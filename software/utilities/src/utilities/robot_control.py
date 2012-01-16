import roslib 
roslib.load_manifest('utilities')
import rospy
import sys
import actionlib
import threading
import functools

# Messages
from msg_and_srv.msg import DistMsg

# Services
from msg_and_srv.srv import SledIOCmd 
from msg_and_srv.srv import GetSledIOMode
from msg_and_srv.srv import NodeEnable 
from msg_and_srv.srv import GetBounds
from msg_and_srv.srv import SetBounds
from msg_and_srv.srv import GetJoystickMax
from msg_and_srv.srv import SetJoystickMax

# Actions
from actions.msg import SetptOutscanAction
from actions.msg import SetptOutscanGoal

# Constants
DFLT_IN_POSITION_TOL = 0.005

class Robot_Control(object):
    """
    Interface for controlling the sled robot. 
    """

    def __init__(self):

        self.lock = threading.Lock()
        self.inPositionTol = DFLT_IN_POSITION_TOL

        # Subscribe to messages
        self.distanceSubscriber = rospy.Subscriber(
                'distance', 
                DistMsg, 
                self.distMsgHandler,
                )

        self.update_rate = rospy.get_param('update_rate')

        # Initialize variables for storing position and velocity. Note These
        # variables (starting with an _) should not be accessed directly.
        # Instead use the position and velocity properties. 
        self._position = None
        self._velocity = None

        # Set up service proxies
        rospy.wait_for_service('sled_io_cmd')
        self.sledIOCmdProxy = rospy.ServiceProxy('sled_io_cmd',SledIOCmd)

        rospy.wait_for_service('get_sled_io_mode')
        self.getSledIOModeProxy = rospy.ServiceProxy('get_sled_io_mode',GetSledIOMode)

        rospy.wait_for_service('joystick_enable')
        self.joystickEnableProxy = rospy.ServiceProxy('joystick_enable',NodeEnable)

        rospy.wait_for_service('get_bounds')
        self.getBoundsProxy = rospy.ServiceProxy('get_bounds',GetBounds)

        rospy.wait_for_service('set_bounds')
        self.setBoundsProxy = rospy.ServiceProxy('set_bounds',SetBounds)

        rospy.wait_for_service('get_joystick_max_velo')
        self.getJoystickMaxVeloProxy = rospy.ServiceProxy(
                'get_joystick_max_velo',
                GetJoystickMax
                )

        rospy.wait_for_service('set_joystick_max_velo')
        self.setJoystickMaxVeloProxy = rospy.ServiceProxy(
                'set_joystick_max_velo',
                SetJoystickMax
                )

        # Setup action clients
        self.setptActionClient = actionlib.SimpleActionClient(
                'setpt_action', 
                SetptOutscanAction
                )
        self.setptActionClient.wait_for_server()

    @property
    def dt(self):
        return 1.0/self.update_rate

    @property
    def position(self):
        """
        Current sled position - handles lock so users don't have to worry
        about it. 
        """
        with self.lock:
            position = self._position
        return position

    @property
    def velocity(self):
        """
        Current seld velocity - handles lock so users don't have to worry 
        about it.
        """
        with self.lock:
            velocity = self._velocity
        return velocity

    def distMsgHandler(self,data):
        """
        Handler for messages from the distance sensor. Gets the current
        position and velocity form the distance sensor.
        """
        with self.lock:
            self._position = data.distance_kalman
            self._velocity = data.velocity_kalman

    def enableSledIO(self):
        """
        Enable io to sled electronics
        """
        resp = self.sledIOCmdProxy('set mode', 'motor_cmd')

    def disableSledIO(self):
        """
        Disable io to sled electronics
        """
        resp = self.sledIOCmdProxy('set mode', 'off')

    def getSledIOMode(self):
        """
        Returns the current sled IO mode. Currently there are only two
        possible values - off, and motor_cmd.
        """
        resp = self.getSledIOModeProxy()
        return resp.sled_io_mode

    def isSledIOEnabled(self):
        """
        Checks whether or not sled io is enabled and returns True or False.
        Current sled io is considered to be disabled if the sled_io_node is in
        'off' mode and it is considered to be enabled otherwise.        
        """
        mode = self.getSledIOMode()
        if mode.lower() == 'off':
            return False
        else:
            return True

    def enableJoystickMode(self):
        """
        Enable joystick control mode. 
        """
        resp = self.joystickEnableProxy(True)
        status = resp.status
        message = resp.message
        return status, message 

    def disableJoystickMode(self):
        """
        Disable joystick control mode
        """
        resp = self.joystickEnableProxy(False)
        status = resp.status
        message = resp.message
        return status, message 

    def getBounds(self):
        """
        Get the current bounds settings for the sled.

        resp = (lower, upper, lower_min, upper_max)
        """
        resp = self.getBoundsProxy()
        lower = resp.lower_bound
        upper = resp.upper_bound
        lowerMin = resp.lower_bound_min
        upperMax = resp.upper_bound_max
        return lower, upper, lowerMin, upperMax 

    def setBounds(self,lower,upper):
        """
        Set the current sled bounds. 

        resp = True for success, False for failure.
        """
        resp = self.setBoundsProxy(lower,upper)
        return resp.flag

    def getJoystickMaxVelo(self):
        """
        Returns the current percent max velocity setting for the
        motor_cmd_joystick_source node.
        """
        resp = self.getJoystickMaxVeloProxy()
        return resp.percent_max_velo


    def setJoystickMaxVelo(self,percent_max_velo):
        """
        Sets the current percent max velocity setting for the
        motor_cmd_joystick_source node.

        Return True for success and False for failure
        """
        resp = self.setJoystickMaxVeloProxy(percent_max_velo)
        return resp.flag

    def startSetptOutscan(self,setptValues,feedback_cb=None,done_cb=None):
        """
        Starts a setpt outscan on the setpt action server using
        the setpt action client.
        """
        startPos = setptValues[0]
        if abs(self.position - startPos) > self.inPositionTol:
            # The sled is not at the start position for the setpt
            # trajectory - abort run.
            raise ValueError, 'unable to start outscan - not in position'
        else:
            # We are good to go ... setup action server goal and send
            # it to to the setpt action server.
            done_cb_wrapped = wrapOutscanDone(done_cb)

            # Setup goal
            goal = SetptOutscanGoal()
            goal.coord_frame = 'absolute'
            goal.position_array = list(setptValues)

            # Send gaol to action server
            self.setptActionClient.send_goal(
                    goal,
                    feedback_cb=feedback_cb,
                    done_cb=done_cb_wrapped
                    )

    def stopSetptOutscan(self):
        """
        Stops the current setpt outscan on the setpt action server.
        """
        self.setptActionClient.cancel_all_goals()

    def startActuatorOutscan(self,actuatorArray,callback):
        pass

    def stopActuatorOutscan(self):
        pass
    

# Decorators
#-------------------------------------------------------------------------------------
def wrapOutscanDone(f):
    functools.wraps(f)
    def decorated(state,result):
        state = actionState2Str(state)
        f(state,result)
        return
    return decorated

# Utility
#---------------------------------------------------------------------------------------
actionStateTable = {
        0: 'pending',
        1: 'active',
        2: 'preempted',
        3: 'succeeded',
        4: 'aborted',
        5: 'rejected',
        6: 'preempting',
        7: 'recalling',
        8: 'recalled',
        9: 'lost',
        }

def actionState2Str(state):
    try:
        stateStr = actionStateTable[state]
    except KeyError:
        stateStr = 'unknown'
    return stateStr
