import roslib 
roslib.load_manifest('utilities')
import rospy
import sys

# Services
from msg_and_srv.srv import SledIOCmd 
from msg_and_srv.srv import GetSledIOMode
from msg_and_srv.srv import NodeEnable 
from msg_and_srv.srv import GetBounds
from msg_and_srv.srv import SetBounds
from msg_and_srv.srv import GetJoystickMax
from msg_and_srv.srv import SetJoystickMax

class RobotControl(object):

    def __init__(self):

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



