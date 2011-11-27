import roslib 
roslib.load_manifest('utilities')
import rospy
import sys

# Services
from msg_and_srv.srv import SledIOCmd 
from msg_and_srv.srv import NodeEnable 
from msg_and_srv.srv import GetBounds
from msg_and_srv.srv import SetBounds

class RobotControl(object):

    def __init__(self):

        # Set up service proxies
        rospy.wait_for_service('sled_io_cmd')
        self.sledIOCmdProxy = rospy.ServiceProxy('sled_io_cmd',SledIOCmd)

        rospy.wait_for_service('joystick_enable')
        self.joystickEnableProxy = rospy.ServiceProxy('joystick_enable',NodeEnable)

        rospy.wait_for_service('get_bounds')
        self.getBoundsProxy = rospy.ServiceProxy('get_bounds',GetBounds)

        rospy.wait_for_service('set_bounds')
        self.setBoundsProxy = rospy.ServiceProxy('set_bounds',SetBounds)

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



