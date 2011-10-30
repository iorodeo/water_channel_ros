import roslib 
roslib.load_manifest('utilities')
import rospy
import sys
from msg_and_srv.srv import SledIOCmd 
from msg_and_srv.srv import NodeEnable 

class ModeSwitcher(object):

    def __init__(self):
        rospy.wait_for_service('sled_io_cmd')
        self.sledIOCmdProxy = rospy.ServiceProxy('sled_io_cmd',SledIOCmd)

        rospy.wait_for_service('joystick_enable')
        self.joystickEnableProxy = rospy.ServiceProxy('joystick_enable',NodeEnable)

    def enableSledIO(self):
        resp = self.sledIOCmdProxy('set mode', 'motor_cmd')
        return resp

    def disableSledIO(self):
        resp = self.sledIOCmdProxy('set mode', 'off')
        return resp

    def enableJoystickMode(self):
        resp = self.joystickEnableProxy(True)
        return resp

    def disableJoystickMode(self):
        resp = self.joystickEnableProxy(False)
        return resp



