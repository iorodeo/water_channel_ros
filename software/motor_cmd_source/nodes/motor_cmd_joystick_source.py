#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('motor_cmd_source')
import rospy
import threading
import math

# Messages
from joy.msg import Joy
from std_msgs.msg import Header
from msg_and_srv.msg import MotorCmdMsg

# Services
from msg_and_srv.srv import NodeEnable
from msg_and_srv.srv import NodeEnableResponse
from msg_and_srv.srv import GetJoystickMax
from msg_and_srv.srv import GetJoystickMaxResponse
from msg_and_srv.srv import SetJoystickMax
from msg_and_srv.srv import SetJoystickMaxResponse

class MotorCmd_Joystick_Source(object):
    """
    A class for generating motor commands from a joystick input device.
    """
    def __init__(self):
        self.initialized = False
        self.setpt_update_rate = rospy.get_param("update_rate",50.0)
        self.rate = rospy.Rate(self.setpt_update_rate)
        self.dt = 1/self.setpt_update_rate
        self.lock =  threading.Lock()
        self.enabled = False
        self.motor_cmd = 0.0

        # Setpt source parameters
        self.max_motor_cmd = rospy.get_param('motor_cmd_max_value',4095)
        self.percent_max_velo = rospy.get_param('joystick_max_velo_default', 25.0)

        # Setup subscriber to joystick topic
        self.joystick_sub = rospy.Subscriber('joy', Joy, self.joystick_callback)

        # Setup setpt topic
        self.motor_cmd_msg = MotorCmdMsg()
        self.motor_cmd_pub = rospy.Publisher('motor_cmd', MotorCmdMsg)

        # Setup enable/disable service 
        self.node_enable_srv = rospy.Service(
                'joystick_enable', 
                NodeEnable, 
                self.handle_node_enable
                ) 

        # Setup get/set max velocity services
        self.get_max_velo_srv = rospy.Service(
                'get_joystick_max_velo',
                GetJoystickMax,
                self.handle_get_max_velo
                )

        self.set_max_velo_srv = rospy.Service(
                'set_joystick_max_velo',
                SetJoystickMax,
                self.handle_set_max_velo
                )

        self.initialized = True

    def handle_node_enable(self,req):
        """
        Handles node enable and disable requests
        """
        with self.lock:
            if req.enable:
                self.enabled = True
            else:
                self.enabled = False
        message = ''
        return NodeEnableResponse(self.enabled,message)

    def handle_get_max_velo(self,req):
        """
        Handles requests for getting the current percent maximum velocity
        setting for the node.
        """
        with self.lock:
            return GetJoystickMaxResponse(self.percent_max_velo)

    def handle_set_max_velo(self,req):
        """
        Handles requests for setting the current percent maximum velocity
        setting.
        """
        flag = False
        with self.lock:
            if req.percent_max_velo >= 0 and req.percent_max_velo <= 100:
                self.percent_max_velo = req.percent_max_velo
                flag = True
        return SetJoystickMaxResponse(flag)

    def joystick_callback(self,data):
        """
        Callback function for joystick messages
        """
        if self.initialized:
            with self.lock:
                self.max_value = 0.01*self.percent_max_velo*self.max_motor_cmd
                self.motor_cmd = data.axes[0]*self.max_value

    def update(self):
        """
        Update function - publishes motor command messages
        """
        with self.lock:
            if self.enabled:
                self.motor_cmd_msg.header.stamp = rospy.get_rostime()
                self.motor_cmd_msg.motor_cmd = self.motor_cmd
                self.motor_cmd_pub.publish(self.motor_cmd_msg)

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    rospy.init_node('motor_cmd')
    src = MotorCmd_Joystick_Source()
    while not rospy.is_shutdown():
        src.update()
        src.rate.sleep()
