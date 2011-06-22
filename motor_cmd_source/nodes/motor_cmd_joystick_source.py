#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('motor_cmd_source')
import rospy
import threading
import math
from joy.msg import Joy
from std_msgs.msg import Header
from motor_cmd_source.msg import MotorCmdMsg 

class MotorCmdSource(object):

    def __init__(self):
        self.initialized = False
        self.setpt_update_rate = rospy.get_param("setpt_update_rate",50)
        self.rate = rospy.Rate(self.setpt_update_rate)
        self.dt = 1/self.setpt_update_rate

        self.lock =  threading.Lock()

        self.motor_cmd = 0.0

        # Setpt source parameters
        self.maxValue = rospy.get_param("motor_cmd_max_value",2000)
        self.minValue = -self.maxValue

        # Setup subscriber to joystick topic
        self.joystick_sub = rospy.Subscriber('joy', Joy, self.joystick_callback)

        # Setup setpt topic
        self.motor_cmd_msg = MotorCmdMsg()
        self.motor_cmd_pub = rospy.Publisher('motor_cmd', MotorCmdMsg)

        self.initialized = True

    def update(self):
        self.motor_cmd_msg.header.stamp = rospy.get_rostime()
        with self.lock:
            self.motor_cmd_msg.motor_cmd = self.motor_cmd
            self.motor_cmd_pub.publish(self.motor_cmd_msg)

    def joystick_callback(self,data):
        if self.initialized:
            with self.lock:
                print data.axes[0]
                self.motor_cmd = data.axes[0]*self.maxValue

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    rospy.init_node('motor_cmd')
    src = MotorCmdSource()
    while not rospy.is_shutdown():
        src.update()
        src.rate.sleep()
