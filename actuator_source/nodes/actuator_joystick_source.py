#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('actuator_source')
import rospy
import threading
import math
from joy.msg import Joy
from std_msgs.msg import Header
from actuator_source.msg import ActuatorMsg 

class ActuatorSource(object):

    def __init__(self):
        self.initialized = False
        self.update_rate = rospy.get_param("actuator_update_rate",50)
        self.rate = rospy.Rate(self.update_rate)
        self.dt = 1/self.update_rate

        self.lock =  threading.Lock()

        # Setpt source parameters
        self.max_value = rospy.get_param("actuator_max_value",2000)
        self.min_value = rospy.get_param("actuator_min_value",1000)
        self.actuator_type = rospy.get_param("actuator_type", "pwm0")
        self.actuaotr_type = self.actuator_type.lower()

        self.mid_point_value = 0.5*(self.max_value + self.min_value)
        self.slope = self.max_value - self.mid_point_value
        self.actuator_value = self.mid_point_value

        # Setup subscriber to joystick topic
        self.joystick_sub = rospy.Subscriber('joy', Joy, self.joystick_callback)

        # Setup actuator topic
        self.actuator_msg = ActuatorMsg()
        self.actuator_pub = rospy.Publisher('actuator', ActuatorMsg)

        self.initialized = True

    def update(self):
        self.actuator_msg.header.stamp = rospy.get_rostime()
        self.actuator_msg.type = self.actuator_type
        with self.lock:
            self.actuator_msg.value = self.actuator_value
            self.actuator_pub.publish(self.actuator_msg)

    def joystick_callback(self,data):
        if self.initialized:
            with self.lock:
                self.actuator_value = data.axes[0]*self.slope + self.mid_point_value

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    rospy.init_node('actuator')
    src = ActuatorSource()
    while not rospy.is_shutdown():
        src.update()
        src.rate.sleep()
