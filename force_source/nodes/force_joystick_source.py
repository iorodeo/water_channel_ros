#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('force_source')
import rospy
import threading
import math
from joy.msg import Joy
from std_msgs.msg import Header
from force_source.msg import ForceMsg

class ForceSource(object):

    def __init__(self):
        self.initialized = False
        self.force_update_rate = rospy.get_param("force_update_rate",50)
        self.rate = rospy.Rate(self.force_update_rate)
        self.dt = 1.0/self.force_update_rate
        self.lock =  threading.Lock()

        # Force source parameters
        self.force = 0.0 
        self.force_calibration = rospy.get_param("force_calibration", 10.0)

        # Setup subscriber to joystick topic
        self.joystick_sub = rospy.Subscriber('joy', Joy, self.joystick_callback)

        # Setup force topic
        self.forceMsg = ForceMsg()
        self.force_pub = rospy.Publisher('force', ForceMsg)

        self.initialized = True

    def update(self):
        self.forceMsg.header.stamp = rospy.get_rostime()
        with self.lock:
            self.forceMsg.dt = self.dt
            self.forceMsg.force = self.force
            self.force_pub.publish(self.forceMsg)

    def joystick_callback(self,data):
        if self.initialized:
            with self.lock:
                self.force = data.axes[0]*self.force_calibration


# -----------------------------------------------------------------------------
if __name__ == '__main__':
    rospy.init_node('force')
    setpt = ForceSource()
    while not rospy.is_shutdown():
        setpt.update()
        setpt.rate.sleep()
