#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('setpt_source')
import rospy
import threading
import math
from joy.msg import Joy
from std_msgs.msg import Header
from setpt_source.msg import SetptMsg

class SetptSource(object):

    def __init__(self):
        self.initialized = False
        self.setpt_update_rate = rospy.get_param("setpt_update_rate",50)
        self.rate = rospy.Rate(self.setpt_update_rate)
        self.dt = 1/self.setpt_update_rate

        self.lock =  threading.Lock()

        # Setpt source parameters
        self.vel_max = rospy.get_param("velocity_max",1000)
        self.acc_max = rospy.get_param("acceleration_max",1000)
        self.vel_setpt = 0
        self.vel_setpt_goal = 0
        self.pos_setpt = 0

        # Setup subscriber to joystick topic
        self.joystick_sub = rospy.Subscriber('joy', Joy, self.joystick_callback)

        # Setup setpt topic
        self.setptMsg = SetptMsg()
        self.setpt_rel_pub = rospy.Publisher('setpt_rel', SetptMsg)

        self.initialized = True

    def update(self):
        self.setptMsg.header.stamp = rospy.get_rostime()
        with self.lock:
            acc = (self.vel_setpt_goal - self.vel_setpt)/self.dt
            if self.acc_max < abs(acc):
                acc = math.copysign(self.acc_max,acc)
            vel_inc = acc*self.dt
            self.vel_setpt += vel_inc
            pos_inc = self.vel_setpt*self.dt
            self.pos_setpt += pos_inc
            self.setptMsg.velocity = self.vel_setpt
            self.setptMsg.position = self.pos_setpt
            self.setpt_rel_pub.publish(self.setptMsg)

    def joystick_callback(self,data):
        if self.initialized:
            with self.lock:
                self.vel_setpt_goal = data.axes[0]*self.vel_max

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    rospy.init_node('setpt_source')
    setpt = SetptSource()
    while not rospy.is_shutdown():
        setpt.update()
        setpt.rate.sleep()
