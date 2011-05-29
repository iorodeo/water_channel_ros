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
# from distance_118x.msg import DistMsg

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
        # self.have_pos = False
        # self.pos = None
        # self.pos_initial = None
        # self.pos_initial = 0

        # Setup subscriber to joystick topic
        self.dist_sub = rospy.Subscriber('joy', Joy, self.joystick_callback)

        # # Setup subscriber to distance topic
        # self.dist_sub = rospy.Subscriber('distance', DistMsg, self.dist_callback)

        # Setup setpt topic
        self.setptMsg = SetptMsg()
        self.setpt_pub = rospy.Publisher('setpt', SetptMsg)

        stamp = rospy.get_rostime()
        self.start_t  = self.get_time()
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
            self.setpt_pub.publish(self.setptMsg)
        # with self.lock:
        #     if self.have_pos == True:
        #         self.pos_setpt = self.pos_initial + cos_pos
        #         self.vel_setpt = cos_vel
        #         self.setptMsg.position = self.pos_setpt
        #         self.setptMsg.velocity = self.vel_setpt
        #         self.setptMsg.error = self.pos_setpt - self.pos
        #         self.setpt_pub.publish(self.setptMsg)

    def joystick_callback(self,data):
        with self.lock:
            self.vel_setpt_goal = data.axes[0]*self.vel_max

    # def dist_callback(self,data):
    #     with self.lock:
    #         if self.have_pos == False:
    #             self.pos_initial = data.distance
    #             self.have_pos = True
    #         self.pos = data.distance

    def get_time(self):
        stamp = rospy.get_rostime()
        return stamp.secs + stamp.nsecs*1.0e-9

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    rospy.init_node('setpt_source')
    setpt = SetptSource()
    while not rospy.is_shutdown():
        setpt.update()
        setpt.rate.sleep()
