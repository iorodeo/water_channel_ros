#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('setpt_source')
import rospy
import threading
import math
from std_msgs.msg import Header
from setpt_source.msg import SetptMsg
from force_source.msg import ForceMsg

import dynamics

class DynamicsSetptSource(object):

    def __init__(self):
        mass = rospy.get_param("mass", 1.0)
        damping = rospy.get_param("damping", 0.0)

        self.dynamics = dynamics.Dynamics()
        self.dynamics.mass = mass
        self.dynamics.damping = damping
        self.dynamics.position = 0.0
        self.dynamics.velocity = 0.0

        # Setup subscriber to force topic
        self.force_sub = rospy.Subscriber('force', ForceMsg, self.force_callback)

        # Setup setpt topic
        self.setptMsg = SetptMsg()
        self.setptPub = rospy.Publisher('setpt_rel', SetptMsg)

    def force_callback(self,data):
        self.dynamics.update(data.force, data.dt)
        self.setptMsg.header.stamp = rospy.get_rostime()
        self.setptMsg.position = self.dynamics.position
        self.setptMsg.velocity = self.dynamics.velocity
        self.setptPub.publish(self.setptMsg)

if __name__ == '__main__':
    rospy.init_node('dynamics')
    dynamics_node = DynamicsSetptSource()
    rospy.spin()
# 
