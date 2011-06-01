#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('setpt_source')
import rospy
import threading
import math
from setpt_source.msg import SetptMsg
from distance_118x.msg import DistMsg

class SetptRelToAbs(object):

    def __init__(self):
        self.initialized = False

        self.lock =  threading.Lock()

        # Setpt source parameters
        self.have_pos = False
        self.pos = None
        self.pos_initial = None
        #self.pos_initial = 10000

        # Setup subscriber to setpt_rel topic
        self.setpt_rel_sub = rospy.Subscriber('setpt_rel', SetptMsg, self.setpt_rel_callback)

        # # Setup subscriber to distance topic
        self.dist_sub = rospy.Subscriber('distance', DistMsg, self.dist_callback)

        # Setup setpt topic
        self.setptMsg = SetptMsg()
        self.setpt_pub = rospy.Publisher('setpt', SetptMsg)

        self.initialized = True

    def setpt_rel_callback(self,data):
        if self.initialized:
            self.setptMsg.header = data.header
            with self.lock:
                if self.have_pos:
                    self.setptMsg.velocity = data.velocity
                    self.setptMsg.position = data.position + self.pos_initial
                    self.setptMsg.error = self.setptMsg.position - self.pos 
                    self.setpt_pub.publish(self.setptMsg)

    def dist_callback(self,data):
        with self.lock:
            if self.have_pos == False:
                self.pos_initial = data.distance
                self.have_pos = True
            self.pos = data.distance


# -----------------------------------------------------------------------------
if __name__ == '__main__':
    rospy.init_node('setpt_rel_to_abs')
    setpt_rel_to_abs = SetptRelToAbs()
    rospy.spin()
