#!/usr/bin/env python
from __future__ import division

import roslib
roslib.load_manifest('setpt_source')
import rospy
import actionlib

import setpt_source.msg

# import threading
# import math
# from std_msgs.msg import Header


class SetptActionServer(object):
    def __init__(self):
        self.initialized = False

        self.setpt_update_rate = rospy.get_param("setpt_update_rate",50)
        self.rate = rospy.Rate(self.setpt_update_rate)
        self.dt = 1/self.setpt_update_rate

        self.position_final = None

        # Setup action server
        self.feedback = setpt_source.msg.SetptFeedback()
        self.result = setpt_source.msg.SetptResult()
        self.action_server = actionlib.SimpleActionServer("setpt_action", setpt_source.msg.SetptAction, execute_cb=self.execute_cb, auto_start=False)
        self.action_server.start()

        # Setup setpt and setpt_rel topics
        self.setptMsg = setpt_source.msg.SetptMsg()
        self.setpt_abs_pub = rospy.Publisher('setpt', setpt_source.msg.SetptMsg)
        self.setpt_rel_pub = rospy.Publisher('setpt_rel', setpt_source.msg.SetptMsg)

        self.initialized = True

    def execute_cb(self,goal):
        coord_frame = goal.coord_frame.lower()
        print coord_frame
        if coord_frame == 'absolute':
            self.pub = self.setpt_abs_pub
        elif coord_frame == 'relative':
            self.pub = self.setpt_rel_pub
        else:
            raise TypeError('Unknown coord_frame type')

        position_count = len(goal.position_array)
        secs_to_completion = position_count * self.dt

        for position in goal.position_array:
            self.setptMsg.header.stamp = rospy.get_rostime()
            self.setptMsg.position = position
            self.pub.publish(self.setptMsg)
            secs_to_completion -= self.dt
            self.feedback.secs_to_completion = secs_to_completion
            self.action_server.publish_feedback(self.feedback)
            self.position_final = position
            self.rate.sleep()

        if self.position_final is not None:
            self.result.position = self.position_final
            self.action_server.set_succeeded(self.result)

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    rospy.init_node('setpt_action_server')
    setpt_action_server = SetptActionServer()
    rospy.spin()
