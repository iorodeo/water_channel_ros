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
        self.action_server = actionlib.SimpleActionServer("setpt", setpt_source.msg.SetptAction, execute_cb=self.execute_cb, auto_start=False)
        self.action_server.start()

        # Setup setpt and setpt_rel topics
        self.setptMsg = setpt_source.msg.SetptMsg()
        self.setpt_abs_pub = rospy.Publisher('setpt', setpt_source.msg.SetptMsg)
        self.setpt_rel_pub = rospy.Publisher('setpt_rel', setpt_source.msg.SetptMsg)

        self.initialized = True

    def execute_cb(self,goal):
        coord_frame = goal.coord_frame.lower()
        if 'abs' in coord_frame:
            self.pub = self.setpt_abs_pub
        elif 'rel' in coord_frame:
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

  # # create messages that are used to publish feedback/result
  # _feedback = actionlib_tutorials.msg.FibonacciFeedback()
  # _result   = actionlib_tutorials.msg.FibonacciResult()

  # def __init__(self, name):
  #   self._action_name = name
  #   self._as = actionlib.SimpleActionServer(self._action_name, actionlib_tutorials.msg.FibonacciAction, execute_cb=self.execute_cb)
  #   self._as.start()

  # def execute_cb(self, goal):
  #   # helper variables
  #   r = rospy.Rate(1)
  #   success = True

  #   # append the seeds for the fibonacci sequence
  #   self._feedback.sequence = []
  #   self._feedback.sequence.append(0)
  #   self._feedback.sequence.append(1)

  #   # publish info to the console for the user
  #   rospy.loginfo('%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i' % (self._action_name, goal.order, self._feedback.sequence[0], self._feedback.sequence[1]))

  #   # start executing the action
  #   for i in xrange(1, goal.order):
  #     # check that preempt has not been requested by the client
  #     if self._as.is_preempt_requested():
  #       rospy.loginfo('%s: Preempted' % self._action_name)
  #       self._as.set_preempted()
  #       success = False
  #       break
  #     self._feedback.sequence.append(self._feedback.sequence[i] + self._feedback.sequence[i-1])
  #     # publish the feedback
  #     self._as.publish_feedback(self._feedback)
  #     # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
  #     r.sleep()

  #   if success:
  #     self._result.sequence = self._feedback.sequence
  #     rospy.loginfo('%s: Succeeded' % self._action_name)
  #     self._as.set_succeeded(self._result)


# -----------------------------------------------------------------------------
if __name__ == '__main__':
    rospy.init_node('setpt_action_server')
    setpt_action_server = SetptActionServer()
    rospy.spin()
