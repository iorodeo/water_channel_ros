#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('setpt_source')
import rospy
import threading
import actionlib
import actions.msg
import msg_and_srv.msg


class SetptActionServer(object):
    """
    Action server, based on SimpleActionServer, for outscanning arrays of
    set points.
    """

    def __init__(self):
        self.lock = threading.Lock()
        self.initialized = False
        self.abort = False

        self.setpt_update_rate = rospy.get_param("setpt_update_rate",50)
        self.rate = rospy.Rate(self.setpt_update_rate)
        self.dt = 1/self.setpt_update_rate

        self.position_final = None

        # Setup action server
        self.feedback = actions.msg.SetptOutscanFeedback()
        self.result = actions.msg.SetptOutscanResult()
        self.server = actionlib.SimpleActionServer(
                "setpt_action", 
                actions.msg.SetptOutscanAction, 
                execute_cb=self.execute_cb, 
                auto_start=False
                )
        self.server.preempt_callback = self.preempt_callback
        self.server.start()

        # Setup setpt and setpt_rel topics
        self.setptMsg = msg_and_srv.msg.SetptMsg()
        self.setpt_abs_pub = rospy.Publisher('setpt', msg_and_srv.msg.SetptMsg)
        self.setpt_rel_pub = rospy.Publisher('setpt_rel', msg_and_srv.msg.SetptMsg)

        self.initialized = True

    def preempt_callback(self):
        """
        Used for aborting runs - if a preempt is recieved we abort. This
        happens when the cancel_all_goals client function is called.
        """
        with self.lock:
            self.abort = True

    def execute_cb(self,goal): 
        """
        Callback for executing goal - outscans array out setpt values. 
        """
        self.abort = False
        coord_frame = goal.coord_frame.lower()
        if coord_frame == 'absolute':
            self.pub = self.setpt_abs_pub
        elif coord_frame == 'relative':
            self.pub = self.setpt_rel_pub
        else:
            raise TypeError('Unknown coord_frame type')

        position_count = len(goal.position_array)
        secs_to_completion = position_count * self.dt

        for i, position in enumerate(goal.position_array):

            # Check whether or not an abort signal has been sent.
            with self.lock:
                if self.abort:
                    self.result.position = self.position_final
                    self.server.set_aborted(self.result)
                    return

            # Publish setpt message
            self.setptMsg.header.stamp = rospy.get_rostime()
            self.setptMsg.position = position
            self.pub.publish(self.setptMsg)

            # Publish feedback message
            secs_to_completion -= self.dt
            percent_complete = (100*i)/position_count
            self.feedback.secs_to_completion = secs_to_completion
            self.feedback.percent_complete = percent_complete
            self.server.publish_feedback(self.feedback)

            self.position_final = position
            self.rate.sleep()

        if self.position_final is not None:
            self.result.position = self.position_final
            self.server.set_succeeded(self.result)

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    rospy.init_node('position_server')
    setpt_action_server = SetptActionServer()
    rospy.spin()
