#!/usr/bin/env python
from __future__ import division

import roslib
roslib.load_manifest('actuator_source')
import rospy
import threading
import actionlib
import actions.msg
import msg_and_srv.msg


class ActuatorActionServer(object):

    def __init__(self):
        self.lock = threading.Lock()
        self.initialized = False
        self.abort = False

        self.setpt_update_rate = rospy.get_param("actuator_update_rate",50)
        self.max_value = rospy.get_param("actuator_max_value",2000)
        self.min_value = rospy.get_param("actuator_min_value",1000)
        self.default_value = rospy.get_param("actuator_default_value",1500)

        self.rate = rospy.Rate(self.setpt_update_rate)
        self.dt = 1/self.setpt_update_rate

        self.value_final = None

        # Setup action server
        self.feedback = actions.msg.ActuatorOutscanFeedback()
        self.result = actions.msg.ActuatorOutscanResult()
        self.server = actionlib.SimpleActionServer(
                "actuator_action", 
                actions.msg.ActuatorOutscanAction, 
                execute_cb=self.execute_cb, 
                auto_start=False
                )
        self.server.preempt_callback = self.preempt_callback
        self.server.start()

        # Setup actuator topic and initialize
        self.actuator_msg = msg_and_srv.msg.ActuatorMsg()
        self.actuator_pub = rospy.Publisher('actuator', msg_and_srv.msg.ActuatorMsg)
        self.initialized = True

    def preempt_callback(self):
        """
        Used for aborting runs - if a preempt is recieved we abort. This
        happens when the cancel_all_goals client function is called.
        """
        with self.lock:
            self.abort = True

    def execute_cb(self,goal):
        self.abort = False
        count = len(goal.actuator_array)
        secs_to_completion = count*self.dt

        for i, value in enumerate(goal.actuator_array):

            # Check whether or not an abort signal has been sent.
            with self.lock:
                if self.abort:
                    self.result.value = self.value_final
                    self.server.set_aborted(self.result)
                    self.actuator_msg.header.stamp = rospy.get_rostime()
                    self.actuator_msg.type = goal.type
                    self.actuator_msg.value = self.default_value 
                    self.actuator_pub.publish(self.actuator_msg)
                    return

            # Clamp output value between max and min
            value = min([value,self.max_value])
            value = max([value,self.min_value])

            # Publish actuator message
            self.actuator_msg.header.stamp = rospy.get_rostime()
            self.actuator_msg.type = goal.type
            self.actuator_msg.value = value 
            self.actuator_pub.publish(self.actuator_msg)

            # Publish feedback
            secs_to_completion -= self.dt
            percent_complete = 100*i/count
            self.feedback.secs_to_completion = secs_to_completion
            self.feedback.percent_complete = percent_complete
            self.server.publish_feedback(self.feedback)

            self.value_final = value 
            self.rate.sleep()

        if self.value_final is not None:
            self.result.value = self.value_final
            self.server.set_succeeded(self.result)

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    rospy.init_node('actuator_server')
    setpt_action_server = ActuatorActionServer()
    rospy.spin()
