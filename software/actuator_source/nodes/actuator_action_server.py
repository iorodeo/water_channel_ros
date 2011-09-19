#!/usr/bin/env python
from __future__ import division

import roslib
roslib.load_manifest('actuator_source')
import rospy
import actionlib
import actions.msg
import msg_and_srv.msg


class ActuatorActionServer(object):

    def __init__(self):
        self.initialized = False

        self.setpt_update_rate = rospy.get_param("actuator_update_rate",50)
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
        self.server.start()

        # Setup actuator topic and initialize
        self.actuator_msg = msg_and_srv.msg.ActuatorMsg()
        self.actuator_pub = rospy.Publisher('actuator', msg_and_srv.msg.ActuatorMsg)
        self.initialized = True

    def execute_cb(self,goal):
        count = len(goal.actuator_array)
        secs_to_completion = count*self.dt

        for value in goal.actuator_array:

            # Publish actuator message
            self.actuator_msg.header.stamp = rospy.get_rostime()
            self.actuator_msg.type = goal.type
            self.actuator_msg.value = value 
            self.actuator_pub.publish(self.actuator_msg)

            # Publish feedback
            secs_to_completion -= self.dt
            self.feedback.secs_to_completion = secs_to_completion
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
