#! /usr/bin/env python
from __future__ import division

import roslib
roslib.load_manifest('setpt_source')
import rospy
import actionlib

import msg_and_srv.msg

def setpt_action_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (SetptAction) to the constructor.
    client = actionlib.SimpleActionClient('setpt_action', msg_and_srv.msg.SetptAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = msg_and_srv.msg.SetptGoal()
    goal.coord_frame = 'relative'
    goal.position_array = range(1000)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A SetptResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('setpt_action_client')
        result = setpt_action_client()
        print "Result: " + str(result)
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
