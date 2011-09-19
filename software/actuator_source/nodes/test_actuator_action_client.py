#! /usr/bin/env python
from __future__ import division

import roslib
roslib.load_manifest('actuator_source')
import rospy
import actionlib
import actions.msg

def actuator_action_client():

    # Creates the SimpleActionClient, passing the type of the action
    # (SetptAction) to the constructor.
    client = actionlib.SimpleActionClient(
            'actuator_action', 
            actions.msg.ActuatorOutscanAction
            )

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    print client.__dict__


    # Creates a goal to send to the action server.
    goal = actions.msg.ActuatorOutscanGoal()
    goal.type = 'pwm0'
    goal.actuator_array = range(1000)

    # Sends the goal to the action server.
    client.send_goal(goal,feedback_cb=feedback_cb)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  

def feedback_cb(data):
    print 'secs to completion %1.2f'%(data.secs_to_completion,)

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('actuator_action_client')
        result = actuator_action_client()
        print "Result: " + str(result)
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
