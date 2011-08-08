#! /usr/bin/env python
from __future__ import division

import roslib
roslib.load_manifest('setpt_source')
import rospy
import actionlib
import action_servers.msg
from msg_and_srv.srv import GetRamp
from msg_and_srv.srv import RelToAbsCmd
import sys

def ramp_action_client(pos,max_velo=500.0,accel=100.0):
    setpt_update_rate = rospy.get_param("setpt_update_rate",50)
    dt = 1.0/setpt_update_rate

    # Create the action client     
    client = actionlib.SimpleActionClient('setpt_action', action_servers.msg.SetptAction)
    client.wait_for_server()

    # Service Proxies 
    rospy.wait_for_service('get_ramp')
    get_ramp = rospy.ServiceProxy('get_ramp', GetRamp)
    rospy.wait_for_service('setpt_rel_cmd')
    rel_to_abs_cmd = rospy.ServiceProxy('setpt_rel_cmd', RelToAbsCmd)

    # Get ramp array
    response = get_ramp(0.0,pos,max_velo, accel,dt)
    ramp = response.ramp

    # Reset Rel2Abs server
    rtn_flag = rel_to_abs_cmd('reset')
    print rtn_flag
    if rtn_flag == False:
        raise IOError, "unable to reset setpt_rel_to_abs"

    # Creates a goal to send to the action server.
    goal = action_servers.msg.SetptGoal()
    goal.coord_frame = 'relative'
    goal.position_array = ramp 

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A SetptResult

if __name__ == '__main__':

    pos_step = float(sys.argv[1])

    try:
        rospy.init_node('ramp_action_client')
        result = ramp_action_client(pos_step)
        print "Result: " + str(result)
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
