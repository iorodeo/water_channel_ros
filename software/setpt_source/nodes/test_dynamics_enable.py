#!/usr/bin/env python
"""
test_dynamics_enable.py

This node tests dynamics_enable service provided by the setpt_dynamics_source
node. 
"""
import roslib 
roslib.load_manifest('setpt_source')
import rospy
from msg_and_srv.srv import NodeEnable 

def node_enable(value):
    rospy.wait_for_service('dynamics_enable')
    try: 
        enable_cmd = rospy.ServiceProxy('dynamics_enable',NodeEnable)
        resp = enable_cmd(value)
        print resp
    except rospy.ServiceException, e:
        print 'Service call failed: %s'%(e,)

if __name__ == '__main__':

    import sys

    # Get value (True/False) from command line
    value_str = sys.argv[1].lower()
    if value_str == 'true':
        value = True
    else:
        value = False

    # Send value to node
    node_enable(value)

