#!/usr/bin/env python
"""
logger_enable_test.py

This node tests logger_enable service provided by the various logging nodes. 
"""
import roslib 
roslib.load_manifest('data_loggers')
import rospy
from msg_and_srv.srv import NodeEnable 

def node_enable(value):
    rospy.wait_for_service('logger_enable')
    try: 
        enable_cmd = rospy.ServiceProxy('logger_enable',NodeEnable)
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

    # Send value to logger node
    node_enable(value)

