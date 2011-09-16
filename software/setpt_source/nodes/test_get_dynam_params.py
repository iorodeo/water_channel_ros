#!/usr/bin/env python
"""
test_get_dynam_params.py

This node test the get_dynam_params service of the setpt_dynamics_source node

"""
import roslib 
roslib.load_manifest('setpt_source')
import rospy
from msg_and_srv.srv import GetDynamParams 

def get_dynam_params():
    rospy.wait_for_service('get_dynam_params')
    try: 
        get_cmd = rospy.ServiceProxy('get_dynam_params',GetDynamParams)
        resp = get_cmd()
        print 'mass:', resp.mass
        print 'damping:', resp.damping
    except rospy.ServiceException, e:
        print 'Service call failed: %s'%(e,)

if __name__ == '__main__':

    import sys

    # Send values to node
    get_dynam_params()
