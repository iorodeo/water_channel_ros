#!/usr/bin/env python
"""
test_set_dynam_params.py

This node test the set_dynam_params service of the setpt_dynamics_source node

"""
import roslib 
roslib.load_manifest('setpt_source')
import rospy
from msg_and_srv.srv import SetDynamParams 

def set_dynam_params(mass,damping):
    rospy.wait_for_service('set_dynam_params')
    try: 
        set_cmd = rospy.ServiceProxy('set_dynam_params',SetDynamParams)
        resp = set_cmd(mass,damping)
        print resp
    except rospy.ServiceException, e:
        print 'Service call failed: %s'%(e,)

if __name__ == '__main__':

    import sys

    # Get mass and damping from command line arguments
    mass = float(sys.argv[1])
    damping = float(sys.argv[2])

    # Send values to node
    set_dynam_params(mass,damping)
