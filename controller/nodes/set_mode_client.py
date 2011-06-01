#!/usr/bin/env python
import roslib 
roslib.load_manifest('controller')
import rospy
import sys
from controller.srv import ControllerCmd 


def set_mode(mode):
    rospy.wait_for_service('controller_cmd')
    try:
        controller_cmd = rospy.ServiceProxy('controller_cmd',ControllerCmd)
        controller_cmd('set mode', mode)
    except rospy.ServiceException, e:
        print 'Service call failed: %s'%(e,)

if __name__ == '__main__':
    mode = sys.argv[1]
    set_mode(mode)

