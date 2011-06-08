#!/usr/bin/env python
import roslib 
roslib.load_manifest('controller')
import rospy
import sys
from controller.srv import ControllerCmd 


def set_mode(mode,delay=0.0):
    rospy.wait_for_service('controller_cmd')
    rospy.sleep(delay)
    try:
        controller_cmd = rospy.ServiceProxy('controller_cmd',ControllerCmd)
        controller_cmd('set mode', mode)
    except rospy.ServiceException, e:
        print 'Service call failed: %s'%(e,)

if __name__ == '__main__':
    mode = sys.argv[1]
    if len(sys.argv) > 2:
        delay = float(sys.argv[2])
    set_mode(mode,delay=delay)

