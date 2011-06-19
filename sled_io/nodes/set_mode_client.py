#!/usr/bin/env python
import roslib 
roslib.load_manifest('sled_io')
import rospy
import sys
from sled_io.srv import ControllerCmd 

def set_mode():
    mode = rospy.get_param('controller_mode', 'off')
    delay = rospy.get_param('set_mode_delay', 2.5)
    rospy.wait_for_service('sled_io_cmd')
    rospy.sleep(delay)
    try:
        controller_cmd = rospy.ServiceProxy('sled_io_cmd',ControllerCmd)
        controller_cmd('set mode', mode)
    except rospy.ServiceException, e:
        print 'Service call failed: %s'%(e,)

if __name__ == '__main__':
    set_mode()

