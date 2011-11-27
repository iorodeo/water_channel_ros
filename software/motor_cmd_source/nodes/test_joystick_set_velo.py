#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('motor_cmd_source')
import rospy
from test_joystick_get_velo import get_max_velo

from msg_and_srv.srv import SetJoystickMax

def set_max_velo(value):
    rospy.wait_for_service('set_joystick_max_velo')
    flag = None
    try:
        proxy = rospy.ServiceProxy('set_joystick_max_velo',SetJoystickMax)
        resp = proxy(value)
        flag = resp.flag
    except rospy.ServiceException, e:
        print 'service call failed: %s'%(str(e),)
    return flag 

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    import sys
    value = float(sys.argv[1])
    flag = set_max_velo(value)
    print 'flag: ', flag
    print 
    value = get_max_velo()
    print 'percent max velo', value

    




