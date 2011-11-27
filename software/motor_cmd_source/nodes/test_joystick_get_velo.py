#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('motor_cmd_source')
import rospy

from msg_and_srv.srv import GetJoystickMax

def get_max_velo():
    rospy.wait_for_service('get_joystick_max_velo')
    value = None
    try:
        proxy = rospy.ServiceProxy('get_joystick_max_velo',GetJoystickMax)
        resp = proxy()
        value = resp.percent_max_velo
    except rospy.ServiceException, e:
        print 'service call failed: %s'%(str(e),)
    return value

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    value = get_max_velo()
    print value



