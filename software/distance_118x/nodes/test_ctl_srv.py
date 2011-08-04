#!/usr/bin/env python
import roslib
roslib.load_manifest('distance_118x')
import sys
import rospy
from msg_and_srv.srv import *

def set_laser(state):

    if not state in ('on', 'off'):
        return

    rospy.wait_for_service('distance_ctl')
    try:
        distance_ctl = rospy.ServiceProxy('distance_ctl', DistSensorCtl)
        distance_ctl('laser', state)
    except rospy.ServiceException, e:
        print 'Service call failed: %s'%e

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    state = sys.argv[1]
    set_laser(state)
        




