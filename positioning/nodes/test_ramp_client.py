#!/usr/bin/env python
import roslib 
roslib.load_manifest('positioning')
import rospy
import matplotlib.pylab as pylab
from positioning.srv import * 

def get_ramp(pos_0, pos_1, max_velo, accel, dt):
    rospy.wait_for_service('get_ramp')
    get_ramp = rospy.ServiceProxy('get_ramp', GetRamp)
    try:
        response = get_ramp(pos_0,pos_1,max_velo, accel,dt)
    except rospy.ServiceException, e:
        print 'Service call failed: %s'%(e,)
    return response.ramp


# -----------------------------------------------------------------------------
if __name__ == '__main__':
    pos_0 = 0
    pos_1 = 2000.0
    max_velo = 1000.0
    accel = 20.0
    dt = 1.0/50.0
    ramp = get_ramp(pos_0, pos_1, max_velo, accel, dt)
    print 'ramp len: ', len(ramp)
    pylab.plot(ramp)
    pylab.show()
