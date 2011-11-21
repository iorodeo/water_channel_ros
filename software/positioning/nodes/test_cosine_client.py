#!/usr/bin/env python
import roslib 
roslib.load_manifest('positioning')
import rospy
import scipy
import matplotlib.pylab as pylab
from msg_and_srv.srv import GetCosine

def get_cosine(amplitude,period,cycles,dt):
    rospy.wait_for_service('get_cosine')
    get_cosine_proxy = rospy.ServiceProxy('get_cosine', GetCosine)
    response = None
    try:
        response = get_cosine_proxy(amplitude,period,cycles,dt)
    except rospy.ServiceException, e:
        print 'Service call failed: %s'%(e,)
    return response.cosine

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    amplitude = 2.5
    period = 10.0
    cycles = 4
    dt = 1.0/50.0
    cosine = get_cosine(amplitude,period,cycles,dt)
    t = scipy.arange(len(cosine))*dt
    pylab.plot(t,cosine)
    pylab.show()
