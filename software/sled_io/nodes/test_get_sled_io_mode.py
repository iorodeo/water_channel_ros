#!/usr/bin/env python
import roslib 
roslib.load_manifest('sled_io')
import rospy

from msg_and_srv.srv import GetSledIOMode


def get_sled_io_mode():
    rospy.wait_for_service('get_sled_io_mode')
    mode = None
    try:
        proxy = rospy.ServiceProxy('get_sled_io_mode',GetSledIOMode)
        resp = proxy()
        mode = resp.sled_io_mode
    except rospy.ServiceException,e:
        print 'service request failed: %s'%(str(e),)
    return mode

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    mode = get_sled_io_mode()
    print 
    print 'mode = ', mode
    print
