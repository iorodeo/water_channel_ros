#!/usr/bin/env python
import roslib 
roslib.load_manifest('file_servers')
import rospy
import sys
from msg_and_srv.srv import LoadPositionArray


def get_position_array(filename):
    rospy.wait_for_service('load_position_array')
    try:
        load_position_array = rospy.ServiceProxy('load_position_array', LoadPositionArray)
        response = load_position_array(filename)
    except rospy.ServiceException, e:
        print 'Service call failed: %s'%(e,)

    print 'status: ', response.status
    print 'message: ', response.message
    print 'coord_frame: ', response.coord_frame
    print 'len(position_array): ', len(response.position_array)
    
# ------------------------------------------------------------------
if __name__ == '__main__':
    filename = sys.argv[1]
    get_position_array(filename)



