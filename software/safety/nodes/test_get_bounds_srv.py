import roslib
roslib.load_manifest('safety')
import rospy

# Services
from msg_and_srv.srv import GetBounds

def get_bounds():
    rospy.wait_for_service('get_bounds')
    response = None
    try:
        get_bounds_proxy = rospy.ServiceProxy('get_bounds',GetBounds)
        response = get_bounds_proxy()
    except rospy.ServiceException, e:
        print 'sevice call failed: %s'%(str(e),)
    return response

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    response = get_bounds()
    print 'get_bounds reponse:'
    print response
