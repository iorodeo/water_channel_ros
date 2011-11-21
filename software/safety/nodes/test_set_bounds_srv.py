import roslib
roslib.load_manifest('safety')
import rospy
from test_get_bounds_srv import get_bounds

# Services
from msg_and_srv.srv import SetBounds

def set_bounds(lower_bound,upper_bound):
    rospy.wait_for_service('set_bounds')
    response = None
    try:
        set_bounds_proxy = rospy.ServiceProxy('set_bounds',SetBounds)
        response = set_bounds_proxy(lower_bound,upper_bound)
    except rospy.ServiceException, e:
        print 'sevice call failed: %s'%(str(e),)
    return response

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    print 'get original bounds'
    orig_bounds = get_bounds()
    print 'original bounds:'
    print orig_bounds
    print 

    print 'set bounds to new values'
    new_lower_bound = 500
    new_upper_bound = 6000
    response = set_bounds(new_lower_bound,new_upper_bound)
    new_bounds = get_bounds()
    print 'set bounds reponse:', response
    print 'new bounds:'
    print new_bounds
    print

    print 'reset bounds to original values'
    lower_bound = orig_bounds.lower_bound
    upper_bound = orig_bounds.upper_bound
    reponse = set_bounds(lower_bound,upper_bound)
    new_bounds = get_bounds()
    print 'set bounds reponse:', response
    print 'new bounds:'
    print new_bounds
    print

