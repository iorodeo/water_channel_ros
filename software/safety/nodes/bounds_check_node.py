#!/usr/bin/env python
import roslib
roslib.load_manifest('safety')
import rospy
import threading
import math

# Messages
from msg_and_srv.msg import DistMsg

# Services
from msg_and_srv.srv import SledIOCmd 
from msg_and_srv.srv import GetBounds
from msg_and_srv.srv import GetBoundsResponse
from msg_and_srv.srv import SetBounds
from msg_and_srv.srv import SetBoundsResponse

DEFAULT_LOWER_BOUND = 2.0
DEFAULT_UPPER_BOUND = 4.0
DEFAULT_LOWER_BOUND_MIN = 1.0
DEFAULT_UPPER_BOUND_MAX = 30.0

class BoundsChecker(object):
    """
    Bounds checker node - monitors distance data and checks whether or
    not the sled is within the user defined boundaries. If not it shuts
    down the sled io. 
    """

    def __init__(self):

        self.lock = threading.Lock()

        # Get upper and lower distance bounds
        self.lower_bound = rospy.get_param('dist_lower_bound', DEFAULT_LOWER_BOUND)
        self.upper_bound = rospy.get_param('dist_upper_bound', DEFAULT_UPPER_BOUND)
        self.lower_bound_min = rospy.get_param('dist_lower_bound_min', DEFAULT_LOWER_BOUND_MIN)
        self.upper_bound_max = rospy.get_param('dist_upper_bound_max', DEFAULT_UPPER_BOUND_MAX)
        
        # Set up get and set bounds services
        self.get_bounds_srv = rospy.Service('get_bounds',GetBounds, self.handle_get_bounds)
        self.set_bounds_srv = rospy.Service('set_bounds',SetBounds, self.handle_set_bounds)

        # Wait for sled io service to become available
        rospy.wait_for_service('sled_io_cmd')
        self.sled_io_cmd = rospy.ServiceProxy('sled_io_cmd',SledIOCmd)

        # # Setup subscriber to distance topic
        self.dist_sub = rospy.Subscriber('distance', DistMsg, self.dist_callback)

    def handle_get_bounds(self,req):
        """
        Returns the current bounds settings.
        """
        with self.lock:
            response = (
                    self.lower_bound,
                    self.upper_bound,
                    self.lower_bound_min,
                    self.upper_bound_max,
                    )
        return GetBoundsResponse(*response)

    def handle_set_bounds(self,req):
        """
        Change the current bounds settings.
        """
        with self.lock:
            # Check bounds
            bounds_ok = True 
            if req.lower_bound < self.lower_bound_min: 
                bounds_ok = False
            if req.upper_bound > self.upper_bound_max:
                bounds_ok = False
            if req.lower_bound >= req.upper_bound:
                bounds_ok = False

            # Set bounds - if they passed the checks
            if bounds_ok:
                self.lower_bound = req.lower_bound
                self.upper_bound = req.upper_bound
                response = True
            else:
                response = False
        return SetBoundsResponse(response)

    def dist_callback(self,data):

        distance = data.distance

        if distance < self.lower_bound or distance > self.upper_bound:
            self.sled_io_cmd('set mode', 'off')


if __name__ == '__main__':
    rospy.init_node('bounds_checker')
    checker = BoundsChecker()
    rospy.spin()
