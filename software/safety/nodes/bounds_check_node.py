#!/usr/bin/env python
import roslib
roslib.load_manifest('safety')
import rospy
import threading
import math
from msg_and_srv.msg import DistMsg
from msg_and_srv.srv import SledIOCmd 


class BoundsChecker(object):

    def __init__(self):

        self.lock = threading.Lock()

        # Get upper and lower distance bounds
        self.lower_bound = rospy.get_param('dist_lower_bound', 2000)
        self.upper_bound = rospy.get_param('dist_upper_bound', 4000)

        # Wait for sled io service to become available
        rospy.wait_for_service('sled_io_cmd')
        self.sled_io_cmd = rospy.ServiceProxy('sled_io_cmd',SledIOCmd)

        # # Setup subscriber to distance topic
        self.dist_sub = rospy.Subscriber('distance', DistMsg, self.dist_callback)


    def dist_callback(self,data):

        distance = data.distance

        if distance < self.lower_bound or distance > self.upper_bound:
            self.sled_io_cmd('set mode', 'off')


if __name__ == '__main__':
    rospy.init_node('bounds_checker')
    checker = BoundsChecker()
    rospy.spin()
