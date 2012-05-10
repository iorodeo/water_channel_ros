#!/usr/bin/env python
import roslib
roslib.load_manifest('safety')
import rospy
import threading

# Messages
from msg_and_srv.msg import DistMsg

# Services
from msg_and_srv.srv import SledIOCmd 

class WatchDogDistance(object):
    """
    Watchdog node for the long range laser distance sensor. Compares the last
    distance sensor time stamp with the current time. If this is greater than the
    timeout threshold sled io is shutdown.
    """

    def __init__(self):
        self.lock = threading.Lock()
        self.ready = False
        self.sleep_dt = 1.0/50.0 # Max sensor update rate in 50Hz.
        self.last_stamp = None
        self.timeout= rospy.get_param('distance_timeout', 1.0)
        rospy.init_node('watchdog_distance')

        # Wait for sled io service to become available
        rospy.wait_for_service('sled_io_cmd')
        self.sled_io_cmd = rospy.ServiceProxy('sled_io_cmd',SledIOCmd)

        # # Setup subscriber to distance topic
        self.dist_sub = rospy.Subscriber('distance', DistMsg, self.dist_callback)
        self.ready = True

    def dist_callback(self,data):
        """
        Handles incoming distance sensor messages. 
        """
        with self.lock:
            if not self.ready:
                return
            self.last_stamp = data.header.stamp

    def run(self):
        """
        Compares the last distance sensor time stamp with the current time. If 
        the difference is greater than the timeout threshold sled io is shutdown.
        """
        while not rospy.is_shutdown():
            with self.lock:
                if not self.ready:
                    return
                if self.last_stamp is not None:
                    dt = rospy.get_time() - self.last_stamp.to_sec()
                    if dt > self.timeout:
                        self.sled_io_cmd('set mode', 'off')

        rospy.sleep(self.sleep_dt)

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    node = WatchDogDistance()
    node.run()
