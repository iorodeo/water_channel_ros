#!/usr/bin/env python
import roslib
roslib.load_manifest('safety')
import rospy
from std_msgs.msg import Header
from msg_and_srv.msg import WatchDogMsg

class WatchDogPulse(object):

    def __init__(self):
        self.update_rate = rospy.get_param("watchdog_update_rate",10)
        self.rate = rospy.Rate(self.update_rate)

        # Setup watchdog pulse topic 
        self.watchdog_msg = WatchDogMsg()
        self.watchdog_pub = rospy.Publisher('watchdog_pulse', WatchDogMsg)

    def sleep(self):
        self.rate.sleep()

    def update(self):
        self.watchdog_msg.header.stamp = rospy.get_rostime()
        self.watchdog_pub.publish(self.watchdog_msg)

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    rospy.init_node('watchdog_pulse')
    pulse = WatchDogPulse()
    while not rospy.is_shutdown():
        pulse.update()
        pulse.sleep()
