#!/usr/bin/env python
import roslib 
roslib.load_manifest('setpt_source')
import rospy
import threading
import math
from std_msgs.msg import Header
from msg_and_srv.msg import SetptMsg 
from msg_and_srv.msg import DistMsg 

class SetptSource(object):

    def __init__(self):
        self.lock =  threading.Lock()

        # Setpt source parameters
        self.pos_setpt = 0.0
        self.vel_setpt = 0.0
        self.dt = 0.02
        self.sin_amplitude = 400.0
        self.sin_period = 15.0
        self.have_pos = False
        self.pos = None
        self.pos_initial = None

        # Setup subscriber to distance topic
        self.dist_sub = rospy.Subscriber('distance', DistMsg, self.dist_callback)

        # Setup setpt topic
        self.setptMsg = SetptMsg()
        self.setpt_pub = rospy.Publisher('setpt', SetptMsg)

        # Initialize node
        rospy.init_node('setpt_source')

        stamp = rospy.get_rostime()
        self.start_t  = self.get_time()

    def run(self):
        while not rospy.is_shutdown():
            self.setptMsg.header.stamp = rospy.get_rostime()
            sin_pos, sin_vel = self.cos()
            with self.lock:
                if self.have_pos == True:
                    self.pos_setpt = self.pos_initial + sin_pos
                    self.vel_setpt = sin_vel
                    self.setptMsg.position = self.pos_setpt
                    self.setptMsg.velocity = self.vel_setpt
                    self.setptMsg.error = self.pos_setpt - self.pos
                    self.setpt_pub.publish(self.setptMsg)
            rospy.sleep(self.dt)

    def dist_callback(self,data):
        with self.lock:
            if self.have_pos == False:
                self.pos_initial = data.distance
                self.have_pos = True
            self.pos = data.distance

    def get_time(self):
        stamp = rospy.get_rostime()
        return stamp.secs + stamp.nsecs*1.0e-9

    def sin(self):
        curr_t = self.get_time()
        t = curr_t - self.start_t
        const = 2.0*math.pi/self.sin_period
        pos = self.sin_amplitude*math.sin(const*t)
        vel = const*self.sin_amplitude*math.cos(const*t)
        return pos, vel

    def cos(self):
        curr_t = self.get_time()
        t = curr_t - self.start_t
        const = 2.0*math.pi/self.sin_period
        pos = self.sin_amplitude*(math.cos(const*t) - 1.0)
        vel = -const*self.sin_amplitude*math.sin(const*t)
        return pos, vel



# -----------------------------------------------------------------------------
if __name__ == '__main__':
    setpt = SetptSource()
    try:
        setpt.run()
    except rospy.ROSInterruptException:
        pass
