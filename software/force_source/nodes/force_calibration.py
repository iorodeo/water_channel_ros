#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('force_source')
import rospy
import threading
from std_msgs.msg import Header
from msg_and_srv.msg import ForceMsg
from msg_and_srv.msg import AnalogInMsg

class Force_Calibration(object):

    def __init__(self):

        self.initialized = False
        self.zeroed = False
        self.lock = threading.Lock()

        # Force parameters
        self.force = 0.0
        self.force_zero = 0.0
        self.force_zero_cnt = 0

        self.force_calibration = rospy.get_param("force_calibration", 1.0)
        self.force_zeroing_num = rospy.get_param("force_zeroing_num", 25)
        self.force_ain_channel = rospy.get_param("force_ain_channel" , 0)

        # Set up subscriber to analog input topic
        self.ain_sub = rospy.Subscriber('analog_input', AnalogInMsg, self.ain_callback)

        # Set up publisher on force topic
        self.force_msg = ForceMsg()
        self.force_pub = rospy.Publisher('force', ForceMsg)

        self.t_last = rospy.get_rostime().to_sec()
        self.initialized = True

    def ain_callback(self,data):

        with self.lock:
            ros_time = rospy.get_rostime()
            ain_value = data.values[self.force_ain_channel]

            if self.zeroed == False:
                # Sensor in not zeroed get zero value
                self.force_zero += ain_value*self.force_calibration
                self.force_zero_cnt += 1
                if self.force_zero_cnt == self.force_zeroing_num:
                    self.force_zero = self.force_zero/float(self.force_zeroing_num)
                    self.zeroed = True

                # While zeroing publish zero for force values
                self.force = 0.0

            else:
                # Sensor is zeroed publish actual sensor values 
                self.force = ain_value*self.force_calibration - self.force_zero

            # Get timing information
            t = ros_time.to_sec()
            dt = t - self.t_last
            self.t_last = t

            # Create force message and publish
            self.force_msg.header.stamp = ros_time
            self.force_msg.dt = dt 
            self.force_msg.force = self.force
            self.t_last = t
            self.force_pub.publish(self.force_msg)

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    rospy.init_node('force')
    fcal = Force_Calibration()
    while not rospy.is_shutdown():
        rospy.spin()
