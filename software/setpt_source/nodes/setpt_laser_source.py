#!/usr/bin/env python
"""
setpt_ain_source.py

Set point source for inertial trajectory system.  

"""
from __future__ import division
import roslib
roslib.load_manifest('setpt_source')
import rospy
import threading
import math

# Messages
from std_msgs.msg import Header
from msg_and_srv.msg import SetptMsg
from msg_and_srv.msg import AnalogInMsg
from msg_and_srv.msg import DistMsg 

# Services
from msg_and_srv.srv import NodeEnable
from msg_and_srv.srv import NodeEnableResponse

AIN_NUM = 1
AIN_INT_TO_VOLTS = 5.0/2047.0
LASER_AIN_OFFSET = 1.0/AIN_INT_TO_VOLTS
LASER_CAL =  AIN_INT_TO_VOLTS*(0.1/4.0)
LASER_RANGE_START= 0.05 # m

class LaserSetptSource(object):

    def __init__(self):
        self.lock = threading.Lock()
        self.ain_num = AIN_NUM 
        self.laser_ain_offset = LASER_AIN_OFFSET
        self.laser_cal = LASER_CAL
        self.laser_range_start = LASER_RANGE_START
        self.setpt_offset = rospy.get_param('laser_setpt_offset', 0.1) 
        self.setpt_sign = rospy.get_param('laser_setpt_sign', -1.0)
        self.setpt_msg = SetptMsg()

        self.velocity = None
        self.ready = False
        self.enabled = False
        rospy.init_node('laser_setpt_source')

        # Setup services
        self.node_enable_srv = rospy.Service(
                'laser_setpt_enable', 
                NodeEnable, 
                self.handle_node_enable
                ) 

        # Publishers and subscribers
        self.setpt_pub = rospy.Publisher('setpt', SetptMsg)

        self.ain_sub = rospy.Subscriber(
                'analog_input',
                AnalogInMsg,
                self.handle_ain_msg
                )

        self.dist_sub = rospy.Subscriber(
                'distance',
                DistMsg,
                self.handle_dist_msg,
                )
        self.ready = True

    def handle_node_enable(self,req):
        """
        Enable or disable the node based on the request.
        """
        with self.lock:
            if req.enable:
                self.enabled = True
            else:
                self.enabled = False
        message = ''
        return NodeEnableResponse(self.enabled,message)


    def handle_ain_msg(self,data):
        """
        Handles incoming analog input messages. Converts the analog inputs to a
        set point signal to be published on the setpt topic.
        """
        if not self.ready or self.velocity is None:
            return
        ain = data.values[self.ain_num] - self.laser_ain_offset
        pos = ain*self.laser_cal + self.laser_range_start
        setpt_pos = self.setpt_sign*(pos - self.setpt_offset)

        with self.lock:
            velocity = self.velocity
            enabled = self.enabled
    
        if enabled:
            self.setpt_msg.header.stamp = rospy.get_rostime()
            self.setpt_msg.position = setpt_pos 
            self.setpt_msg.velocity = velocity 
            self.setpt_pub.publish(self.setpt_msg)

    def handle_dist_msg(self,data):
        """
        Handles incoming distance sensor messages. Get current sled velocity
        for use as a feed forward term in the controller.
        """
        if not self.ready:
            return
        with self.lock:
            self.velocity = data.velocity_kalman

    def run(self):
        rospy.spin()


# -----------------------------------------------------------------------------
if __name__ == '__main__':

    node = LaserSetptSource()
    node.run()


