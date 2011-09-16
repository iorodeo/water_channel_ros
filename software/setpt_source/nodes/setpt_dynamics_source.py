#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('setpt_source')
import rospy
import threading
import math
import dynamics

# Messages
from std_msgs.msg import Header
from msg_and_srv.msg import SetptMsg
from msg_and_srv.msg import ForceMsg

# Services
from msg_and_srv.srv import NodeEnable
from msg_and_srv.srv import NodeEnableResponse

class DynamicsSetptSource(object):

    def __init__(self):
        mass = rospy.get_param("mass", 1.0)
        damping = rospy.get_param("damping", 0.0)

        self.lock = threading.Lock()
        self.enabled = False
        
        self.dynamics = dynamics.Dynamics()
        self.dynamics.mass = mass
        self.dynamics.damping = damping
        self.dynamics.reset()

        # Setup enable service
        self.node_enable_srv = rospy.Service('dynamics_enable', NodeEnable, self.handle_node_enable) 

        # Setup subscriber to force topic
        self.force_sub = rospy.Subscriber('force', ForceMsg, self.force_callback)

        # Setup setpt topic
        self.setptMsg = SetptMsg()
        self.setptPub = rospy.Publisher('setpt_rel', SetptMsg)


    def handle_node_enable(self,req):
        if req.enable:
            # Enable dynamics node
            with self.lock:
                self.dynamics.reset()
                self.enabled = True
        else:
            # Disable dynamics node
            with self.lock:
                self.enabled = False
        message = ''
        return NodeEnableResponse(self.enabled,message)

    def force_callback(self,data):
        with self.lock:
            if self.enabled:
                self.dynamics.update(data.force, data.dt)
                self.setptMsg.header.stamp = rospy.get_rostime()
                self.setptMsg.position = self.dynamics.position
                self.setptMsg.velocity = self.dynamics.velocity
                self.setptPub.publish(self.setptMsg)

if __name__ == '__main__':
    rospy.init_node('dynamics')
    dynamics_node = DynamicsSetptSource()
    rospy.spin()
# 
