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
from msg_and_srv.srv import SetDynamParams
from msg_and_srv.srv import SetDynamParamsResponse
from msg_and_srv.srv import GetDynamParams
from msg_and_srv.srv import GetDynamParamsResponse

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

        # Setup services
        self.node_enable_srv = rospy.Service(
                'dynamics_enable', 
                NodeEnable, 
                self.handle_node_enable
                ) 

        self.set_dynam_params_srv = rospy.Service(
                'set_dynam_params', 
                SetDynamParams, 
                self.handle_set_dynam_params
                )

        self.get_dynam_params_srv = rospy.Service(
                'get_dynam_params',
                GetDynamParams,
                self.handle_get_dynam_params
                )

        # Setup subscribers
        self.force_sub = rospy.Subscriber('force', ForceMsg, self.force_callback)

        # Setup setpt topic
        self.setptMsg = SetptMsg()
        self.setptPub = rospy.Publisher('setpt_rel', SetptMsg)

    def handle_get_dynam_params(self,req):
        """
        Returns the current dynamics parameters
        """
        mass = self.dynamics.mass
        damping = self.dynamics.damping
        return GetDynamParamsResponse(mass,damping)

    def handle_set_dynam_params(self,req):
        """
        Set dynamics parameters based on request.
        """
        message = ''
        status = True
        with self.lock:

            # Update mass
            if req.mass > 0.0:
                self.dynamics.mass = req.mass
            else:
                status = False
                message = 'mass must be greater than zero'

            # Update damping
            if req.damping >= 0.0:
                self.dynamics.damping = req.damping
            else:
                status = False
                message = 'damping must be greater than or equal to zero'

        return SetDynamParamsResponse(status,message)

    def handle_node_enable(self,req):
        """
        Enable or disable the dynamics node based on the request.
        """
        with self.lock:
            if req.enable:
                # Enable dynamics node
                self.dynamics.reset()
                self.enabled = True
            else:
                # Disable dynamics node
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

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    rospy.init_node('dynamics')
    dynamics_node = DynamicsSetptSource()
    rospy.spin()
# 
