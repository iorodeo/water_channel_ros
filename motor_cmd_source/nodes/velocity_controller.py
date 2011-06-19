#!/usr/bin/env python
import roslib 
roslib.load_manifest('motor_cmd_source')
import rospy
import threading
import math
import pid_controller
import velocity_feedforward
from std_msgs.msg import Header
from distance_118x.msg import DistMsg 
from motor_cmd_source.msg import MotorCmdMsg 
from motor_cmd_source.msg import PIDMsg 
from setpt_source.msg import SetptMsg

class VelocityController(object):

    def __init__(self):
        self.lock = threading.Lock()
        self.haveSensorData = False
        self.position = None
        self.velocity = None
        self.setptPosition = None
        self.setptVelocity = None
        self.error = None
        self.ffValue = None
        self.motorCmd = None

        self.controller = pid_controller.PIDController()
        self.controller.pgain = rospy.get_param('controller_pgain', 0.5)
        self.controller.igain = rospy.get_param('controller_igain', 0.0)
        self.controller.dgain = rospy.get_param('controller_dgain', 0.0)
        self.controller.outputMax = rospy.get_param('controller_max_output', 4000)
        self.controller.outputMin = rospy.get_param('controller_min_output', -4000)
        
        self.feedForward = velocity_feedforward.VelocityFeedForward()
        self.feedForward.posCoeff = rospy.get_param('feedforward_pos_coeff', 5.375)
        self.feedForward.negCoeff = rospy.get_param('feedforward_neg_coeff', 5.4)
        self.feedForward.posOffset = rospy.get_param('feedforward_pos_offset', 93.0)
        self.feedForward.negOffset = rospy.get_param('feedforward_neg_offset', -69.35)
        self.controller.ffFunc = self.feedForward.func

        # Setup subscriber setpt topic 
        self.setptSub = rospy.Subscriber('setpt', SetptMsg, self.setptCallback)
        self.distanceSub = rospy.Subscriber('distance', DistMsg, self.distanceCallback)

        # Setup setpt topic
        self.motorCmdMsg = MotorCmdMsg()
        self.motorCmdPub = rospy.Publisher('motor_cmd', MotorCmdMsg)
        self.PIDMsg = PIDMsg()
        self.PIDPub = rospy.Publisher('pid_terms', PIDMsg)

    def setptCallback(self,data):
        with self.lock:
            if self.haveSensorData==True:
                self.setptPosition = data.position
                self.setptVelocity = data.velocity
                self.error = self.setptVelocity - self.velocity
                self.ffValue = self.setptVelocity
                self.motorCmd = self.controller.update(self.error,self.ffValue)
                print self.motorCmd

        stamp = rospy.get_rostime()
        # Write motor command message
        self.motorCmdMsg.header.stamp = stamp 
        self.motorCmdMsg.motor_cmd = self.motorCmd
        self.motorCmdPub.publish(self.motorCmdMsg)
        # Write pid controller message
        self.PIDMsg.header.stamp = stamp
        self.PIDMsg.ffTerm = self.controller.ffTerm
        self.PIDMsg.pTerm = self.controller.pTerm
        self.PIDMsg.iTerm = self.controller.iTerm
        self.PIDMsg.dTerm = self.controller.dTerm
        self.PIDPub.publish(self.PIDMsg)

    def distanceCallback(self,data):
        with self.lock:
            self.haveSensorData = True
            self.position = data.distance_kalman
            self.velocity = data.velocity_kalman


# -----------------------------------------------------------------------------
if __name__ == '__main__':
    rospy.init_node('velocity_controller')
    ctl = VelocityController()
    rospy.spin()
        

