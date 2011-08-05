#!/usr/bin/env python
import roslib 
roslib.load_manifest('motor_cmd_source')
import rospy
import threading
import math
import pid_controller
import velocity_feedforward
from std_msgs.msg import Header
from msg_and_srv.msg import DistMsg 
from msg_and_srv.msg import MotorCmdMsg
from msg_and_srv.msg import PIDMsg
from setpt_source.msg import SetptMsg
from gain_scheduler import GainScheduler

class BaseController(object):

    def __init__(self):
        self.lock = threading.Lock()
        self.haveSensorData = False
        self.haveSetptData = False
        self.position = None
        self.velocity = None
        self.setptPosition = None
        self.setptVelocity = None
        self.error = None
        self.ffValue = None
        self.motorCmd = None
        self.update_rate = 0.001
        self.rate = rospy.Rate(self.update_rate)

        self.controller = pid_controller.PIDController()
        self.controller.pgain = rospy.get_param('controller_pgain', 0.5)
        self.controller.igain = rospy.get_param('controller_igain', 0.0)
        self.controller.dgain = rospy.get_param('controller_dgain', 0.0)
        self.controller.outputMax = rospy.get_param('controller_max_output', 4000)
        self.controller.outputMin = rospy.get_param('controller_min_output', -4000)
        
        self.feedForward = velocity_feedforward.VelocityFeedForward()
        self.feedForward.coeff = rospy.get_param('feedforward_coeff', 5.6)
        self.controller.ffFunc = self.feedForward.func

        self.pgainScheduler = GainScheduler(width_x=20.0, width_y=20.0, min_gain=0.5,max_gain=5.0) 

        # Setup subscriber setpt topic 
        self.setptSub = rospy.Subscriber('setpt', SetptMsg, self.setptCallback)
        self.distanceSub = rospy.Subscriber('distance', DistMsg, self.distanceCallback)

        # Setup publisher for motor command topic
        self.motorCmdMsg = MotorCmdMsg()
        self.motorCmdPub = rospy.Publisher('motor_cmd', MotorCmdMsg)
        self.PIDMsg = PIDMsg()
        self.PIDPub = rospy.Publisher('pid_terms', PIDMsg)

    def getMotorCmd(self): 
        """ 
        Get motor command dummy function - child classes should  implement
        there own verisons of this.  
        """ 
        pass

    def setptCallback(self,data):
        with self.lock:
            self.haveSetptData = True
            self.setptPosition = data.position
            self.setptVelocity = data.velocity

    def distanceCallback(self,data):
        with self.lock:
            self.haveSensorData = True
            self.position = data.distance_kalman
            self.velocity = data.velocity_kalman
            if self.haveSetptData == True:
                self.getMotorCmd()
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


# -----------------------------------------------------------------------------
if __name__ == '__main__':
    rospy.init_node('base_controller')
    ctl = BaseController()
    rospy.spin()
        

