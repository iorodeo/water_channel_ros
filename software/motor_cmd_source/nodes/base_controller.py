#!/usr/bin/env python
import roslib 
roslib.load_manifest('motor_cmd_source')
import rospy
import threading
import math
import pid_controller
import velocity_feedforward
from gain_scheduler import GainScheduler

# Messages
from std_msgs.msg import Header
from msg_and_srv.msg import DistMsg 
from msg_and_srv.msg import MotorCmdMsg
from msg_and_srv.msg import PIDMsg
from setpt_source.msg import SetptMsg

# Services
from msg_and_srv.srv import NodeEnable
from msg_and_srv.srv import NodeEnableResponse

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
        self.enabled = False

        self.controller = pid_controller.PIDController()
        self.controller.pgain = rospy.get_param('controller_pgain', 0.5)
        self.controller.igain = rospy.get_param('controller_igain', 0.0)
        self.controller.dgain = rospy.get_param('controller_dgain', 0.0)
        self.controller.outputMax = rospy.get_param('controller_max_output', 4000)
        self.controller.outputMin = rospy.get_param('controller_min_output', -4000)
        
        self.feedForward = velocity_feedforward.VelocityFeedForward()
        self.feedForward.coeff = rospy.get_param('controller_ffcoeff', 5.6)
        self.controller.ffFunc = self.feedForward.func

        pgain = self.controller.pgain
        self.pgainScheduler = GainScheduler(0.1*pgain,pgain,0.075) 

        # Setup subscriber setpt topic 
        self.setptSub = rospy.Subscriber('setpt', SetptMsg, self.setptCallback)
        self.distanceSub = rospy.Subscriber('distance', DistMsg, self.distanceCallback)

        # Setup publisher for motor command topic
        self.motorCmdMsg = MotorCmdMsg()
        self.motorCmdPub = rospy.Publisher('motor_cmd', MotorCmdMsg)
        self.PIDMsg = PIDMsg()
        self.PIDPub = rospy.Publisher('pid_terms', PIDMsg)

        # Setup enable/disable service 
        self.nodeEnableSrv = rospy.Service(
                'controller_enable', 
                NodeEnable, 
                self.handleNodeEnable
                ) 

    def handleNodeEnable(self,req):
        with self.lock:
            if req.enable:
                self.enabled = True
            else:
                self.enabled = False
                self.motorCmd = 0
                self.publishMotorCmd()

        message = ''
        return NodeEnableResponse(self.enabled,message)


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
            if self.enabled and self.haveSetptData == True:
                self.getMotorCmd()
                stamp = rospy.get_rostime()
                self.publishMotorCmd(stamp)
                self.publishPIDInfo(stamp)

    def publishMotorCmd(self,stamp=None):
        if stamp is None:
            stamp = rospy.get_rostime()
        self.motorCmdMsg.header.stamp = stamp 
        self.motorCmdMsg.motor_cmd = self.motorCmd
        self.motorCmdPub.publish(self.motorCmdMsg)
        

    def publishPIDInfo(self,stamp):
        if stamp is None:
            stamp = rospy.get_rostime()
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
        

