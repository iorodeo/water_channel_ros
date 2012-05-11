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
from msg_and_srv.srv import GetControllerGains
from msg_and_srv.srv import GetControllerGainsResponse
from msg_and_srv.srv import SetControllerGains
from msg_and_srv.srv import SetControllerGainsResponse


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

        self.pgain = rospy.get_param('controller_pgain', 0.5)
        self.pgainSchedMin = rospy.get_param('controller_pgain_sched_min', 0.1)
        self.pgainSchedWid = rospy.get_param('controller_pgain_sched_wid', 0.075)
        self.igain = rospy.get_param('controller_igain', 0.0)
        self.dgain = rospy.get_param('controller_dgain', 0.0)
        self.outputMax = rospy.get_param('controller_max_output', 4000)
        self.outputMin = rospy.get_param('controller_min_output', -4000)
        self.ffcoeff = rospy.get_param('controller_ffcoeff', 5.6)
        self.createController()

        # Setup subscriber setpt topic 
        self.setptSub = rospy.Subscriber('setpt', SetptMsg, self.setptCallback)
        self.distanceSub = rospy.Subscriber('distance', DistMsg, self.distanceCallback)

        # Setup publisher for motor command topic
        self.motorCmdMsg = MotorCmdMsg()
        self.motorCmdPub = rospy.Publisher('motor_cmd', MotorCmdMsg)
        self.PIDMsg = PIDMsg()
        self.PIDPub = rospy.Publisher('pid_terms', PIDMsg)

        # Set up gain setting service
        self.setGainsSrv = rospy.Service(
                'controller_set_gains',
                SetControllerGains,
                self.handleSetGains,
                )

        self.getGainsSrv = rospy.Service(
                'controller_get_gains',
                GetControllerGains,
                self.handleGetGains,
                )

        # Setup enable/disable service 
        self.nodeEnableSrv = rospy.Service(
                'controller_enable', 
                NodeEnable, 
                self.handleNodeEnable,
                ) 

    def createController(self):
        """
        Creates a new gain scheduling controller based on the current
        gain settings.
        """
        self.feedForward = velocity_feedforward.VelocityFeedForward()
        self.feedForward.coeff = self.ffcoeff 

        self.controller = pid_controller.PIDController(
                self.pgain,
                self.igain,
                self.dgain,
                self.outputMin,
                self.outputMax,
                self.feedForward.func
                )
        
        self.pgainScheduler = GainScheduler(
                self.pgainSchedMin*self.pgain,
                self.pgain,
                self.pgainSchedWid,
                ) 

    def handleSetGains(self,req):
        """
        Handles reqeusts to set the current controller gains.
        """
        flag = True
        message = ''

        pgain = req.pgain
        igain = req.igain
        dgain = req.dgain

        if pgain < 0:
            flag = False
            message = 'pgain must be >= 0,'

        if igain < 0:
            flag = False
            message = '{0} igain must be >= 0,'

        if dgain < 0:
            flag = False
            message = '{0} dgain must be >= 0'

        with self.lock:
            if flag:
                self.pgain = pgain
                self.igain = igain
                self.dgain = dgain
                self.createController()

        return SetControllerGainsResponse(flag, message)

    def handleGetGains(self,req):
        """
        Handles request for the current PID controller gains.
        """
        with self.lock:
            pgain = self.pgain
            igain = self.igain
            dgain = self.dgain
        return GetControllerGainsResponse(pgain,igain,dgain)

    def handleNodeEnable(self,req):
        """
        Handles request to enable and disable the controller
        """
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
        

