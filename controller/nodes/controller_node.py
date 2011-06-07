#!/usr/bin/env python
import roslib 
roslib.load_manifest('controller')
import rospy
import threading
import time
from controller_comm import ControllerComm
# Messages
from setpt_source.msg import SetptMsg 
from distance_118x.msg import DistMsg 
from motor_cmd_source.msg import MotorCmdMsg
# Services
from controller.srv import * 

class Controller(object):

    def __init__(self):
        self.lock =  threading.Lock()

        self.dev = ControllerComm()
        self.dev.setModeOff()

        self.sleep_dt = 0.001

        self.pos = None
        self.pos_setpt = None
        self.vel_setpt = None
        self.motor_cmd = None

        self.have_new_pos = False
        self.have_new_setpt = False
        self.have_new_motor_cmd = False
        self.have_pos = False

        # Setup subscriber to distance and set point topics
        self.distance_sub = rospy.Subscriber('distance', DistMsg, self.distance_callback)
        self.setpoint_sub = rospy.Subscriber('setpt', SetptMsg, self.setpt_callback)
        self.motor_cmd_sub = rospy.Subscriber('motor_cmd', MotorCmdMsg, self.motor_cmd_callback)
        
        # Setup controller service
        self.srv = rospy.Service('controller_cmd', ControllerCmd, self.handle_controller_cmd)

        # Add shutdown code
        rospy.on_shutdown(self.error_stop)

        # Initialize nodes
        rospy.init_node('controller')

    def run(self):

        while not rospy.is_shutdown():

            with self.lock:

                # Send position data to controller
                if self.have_new_pos == True:
                    self.dev.sendPosition(self.pos)
                    self.have_new_pos = False
                    if self.have_pos == False:
                        self.dev.sendSetPoint(self.pos, 0.0)
                        self.have_pos = True

                # Send new setpt data to controller
                if self.have_new_setpt == True:
                    self.dev.sendSetPoint(self.pos_setpt, self.vel_setpt)
                    self.have_new_setpt = False

                # Send motor command to controller
                if self.have_new_motor_cmd == True:
                    self.dev.sendMotorCmd(self.motor_cmd)
                    self.have_new_motor_cmd = False

            # Get data from controller
            if 1:
                data = self.dev.readInWaiting()
                if data:
                    try:
                        for x in data[-1]:
                            print '%1.2f, '%(float(x),) , 
                            pass
                        print
                    except:
                        print 'data error'

        rospy.sleep(self.sleep_dt)

    def handle_controller_cmd(self,req):
        if req.cmd == 'set mode':
            mode = req.valueString

            if mode.lower() == 'off':
                with self.lock:
                    self.dev.setModeOff()

            elif mode.lower() == 'tracking':
                with self.lock:
                    if self.have_pos:
                        self.dev.setModeTracking()

            elif mode.lower() == 'motor_cmd':
                with self.lock:
                    self.dev.setModeMotorCmd()

            else:
                pass

        return ControllerCmdResponse()

    def distance_callback(self,data):
        with self.lock:
            self.have_new_pos = True
            self.pos = data.distance

    def setpt_callback(self,data):
        with self.lock:
            self.have_new_setpt = True
            self.pos_setpt = data.position
            self.vel_setpt = data.velocity

    def motor_cmd_callback(self,data):
        with self.lock:
            self.have_new_motor_cmd = True
            self.motor_cmd = data.motor_cmd

    def error_stop(self):
        for i in range(0,10):
            #print 'setModeOff', i
            self.dev.setModeOff();
            time.sleep(0.1)
            


# -----------------------------------------------------------------------------
if __name__ == '__main__':
    ctl = Controller()
    try:
        ctl.run()
    except rospy.ROSInterruptException:
        pass
