#!/usr/bin/env python
import roslib 
roslib.load_manifest('sled_io')
import rospy
import threading
import time
from sled_comm import SledIOComm
from motor_cmd_source.msg import MotorCmdMsg
from safety.msg import WatchDogMsg
from sled_io.srv import * 

class SledIO(object):

    def __init__(self):
        self.lock =  threading.Lock()

        self.dev = SledIOComm()
        self.dev.setModeOff()
        self.sleep_dt = 0.001
        self.motor_cmd = None

        # Setup subscriber to motor_cmd topic and watchdog pulse 
        self.motor_cmd_sub = rospy.Subscriber('motor_cmd', MotorCmdMsg, self.motor_cmd_callback)
        self.watchdog_sub = rospy.Subscriber('watchdog_pulse', WatchDogMsg, self.watchdog_callback)
        
        # Setup controller service
        self.srv = rospy.Service('sled_io_cmd', SledIOCmd, self.handle_sled_io_cmd)

        # Add shutdown code
        rospy.on_shutdown(self.error_stop)

        # Initialize nodes
        rospy.init_node('sled_control_io')

    def run(self):

        while not rospy.is_shutdown():
            with self.lock:

                #------------------------------------------------------------------
                # For testing
                # -----------------------------------------------------------------
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
                if 0:
                    data = self.dev.readInWaiting(conv2float=False)
                    if data:
                        for val in data:
                            try:
                                valsplit = val.split()
                                cmdval = int(valsplit[0])
                                if not cmdval in (55,58):
                                    print 'CE',  val
                            except:
                                print 'DE', val
            # -----------------------------------------------------------------
            rospy.sleep(self.sleep_dt)

    def handle_sled_io_cmd(self,req):
        if req.cmd == 'set mode':
            mode = req.valueString
            if mode.lower() == 'off':
                with self.lock:
                    self.dev.setModeOff()
            elif mode.lower() == 'motor_cmd':
                with self.lock:
                    self.dev.setModeMotorCmd()
            else:
                pass

        return SledIOCmdResponse()

    def motor_cmd_callback(self,data):
        with self.lock:
            self.motor_cmd = data.motor_cmd
            self.dev.sendMotorCmd(self.motor_cmd)

    def watchdog_callback(self,data):
        with self.lock:
            self.dev.sendWatchDogPulse()

    def error_stop(self):
        for i in range(0,10):
            self.dev.setModeOff();
            time.sleep(0.1)
            

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    ctl = SledIO()
    try:
        ctl.run()
    except rospy.ROSInterruptException:
        pass
