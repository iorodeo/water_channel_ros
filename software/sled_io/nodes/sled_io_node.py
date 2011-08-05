#!/usr/bin/env python
import roslib 
roslib.load_manifest('sled_io')
import rospy
import threading
import time
from sled_comm import SledIOComm
from msg_and_srv.msg import AnalogInMsg
from msg_and_srv.msg import MotorCmdMsg
from msg_and_srv.msg import WatchDogMsg
from msg_and_srv.msg import ActuatorMsg
from msg_and_srv.srv import SledIOCmd
from msg_and_srv.srv import SledIOCmdResponse

class SledIO(object):

    def __init__(self):
        self.lock =  threading.Lock()

        self.dev = SledIOComm(port='/dev/USB_Controller',baudrate=115200,timeout=0.1)
        self.dev.setModeOff()
        self.sleep_dt = 0.01
        self.motor_cmd = None

        # Setup analog input publisher 
        self.ain_msg = AnalogInMsg()
        self.ain_pub = rospy.Publisher('analog_input', AnalogInMsg)

        # Setup subscriber to motor_cmd topic and watchdog pulse 
        self.motor_cmd_sub = rospy.Subscriber('motor_cmd', MotorCmdMsg, self.motor_cmd_callback)
        self.watchdog_sub = rospy.Subscriber('watchdog_pulse', WatchDogMsg, self.watchdog_callback)
        self.actuator_sub = rospy.Subscriber('actuator', ActuatorMsg, self.actuator_callback)
        
        # Setup controller service
        self.srv = rospy.Service('sled_io_cmd', SledIOCmd, self.handle_sled_io_cmd)

        # Add shutdown code
        rospy.on_shutdown(self.error_stop)

        # Initialize nodes
        rospy.init_node('sled_control_io')

        self.dev.setDataStreamOn()

    def run(self):
        while not rospy.is_shutdown():
            with self.lock:
                # Get data stream from controller
                data = self.dev.readInWaiting()
                if data:
                    self.ain_msg.header.stamp = rospy.get_rostime()
                    self.ain_msg.values = data[-1]
                    self.ain_pub.publish(self.ain_msg)
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

    def actuator_callback(self,data):
        actuator_type = data.type
        actuator_value = data.value
        if actuator_type in ('pwm0', 'pwm1'):
            pwm_num = int(actuator_type[-1])
            with self.lock:
                self.dev.sendActuatorPWM(pwm_num, actuator_value)

    def error_stop(self):
        for i in range(0,10):
            self.dev.setModeOff();
            self.dev.setDataStreamOff()
            time.sleep(0.1)
            

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    ctl = SledIO()
    try:
        ctl.run()
    except rospy.ROSInterruptException:
        pass
