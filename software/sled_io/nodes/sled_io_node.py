#!/usr/bin/env python
import roslib 
roslib.load_manifest('sled_io')
import rospy
import threading
import time
from sled_comm import SledIOComm

# Messages
from msg_and_srv.msg import AnalogInMsg
from msg_and_srv.msg import MotorCmdMsg
from msg_and_srv.msg import WatchDogMsg
from msg_and_srv.msg import ActuatorMsg

# Services
from msg_and_srv.srv import SledIOCmd
from msg_and_srv.srv import SledIOCmdResponse
from msg_and_srv.srv import GetSledIOMode
from msg_and_srv.srv import GetSledIOModeResponse

class SledIO(object):

    def __init__(self):
        self.lock =  threading.Lock()
        # Open serial device. Note, this is a bit of a kludge - sometimes the
        # device is opened, but no data appears on the serial stream. I'm not sure
        # why this is happen. In these case the device is closed and re-opened until it
        # can read data. 
        dev_flag = False
        while dev_flag == False:
            self.dev = SledIOComm(port='/dev/USB_Controller',baudrate=115200,timeout=0.3)
            self.dev.setModeOff()
            self.dev.setDataStreamOn()
            line = self.dev.readline()
            if not line:
                self.dev.close()
                del(self.dev)
            else:
                dev_flag = True
            
        self.sleep_dt = 0.01
        self.motor_cmd = None
        self.mode = 'off'
        self.pwm_default_value = rospy.get_param('actuator_default_value', 1500)

        # Setup analog input publisher 
        self.ain_msg = AnalogInMsg()
        self.ain_pub = rospy.Publisher('analog_input', AnalogInMsg)

        # Setup subscriber to motor_cmd topic and watchdog pulse 
        self.motor_cmd_sub = rospy.Subscriber('motor_cmd', MotorCmdMsg, self.motor_cmd_callback)
        self.watchdog_sub = rospy.Subscriber('watchdog_pulse', WatchDogMsg, self.watchdog_callback)
        self.actuator_sub = rospy.Subscriber('actuator', ActuatorMsg, self.actuator_callback)
        
        # Setup motor command and get mode services
        self.motor_cmd_srv = rospy.Service('sled_io_cmd', SledIOCmd, self.handle_sled_io_cmd)
        self.get_io_mode_srv = rospy.Service('get_sled_io_mode', GetSledIOMode, self.handle_get_io_mode)

        # Add shutdown code
        rospy.on_shutdown(self.error_stop)

        # Initialize nodes
        rospy.init_node('sled_control_io')


    def run(self):
        """
        Main run loop receives analog data from the sled io device and pulishes
        on the analog io topic.
        """
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
        """
        Handles sled io command requests
        """
        if req.cmd == 'set mode':
            mode = req.valueString
            if mode.lower() == 'off':
                with self.lock:
                    self.dev.setModeOff()
                    self.mode = mode.lower()
            elif mode.lower() == 'motor_cmd':
                with self.lock:
                    self.dev.setModeMotorCmd()
                    self.mode = mode.lower()
            else:
                pass
        elif req.cmd == 'set default pwm':
            with self.lock: 
                for n in (0,1):
                    self.dev.sendActuatorPWM(n, self.pwm_default_value)

        return SledIOCmdResponse()

    def handle_get_io_mode(self,req):
        """
        Handles get sled io mode requests
        """
        with self.lock:
            mode = self.mode
        return GetSledIOModeResponse(mode)

    def motor_cmd_callback(self,data):
        """
        Callback for motor command messages
        """
        with self.lock:
            self.motor_cmd = data.motor_cmd
            self.dev.sendMotorCmd(self.motor_cmd)

    def watchdog_callback(self,data):
        """
        Callback for watchdog messages
        """
        with self.lock:
            self.dev.sendWatchDogPulse()

    def actuator_callback(self,data):
        """
        Callback for actuator messages
        """
        actuator_type = data.type
        actuator_value = data.value
        if actuator_type in ('pwm0', 'pwm1'):
            pwm_num = int(actuator_type[-1])
            with self.lock:
                self.dev.sendActuatorPWM(pwm_num, actuator_value)

    def error_stop(self):
        """
        Clean up function for errors
        """
        for i in range(0,10):
            with self.lock:
                self.dev.setModeOff();
                self.dev.setDataStreamOff()
                self.dev.flushInput()
                self.dev.flushOutput()
            time.sleep(0.1)
            

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    ctl = SledIO()
    try:
        ctl.run()
    except rospy.ROSInterruptException:
        pass
