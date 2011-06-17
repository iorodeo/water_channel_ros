#!/usr/bin/env python
import roslib 
roslib.load_manifest('distance_118x')
import rospy
import threading
import random
from std_msgs.msg import Header
from distance_118x.msg import DistMsg 
from distance_118x.srv import *
from distance_sensor_118x import DistanceSensor
import filters

class DistSensorNode(object):

    def __init__(self, scale_factor=1000.0,port='/dev/USB_Distance'):
        """
        Initilize distance sensor node.
        """
        self.lock =  threading.Lock()
        self.distMsg = DistMsg()

        self.kalman = filters.KalmanFilter()

        # Setup distance sensor
        self.fakeit = rospy.get_param('fake_distance_sensor',False)
        if self.fakeit == False:
            self.dev = DistanceSensor(port) 
            self.dev.open()
            self.dev.setScaleFactor(scale_factor)
            self.dev.laserOn()
        self.laserOn = True

        # Setup publisher and control service
        self.pub = rospy.Publisher('distance', DistMsg)
        self.srv = rospy.Service('distance_ctl', DistSensorCtl, self.handle_distance_ctl)

        # Shutdown code
        rospy.on_shutdown(self.on_shutdown)

        # Initialize node
        rospy.init_node('distance_sensor')
        self.kalman_startup_delay = rospy.get_param('kalman_startup_delay', 1.0)
        self.start_t = rospy.get_rostime()

    def on_shutdown(self):
        if self.fakeit == False:
            self.dev.laserOff()

    def handle_distance_ctl(self,req):
        if req.cmd == 'laser': 
            if req.valueString == 'on': 
                with self.lock: 
                    if self.fakeit == False:
                        self.dev.laserOn()
                        self.dev.startDistTracking('50hz')
                    self.laserOn = True 
                        
            else:
                with self.lock:
                    if self.fakeit == False:
                        self.dev.laserOff()
                    self.laserOn = False
        return DistSensorCtlResponse() 

    def stream(self):
        """
        Streams data from the sensor. 
        """
        if self.fakeit == False:
            self.dev.startDistTracking('50hz')
        while not rospy.is_shutdown():
            with self.lock:
                if self.laserOn == True:
                    try:
                        if self.fakeit == False:
                            value = self.dev.readSample(convert='float')
                        else:
                            value = random.normalvariate(1000,2.0) 
                    except:
                        value = None
                else:
                    value = None
            if not value is None:

                t = rospy.get_rostime()
                self.distMsg.header.stamp = t
                self.distMsg.distance = value

                # Get Kalman filtered distance and velocity
                distKalman, veloKalman  = self.kalman.update(value,t.to_sec())
                if distKalman:
                    self.distMsg.distance_kalman = distKalman
                else:
                    self.distMsg.distance_kalman = value 
                if veloKalman:
                    self.distMsg.velocity_kalman = veloKalman
                else:
                    self.distMsg.velocity_kalman = 0.0

                # Publish messages after a suitable delay to allow kalman filter to converge
                if t.to_sec() - self.start_t.to_sec() > self.kalman_startup_delay:
                    self.pub.publish(self.distMsg)


# -----------------------------------------------------------------------------
if __name__ == '__main__':
    dist_node = DistSensorNode()
    try:
        dist_node.stream()
    except rospy.ROSInterruptException:
        pass



