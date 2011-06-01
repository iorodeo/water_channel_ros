#!/usr/bin/env python
import roslib 
roslib.load_manifest('distance_118x')
import rospy
import threading
from std_msgs.msg import Header
from distance_118x.msg import DistMsg 
from distance_118x.srv import *
from distance_sensor_118x import DistanceSensor

class DistSensorNode(object):

    def __init__(self, scale_factor=1000.0,port='/dev/USB_Distance'):
        """
        Initilize distance sensor node.
        """
        self.lock =  threading.Lock()

        # Setup distance sensor
        self.dev = DistanceSensor(port) 
        self.dev.open()
        self.dev.setScaleFactor(scale_factor)
        self.distMsg = DistMsg()
        self.dev.laserOn()
        self.laserOn = True

        # Setup publisher and control service
        self.pub = rospy.Publisher('distance', DistMsg)
        self.srv = rospy.Service('distance_ctl', DistSensorCtl, self.handle_distance_ctl)

        # Initialize node
        rospy.init_node('distance_sensor')

    def handle_distance_ctl(self,req):
        if req.cmd == 'laser':
            if req.valueString == 'on':
                with self.lock:
                    self.dev.laserOn()
                    self.dev.startDistTracking('50hz')
                    self.laserOn = True 
                    
            else:
                with self.lock:
                    self.dev.laserOff()
                    self.laserOn = False
        return DistSensorCtlResponse() 

    def stream(self):
        """
        Streams data from the sensor. 
        """
        self.dev.startDistTracking('50hz')
        while not rospy.is_shutdown():
            with self.lock:
                if self.laserOn == True:
                    try:
                        value = self.dev.readSample(convert='float')
                    except:
                        value = None
                else:
                    value = None
            if not value is None:
                self.distMsg.header.stamp = rospy.get_rostime()
                self.distMsg.distance = value
                self.pub.publish(self.distMsg)

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    dist_node = DistSensorNode()
    try:
        dist_node.stream()
    except rospy.ROSInterruptException:
        pass



