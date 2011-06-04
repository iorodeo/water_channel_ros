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

class DistSensorNode(object):

    def __init__(self, scale_factor=1000.0,port='/dev/USB_Distance',fakeit=False):
        """
        Initilize distance sensor node.
        """
        self.lock =  threading.Lock()
        self.fakeit = fakeit
        self.distMsg = DistMsg()

        # Setup distance sensor
        if self.fakeit == False:
            self.dev = DistanceSensor(port) 
            self.dev.open()
            self.dev.setScaleFactor(scale_factor)
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
                self.distMsg.header.stamp = rospy.get_rostime()
                self.distMsg.distance = value
                self.pub.publish(self.distMsg)

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    import sys
    if len(sys.argv) > 1:
        mode = sys.argv[1]
        if mode.lower() == 'true':
            print 'faking distance sensor!'
            fakeit = True
        elif mode.lower() == 'false':
            fakeit = False
        else:
            raise ValueError, 'unknown mode = %s'%(mode,)
    else:
        fakeit = False

    dist_node = DistSensorNode(fakeit=fakeit)
    try:
        dist_node.stream()
    except rospy.ROSInterruptException:
        pass



