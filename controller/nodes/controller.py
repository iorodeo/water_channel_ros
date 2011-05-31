#!/usr/bin/env python
import roslib 
roslib.load_manifest('setpt_source')
import rospy
import threading
import time
from setpt_source.msg import SetptMsg 
from distance_118x.msg import DistMsg 
from ControllerComm import ControllerComm

class Controller(object):

    def __init__(self):
        self.lock =  threading.Lock()

        self.dev = ControllerComm()
        self.dev.setModeOff()
        #self.dev.setModeTracking()

        self.pos = None
        self.pos_setpt = None
        self.vel_setpt = None
        self.sleep_dt = 0.001
        self.have_new_pos = False
        self.have_new_setpt = False
        self.start_tracking = False
        self.have_pos = False

        # Setup subscriber to distance topic
        self.distance_sub = rospy.Subscriber('distance', DistMsg, self.distance_callback)
        self.setpoint_sub = rospy.Subscriber('setpt', SetptMsg, self.setpt_callback)

        # Add shutdown code
        rospy.on_shutdown(self.errorStop)

        # Initialize nodes
        rospy.init_node('controller')

    def run(self):

        while not rospy.is_shutdown():

            with self.lock:
                # Send position data to controller
                if self.have_new_pos == True:
                    #print 'P', self.pos
                    self.dev.sendPosition(self.pos)
                    self.have_new_pos = False
                    if self.have_pos == False:
                        self.dev.sendSetPoint(self.pos, 0.0)
                        self.dev.setModeTracking()
                        self.have_pos = True

                # Send new setpt data to controller
                if self.have_new_setpt == True:
                    #print 'S', self.pos_setpt
                    self.dev.sendSetPoint(self.pos_setpt, self.vel_setpt)
                    self.have_new_setpt = False

            data = self.dev.readInWaiting()
            if data:
                try:
                    for x in data[-1]:
                        print '%1.2f, '%(float(x),) , 
                    print
                except:
                    print 'data error'

        rospy.sleep(self.sleep_dt)

    def distance_callback(self,data):
        with self.lock:
            self.have_new_pos = True
            self.pos = data.distance

    def setpt_callback(self,data):
        with self.lock:
            self.have_new_setpt = True
            self.pos_setpt = data.position
            self.vel_setpt = data.velocity

    def errorStop(self):
        for i in range(0,10):
            print 'setModeOff', i
            self.dev.setModeOff();
            time.sleep(0.1)
            


# -----------------------------------------------------------------------------
if __name__ == '__main__':
    ctl = Controller()
    try:
        ctl.run()
    except rospy.ROSInterruptException:
        pass
