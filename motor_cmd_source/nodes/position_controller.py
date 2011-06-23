#!/usr/bin/env python
import roslib 
roslib.load_manifest('motor_cmd_source')
import rospy
from base_controller import BaseController

class PositionController(BaseController):

    def getMotorCmd(self):
        self.error = self.setptPosition - self.position
        self.ffValue = self.setptVelocity

        ## Use continuous gain schedule - to make motion smooth
        ##----------------------------------------------------------------------
        #if abs(self.error) < 5:
        #    self.controller.pgain = 1.0
        #elif abs(self.error) < 10:
        #    self.controller.pgain = 2.0
        #elif abs(self.error) < 15:
        #    self.controller.pgain = 5.0
        #elif abs(self.error) < 20:
        #    self.controller.pgain = 8.0
        #else:
        #    self.controller.pgain = 12.0
        ## ---------------------------------------------------------------------
        
        self.motorCmd = self.controller.update(self.error,self.ffValue)

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    rospy.init_node('position_controller')
    ctl = PositionController()
    rospy.spin()
        

