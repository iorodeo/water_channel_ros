#!/usr/bin/env python
import roslib 
roslib.load_manifest('motor_cmd_source')
import rospy
from base_controller import BaseController

class PositionController(BaseController):

    def getMotorCmd(self):
        self.error = self.setptPosition - self.position
        self.ffValue = self.setptVelocity
        self.motorCmd = self.controller.update(self.error,self.ffValue)

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    rospy.init_node('position_controller')
    ctl = PositionController()
    rospy.spin()
        

