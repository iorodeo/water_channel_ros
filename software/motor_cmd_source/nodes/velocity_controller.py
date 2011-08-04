#!/usr/bin/env python
import roslib 
roslib.load_manifest('motor_cmd_source')
import rospy
from base_controller import BaseController

class VelocityController(BaseController):

    def getMotorCmd(self):
        self.error = self.setptVelocity - self.velocity
        self.ffValue = self.setptVelocity
        self.motorCmd = self.controller.update(self.error,self.ffValue)

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    rospy.init_node('velocity_controller')
    ctl = VelocityController()
    rospy.spin()
        

