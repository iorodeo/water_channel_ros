#!/usr/bin/env python
import roslib 
roslib.load_manifest('positioning')
import rospy
import positioning_moves 
from positioning.srv import * 

class RampServer(object):

    def __init__(self):
        self.srv = rospy.Service('get_ramp', GetRamp, self.handle_ramp_request)
        rospy.init_node('ramp_server')

    def run(self):
        rospy.spin()

    def handle_ramp_request(self,req):
        ramp = positioning_moves.get_ramp(
                req.pos_0,
                req.pos_1,
                req.max_velo,
                req.accel,
                req.dt
                )
        return GetRampResponse(list(ramp))

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    ramp_serv = RampServer()
    ramp_serv.run()