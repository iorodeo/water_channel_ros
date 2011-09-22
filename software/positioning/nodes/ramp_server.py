#!/usr/bin/env python
import roslib 
roslib.load_manifest('positioning')
import rospy
import utilities
from msg_and_srv.srv import GetRamp
from msg_and_srv.srv import GetRampResponse

class RampServer(object):

    def __init__(self):
        self.srv = rospy.Service('get_ramp', GetRamp, self.handle_ramp_request)
        rospy.init_node('ramp_server')

    def run(self):
        rospy.spin()

    def handle_ramp_request(self,req):
        ramp = utilities.get_ramp(
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
