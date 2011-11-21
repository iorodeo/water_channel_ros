#!/usr/bin/env python
import roslib 
roslib.load_manifest('positioning')
import rospy
import utilities
from msg_and_srv.srv import GetCosine
from msg_and_srv.srv import GetCosineResponse

class CosineServer(object):

    def __init__(self):
        self.srv = rospy.Service('get_cosine', GetCosine, self.handle_cosine_request)
        rospy.init_node('cosine_server')

    def run(self):
        rospy.spin()

    def handle_cosine_request(self,req):
        cosine = utilities.get_cosine(
                req.amplitude,
                req.period,
                req.cycles,
                req.dt
                )
        return GetCosineResponse(list(cosine))

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    cosine_serv = CosineServer()
    cosine_serv.run()
