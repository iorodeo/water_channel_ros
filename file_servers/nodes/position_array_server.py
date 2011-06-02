#!/usr/bin/env python
import roslib 
roslib.load_manifest('file_servers')
import rospy
from file_servers.srv import * 
import os

class PositionArrayServer(object):

    def __init__(self):

        # Setup controller service
        self.srv = rospy.Service('load_position_array', LoadPositionArray, self.handle_load_request)

        # Initialize nodes
        rospy.init_node('position_array_server')

    def run(self):
        rospy.spin()

    def handle_load_request(self, req):
        status = False
        message = ''
        coord_frame = ''
        position_array = []
        try:
            with open(req.filename,'r') as f:
                coord_frame = f.readline().strip()
                for data in f.readlines():
                    position_array.append(float(data))
                status = True
        except Exception, e:
            message = e.__str__()
        return LoadPositionArrayResponse(status,message,coord_frame,position_array)
        

# --------------------------------------------
if __name__ == '__main__':
    posArrayServer = PositionArrayServer()
    posArrayServer.run()
