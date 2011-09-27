#!/usr/bin/env python
import roslib 
roslib.load_manifest('file_servers')
import rospy
import threading
from hdf5_run_reader import HDF5_Run_Reader

# Services
from msg_and_srv.srv import SetRunFile 
from msg_and_srv.srv import SetRunFileResponse
from msg_and_srv.srv import GetRunData
from msg_and_srv.srv import GetRunDataResponse

class Base_Run_File_Server(object):

    def __init__(self):

        self.run_reader = None
        self.lock = threading.Lock()

        # Setup services
        self.set_file_srv = rospy.Service(
                'set_run_file', 
                SetRunFile, 
                self.handle_set_run_file
                )
        self.get_run_data_srv = rospy.Service(
                'get_run_data',
                GetRunData,
                self.handle_get_run_data
                )
        # Initialize nodes
        rospy.init_node('position_array_server')

    def run(self):
        rospy.spin()

    def handle_set_run_file(self,req):
        with self.lock:
            try:
                self.run_reader = HDF5_Run_Reader(req.filename)
            except IOError, message:
                return SetRunFileResponse(False,message,'',0)
            mode = self.run_reader.mode
            number_of_runs = self.run_reader.number_of_runs
            return SetRunFileResponse(True,'',mode,number_of_runs)

    def handle_get_run_data(self,req):
        with self.lock:
            run = self.run_reader.get_run(req.run_number)
            return SetRunfileResponse(True,'',[],0.0,0.0)


# -------------------------------------------------------------------------------
if __name__ == '__main__':
    file_server = Base_Run_File_Server()
    file_server.run()
