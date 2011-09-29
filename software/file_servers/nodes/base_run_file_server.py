#!/usr/bin/env python
"""
base_run_file_server.py

Base clase for for run files server nodes. Services allow setting the current 
hdf5 run file, loading runs from the file, and clearing the current run file.
"""
import roslib 
roslib.load_manifest('file_servers')
import rospy
import threading
from hdf5_run_reader import HDF5_Run_Reader

# Services
from msg_and_srv.srv import SetRunFile 
from msg_and_srv.srv import SetRunFileResponse
from msg_and_srv.srv import GetRunFile
from msg_and_srv.srv import GetRunFileResponse
from msg_and_srv.srv import ClearRunFile
from msg_and_srv.srv import ClearRunFileResponse
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
        self.get_file_srv = rospy.Service(
                'get_run_file',
                GetRunFile, 
                self.handle_get_run_file
                )
        self.clear_file_srv = rospy.Service(
                'clear_run_file',
                ClearRunFile,
                self.handle_clear_run_file
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
        """
        Handles request for setting the current run file.
        """
        with self.lock:

            # Close current run reader is file is open
            if self.run_reader is not None:
                self.run_reader.close()
                self.run_reader = None

            # Open new file with run reader
            try:
                self.run_reader = HDF5_Run_Reader(req.filename)
                mode = self.run_reader.get_mode()
                number_of_runs = self.run_reader.number_of_runs
                return SetRunFileResponse(True,'',mode,number_of_runs)
            except Exception, e:
                message = '%s'%(e,)
                return SetRunFileResponse(False,message,'',0)

    def handle_get_run_file(self,req):
        """
        Handles request for information about the current run file.
        """
        with self.lock:
            if self.run_reader is not None:
                filename = self.run_reader.filename
                number_of_runs = self.run_reader.number_of_runs
                mode = self.run_reader.get_mode()
                return GetRunFileResponse(True,'',filename,mode,number_of_runs)
            else:
                message = 'Run file is not set'
                return GetRunFileResponse(False,message,'','',0)

    def handle_clear_run_file(self,req):
        """
        Clears the current run file from the server.
        """
        with self.lock:
            try:
                self.run_reader.close()
                status = True
            except:
                status = False
            self.run_reader = None
        return ClearRunFileResponse(status)

    def handle_get_run_data(self,req):
        """
        Note, child classes should implement their own verion of this funcion.
        """
        with self.lock:
            if self.run_reader is not None:
                run = self.run_reader.get_run(req.run_number)
                filename = self.run_reader.filename
                return GetRunDataResponse(True,'',filename,[],0.0,0.0)
            else:
                messge = 'run file is not set'
                return GetRunDataResponse(False,message,'',[],0.0,0.0)

# -------------------------------------------------------------------------------
if __name__ == '__main__':
    file_server = Base_Run_File_Server()
    file_server.run()
