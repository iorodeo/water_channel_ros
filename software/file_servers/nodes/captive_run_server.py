#!/usr/bin/env python
"""
captive_file_server.py

Run file server node for captive trajectory mode.  Services allow setting the current 
hdf5 run file, loading runs from the file, and clearing the current run file.
"""
import roslib 
roslib.load_manifest('file_servers')
import rospy
import threading
from base_run_server import Base_Run_Server
from base_run_server import get_values_array

# Services
from msg_and_srv.srv import GetRunDataResponse

class Captive_Run_Server(Base_Run_Server):

    def handle_get_run_data(self,req):
        """
        Get run data from hdf5 file for captive trajectory run
        """
        with self.lock:
            if self.run_reader is not None:
                filename = self.run_reader.filename
                dt = self.get_actuator_dt()
                run = self.run_reader.get_run(req.run_number)
                values = get_values_array(run,dt)
                mass = float(run['mass'][0])
                damping = float(run['damping'][0])
                return GetRunDataResponse(True,'',filename,values,mass,damping)
            else:
                message = 'run file is not set'
                return GetRunDataResponse(False,message,'',[],0.0,0.0)

# -------------------------------------------------------------------------------
if __name__ == '__main__':
    run_server = Captive_Run_Server('captive_run_server')
    run_server.run()


