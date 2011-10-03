#!/usr/bin/env python
"""
base_run_server.py

Base class for run files server nodes. Services allow setting the current 
hdf5 run file, loading runs from the file, and clearing the current run file.
"""
import roslib 
roslib.load_manifest('file_servers')
import rospy
import threading
import numpy
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

class Base_Run_Server(object):

    def __init__(self,node_name):

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
        rospy.init_node(node_name)

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
                message = 'run file is not set'
                return GetRunDataResponse(False,message,'',[],0.0,0.0)

    def get_actuator_dt(self):
        """
        Gets the actuator update rate from the parameter server
        """
        dt = 1.0/rospy.get_param('/actuator_update_rate',50.0)
        return dt

# Utility functions -----------------------------------------------------------

def get_values_array(run,dt):
    """
    Get array of actuator values based on type.
    """
    run_type = run.attrs['type'].lower()
    if run_type == 'constant':
        params = run['params']
        values_array = get_constant_array(params,dt)
    elif run_type == 'trapezoidal':
        params = run['params']
        values_array = get_trapezoidal_array(params,dt)
    elif run_type == 'array':
        values_array = run['values']
    else:
        raise ValueError, 'unknow run type: %s'%(run_type,)
    return values_array

def get_constant_array(params,dt):
    """
    Creates values array for runs of type constant.
    """
    T = float(params['T'][0])
    value = float(params['value'][0])
    N = int(T/dt)
    values_array = value*numpy.ones((N,))
    return values_array

def get_trapezoidal_array(params,dt):
    """
    Create values array for runs of type trapezoidal
    """
    # Extract parameters
    T = float(params['T'][0])
    value_0 = float(params['value_0'][0])
    value_1 = float(params['value_1'][0])
    rate = abs(float(params['rate'][0]))
    N = int(T/dt)

    # Create trapezoidal profile
    array_0 = numpy.arange(N)
    array_1 = numpy.ones((N,))
    array_2 = (N-1) - array_0

    if (value_1 - value_0) >= 0:
        print '1'
        array_0 = dt*rate*array_0 + value_0
        array_1 = value_1*array_1
        array_2 = dt*rate*array_2 + value_0
        joint_array = numpy.array([array_0, array_1, array_2])
        values_array = numpy.min(joint_array,axis=0)
    else:
        print '2'
        array_0 = -dt*rate*array_0 + value_0
        array_1 = value_1*array_1
        array_2 = -dt*rate*array_2 + value_0
        joint_array = numpy.array([array_0, array_1, array_2])
        values_array = numpy.max(joint_array,axis=0)
    return values_array

# -------------------------------------------------------------------------------
if __name__ == '__main__':
    run_server = Base_Run_Server('base_run_server')
    run_server.run()
