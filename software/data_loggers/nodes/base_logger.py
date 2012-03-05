#!/usr/bin/env python
"""
Base class for data logger nodes 
"""
import roslib 
roslib.load_manifest('data_loggers')
import rospy
import threading
import os
import os.path
import numpy
from utilities.hdf5_logger import HDF5_Logger

# Messages
from msg_and_srv.msg import DistMsg 
from msg_and_srv.msg import SetptMsg
from msg_and_srv.msg import ActuatorMsg
from msg_and_srv.msg import MotorCmdMsg
from msg_and_srv.msg import AnalogInMsg

# Services
from msg_and_srv.srv import SetLogFile
from msg_and_srv.srv import SetLogFileResponse
from msg_and_srv.srv import NodeEnable
from msg_and_srv.srv import NodeEnableResponse

class Base_Logger(object):

    def __init__(self):
        self.lock = threading.Lock()
        default_log_dir = os.path.join(os.environ['HOME'],'ros_logs')
        self.directory = rospy.get_param('default_log_dir', default_log_dir)
        self.filename = rospy.get_param('default_log_file', 'default_log_file.hdf5')
        self.logging_rate = rospy.get_param('logging_rate', 50.0)
        self.enabled = False
        self.logger = None
        self.start_time = None
        self.trial_str = None

        # Initialize all data values to None
        self.init_data_values()

        # Setup services
        self.set_log_file_srv = rospy.Service('set_logger_file', SetLogFile, self.handle_set_log_file)
        self.node_enable_srv = rospy.Service('logger_enable', NodeEnable, self.handle_node_enable) 
        
        # Subscribe to messages - common to all logs
        self.dist_sub = rospy.Subscriber('distance', DistMsg, self.dist_msg_callback)
        self.setpt_sub = rospy.Subscriber('setpt', SetptMsg, self.setpt_msg_callback)
        self.actuator_sub = rospy.Subscriber('actuator', ActuatorMsg, self.actuator_msg_callback)
        self.motor_sub = rospy.Subscriber('motor_cmd', MotorCmdMsg, self.motor_msg_callback)
        self.analog_sub = rospy.Subscriber('analog_input', AnalogInMsg, self.analog_msg_callback) 

        rospy.on_shutdown(self.on_shutdown)

    def init_node(self):
        rospy.init_node('data_logger')
        self.rate = rospy.Rate(self.logging_rate)

    def have_data(self):
        """
        Determine whether or not we have data form logging.
        Note, should be called on with the threading lock. 
        """
        test = True 
        #test &= self.force_data is not None
        test &= self.dist_data is not None
        test &= self.setpt_data is not None
        #test &= self.actuator_data is not None
        test &= self.motor_data is not None
        test &= self.analog_data is not None
        return test

    def dist_msg_callback(self, data):
        """
        Callback for handling distance messages
        """
        with self.lock:
            if self.enabled:
                self.dist_data = data

    def setpt_msg_callback(self, data):
        """
        Callback for handling setpt messages
        """
        with self.lock:
            if self.enabled:
                self.setpt_data = data

    def actuator_msg_callback(self, data):
        """
        Callback for handling actuator messages. On fist call after reset
        set attribute for actuator type. 
        """
        with self.lock:
            if self.enabled:
                if self.actuator_data is None:
                    self.logger.add_attribute(self.actuator_path,'type',data.type)
                self.actuator_data = data

    def motor_msg_callback(self, data):
        """
        Callback for handling motor cmd messages
        """
        with self.lock:
            if self.enabled:
                self.motor_data = data

    def analog_msg_callback(self, data):
        """
        Callback for handling analog input messages. Note on first call
        after reset create analog input dataset in log file.
        """
        with self.lock:
            if self.enabled:
                if self.analog_data is None: 
                    # First call since reset - create analog input dataset
                    n = len(data.values)
                    self.logger.add_dataset(self.analog_input_path, (n,))
                    self.logger.add_attribute(self.analog_input_path, 'unit', 'V')
                self.analog_data = data
                

    def run(self):
        """
        Main loop - writes data to log file if node is enabled and we have
        data in all 
        """
        while not rospy.is_shutdown():
            with self.lock:
                if self.enabled and self.have_data():
                    self.write_data()
            self.rate.sleep()

    def write_data(self):
        """
        Write data to log file.
        """
        # Time data
        rostime = rospy.get_rostime()
        time = rostime.to_sec() - self.start_time
        self.logger.add_dataset_value(self.time_path, time)

        # Distance sensor data
        self.logger.add_dataset_value(self.distance_raw_path, self.dist_data.distance)
        self.logger.add_dataset_value(self.distance_kalman_path, self.dist_data.distance_kalman)
        self.logger.add_dataset_value(self.velocity_kalman_path, self.dist_data.velocity_kalman)

        # Set point data 
        self.logger.add_dataset_value(self.setpt_position_path, self.setpt_data.position)
        self.logger.add_dataset_value(self.setpt_velocity_path, self.setpt_data.velocity)
        self.logger.add_dataset_value(self.setpt_error_path, self.setpt_data.error,)

        # Actuator data
        if not self.actuator_data is None:
            self.logger.add_dataset_value(self.actuator_path, self.actuator_data.value)

        # Motor command data
        self.logger.add_dataset_value(self.motor_cmd_path, self.motor_data.motor_cmd)

        # Analog input data
        self.logger.add_dataset_value(self.analog_input_path, self.analog_data.values)

    def handle_set_log_file(self,req):
        """
        Handle set log file service. Gets log file name and directory.
        """
        with self.lock:
            if not self.enabled:
                self.directory = req.directory
                self.filename = req.filename
                if not self.check_log_dir():
                    status = False
                    message = 'log directory does not exist'
                else:
                    status = True
                    message = ''
            else:
                status = False
                message = 'cannot set log director while node is enabled'
        return SetLogFileResponse(status,message)

    def check_log_dir(self):
        """
        Check whether or not log dir exists
        """
        return os.path.isdir(self.directory)


    def handle_node_enable(self,req):
        """
        Handle node enable service. Starts and stops loggin. 
        """
        message = ''
        with self.lock:
            if req.enable:
                # Enable logging mode
                if not self.enabled:
                    try:
                        self.create_hdf5_logger()
                        self.enabled = True
                        rostime = rospy.get_rostime()
                        self.start_time = rostime.to_sec()
                    except IOError, e:
                         self.enabled = False
                         message = 'unable to open file: %s'%(e,)
            else:
                # Disable logging node and reset data to None
                self.enabled = False
                del self.logger
                self.logger = None
                self.init_data_values()

        return NodeEnableResponse(self.enabled,message)

    def create_hdf5_logger(self):
        """
        Creates hdf5 data logger, adds all groups, datasets and attributes 
        except the analog input dataset which because of its possibly variable
        size is created on the first callback of the analog input message.
        """
        # Create HDF5 data logger
        filepath = os.path.join(self.directory,self.filename)
        if os.path.isfile(filepath):
            log_exists = True
        else:
            log_exists = False
        self.logger = HDF5_Logger(filepath,'a') 
        if not log_exists:
            # Create top level information for log file - as the log file
            # has just been created.
            self.top_info_path = '/info'
            self.logger.add_group(self.top_info_path)
            self.logger.add_datetime(self.top_info_path)
            self.logger.add_attribute(self.top_info_path,'type', 'log_file')

        # Get number of current trial
        trial_list = [grp for grp in self.logger.list('/') if grp[:5] == 'trial']
        trial_num = len(trial_list)
        self.trial_str = 'trial_%d'%(trial_num,)

        # Create trial group
        self.logger.add_group('/%s'%(self.trial_str,))

        # Create info group 
        self.trial_info_path = '/%s/info'%(self.trial_str,)
        self.logger.add_group(self.trial_info_path)
        self.logger.add_datetime(self.trial_info_path)

        # Create time dataset
        self.time_path = '/%s/data/time'%(self.trial_str,)
        self.logger.add_dataset(self.time_path, (1,))
        self.logger.add_attribute(self.time_path, 'unit', 's')

        # Create distance sensor dataset
        self.distance_raw_path = '/%s/data/distance/distance_raw'%(self.trial_str,)
        self.logger.add_dataset(self.distance_raw_path, (1,))
        self.logger.add_attribute(self.distance_raw_path, 'unit', 'mm')

        self.distance_kalman_path = '/%s/data/distance/distance_kalman'%(self.trial_str,)
        self.logger.add_dataset(self.distance_kalman_path, (1,))
        self.logger.add_attribute(self.distance_kalman_path, 'unit', 'mm')

        self.velocity_kalman_path = '/%s/data/distance/velocity_kalman'%(self.trial_str,)
        self.logger.add_dataset(self.velocity_kalman_path, (1,))
        self.logger.add_attribute(self.velocity_kalman_path, 'unit', 'mm/s')

        # Create setpt dataset
        self.setpt_position_path = '/%s/data/setpt/position'%(self.trial_str,)
        self.logger.add_dataset(self.setpt_position_path, (1,))
        self.logger.add_attribute(self.setpt_position_path, 'unit', 'mm')

        self.setpt_velocity_path = '/%s/data/setpt/velocity'%(self.trial_str,)
        self.logger.add_dataset(self.setpt_velocity_path, (1,))
        self.logger.add_attribute(self.setpt_velocity_path, 'unit', 'mm/s')

        self.setpt_error_path = '/%s/data/sept/error'%(self.trial_str,)
        self.logger.add_dataset(self.setpt_error_path, (1,))
        self.logger.add_attribute(self.setpt_error_path, 'unit', 'mm')

        # Create actuator actuator dataset 
        self.actuator_path = '/%s/data/actuator'%(self.trial_str,)
        self.logger.add_dataset(self.actuator_path,(1,))
        self.logger.add_attribute(self.actuator_path,'unit','us')

        # Create motor command dataset
        self.motor_cmd_path = '/%s/data/motor_cmd'%(self.trial_str,)
        self.logger.add_dataset(self.motor_cmd_path, (1,))
        self.logger.add_attribute(self.motor_cmd_path, 'unit', '12-bit int')

        # Note, Create analog input dataset not created until first
        # analog input callback in order to get array size
        self.analog_input_path = '/%s/data/analog_input'%(self.trial_str,)

    def init_data_values(self):
        """
        Initializes all data values to None
        """
        self.dist_data = None
        self.setpt_data = None
        self.actuator_data = None
        self.motor_data = None
        self.analog_data = None

    def on_shutdown(self):
        with self.lock:
            self.enabled = False
            try:
                del self.logger
                self.logger = None
            except:
                pass

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    logger = Base_Logger()
    logger.init_node()
    try:
        logger.run()
    except rospy.ROSInterruptException:
        pass





        
