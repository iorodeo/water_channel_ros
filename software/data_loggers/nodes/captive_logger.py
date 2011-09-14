#!/usr/bin/env python
"""
Data logger for captivie trajectory mode. 
"""
import roslib 
roslib.load_manifest('data_loggers')
import rospy
import threading
import os
import os.path
from  hdf5_logger import HDF5_Logger

# Messages
from msg_and_srv.msg import ForceMsg
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

class Captive_Logger(object):

    def __init__(self):
        self.lock = threading.Lock()
        default_log_dir = os.path.join(os.environ['HOME'],'ros_logs')
        self.directory = rospy.get_param('default_log_dir', default_log_dir)
        self.filename = rospy.get_param('default_log_file', 'default_log_file.hdf5')
        self.logging_rate = rospy.get_param('logging_rate', 50.0)
        self.enabled = False
        self.logger = None
        self.start_time = None

        # Initialize all data values to None
        self.init_data_values()

        # Setup services
        self.set_log_file_srv = rospy.Service('set_logger_file', SetLogFile, self.handle_set_log_file)
        self.node_enable_srv = rospy.Service('logger_enable', NodeEnable, self.handle_node_enable) 

        # Subscribe to messages
        self.force_sub = rospy.Subscriber('force', ForceMsg, self.force_msg_callback)
        self.dist_sub = rospy.Subscriber('distance', DistMsg, self.dist_msg_callback)
        self.setpt_sub = rospy.Subscriber('setpt', SetptMsg, self.setpt_msg_callback)
        self.actuator_sub = rospy.Subscriber('actuator', ActuatorMsg, self.actuator_msg_callback)
        self.motor_sub = rospy.Subscriber('motor_cmd', MotorCmdMsg, self.motor_msg_callback)
        self.analog_sub = rospy.Subscriber('analog_input', AnalogInMsg, self.analog_msg_callback)

        rospy.on_shutdown(self.on_shutdown)
        rospy.init_node('data_logger')
        self.rate = rospy.Rate(self.logging_rate)

    def have_data(self):
        """
        Determine whether or not we have data form logging.
        Note, should be called on with the threading lock. 
        """
        test = True 
        test &= self.force_data is not None
        test &= self.dist_data is not None
        test &= self.setpt_data is not None
        test &= self.actuator_data is not None
        test &= self.motor_data is not None
        test &= self.analog_data is not None
        return test

    def force_msg_callback(self, data):
        """
        Callback for handling force sensor messages
        """
        with self.lock:
            if self.enabled:
                self.force_data = data

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
                    self.logger.add_attribute('/data/actuator','type',data.type)
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
                    self.logger.add_dataset('/data/analog_input', (n,))
                    self.logger.add_attribute('/data/analog_input', 'unit', 'V')
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
        print 'writing data'

        # Time data
        rostime = rospy.get_rostime()
        time = rostime.to_sec() - self.start_time
        self.logger.add_dataset_value('/data/time', time)

        # Force sensor data
        self.logger.add_dataset_value( '/data/force', self.force_data.force)

        # Distance sensor data
        self.logger.add_dataset_value('/data/distance/distance_raw', self.dist_data.distance)
        self.logger.add_dataset_value('/data/distance/distance_kalman', self.dist_data.distance_kalman)
        self.logger.add_dataset_value('/data/distance/velocity_kalman', self.dist_data.velocity_kalman)

        # Set point data 
        self.logger.add_dataset_value('/data/setpt/position', self.setpt_data.position)
        self.logger.add_dataset_value('/data/setpt/velocity', self.setpt_data.velocity)
        self.logger.add_dataset_value('/data/setpt/error', self.setpt_data.error,)

        # Actuator data
        self.logger.add_dataset_value('/data/actuator', self.actuator_data.value)

        # Motor command data
        self.logger.add_dataset_value('/data/motor_cmd', self.motor_data.motor_cmd)

        # Analog input data
        self.logger.add_dataset_value('/data/analog_input', self.analog_data.values)

    def handle_set_log_file(self,req):
        """
        Handle set log file service. Gets log file name and directory.
        """
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
        print req.enable
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
        print filepath
        self.logger = HDF5_Logger(filepath) 

        # Create info group 
        self.logger.add_group('/info')
        self.logger.add_datetime('/info')
        self.logger.add_attribute('/info', 'mode', 'captive_trajectory')

        # Create time dataset
        self.logger.add_dataset('/data/time', (1,))
        self.logger.add_attribute('/data/time', 'unit', 's')

        # Create force dataset
        self.logger.add_dataset('/data/force',(1,))
        self.logger.add_attribute('/data/force', 'unit', 'N')

        # Create distance sensor dataset
        self.logger.add_dataset('/data/distance/distance_raw', (1,))
        self.logger.add_dataset('/data/distance/distance_kalman', (1,))
        self.logger.add_dataset('/data/distance/velocity_kalman', (1,))
        self.logger.add_attribute('/data/distance/distance_raw', 'unit', 'mm')
        self.logger.add_attribute('/data/distance/distance_kalman', 'unit', 'mm')
        self.logger.add_attribute('/data/distance/velocity_kalman', 'unit', 'mm/s')

        # Create setpt dataset
        self.logger.add_dataset('/data/setpt/position', (1,))
        self.logger.add_dataset('/data/setpt/velocity', (1,))
        self.logger.add_dataset('/data/setpt/error', (1,))
        self.logger.add_attribute('/data/setpt/position', 'unit', 'mm')
        self.logger.add_attribute('/data/setpt/velocity', 'unit', 'mm/s')
        self.logger.add_attribute('/data/setpt/error', 'unit', 'mm')

        # Create actuator actuator dataset 
        self.logger.add_dataset('/data/actuator',(1,))
        self.logger.add_attribute('/data/actuator','unit','us')

        # Create motor command dataset
        self.logger.add_dataset('/data/motor_cmd', (1,))
        self.logger.add_attribute('/data/motor_cmd', 'unit', '12-bit int')

        # Note, Create analog input dataset not created until first
        # analog input callback in order to get array size

    def init_data_values(self):
        """
        Initializes all data values to None
        """
        self.force_data = None
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

    logger = Captive_Logger()
    try:
        logger.run()
    except rospy.ROSInterruptException:
        pass





        
