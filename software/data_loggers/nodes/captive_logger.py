#!/usr/bin/env python
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
        self.directory = rospy.get_param('default_log_dir', 'default_log_dir.txt')
        self.filename = rospy.get_param('default_log_file', 'default_log_file.txt')
        self.logging_rate = rospy.get_param('logging_rate', 50.0)
        self.enabled = False
        self.logger = None

        self.force_data = None
        self.dist_data = None
        self.setpt_data = None
        self.actuator_data = None
        self.motor_cmd_data = None
        self.analog_in_data = None

        # Setup services
        self.set_log_file_srv = rospy.Service('set_logger_file', SetLogFile, self.handle_set_log_file)
        self.node_enable_srv = rospy.Service('logger_enable', NodeEnable, self.handle_node_enable) 

        # Subscribe to messages

        rospy.on_shutdown(self.on_shutdown)
        rospy.init_node('data_logger')
        self.rate = rospy.Rate(self.logging_rate)

    def have_data(self):
        """
        Determine whether or not we have data form logging.
        """
        test = True 
        test &= self.force_data is not None
        test &= self.dist_data is not None
        test &= self.setpt_data is not None
        test &= self.actuator_data is not None
        test &= self.motor_cmd_data is not None
        test &= self.analog_in_data is not None
        return test

    def run(self):
        while not rospy.is_shutdown():
            with self.lock:
                print 'hello ',
                if self.enabled:
                    print 'enabled ',
                    if self.have_data():
                        print 'have data ',
                print 
            self.rate.sleep()

    def handle_set_log_file(self,req):
        """
        Handle set log file service 
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
        Handle node enable service
        """
        if req.enable:

            # Enable logging node
            filepath = os.path.join(self.filename,self.directory)
            try:
                self.logger = HDF5_Logger(filepath) 
                self.logger.open()
                self.logger.addDataSet('force'(1,))
                self.logger.addDataAttribute('force', 'unit', 'N')

                self.enabled = True
                status = True
                message = ''
            except IOError, e:
                self.enabled = False
                status = False
                message = 'unable to open file: %s'%(e,)
        else:

            # Disable logging node
            self.enabled = False
            self.logger.close()
            self.logger = None
            status = True
            message = ''
        return NodeEnableResponse(status,message)

    def on_shutdown(self):
        with self.lock:
            self.enabled = False
            try:
                self.fid.close()
            except:
                pass

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    logger = Captive_Logger()
    try:
        logger.run()
    except rospy.ROSInterruptException:
        pass





        
