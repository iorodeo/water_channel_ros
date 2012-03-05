#!/usr/bin/env python
"""
Data logger for position trajectory mode. 
"""
import roslib 
roslib.load_manifest('data_loggers')
import rospy
import os
import os.path
from base_logger import Base_Logger

# Messages
from msg_and_srv.msg import ForceMsg

class Position_Logger(Base_Logger):

    def __init__(self):
        super(Position_Logger,self).__init__()
        self.force_sub = rospy.Subscriber('force', ForceMsg, self.force_msg_callback)
        
    def have_data(self):
        """
        Determine whether or not we have data form logging.
        Note, should be called on with the threading lock. 
        """
        test = super(Position_Logger,self).have_data()
        test &= self.force_data is not None
        return test

    def force_msg_callback(self, data):
        """
        Callback for handling force sensor messages
        """
        with self.lock:
            if self.enabled:
                self.force_data = data

    def write_data(self):
        """
        Write data to log file.
        """
        super(Position_Logger,self).write_data()

        # Write force sensor data
        self.logger.add_dataset_value(self.force_path, self.force_data.force)


    def create_hdf5_logger(self):
        """
        Creates hdf5 data logger, adds all groups, datasets and attributes 
        except the analog input dataset which because of its possibly variable
        size is created on the first callback of the analog input message.
        """
        super(Position_Logger,self).create_hdf5_logger()

        # Set mode attribute for trial to 'captive trajectory'
        self.logger.add_attribute(self.trial_info_path, 'mode', 'position trajectory')

        # Create force dataset
        self.force_path = '/%s/data/force'%(self.trial_str,)
        self.logger.add_dataset(self.force_path,(1,))
        self.logger.add_attribute(self.force_path, 'unit', 'N')

    def init_data_values(self):
        """
        Initializes all data values to None
        """
        super(Position_Logger,self).init_data_values()
        self.force_data = None

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    logger = Position_Logger()
    logger.init_node()
    try:
        logger.run()
    except rospy.ROSInterruptException:
        pass





        
