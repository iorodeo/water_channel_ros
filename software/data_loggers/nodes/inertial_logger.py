#!/usr/bin/env python
"""
Data logger for inertial trajectory mode. 
"""
import roslib 
roslib.load_manifest('data_loggers')
import rospy
import os
import os.path
from position_logger import Position_Logger

class Inertial_Logger(Position_Logger):

    def create_hdf5_logger(self):
        """
        Creates hdf5 data logger. 
        """
        super(Captive_Logger,self).create_hdf5_logger()
        self.logger.add_attribute(self.trial_info_path, 'mode', 'inertial trajectory')

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    logger = Inertial_Logger()
    logger.init_node()
    try:
        logger.run()
    except rospy.ROSInterruptException:
        pass





        
