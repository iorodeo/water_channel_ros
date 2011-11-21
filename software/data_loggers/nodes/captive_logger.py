#!/usr/bin/env python
"""
Data logger for captivie trajectory mode. 
"""
import roslib 
roslib.load_manifest('data_loggers')
import rospy
import os
import os.path
from position_logger import Position_Logger

# Services
from msg_and_srv.srv import GetDynamParams

class Captive_Logger(Position_Logger):

    def create_hdf5_logger(self):
        """
        Creates hdf5 data logger, adds all groups, datasets and attributes 
        except the analog input dataset which because of its possibly variable
        size is created on the first callback of the analog input message.
        """
        super(Captive_Logger,self).create_hdf5_logger()

        # Set mass and damping parameters
        rospy.wait_for_service('get_dynam_params')
        try: 
            get_cmd = rospy.ServiceProxy('get_dynam_params',GetDynamParams)
            resp = get_cmd()
            mass = resp.mass
            damping = resp.damping
        except rospy.ServiceException, e:
            rospy.logerr('unable to get mass and damping parameters for log file')
            mass = numpy.nan
            damping = numpy.nan

        self.mass_path = '/%s/data/mass'%(self.trial_str,)
        self.logger.add_dataset(self.mass_path, (1,))
        self.logger.add_attribute(self.mass_path,'unit', 'kg')
        self.logger.add_dataset_value(self.mass_path, mass)

        self.damping_path = '/%s/data/damping'%(self.trial_str,)
        self.logger.add_dataset(self.damping_path, (1,))
        self.logger.add_attribute(self.damping_path, 'unit', 'kg/s')
        self.logger.add_dataset_value(self.damping_path, damping)

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    logger = Captive_Logger()
    logger.init_node()
    try:
        logger.run()
    except rospy.ROSInterruptException:
        pass





        
