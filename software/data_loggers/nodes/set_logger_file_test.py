#!/usr/bin/env python
"""
set_logger_file_test.py

This node tests the set_logger_file service provided by the various logging
nodes. 
"""
import roslib 
roslib.load_manifest('data_loggers')
import rospy
from msg_and_srv.srv import SetLogFile

def set_logfile(filename,directory):
    rospy.wait_for_service('set_logger_file')
    try: 
        set_logfile_cmd = rospy.ServiceProxy('set_logger_file',SetLogFile)
        resp = set_logfile_cmd(filename,directory)
        print resp
    except rospy.ServiceException, e:
        print 'Service call failed: %s'%(e,)

if __name__ == '__main__':
    import sys
    # Get filename and directory from command line
    filename = sys.argv[1]
    directory = sys.argv[2]
    # Send valus to logger
    set_logfile(filename,directory)


