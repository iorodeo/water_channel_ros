#!/usr/bin/env python
import roslib 
roslib.load_manifest('data_loggers')
import rospy
import threading
import os
import os.path
from distance_118x.msg import DistMsg 
from motor_cmd_source.msg import MotorCmdMsg

class MotorCmdLogger(object):

    def __init__(self):
        self.filename = 'motor_cmd_data.txt'
        self.data_dir = os.path.join(os.environ['HOME'],'ros_logs')
        self.check_data_dir()
        self.data_file = os.path.join(self.data_dir, self.filename)
        self.fid = open(self.data_file, 'w')

        self.distance_data = None
        self.motor_cmd_data = None
        self.lock = threading.Lock()
        self.distance_sub = rospy.Subscriber('distance', DistMsg, self.distance_callback)
        self.motor_cmd_sub = rospy.Subscriber('motor_cmd', MotorCmdMsg, self.motor_cmd_callback)
        rospy.on_shutdown(self.on_shutdown)
        rospy.init_node('motor_cmd_logger')

    def check_data_dir(self):
        if not os.path.isdir(self.data_dir):
            os.path.mkdir(self.data_dir)

    def run(self):
        rospy.spin()

    def distance_callback(self,data):
        with self.lock:
            self.distance_data = data

    def motor_cmd_callback(self,data):
        with self.lock:
            self.motor_cmd_data = data
        if self.distance_data:
            distance = self.distance_data.distance
            motor_cmd = self.motor_cmd_data.motor_cmd
            self.fid.write('%f %f\n'%(motor_cmd, distance))

    def on_shutdown(self):
        self.fid.close()


# -----------------------------------------------------------------------------
if __name__ == '__main__':

    logger = MotorCmdLogger()
    try:
        logger.run()
    except rospy.ROSInterruptException:
        pass
