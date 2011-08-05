#!/usr/bin/env python
import roslib 
roslib.load_manifest('motor_cmd_source')
import rospy
import threading
import math
from std_msgs.msg import Header
from msg_and_srv.msg import MotorCmdMsg

class MotorCmdSinSrc(object):

    def __init__(self):
        self.update_rate = rospy.get_param('update_rate',50)
        self.amplitude = rospy.get_param('sin_amplitude', 300)
        self.period = rospy.get_param('sin_period', 10)

        # Setup motor command  topic
        self.MotorCmdMsg = MotorCmdMsg()
        self.pub = rospy.Publisher('motor_cmd', MotorCmdMsg)

        self.rate = rospy.Rate(self.update_rate)
        self.is_first = True

    def run(self):
        while not rospy.is_shutdown():
            self.MotorCmdMsg.header.stamp = rospy.get_rostime()
            value = self.sin()
            self.MotorCmdMsg.motor_cmd = value
            self.pub.publish(self.MotorCmdMsg)
            self.rate.sleep()
            
    def sin(self):
        if self.is_first == True:
            self.is_first = False
            self.t_start = rospy.get_rostime().to_sec()
        t  = rospy.get_rostime().to_sec() - self.t_start
        value = self.amplitude*math.sin(2.0*math.pi*t/self.period)
        return value


# -----------------------------------------------------------------------------
if __name__ == '__main__':
    # Initialize node
    rospy.init_node('motor_cmd_sin_source')
    cmdSource = MotorCmdSinSrc()
    try:
        cmdSource.run()
    except rospy.ROSInterruptException:
        pass


