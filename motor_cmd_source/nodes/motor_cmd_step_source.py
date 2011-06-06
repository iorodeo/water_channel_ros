#!/usr/bin/env python
import roslib 
roslib.load_manifest('motor_cmd_source')
import rospy
import threading
import math
from std_msgs.msg import Header
from motor_cmd_source.msg import MotorCmdMsg 


class TestMotorCmdSrc(object):

    def __init__(self, t_low=3.0, t_high=5.0, step_list=[200,300]):
        self.update_rate = 50

        # Step function parameters
        self.t_low = t_low
        self.t_high= t_high
        self.step_list = step_list
        self.step_cnt = 0

        self.t0 = self.t_low
        self.t1 = self.t_low + self.t_high
        self.t2 = 2*self.t_low + self.t_high
        self.t3 = 2*self.t_low + 2*self.t_high
        self.t4 = 3*self.t_low + 2*self.t_high

        # Setup motor command  topic
        self.MotorCmdMsg = MotorCmdMsg()
        self.pub = rospy.Publisher('motor_cmd', MotorCmdMsg)

        # Initialize node
        rospy.init_node('motor_cmd_step_source')
        self.t_step_start = rospy.get_time()
        self.rate = rospy.Rate(self.update_rate)

    def run(self):
        while not rospy.is_shutdown():
            self.MotorCmdMsg.header.stamp = rospy.get_rostime()
            t, value = self.stepFunc()
            if t > self.t4:
                self.step_cnt += 1
                if self.step_cnt >= len(self.step_list):
                    break
                else:
                    self.t_step_start = rospy.get_time()
            self.MotorCmdMsg.motor_cmd = value
            self.pub.publish(self.MotorCmdMsg)
            self.rate.sleep()
            

    def stepFunc(self):
        t_curr  = rospy.get_time() - self.t_step_start
        if t_curr < self.t0:
            value = 0
        elif t_curr >= self.t0 and t_curr < self.t1:
            value = self.step_list[self.step_cnt]
        elif t_curr >= self.t1 and t_curr < self.t2:
            value = 0
        elif t_curr >= self.t2 and t_curr < self.t3:
            value = -self.step_list[self.step_cnt]
        else:
            value = 0 
        return t_curr, value


# -----------------------------------------------------------------------------
if __name__ == '__main__':
    step_list = [100,200,300,400,500,600,700,800,900,1000]
    #step_list = range(10,210,10)
    cmdSource = TestMotorCmdSrc(step_list=step_list)
    try:
        cmdSource.run()
    except rospy.ROSInterruptException:
        pass


