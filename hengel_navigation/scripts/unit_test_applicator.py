#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion, PoseStamped
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from hengel_navigation.msg import ValveInput, OperationMode
import tf
from math import radians, copysign, sqrt, pow, pi, atan2, sin, floor, cos
from tf.transformations import euler_from_quaternion
import numpy as np
import sys
#from PIL import Image
#import PIL.Image
import time
import os

#MARKER_DOWN = 1023
#MARKER_DOWN = 870
# MARKER_DOWN = 890
MARKER_DOWN = 2420
#MARKER_DOWN=900
#MARKER_UP = 512
MARKER_UP = 2000


class UnitTestApplicator():
    def __init__(self):
        self.initial_setting()
        self.run()

    def initial_setting(self):
        rospy.init_node('unit_test_applicator', anonymous=False, disable_signals=True)


        self.valve_operation_mode = OperationMode()
        self.valve_operation_mode.mode = 1
        self.valve_angle_input = ValveInput()

        self.valve_status = MARKER_UP

        self.r = rospy.Rate(50)  #50hz

        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.valve_angle_publisher = rospy.Publisher(
            '/valve_input', ValveInput, queue_size=5)
        self.valve_operation_mode_publisher = rospy.Publisher(
            '/operation_mode', OperationMode, queue_size=5)
        self.valve_operation_mode_publisher.publish(
            self.valve_operation_mode)

        self.loop_cnt = 0

        print("initialized")

    def run(self):
        self.wait_for_seconds(2)
        print("while loop start")
        while(True):
            try:
                self.loop_cnt=self.loop_cnt+1
                if self.loop_cnt==50:
                    self.loop_cnt=0
                    self.valve_status = MARKER_UP if self.valve_status==MARKER_DOWN else MARKER_DOWN
                    print("valve_status = "+("MARKER_UP" if self.valve_status==MARKER_UP else "MARKER_DOWN"))
                self.valve_angle_input.goal_position = self.valve_status
                self.valve_angle_publisher.publish(
                    self.valve_angle_input)

                self.r.sleep()

            except KeyboardInterrupt:
                print("Got KeyboardInterrupt")
                break

    def wait_for_seconds(self, _input):
        cnt_loop = (int)(_input * 50.0)
        for i in range(cnt_loop):
            self.r.sleep()

if __name__ == '__main__':
    try:
        UnitTestApplicator()
        print("End of Main Function")

    except Exception as e:
        print(e)
        rospy.loginfo("shutdown program.")
        sys.exit()
