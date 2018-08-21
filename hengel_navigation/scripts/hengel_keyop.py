#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion, PoseStamped
from std_msgs.msg import Float32, Time, Int32
from sensor_msgs.msg import Image, CompressedImage
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from hengel_navigation.msg import ValveInput, OperationMode
import tf
from math import radians, copysign, sqrt, pow, pi, atan2, sin, floor, cos, asin
from tf.transformations import euler_from_quaternion
import numpy as np
import sys
import time
import os
import cv2
import cv_bridge
import logging
import matplotlib.pyplot as plt

class Keyop():
    def __init__(self):
        rospy.init_node('hengel_keyop', anonymous=False)

        self.delta1 = 0.0
        self.delta2 = 0.0

        self.pub_delta_theta_1 = rospy.Publisher('/delta_theta_1', Float32, queue_size=5)
        self.pub_delta_theta_2 = rospy.Publisher('/delta_theta_2', Float32, queue_size=5)
        rospy.Subscriber('/keyboard_input', Int32, self.callback)

        self.r=rospy.Rate(50)

        self.run()

    def callback(self, _keyboard):
        increment = 0.1
        if _keyboard==0: #straight
            self.delta1 += increment
            self.delta2 += increment
        elif _keyboard == 1: #backward
            self.delta1 -= increment
            self.delta2 -= increment
        elif _keyboard ==2: #right
            self.delta1 += increment
            self.delta2 -= increment
        elif _keyboard ==3: # left
            self.delta1 -= increment
            self.delta2 -= increment
        elif _keyboard == 4: #stop
            self.delta1 = 0.0
            self.delta2 = 0.0
    
    def run(self):
        while True:
            if rospy.is_shutdown():
                break
            try:
                self.pub_delta_theta_1.publish(self.delta1)
                self.pub_delta_theta_2.publish(self.delta2)
                self.r.sleep()
                
            except KeyboardInterrupt:
                self.pub_delta_theta_1.publish(0.0)
                self.pub_delta_theta_2.publish(0.0)
                print("Got KeyboardInterrupt")
                rospy.signal_shutdown("KeyboardInterrupt")
                break

if __name__=='__main__':
    Keyop()