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


class Keyop_PUblisher():
    def __init__(self):
        rospy.init_node('hengel_keyop_publisher', anonymous=False)
        self.pub=rospy.Publisher('/keyboard_input', Int32, queue_size=5)

        self.run()

    def run(self):
        while True:
            try:
                key_input = raw_input("w: forward, x: backward, a:left, d: right")
                if key_input=='a':
                    self.pub.publish(3)
                elif key_input=='w':
                    self.pub.publish(0)
                elif key_input=='d':
                    self.pub.publish(2)
                elif key_input=='x':
                    self.pub.publish(1)
                elif key_input=='s':
                    self.pub.publish(4)
            except KeyboardInterrupt:
                print("Got KeyboardInterrupt")
                rospy.signal_shutdown("KeyboardInterrupt")
                break

if __name__=='__main__':
    Keyop_PUblisher()