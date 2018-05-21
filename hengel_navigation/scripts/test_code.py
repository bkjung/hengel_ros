#!/usr/bin/env python

# import rospy
from geometry_msgs.msg import Twist, Point, Quaternion, PoseStamped
from std_msgs.msg import Float32
import tf
from math import radians, copysign, sqrt, pow, pi, atan2, sin, floor, cos
from tf.transformations import euler_from_quaternion
import numpy as np
import sys
from PIL import Image
import time
import os
from navigation_control import NavigationControl
import cv2



package_base_path = os.path.abspath(os.path.join(os.path.dirname(__file__),"../.."))
os.system("mkdir -p "+package_base_path+"/hengel_path_manager/output_pathmap")
os.system("mkdir -p"+package_base_path+"/hengel_path_manager/waypnts")


class GoToPoint():
    def __init__(self):
        self.arr_path=[]
        self.draw_start_index=[]
        print("Type point to go: ")
        self.x=raw_input("x: ")
        print(self.x)
        self.y=raw_input("y: ")
        print(self.y)
        self.theta=raw_input("theta: ")
        print(self.theta)

        self.make_path()
        self.run(self.theta)
        
    def make_path(self):
        path_to_point.append([0.0, 0.0])
        path_to_point.append([(float)(self.x), (float)(self.y)])
        self.arr_path.append(path_to_point)

    def run(self, _theta):
        # NavigationControl(self.arr_path, self.draw_start_index)
        NavigationControl(self.arr_path, _theta)

if __name__ == '__main__':
    try:
        GoToPoint()

        print("End of Main Function")

    except Exception as e:
        print(e)
        # rospy.loginfo("shutdown program.")
