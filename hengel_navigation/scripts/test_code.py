#!/usr/bin/env python

# import rospy
from geometry_msgs.msg import Twist, Point, Quaternion, PoseStamped
from std_msgs.msg import Float32
import tf
from math import radians, copysign, sqrt, pow, pi, atan2, sin, floor, cos
from tf.transformations import euler_from_quaternion
import sys
import time
import os
from navigation_control import NavigationControl



package_base_path = os.path.abspath(os.path.join(os.path.dirname(__file__),"../.."))
os.system("mkdir -p "+package_base_path+"/hengel_path_manager/output_pathmap")
os.system("mkdir -p "+package_base_path+"/hengel_path_manager/waypnts")


class GoToPoint():
    def __init__(self):
        self.path_to_point=[]
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
        self.path_to_point.append([0.0, 0.0])
        self.path_to_point.append([(float)(self.x), (float)(self.y)])
        self.arr_path.append(self.path_to_point)

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
