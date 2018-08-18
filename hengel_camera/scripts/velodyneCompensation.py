#!/usr/bin/env python
import rospy
import sys
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Float32, Time, Header
from sensor_msgs.msg import Image, CompressedImage
from markRobotView import RobotView
from math import radians, copysign, sqrt, pow, pi, atan2, sin, floor, cos, asin,ceil
import numpy as np
import time
import os
import copy
from os.path import expanduser
import message_filters
import collections
from feature_match import FeatureMatch
from matplotlib import pyplot as plt
from hengel_camera.msg import CmpImg
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
from cv_bridge import CvBridge
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')

class VelodyneCompensation():
    def __init__(self):
        rospy.init_node('hengel_velodyne_compensation', anonymous=False)
        self.origin=rospy.wait_for_message('/midpoint', Point)
        rospy.Subscriber('/blam/blam_slam/odometry_integrated_estimate', PoseStamped, self.callback)
        self.pub=rospy.Publisher('/offset_change', Point, queue_size=5)

        self.callback_midpoint=message_filters.Subscriber('/midpoint',Point)
        self.callback_velodyne=message_filters.Subscriber('/blam/blam_slam/odometry_integrated_estimate', PoseStamped)
        self.ts=message_filters.ApproximateTimeSynchronizer([self.callback_midpoint, self.callback_velodyne], 10, 0.1, allow_headerless=True)

        self.ts.registerCallback(self.sync_callback)

    def sync_callback(self, _midpnt, _pose):
        print("origin: %d, %d, %d" %(self.origin.x, self.origin.y, self.origin.z))
        offset=Point()
        move_x =_pose.pose.position.x-self.origin.x
        move_y =_pose.pose.position.y-self.origin.y
        move_th =_pose.pose.orientation.z -self.origin.z

        print("velodyne: %d, %d, %d" %(_pose.pose.position.x, _pose.pose.position.y, _pose.pose.orientation))
        mid_x = _midpnt.x
        mid_y = _midpnt.y
        mid_th = _midpnt.z

        offset.x = (-move_x) - mid_x
        offset.y = move_y - mid_y
        offset.z = - move_th - midpnt.z

        self.pub.publish(offset)

if __name__=='__main__':
    VelodyneCompensation()