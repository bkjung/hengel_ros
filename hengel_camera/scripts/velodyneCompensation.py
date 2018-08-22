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
        # rospy.Subscriber('/blam/blam_slam/odometry_integrated_estimate', PoseStamped, self.callback)
        self.pub=rospy.Publisher('/offset_change', Point, queue_size=5)

        self.callback_midpoint=message_filters.Subscriber('/midpoint',Point)
        self.callback_velodyne=message_filters.Subscriber('/blam/blam_slam/odometry_integrated_estimate', PoseStamped)
        self.ts=message_filters.ApproximateTimeSynchronizer([self.callback_midpoint, self.callback_velodyne], 10, 0.1, allow_headerless=True)

        self.ts.registerCallback(self.sync_callback)

        self.endPoint_callback=message_filters.Subscriber('/endpoint', Point)
        self.midPoint_callback=message_filters.Subscriber('/midpoint', Point)
        self.midPoint_time_callback=message_filters.Subscriber('/midpoint_time', Time)

        self.ts_2=message_filters.ApproximateTimeSynchronizer([self.endPoint_callback, self.midPoint_callback, self.midPoint_time_callback], 10, 0.1, allow_headerless=True)
        self.ts_2.registerCallback(self.sync_virtual_callback)

        self.pub_virtual_map=rospy.Publisher('/virtual_map', CompressedImage, queue_size=3)

        self.initialize()

        rospy.spin()

    def initialize(self):
        self.bridge=CvBridge()
        self.pixMetRatio=250
        self.cropped_virtual_map=np.full((1280,1280),255).astype('uint8')
        self.virtual_map=np.full((int(self.pixMetRatio*self.height), int(self.pixMetRatio*self.width)), 255)
        self.app_robotview=RobotView(self.virtual_map) # Add the endpoint into the virtual map

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

    def sync_virtual_callback(self, _endPoint, _midPoint, _midPointTime):
        if not self.isNavigationStarted:
            self.isNavigationStarted = True
        # print("sync virtual")
        # _time=time.time()

        # if not self.isProcessingVirtualmapTime:
        # self.mid_predict_canvas_x.appendleft(_midPoint.x)
        # self.mid_predict_canvas_y.appendleft(_midPoint.y)
        # self.mid_predict_canvas_th.appendleft(_midPoint.z)
        # self.mid_predict_canvas_time.appendleft(_midPointTime.data.to_nsec())

        # self.recent_pts.appendleft((_midPoint.x, _midPoint.y))

        # self.virtual_map = self.app_robotview.run(_midPoint, _endPoint)
        self.app_robotview.run(_midPoint, _endPoint)
        self.virtual_map = self.app_robotview.img

        # ttime=Float32()
        # ttime.data=float(time.time()-_time)

        #PUBLISHING VIRTUAL MAP, but currently the msg cannot be viewed at rqt (supposedly because of msgtype mismatch)
        virtual_map_msg=self.bridge.cv2_to_compressed_imgmsg(self.virtual_map)
        self.pub_virtual_map.publish(virtual_map_msg)


if __name__=='__main__':
    VelodyneCompensation()
