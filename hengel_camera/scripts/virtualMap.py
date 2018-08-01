#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from sensor_msgs.msg import Image, CompressedImage
from hengel_camera.markRobotView import RobotView
from math import radians, copysign, sqrt, pow, pi, atan2, sin, floor, cos, asin
import numpy as np
import sys
import time
import os
import cv2
from cv_bridge import CvBridge
import message_filters

class VisualCompensation():
    def __init__(self):
        word= raw_input("WHAT IS THE WIDTH AND HEIGHT OF CANVAS?\n Type: ")
        self.width=float(word.split()[0])
        self.height=float(word.split()[1])

        self.initialize()
    def initialize(self):
        rospy.init_node('hengel_camera_compensation', anonymous=False)
        self.pixMetRatio=500
        self.img=np.full((int(self.pixMetRatio*self.height), int(self.pixMetRatio*self.width)), 255)
        self.endPoint_callback=message_filters.Subscriber('/endpoint', Point)
        self.midPoint_callback=message_filters.Subscriber('/midpoint', Point)

        self.ts=message_filters.ApproximateTimeSynchronizer([self.endPoint_callback, self.midPoint_callback], 10, 0.1, allow_headerless=True)
        self.ts.registerCallback(self.sync_callback)

        self.pub_virtual_map=rospy.Publisher('/virtual_map', CompressedImage, queue_size=3)
        rospy.spin()

    def sync_callback(self, _endPoint, _midPoint):
        app=RobotView(self.img, _midPoint, _endPoint)
        print("run")
        self.img = app.run()

        bridge=CvBridge()
        virtual_map_msg=bridge.cv2_to_compressed_imgmsg(self.img)
        self.pub_virtual_map.publish(virtual_map_msg)

if __name__=='__main__':
    VisualCompensation()
