#!/usr/bin/env python

import rospy
import numpy as np
from sersor_msgs.msg import Image
import cv2

#class Undistort():
#    def __init__(self):
#        rospy.init_node('unidstortion', anonymous=True)
#        rawImgSubscriber=rospy.Subscriber('/wide_cam_1/image_raw', Image, self.callback_undistort)
#        rate=rospy.Rate(10)
#
#    def callback_undistort(self,_img):
#        bridge=CvBridge()
#        self.rawimg=bridge.imgmsg_to_cv2(_img, "rgb8")
#        mtx=[[814.518716, 0, 307.439908],[0,773.806237, 228.559015],[0,0,1]]
#        dst=[-1.277346, 1.545611, -0.022366, -0.002066]
#
#        self.undistImg=cv2.undistort(self.rawimg, mtx, dst, None, mtx)


def callback_undistort(_img):
    bridge_CvBridge()
    rawimg=bridge.imgmsg_to_cv2(_img, "rgb8")
    mtx=[[814.518716, 0, 307.439908],[0,773.806237, 228.559015],[0,0,1]]
    dst=[-1.277346, 1.545611, -0.022366, -0.002066]
    undistImg=cv2.undistort(rawimg, mtx, dst, None, mtx)
    cv2.imshow('undistort', undistImg)


if __name__=="__main__":
    rospy.init_node('undistortion', anonymous=True)
    rawImgSubscriber=rospy.Subscriber('/usb_cam/image_raw', Image, callback_undistort)
    rate=rospy.Rate(10)


