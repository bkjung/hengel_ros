#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
import cv2
from cv_bridge import CvBridge, CvBridgeError

class Undistort():
    def __init__(self):
        rospy.init_node('undistortion', anonymous=True)

        rospy.Subscriber('/wide_cam_1/image_raw/compressed', CompressedImage, self.callback_undistort1)
        rospy.Subscriber('/wide_cam_2/image_raw/compressed', CompressedImage, self.callback_undistort2)
        rospy.Subscriber('/wide_cam_3/image_raw/compressed', CompressedImage, self.callback_undistort3)
        rospy.Subscriber('/wide_cam_4/image_raw/compressed', CompressedImage, self.callback_undistort4)
        rospy.spin()
        # self.cameraInfoSubscriber=rospy.Subscriber('/usb_cam/camera_info',CameraInfo, self.callback_info)
    #    self.undistImgPublisher=rospy.Publisher('/undist_img/image_raw',Image, queue_size=3)
        self.rate=rospy.Rate(10)

    def callback_undistort1(self,_img):
        print("callback")
        try:
            bridge=CvBridge()
            rawimg=bridge.compressed_imgmsg_to_cv2(_img, "bgr8")
        except CvBridgeError as e:
            print(e)
        print("DEBUG-1")
        mtx=[[329.446963, 0, 315.383397],[0, 328.784203, 239.504433], [0,0,1]]
        mtx=np.array(mtx)
        dst=[-0.282543, 0.0585483, 0.00022407, -0.00044064]
        dst=np.array(dst)
        if len(mtx)!=0 and len(dst)!=0:
            if rawimg is not None:
                print("DEBUG-2")
                print(type(rawimg))
                undistImg=cv2.undistort(rawimg, mtx, dst, None, mtx)
                print(undistImg)
                # cv2.imshow('raw_img', rawimg)
                cv2.imshow('undist_img_1', undistImg)
                cv2.imwrite('undist_img_1_1.png', undistImg)
                cv2.waitKey(3)
            else:
                print("Image1 is None")

    def callback_undistort2(self,_img):
        try:
            bridge=CvBridge()
            rawimg=bridge.compressed_imgmsg_to_cv2(_img, "bgr8")
        except CvBridgeError as e:
            print(e)
        mtx=[[334.573338, 0, 313.382623],[0, 335.061739, 243.157640], [0,0,1]]
        mtx=np.array(mtx)
        dst=[-0.252637, 0.042737, -0.000955, -0.002136]
        dst=np.array(dst)
        if len(mtx)!=0 and len(dst)!=0:
            if rawimg is not None:
                undistImg=cv2.undistort(rawimg, mtx, dst, None, mtx)
                # cv2.imshow('raw_img', rawimg)
                # cv2.imshow('undist_img_2', undistImg)
                cv2.waitKey(3)
            else:
                print("Image2 is None")

    def callback_undistort3(self,_img):
        try:
            bridge=CvBridge()
            rawimg=bridge.compressed_imgmsg_to_cv2(_img, "bgr8")
        except CvBridgeError as e:
            print(e)
        mtx=[[329.799706, 0, 317.902481], [0, 328.339702, 232.650023], [0,0,1]]
        mtx=np.array(mtx)
        dst=[-0.276948, 0.054472, -0.001206, -0.000234]
        dst=np.array(dst)
        if len(mtx)!=0 and len(dst)!=0:
            if rawimg is not None:
                undistImg=cv2.undistort(rawimg, mtx, dst, None, mtx)
                # cv2.imshow('raw_img', rawimg)
                # cv2.imshow('undist_img_3', undistImg)
                cv2.waitKey(3)
            else:
                print("Image3 is None")

    def callback_undistort4(self,_img):
        try:
            bridge=CvBridge()
            rawimg=bridge.compressed_imgmsg_to_cv2(_img, "bgr8")
        except CvBridgeError as e:
            print(e)
        mtx=[[329.825798, 0, 312.223260],[0, 328.056018, 243.581446], [0,0,1]]
        mtx=np.array(mtx)
        dst=[-0.279630, 0.058004, -0.000491, -0.000337]
        dst=np.array(dst)
        if len(mtx)!=0 and len(dst)!=0:
            if rawimg is not None:
                undistImg=cv2.undistort(rawimg, mtx, dst, None, mtx)
                # cv2.imshow('raw_img', rawimg)
                # cv2.imshow('undist_img_4', undistImg)
                cv2.waitKey(3)
            else:
                print("Image4 is None")


if __name__=="__main__":
    Undistort()
    cv2.destroyAllWindows()

