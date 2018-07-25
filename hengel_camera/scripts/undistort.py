#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
import cv2
from cv_bridge import CvBridge, CvBridgeError

class Undistort():
    def __init__(self):
        rospy.init_node('undistortion', anonymous=True)

        rospy.Subscriber('/genius1/compressed', CompressedImage, self.callback_undistort1)
        rospy.Subscriber('/genius2/compressed', CompressedImage, self.callback_undistort2)
        rospy.Subscriber('/genius3/compressed', CompressedImage, self.callback_undistort3)
        rospy.Subscriber('/genius4/compressed', CompressedImage, self.callback_undistort4)
        rospy.spin()
        # self.cameraInfoSubscriber=rospy.Subscriber('/usb_cam/camera_info',CameraInfo, self.callback_info)
    #    self.undistImgPublisher=rospy.Publisher('/undist_img/image_raw',Image, queue_size=3)
        self.rate=rospy.Rate(10)

    def callback_undistort1(self,_img):
        try:
            print("SUBSCRIBE-1")
            bridge=CvBridge()
            rawimg=bridge.compressed_imgmsg_to_cv2(_img, "bgr8")
            print(rawimg.shape)
        except CvBridgeError as e:
            print(e)
        mtx=[[392.457559, 0, 307.489055],[0, 393.146087, 314.555479], [0,0,1]]
        #mtx=[[329.446963, 0, 315.383397],[0, 328.784203, 239.504433], [0,0,1]]
        mtx=np.array(mtx)
        dst=[-0.005695, -0.017562, -0.000497, 0.001201]
        dst=np.array(dst)
        if len(mtx)!=0 and len(dst)!=0:
            if rawimg is not None:
                undistImg=cv2.undistort(rawimg, mtx, dst, None, mtx)
                # cv2.imshow('raw_img', rawimg)
                cv2.imshow('undist_img_1', undistImg)
                cv2.imwrite('undist_img_1_1.png', undistImg)
                cv2.waitKey(3)
            else:
                print("Image1 is None")

    def callback_undistort2(self,_img):
        try:
            print("SUBSCRIBE-2")
            bridge=CvBridge()
            rawimg=bridge.compressed_imgmsg_to_cv2(_img, "bgr8")
        except CvBridgeError as e:
            print(e)
        mtx=[[397.515439, 0, 427.319496], [0, 396.393346, 359.074317],[0,0,1]]
        mtx=np.array(mtx)
        dst=[0.008050, -0.019082, 0.002712, 0.009123]
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
            print("SUBSCRIBE-3")
            bridge=CvBridge()
            rawimg=bridge.compressed_imgmsg_to_cv2(_img, "bgr8")
        except CvBridgeError as e:
            print(e)
        mtx=[[389.940243, 0, 366.042362],[0, 389.110547, 376.957547],[0,0,1]]
        mtx=np.array(mtx)
        dst=[0.001656, -0.022658, 0.005813, -0.003150]
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
            print("SUBSCRIBE-4")
            bridge=CvBridge()
            rawimg=bridge.compressed_imgmsg_to_cv2(_img, "bgr8")
        except CvBridgeError as e:
            print(e)
        mtx=[[373.550865, 0, 385.121920],[0, 373.744498, 317.403774], [0,0,1]]
        mtx=np.array(mtx)
        dst=[-0.018599, -0.009035, 0.001095, -0.004048]
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

