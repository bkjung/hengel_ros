#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
import cv2
from cv_bridge import CvBridge

class Undistort():
    def __init__(self):
        self.dst=np.array([])
        self.mtx=np.array([])
        self.rawimg=np.ndarray([])

        self.bridge=CvBridge()
        self.rawImgSubscriber=rospy.Subscriber('/wide_cam_1/image_raw', Image, self.callback_undistort1)
        self.rawImgSubscriber=rospy.Subscriber('/wide_cam_2/image_raw', Image, self.callback_undistort2)
        self.rawImgSubscriber=rospy.Subscriber('/wide_cam_3/image_raw', Image, self.callback_undistort3)
        self.rawImgSubscriber=rospy.Subscriber('/wide_cam_4/image_raw', Image, self.callback_undistort4)
        
        # self.cameraInfoSubscriber=rospy.Subscriber('/usb_cam/camera_info',CameraInfo, self.callback_info)
    #    self.undistImgPublisher=rospy.Publisher('/undist_img/image_raw',Image, queue_size=3)
        self.rate=rospy.Rate(10)

        self.img_1=np.ndarray([])
        self.img_2=np.ndarray([])
        self.img_3=np.ndarray([])
        self.img_4=np.ndarray([])

    def callback_info(self, _info):
        self.mtx=[]
        self.dst=np.array(_info.D)
        for i in [0,3,6]:
            self.mtx.append(list(_info.K)[i:i+3])
        self.mtx=np.array(self.mtx)



    def callback_undistort1(self,_img):
        try:
            self.rawimg=self.bridge.imgmsg_to_cv2(_img, "bgr8")
        except CvBridgeError as e:
            print(e)

        ############ 1st Trial ################################
        # self.mtx=[[814.518716, 0, 307.439908],[0,773.806237, 228.559015],[0,0,1]]
        # self.mtx=np.array(self.mtx)
        # self.dst=[-1.277346, 1.545611, -0.022366, -0.002066]
        # self.dst=np.array(self.dst)
        ############ 2nd Trial ################################
        # self.mtx=[[331.350552, 0, 297.584603],[0,331.407182, 227.861193],[0,0,1]]
        # self.mtx=np.array(self.mtx)
        # self.dst=[-0.299195,0.074445,0.000095, -0.000103]
        # self.dst=np.array(self.dst)
        ############# Current Version ##########################
        self.mtx=[[334.573338, 0, 313.382623],[0, 335.061739, 243.157640], [0,0,1]]
        self.mtx=np.array(self.mtx)
        self.dst=[-0.252637, 0.042737, -0.000955, -0.002136]
        self.dst=np.array(self.dst)
        ########################################################
        if len(self.mtx)!=0 and len(self.dst)!=0 and self.rawimg is not None:
            self.img_1=cv2.undistort(self.rawimg, self.mtx, self.dst, None, self.mtx)
            cv2.imshow('raw_img', self.rawimg)
            cv2.imshow('undist_img', self.img_1)
            cv2.waitKey(3)

    def callback_undistort2(self,_img):
        try:
            self.rawimg=self.bridge.imgmsg_to_cv2(_img, "bgr8")
        except CvBridgeError as e:
            print(e)
        ############# Current Version ##########################
        self.mtx=[[334.573338, 0, 313.382623],[0, 335.061739, 243.157640], [0,0,1]]
        self.mtx=np.array(self.mtx)
        self.dst=[-0.252637, 0.042737, -0.000955, -0.002136]
        self.dst=np.array(self.dst)
        ########################################################
        if len(self.mtx)!=0 and len(self.dst)!=0 and self.rawimg is not None:
            self.img_2=cv2.undistort(self.rawimg, self.mtx, self.dst, None, self.mtx)
            cv2.imshow('raw_img', self.rawimg)
            cv2.imshow('undist_img', self.img_2)
            cv2.waitKey(3)

    def callback_undistort3(self,_img):
        try:
            self.rawimg=self.bridge.imgmsg_to_cv2(_img, "bgr8")
        except CvBridgeError as e:
            print(e)
        ############# Current Version ##########################
        self.mtx=[[334.573338, 0, 313.382623],[0, 335.061739, 243.157640], [0,0,1]]
        self.mtx=np.array(self.mtx)
        self.dst=[-0.252637, 0.042737, -0.000955, -0.002136]
        self.dst=np.array(self.dst)
        ########################################################
        if len(self.mtx)!=0 and len(self.dst)!=0 and self.rawimg is not None:
            self.img_3=cv2.undistort(self.rawimg, self.mtx, self.dst, None, self.mtx)
            cv2.imshow('raw_img', self.rawimg)
            cv2.imshow('undist_img', self.img_3)
            cv2.waitKey(3)

    def callback_undistort4(self,_img):
        try:
            self.rawimg=self.bridge.imgmsg_to_cv2(_img, "bgr8")
        except CvBridgeError as e:
            print(e)
        ############# Current Version ##########################
        self.mtx=[[334.573338, 0, 313.382623],[0, 335.061739, 243.157640], [0,0,1]]
        self.mtx=np.array(self.mtx)
        self.dst=[-0.252637, 0.042737, -0.000955, -0.002136]
        self.dst=np.array(self.dst)
        ########################################################
        if len(self.mtx)!=0 and len(self.dst)!=0 and self.rawimg is not None:
            self.img_4=cv2.undistort(self.rawimg, self.mtx, self.dst, None, self.mtx)
            cv2.imshow('raw_img', self.rawimg)
            cv2.imshow('undist_img', self.img_4)
            cv2.waitKey(3)


if __name__=="__main__":
    rospy.init_node('undistortion', anonymous=True)
    Undistort()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")
    cv2.destroyAllWindows()


