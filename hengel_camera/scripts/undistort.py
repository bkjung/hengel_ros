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
        self.undistImg=np.ndarray([])

        self.bridge=CvBridge()
        self.rawImgSubscriber=rospy.Subscriber('/usb_cam/image_raw', Image, self.callback_undistort)
        self.cameraInfoSubscriber=rospy.Subscriber('/usb_cam/camera_info',CameraInfo, self.callback_info)
    #    self.undistImgPublisher=rospy.Publisher('/undist_img/image_raw',Image, queue_size=3)
        rate=rospy.Rate(10)

    def callback_info(self, _info):
        self.mtx=[]
        self.dst=np.array(_info.D)
        for i in [0,3,6]:
            self.mtx.append(list(_info.K)[i:i+3])
        self.mtx=np.array(self.mtx)



    def callback_undistort(self,_img):
        try:        
            self.rawimg=self.bridge.imgmsg_to_cv2(_img, "bgr8")
        except CvBridgeError as e:
            print(e)

        ############ 1st Trial ################################
        # self.mtx=[[814.518716, 0, 307.439908],[0,773.806237, 228.559015],[0,0,1]]
        # se    lf.mtx=np.array(self.mtx)
        # self.dst=[-1.277346, 1.545611, -0.022366, -0.002066]
        # self.dst=np.array(self.dst)
        ############ 2nd Trial (Current Version) ###############
        # self.mtx=[[331.350552, 0, 297.584603],[0,331.407182, 227.861193],[0,0,1]]
        # self.mtx=np.array(self.mtx)
        # self.dst=[-0.299195,0.074445,0.000095, -0.000103]
        # self.dst=np.array(self.dst)
        #######################################################

        if len(self.mtx)!=0 and len(self.dst)!=0 and self.rawimg is not None:
            self.undistImg=cv2.undistort(self.rawimg, self.mtx, self.dst, None, self.mtx)
            cv2.imshow('raw_img', self.rawimg)
            cv2.imshow('undist_img', self.undistImg)
            cv2.waitKey(3)


if __name__=="__main__":
    rospy.init_node('undistortion', anonymous=True)
    Undistort()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")
    cv2.destroyAllWindows()


