#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class Undistort():
    def __init__(self):
        self.rawimg=[]
        self.rawimg=np.ndarray(self.rawimg)
        self.undistImg=[]
        self.undistImg=np.ndarray(self.undistImg)
        self.bridge=CvBridge()
        self.rawImgSubscriber=rospy.Subscriber('/usb_cam/image_raw', Image, self.callback_undistort)
    #    self.undistImgPublisher=rospy.Publisher('/undist_img/image_raw',Image, queue_size=3)
        rate=rospy.Rate(10)
        # try:
        #     while True:
        #         if rospy.is_shutdown():
        #             break
        #         if not self.undistImg is None:
        #             # cv2.imshow('undist', self.undistImg)
        #             cv2.imshow('raw image', self.rawimg)
        #         rate.sleep()
        # except KeyboardInterrupt:
        #     rospy.signal_shutdown("Rospy Shutdown")
        # rospy.signal_shutdown("Keyboard Interrupt")


    def callback_undistort(self,_img):
        try:
        
            self.rawimg=self.bridge.imgmsg_to_cv2(_img, "bgr8")
        except CvBridgeError as e:
            print(e)
        mtx=[[814.518716, 0, 307.439908],[0,773.806237, 228.559015],[0,0,1]]
        mtx=np.array(mtx)
        dst=[-1.277346, 1.545611, -0.022366, -0.002066]
        dst=np.array(dst)

        self.undistImg=cv2.undistort(self.rawimg, mtx, dst, None, mtx)

        cv2.imshow('cv_img', self.undistImg)
        cv2.waitKey(3)


if __name__=="__main__":
    rospy.init_node('undistortion', anonymous=True)
    Undistort()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")
    cv2.destoryAllWindows()


