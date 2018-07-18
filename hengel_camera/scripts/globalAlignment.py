#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
import cv2
from cv_bridge import CvBridge, CvBridgeError
import copy

class globalAlignment():
    def __init__(self):
        rospy.init_node('globalAlignment', anonymous=True)
        print("DEBUG-0")
        rospy.Subscriber('/usb_cam1/undistort/compressed', CompressedImage, self.callbackImg1)
        rospy.Subscriber('/usb_cam2/undistort/compressed', CompressedImage, self.callbackImg2)
        rospy.Subscriber('/usb_cam3/undistort/compressed', CompressedImage, self.callbackImg3)
        rospy.Subscriber('/usb_cam4/undistort/compressed', CompressedImage, self.callbackImg4)

        print("DEBUG-1")
        self.Img1=np.ndarray([])
        self.Img2=np.ndarray([])
        self.Img3=np.ndarray([])
        self.Img4=np.ndarray([])
        print("DEBUG-2")
        rospy.spin()
        rate= rospy.Rate(10)

    def callbackImg1(self, _img):
        print("SUBSCRIBE-2")
        bridge=CvBridge()
        self.Img1=bridge.compressed_imgmsg_to_cv2(_img, "bgr8")
        img_copy=copy.deepcopy(cv2.cvtColor(self.Img1, cv2.COLOR_BGR2GRAY))
        for i in xrange(len(self.Img1)):
            for j in xrange(len(self.Img1[i])):
                if self.Img1[i][j][2]<100:
                    img_copy[i][j]=0
                else:
                    img_copy[i][j]=255

        cv2.imshow("callback1", img_copy)
        cv2.waitKey(3)

    def callbackImg2(self, _img):
        print("SUBSCRIBE-2")
        bridge=CvBridge()
        self.Img2=bridge.compressed_imgmsg_to_cv2(_img, "bgr8")
        img_copy=copy.deepcopy(cv2.cvtColor(self.Img2, cv2.COLOR_BGR2GRAY))
        for i in xrange(len(self.Img2)):
            for j in xrange(len(self.Img2[i])):
                if self.Img2[i][j][2]<100:
                    img_copy[i][j]=0
                else:
                    img_copy[i][j]=255

        cv2.imshow("callback1", img_copy)
        cv2.waitKey(3)

    def callbackImg3(self, _img):
        print("SUBSCRIBE-2")
        bridge=CvBridge()
        self.Img3=bridge.compressed_imgmsg_to_cv2(_img, "bgr8")
        img_copy=copy.deepcopy(cv2.cvtColor(self.Img3, cv2.COLOR_BGR2GRAY))
        for i in xrange(len(self.Img3)):
            for j in xrange(len(self.Img3[i])):
                if self.Img3[i][j][3]<100:
                    img_copy[i][j]=0
                else:
                    img_copy[i][j]=255

        cv2.imshow("callback1", img_copy)
        cv2.waitKey(3)

    def callbackImg4(self, _img):
        print("SUBSCRIBE-2")
        bridge=CvBridge()
        self.Img4=bridge.compressed_imgmsg_to_cv2(_img, "bgr8")
        img_copy=copy.deepcopy(cv2.cvtColor(self.Img4, cv2.COLOR_BGR2GRAY))
        for i in xrange(len(self.Img4)):
            for j in xrange(len(self.Img4[i])):
                if self.Img4[i][j][3]<100:
                    img_copy[i][j]=0
                else:
                    img_copy[i][j]=255

        cv2.imshow("callback1", img_copy)
        cv2.waitKey(3)
if __name__=='__main__':
    globalAlignment()
    cv2.destroyAllWindows()
