#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

class Homography:
    def __init__(self):
        self.initialize()
        rospy.Subscriber("/usb_cam1/undistort/compressed", CompressedImage, self.callback)
        self.pub=rospy.Publisher("/usb_cam1/homography/compressed", CompressedImage, queue_size=1)
        rospy.spin()
    
    def initialize(self):
        rospy.init_node('homography', anonymous=True)
        self.bridge=CvBridge()

    def callback(self, _img):
        cv_img=self.bridge.compressed_imgmsg_to_cv2(_img)
        cv_img=self.homography(cv_img)
        imgmsg=self.bridge.cv2_to_compressed_imgmsg(cv_img)
        self.pub.publish(imgmsg)
        # cv2.imshow('homography', cv_img)
        cv2.waitKey(3)        


    def homography(self, _img):
        objPts=np.zeros((20,2), dtype=np.float32)
        objPts=[[80,480],[240,480],[400,480],[560,480],[720,480],[880,480],
        [1040,480],[240,640],[400,640],[560,640],[720,640],[880,640],[1040,640],[400,800],[560,800],[720,800],[880,800],[560,960],[720,960]]
        objPts=np.array(objPts, np.float32)

        imgPts=np.zeros((20,2), dtype=np.float32)
        imgPts=[[13,368],[202.8,351.2],[384.8,339.8],[566.3,323.6],[749.5,307.5],
        [933,289],[1112,274],[61.4,436.6],[307,418.5],[545.5,397.5],[787.1,376.6],[1025,354],[1263,335],[157.5,570.5],[504.7,540],[855.8,501.9],[1199,481],[389.6,912.9],[1034, 865]]
        imgPts=np.array(imgPts, np.float32)

        homography, status=cv2.findHomography(imgPts, objPts)
        
        return cv2.warpPerspective(_img, homography, (1280, 960))



if __name__=="__main__":
    Homography()
