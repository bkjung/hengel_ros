#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

class Homography:
    def __init__(self, index, img):
        rospy.init_node('homography', anonymous=True)
        self.bridge=CvBridge()
        self.pub=rospy.Publisher("/usb_cam1/homography/compressed", CompressedImage, queue_size=1)
        rospy.spin()

    def callback1(self, _img):
        cv_img=self.bridge.compressed_imgmsg_to_cv2(_img)
        cv_img=self.homography3(cv_img)
        imgmsg=self.bridge.cv2_to_compressed_imgmsg(cv_img)
        self.pub.publish(imgmsg)
        # cv2.imshow('homography', cv_img)
        cv2.waitKey(3)

    def callback3(self, _img):
        cv_img=self.bridge.compressed_imgmsg_to_cv2(_img)
        cv_img=self.homography3(cv_img)
        imgmsg=self.bridge.cv2_to_compressed_imgmsg(cv_img)
        self.pub.publish(imgmsg)
        # cv2.imshow('homography', cv_img)
        cv2.waitKey(3)

    def homography1(self, _img):
        objPts=np.zeros((20,2), dtype=np.float32)
        objPts=[]
        objPts=[point_r[1]objPts = [point_r[1]*(-5.0)+640.0, point_r[2]*(-5.0)+640.0] for point_r in robotPts ]
        objPts=np.array(objPts, np.float32)

        imgPts=np.zeros((20,2), dtype=np.float32)
        imgPts=[]
        imgPts=np.array(imgPts, np.float32)

        homography, status=cv2.findHomography(imgPts, objPts)

        return cv2.warpPerspective(_img, homography, (1280, 960))

    def homography3(self, _img):
        objPts=np.zeros((23,2), dtype=np.float32)
        robotPts=[[15,-30], [5,-30],[-5,-30],[-15,-30],[-25,-30],[-35,-30],[-45,-30],
                [35,-40],[25,-40],[15,-40],[5,-40],[-5,-40],[-15, -40],[-25,-40],[-35,-40],[-45,-40],
                [25,-50],[15,-50],[5,-50],[-5,-50],[-45,-50]]
        objPts = [point_r[1]*(-5.0)+640.0, point_r[2]*(-5.0)+640.0] for point_r in robotPts ]
        objPts=np.array(objPts, np.float32)

        imgPts=np.zeros((23,2), dtype=np.float32)
        imgPts=[[199.8, 550.2], [297,549],[395.5,548],[495.7,548.1],[594.8,547.1],[692.9,545.1],[783.6,542.1],
            [62,464],[142.2,463.8],[225.2,463.2],[308.2,462],[392.5,461.2],[476.9,459.6],[562, 460],[647, 459],[729.8, 457.8],
            [171.2,399.2], [244,398],[316.5,398],[388.5, 345.7],[650.9,343.1]]
        imgPts=np.array(imgPts, np.float32)

        homography, status=cv2.findHomography(imgPts, objPts)

        return cv2.warpPerspective(_img, homography, (1280, 1280))

    def homography4(self, _img):
        objPts=np.zeros((23,2), dtype=np.float32)
        robotPts=[[20,25],[20,15],
                [25,20],[25,0],[25,-20],[25,-40],
                [45,20],[45,0],[45,20],[45,40],[45,60],
                [50,45]]
        objPts = [point_r[1]*(-5.0)+640.0, point_r[2]*(-5.0)+640.0] for point_r in robotPts ]
        objPts=np.array(objPts, np.float32)

        imgPts=np.zeros((23,2), dtype=np.float32)
        imgPts=[[96.2,522],[197,524],
                [166,471], [357.4,473.4],[549,475.6],[733.7, 474.7],
                [222,326],[366,327],[510,329],[652,330],[786,331],
                [65.6,299]]
        imgPts=np.array(imgPts, np.float32)

        homography, status=cv2.findHomography(imgPts, objPts)

        return cv2.warpPerspective(_img, homography, (1280, 1280))

