#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
import os
'''
CAMERA NODE RUNNING AT FULL SPEED
NO IMAGE RECORDING
'''

package_base_path = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "../.."))
os.system("mkdir -p " + package_base_path + "/hengel_navigation/viewpoint_img")


class AroundImage:
    def __init__(self):
        self.Initialize()

    def Initialize(self):
        self.cam_bottom = cv2.VideoCapture(0)
        self.cam_middle = cv2.VideoCapture(2)
        self.cam_top = cv2.VideoCapture(1)
        # self.cam_bottom.set(cv2.CV_CAP_PROP_BUFFERSIZE, 1)
        self.cam_bottom.set(cv2.CAP_PROP_FRAME_WIDTH, 864)
        self.cam_bottom.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cam_bottom.set(cv2.CAP_PROP_FOURCC, int(0x47504A4D))     # COLLECT IMAGE IN MJPG FORM, SOLVE USB HUB BANDWIDTH ISSUE
        # self.cam_middle.set(cv2.CV_CAP_PROP_BUFFERSIZE, 1)
        self.cam_middle.set(cv2.CAP_PROP_FRAME_WIDTH, 864)
        self.cam_middle.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cam_middle.set(cv2.CAP_PROP_FOURCC, int(0x47504A4D))
        # self.cam_top.set(cv2.CV_CAP_PROP_BUFFERSIZE, 1)
        self.cam_top.set(cv2.CAP_PROP_FRAME_WIDTH, 864)
        self.cam_top.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cam_top.set(cv2.CAP_PROP_FOURCC, int(0x47504A4D))

        #Calculate homography
        objPts = np.zeros((3, 4, 2), dtype=np.float32)
        objPts[0] = [[300,1800],[700,1800],[700,1400],[300,1400]] #bottom
        objPts[1] = [[300,1400],[700,1400],[700, 1000],[300,1000]]  #middle_1
        objPts[2] = [[300,1000],[700,1000],[700,600],[300,600]] #top

        imgPts = np.zeros((3, 4, 2), dtype=np.float32)
        imgPts[0] = [[37,462],[581,444],[452,97],[150,100]]  #bottom_1
        imgPts[1] = [[172,352],[417,349],[369,238],[219,239]]  #middle
        imgPts[2] = [[228,402],[390,402],[363,333],[255,333]] #top

        for i in range(3):
            for j in range(4):
                objPts[i][j][0] += 0
                objPts[i][j][1] -= 500
            objPts[i] = np.array(objPts[i], np.float32)
            imgPts[i] = np.array(imgPts[i], np.float32)

        self.homography_bottom = cv2.getPerspectiveTransform(
            imgPts[0], objPts[0])
        self.homography_middle = cv2.getPerspectiveTransform(
            imgPts[1], objPts[1])
        self.homography_top = cv2.getPerspectiveTransform(
            imgPts[2], objPts[2])

    def warp_image(self, image, homography):
        im_out = cv2.warpPerspective(image, homography, (1000,1400))
        return im_out

    # EFFICIENT TRUE/FALSE MASKING - NUMPY MASKING
    # ALLOWS SIMPLE ADDITION OF PIXELS FOR IMAGE STITCHING
    def find_mask(self, image):
        black_range1 = np.array([0, 0, 0])
        im_mask = (cv2.inRange(image, black_range1,
                               black_range1)).astype('bool')
        im_mask_inv = (1 - im_mask).astype('bool')
        im_mask_inv = np.dstack((im_mask_inv, im_mask_inv, im_mask_inv))
        im_mask = np.dstack((im_mask, im_mask, im_mask))
        return im_mask_inv, im_mask

    def takeAroundImage(self):
        try:
            print("take image func started")
            print("time:" + time.strftime("%y%m%d_%H%M%S"))
            bridge = CvBridge()
            for i in xrange(4):
                self.cam_bottom.grab()
                self.cam_middle.grab()
                self.cam_top.grab()
            _, bottom_img = self.cam_bottom.read()
            _, middle_img = self.cam_middle.read()
            _, top_img = self.cam_top.read()

            # cv2.imshow("bottom", bottom_img)
            # cv2.imshow("middle", middle_img)
            # cv2.imshow("top", top_img)

            h, w = bottom_img.shape[:2]
            # optimalMat, roi = cv2.getOptimalNewCameraMatrix(intrin, dist, (w,h), 1, (w,h))
            # undist_bottom = cv2.undistort(bottom_img, intrin, dist, None, intrin)
            # undist_middle= cv2.undistort(middle_img,intrin, dist, None, intrin)

            init_time = time.time()
            im_bottom = self.warp_image(bottom_img,
                                       self.homography_bottom).astype('uint8')
            im_middle = self.warp_image(middle_img,
                                      self.homography_middle).astype('uint8')
            im_top = self.warp_image(top_img,
                                       self.homography_top).astype('uint8')
            print("image warped")

            # MULTIPLY WARPED IMAGE, THEN ADD TO BLANK IMAGE
            im_mask_inv, im_mask = self.find_mask(im_middle)
            bottom_masked = np.multiply(im_bottom, im_mask).astype('uint8')
            middle_masked = np.multiply(im_middle, im_mask_inv).astype('uint8')
            top_masked = np.multiply(im_top, im_mask).astype('uint8')
            summed_image = bottom_masked + middle_masked + top_masked
            summed_image = cv2.resize(
                summed_image, (500, 700), interpolation=cv2.INTER_AREA)
            print("summed image made")
            cv2.imwrite(
                package_base_path +
                "/hengel_navigation/viewpoint_img/warped_image_" +
                time.strftime("%y%m%d_%H%M%S") + '.jpg', summed_image)
            return summed_image

            # SEND IMAGE AS ROS imgmsg
            # summed_image_msg = bridge.cv2_to_imgmsg(summed_image, "bgr8")
            # warp_pub.publish(summed_image_msg)

            # RETURN IMAGE
            # return summed_image
        except KeyboardInterrupt:
            rospy.signal_shutdown("keyboard interrupt")
