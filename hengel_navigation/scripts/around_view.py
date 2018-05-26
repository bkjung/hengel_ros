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
        self.cam_front = cv2.VideoCapture(1)
        self.cam_left = cv2.VideoCapture(0)
        self.cam_right = cv2.VideoCapture(2)
        # self.cam_front.set(cv2.CV_CAP_PROP_BUFFERSIZE, 1)
        self.cam_front.set(cv2.CAP_PROP_FRAME_WIDTH, 864)
        self.cam_front.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cam_front.set(cv2.CAP_PROP_FOURCC, int(0x47504A4D))     # COLLECT IMAGE IN MJPG FORM, SOLVE USB HUB BANDWIDTH ISSUE
        # self.cam_left.set(cv2.CV_CAP_PROP_BUFFERSIZE, 1)
        self.cam_left.set(cv2.CAP_PROP_FRAME_WIDTH, 864)
        self.cam_left.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cam_left.set(cv2.CAP_PROP_FOURCC, int(0x47504A4D))
        # self.cam_right.set(cv2.CV_CAP_PROP_BUFFERSIZE, 1)
        self.cam_right.set(cv2.CAP_PROP_FRAME_WIDTH, 864)
        self.cam_right.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cam_right.set(cv2.CAP_PROP_FOURCC, int(0x47504A4D))

        #Calculate homography
        objPts = np.zeros((3, 4, 2), dtype=np.float32)
        objPts[0] = [[450, 800], [950, 800], [950, 300], [450, 300]]
        objPts[1] = [[950, 800], [1180, 800], [1180, 300], [950, 300]]  #left_1
        objPts[2] = [[220, 800], [450, 800], [450, 300], [220, 300]]

        imgPts = np.zeros((3, 4, 2), dtype=np.float32)
        imgPts[0] = [[109.3, 306.3], [506, 307.7], [417.8, 116],
                     [200, 116.7]]  #front_1
        imgPts[1] = [[194.5, 399.5], [311.8, 317.5], [172.1, 226.3],
                     [74.1, 258.3]]  #left
        imgPts[2] = [[284.5, 275], [402.8, 354.2], [521.1, 212],
                     [425.1, 183.6]]

        for i in range(3):
            for j in range(4):
                objPts[i][j][0] += 200
                objPts[i][j][1] -= 100
            objPts[i] = np.array(objPts[i], np.float32)
            imgPts[i] = np.array(imgPts[i], np.float32)

        self.homography_front = cv2.getPerspectiveTransform(
            imgPts[0], objPts[0])
        self.homography_left = cv2.getPerspectiveTransform(
            imgPts[1], objPts[1])
        self.homography_right = cv2.getPerspectiveTransform(
            imgPts[2], objPts[2])

    def warp_image(self, image, homography):
        im_out = cv2.warpPerspective(image, homography, (1800, 1300))
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
                self.cam_front.grab()
                self.cam_left.grab()
                self.cam_right.grab()
            _, front_img = self.cam_front.read()
            _, left_img = self.cam_left.read()
            _, right_img = self.cam_right.read()

            # cv2.imshow("front", front_img)
            # cv2.imshow("left", left_img)
            # cv2.imshow("right", right_img)

            h, w = front_img.shape[:2]
            # optimalMat, roi = cv2.getOptimalNewCameraMatrix(intrin, dist, (w,h), 1, (w,h))
            # undist_front = cv2.undistort(front_img, intrin, dist, None, intrin)
            # undist_left= cv2.undistort(left_img,intrin, dist, None, intrin)

            init_time = time.time()
            im_front = self.warp_image(front_img,
                                       self.homography_front).astype('uint8')
            im_left = self.warp_image(left_img,
                                      self.homography_left).astype('uint8')
            im_right = self.warp_image(right_img,
                                       self.homography_right).astype('uint8')
            print("image warped")

            # MULTIPLY WARPED IMAGE, THEN ADD TO BLANK IMAGE
            im_mask_inv, im_mask = self.find_mask(im_front)
            front_masked = np.multiply(im_front, im_mask_inv).astype('uint8')
            left_masked = np.multiply(im_left, im_mask).astype('uint8')
            right_masked = np.multiply(im_right, im_mask).astype('uint8')
            summed_image = front_masked + left_masked + right_masked
            summed_image = cv2.resize(
                summed_image, (900, 650), interpolation=cv2.INTER_AREA)
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
