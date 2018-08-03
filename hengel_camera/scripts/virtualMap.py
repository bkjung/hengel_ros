#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from sensor_msgs.msg import Image, CompressedImage
from markRobotView import RobotView
from math import radians, copysign, sqrt, pow, pi, atan2, sin, floor, cos, asin
import numpy as np
import sys
import time
import os
import cv2
from cv_bridge import CvBridge
import message_filters

class VisualCompensation():
    def __init__(self):
        word= raw_input("WHAT IS THE WIDTH AND HEIGHT OF CANVAS?\n Type: ")
        self.width=float(word.split()[0])
        self.height=float(word.split()[1])

        self.initialize()

    def initialize(self):
        rospy.init_node('hengel_camera_compensation', anonymous=False)

        self.bridge=CvBridge()
        self.pixMetRatio=500

        self.mid_predict_canvas_x=0
        self.mid_predict_canvas_y=0
        self.mid_predict_canvas_th=0

        self.img=np.full((int(self.pixMetRatio*self.height), int(self.pixMetRatio*self.width)), 255)
        self.endPoint_callback=message_filters.Subscriber('/endpoint', Point)
        self.midPoint_callback=message_filters.Subscriber('/midpoint', Point)


        self.ts=message_filters.ApproximateTimeSynchronizer([self.endPoint_callback, self.midPoint_callback], 10, 0.1, allow_headerless=True)
        self.ts.registerCallback(self.sync_virtual_callback)

        self.pub_virtual_map=rospy.Publisher('/virtual_map', CompressedImage, queue_size=3)

        self.callback1=message_filters.Subscriber('/genius1/compressed', CompressedImage)
        self.callback2=message_filters.Subscriber('/genius2/compressed', CompressedImage)
        self.callback3=message_filters.Subscriber('/genius3/compressed', CompressedImage)
        self.callback4=message_filters.Subscriber('/genius4/compressed', CompressedImage)

        self.ts=message_filters.ApproximateTimeSynchronizer([self.callback1, self.callback2, self.callback3, self.callback4], 10, 0.1, allow_headerless=True)
        self.ts.registerCallback(self.sync_real_callback)


        rospy.spin()

    def sync_real_callback(self, _img1, _img2, _img3, _img4):
        _time=time.time()
        bridge=CvBridge()
        img1 = self.callback_undistort1(_img1)
        img2 = self.callback_undistort2(_img2)
        img3 = self.callback_undistort3(_img3)
        img4 = self.callback_undistort4(_img4)

        im_mask_inv1, im_mask1=self.find_mask(img1)
        im_mask_inv3, im_mask3=self.find_mask(img3)
        _, im_mask2=self.find_mask(img2)
        _, im_mask4=self.find_mask(img4)

        print(im_mask1)
        img_white=np.full((1280, 1280,3), 255)

        im_mask13=cv2.bitwise_and(np.array(im_mask1), np.array(im_mask3))
        im_mask24=cv2.bitwise_and(np.array(im_mask2), np.array(im_mask4))
        im_mask1234=cv2.bitwise_and(im_mask13, im_mask24)

        img_white_masked=np.multiply(img_white, im_mask1234)
        img2_masked=np.multiply(img2, im_mask13)
        img4_masked=np.multiply(img4, im_mask13)
        summed_image= img1+img2_masked+img3+img4_masked+img_white_masked

        self.crop_image(summed_image)

        # bridge=CvBridge()
        # summed_msg=bridge.cv2_to_compressed_imgmsg(summed_image)
        # self.sum_pub.publish(summed_msg)

    def crop_image(self, _img):
        mid_predict_img_x = self.mid_predict_canvas_x * self.pixMetRatio
        mid_predict_img_y = _img.shape[0] self.mid_predict_canvas_y * self.pixMetRatio
        mid_predict_img_th = self.mid_predict_canvas_th
        half_map_size = 125

        imgPts=np.array([[0,0], [0, 1280], [1280, 1280], [1280,0]])
        obsPts=np.array([[mid_predict_img_x-half_map_size*sin(mid_predict_img_th), mid_predict_img_y-cos(mid_predict_img_th)],
                        [mid_predict_img_x+half_map_size]])

        


    def find_mask(self, img):
        _time=time.time()
        black_range1=np.array([0,0,0])
        # im_mask=(cv2.inRange(img, black_range1, black_range1)).astype('bool')
        im_mask=(cv2.inRange(img, black_range1, black_range1))
        im_mask=np.dstack((im_mask, im_mask, im_mask))
        # im_mask_inv=(1-im_mask).astype('bool')
        im_mask_inv=(1-im_mask)
        return im_mask_inv, im_mask

    def sync_virtual_callback(self, _endPoint, _midPoint):
        app=RobotView(self.img, _midPoint, _endPoint) # Add the endpoint into the virtual map
        self.mid_predict_canvas_x=_midPoint.x
        self.mid_predict_canvas_y=_midPoint.y
        self.mid_predict_canvas_th=_midPoint.z

        self.img = app.run()

        # bridge=CvBridge()
        # virtual_map_msg=bridge.cv2_to_compressed_imgmsg(self.img)
        # self.pub_virtual_map.publish(virtual_map_msg)

    def callback_undistort1(self, _img):
        img=self.bridge.compressed_imgmsg_to_cv2(_img)
        mtx=np.array([[392.457559, 0, 307.489055],[0, 393.146087, 314.555479], [0,0,1]])
        dst=np.array([-0.005695, -0.017562, -0.000497, 0.001201])
        homo1= np.array([[-1.67130692e-01,  5.99743615e+00, -5.29293784e+02],
            [-2.40710331e+00,  4.76090267e+00,  1.55119117e+03],
            [-2.21043846e-04,  7.30990701e-03,  1.00000000e+00]])
        return cv2.warpPerspective( cv2.undistort(img, mtx, dst,None, mtx) , homo1, (1280,1280))

    
    def callback_undistort2(self, _img):
        img=self.bridge.compressed_imgmsg_to_cv2(_img)
        mtx=np.array([[397.515439, 0, 427.319496], [0, 396.393346, 359.074317],[0,0,1]])
        dst=np.array([0.008050, -0.019082, 0.002712, 0.009123])
        homo2= np.array([[-2.36547415e+00,  4.44589419e+00,  1.65240597e+03],
            [-1.11902669e-02,  2.88055561e+00,  2.03902843e+03],
            [-5.36747061e-06,  6.70728023e-03,  1.00000000e+00]])
        return cv2.warpPerspective( cv2.undistort(img, mtx, dst,None, mtx) , homo2, (1280,1280))

    def callback_undistort3(self, _img):
        img=self.bridge.compressed_imgmsg_to_cv2(_img)
        mtx=np.array([[384.740566000322766, 0, 416.6703696819],[0, 386.64723334, 297.593178440],[0,0,1]])
        dst=np.array([-0.0048592546, -0.02278286, 0.00255238134, -0.002026589])
        homo3= np.array([[ 2.55132452e-01,  9.82372337e+00,  4.09600642e+03],
            [ 6.45201391e+00,  1.30885948e+01, -1.66201249e+03],
            [ 3.88669729e-04,  2.00259308e-02,  1.00000000e+00]])
        return cv2.warpPerspective( cv2.undistort(img, mtx, dst,None, mtx) , homo3, (1280,1280))


    def callback_undistort4(self, _img):
        img=self.bridge.compressed_imgmsg_to_cv2(_img)
        mtx=np.array([[387.43952147965115, 0.0, 412.56131546398876], [0.0, 389.1761911600528, 259.39190229663814], [0.0, 0.0, 1.0]])
        dst=np.array([0.005292760390926921, -0.025832001932141472, 0.0005161396135159652, -0.00047231070184728226])
        homo4= np.array([[ 2.57420243e+00,  5.85803823e+00, -4.05003547e+02],
            [-1.15034759e-01,  7.22474987e+00, -7.29546146e+02],
            [-1.92621119e-04,  8.88963498e-03,  1.00000000e+00]])
        return cv2.warpPerspective( cv2.undistort(img, mtx, dst,None, mtx) , homo4, (1280,1280))


if __name__=='__main__':
    VisualCompensation()
