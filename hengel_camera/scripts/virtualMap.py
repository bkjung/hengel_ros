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
import collections
from feature_match import FeatureMatch

class VisualCompensation():
    def __init__(self, _num_pts_delete):
        word= raw_input("WHAT IS THE WIDTH AND HEIGHT OF CANVAS?\n Type: ")
        self.width=float(word.split()[0])
        self.height=float(word.split()[1])
        
        self.num_pts_delete = _num_pts_delete
        self.recent_pts = collections.deque(self.num_pts_delete*[(0.0,0.0)],_num_pts_delete)



        self.initialize()

    def initialize(self):
        rospy.init_node('hengel_camera_compensation', anonymous=False)

        self.bridge=CvBridge()
        self.pixMetRatio=500


        self.img=np.full((int(self.pixMetRatio*self.height), int(self.pixMetRatio*self.width)), 255)
        self.app_robotview=RobotView(self.img) # Add the endpoint into the virtual map


        self.mid_predict_canvas_x=0
        self.mid_predict_canvas_y=0
        self.mid_predict_canvas_th=0

        self.endPoint_callback=message_filters.Subscriber('/endpoint', Point)
        self.midPoint_callback=message_filters.Subscriber('/midpoint', Point)


        self.ts=message_filters.ApproximateTimeSynchronizer([self.endPoint_callback, self.midPoint_callback], 10, 0.1, allow_headerless=True)
        self.ts.registerCallback(self.sync_virtual_callback)

        self.pub_virtual_map=rospy.Publisher('/virtual_map', CompressedImage, queue_size=3)
        self.vision_offset_publisher = rospy.Publisher('/offset_change', Point, queue_size=10)
        self.callback1=message_filters.Subscriber('/genius1/compressed', CompressedImage)
        self.callback2=message_filters.Subscriber('/genius2/compressed', CompressedImage)
        self.callback3=message_filters.Subscriber('/genius3/compressed', CompressedImage)
        self.callback4=message_filters.Subscriber('/genius4/compressed', CompressedImage)
        # self.callback_pi_left=message_filters.Subscriber('/pi_cam_left/compressed', CompressedImage)
        # self.callback_pi_right=message_filters.Subscriber('/pi_cam_right/compressed', CompressedImage)

        # self.ts=message_filters.ApproximateTimeSynchronizer([self.callback1, self.callback2, self.callback3, self.callback4,
        #                                      self.callback_pi_left, self.callback_pi_right ], 10, 0.1, allow_headerless=True)

        self.ts=message_filters.ApproximateTimeSynchronizer([self.callback1, self.callback2, self.callback3, self.callback4], 10, 0.1, allow_headerless=True)
        self.ts.registerCallback(self.sync_real_callback)


        ############################ DEBUG ################################
        self.pub_time_1=rospy.Publisher('/time1', Float32, queue_size=5)
        self.pub_time_2=rospy.Publisher('/time2', Float32, queue_size=5)


        rospy.spin()

    # def sync_real_callback(self, _img1, _img2, _img3, _img4, _img_left, _img_right):
    def sync_real_callback(self, _img1, _img2, _img3, _img4):
        _time=time.time()
        print("sync real")
        img1 = self.undistort1(_img1)
        img2 = self.undistort2(_img2)
        img3 = self.undistort3(_img3)
        img4 = self.undistort4(_img4)
        # img_left=self.undistort_left(_img_left)
        # img_right=self.undistort_right(_img_right)

        im_mask_inv1, im_mask1=self.find_mask(img1)
        im_mask_inv3, im_mask3=self.find_mask(img3)
        _, im_mask2=self.find_mask(img2)
        _, im_mask4=self.find_mask(img4)

        img_white=np.full((1280, 1280,3), 255)

        im_mask13=cv2.bitwise_and(np.array(im_mask1), np.array(im_mask3))
        im_mask24=cv2.bitwise_and(np.array(im_mask2), np.array(im_mask4))
        im_mask1234=cv2.bitwise_and(im_mask13, im_mask24)

        img_white_masked=np.multiply(img_white, im_mask1234)
        img2_masked=np.multiply(img2, im_mask13)
        img4_masked=np.multiply(img4, im_mask13)
        summed_image= img1+img2_masked+img3+img4_masked+img_white_masked

        ttime=Float32()
        cv2.imwrite("/home/bkjung/summed.png", summed_image)
        print(str(time.time()-_time))
        ttime.data=float(time.time()-_time)
        self.pub_time_2.publish(ttime)


        # self.crop_image(summed_image)

        #################
        try:
            fm = FeatureMatch()
            fm.SIFT_FLANN_matching(np.dstack((self.img,self.img, self.img)), summed_image)
            if fm.status == True:
                self.vision_offset_publisher.publish(Point(fm.delta_x, fm.delta_y, fm.delta_theta))
                self.app_robotview.remove_points_during_vision_compensation(self.recent_pts)
                self.img = self.app_robotview.img

                #Initialize Queue
                self.recent_pts = collections.deque(self.num_pts_delete*[(0.0,0.0)],_num_pts_delete)
        except Exception as e:
            print(e)
            sys.exit("flann error")



        
        #################

        print("Visual Calculation Elapsed Time After Camera_Image Input: "+str(time.time()-_time))

        # bridge=CvBridge()
        # summed_msg=bridge.cv2_to_compressed_imgmsg(summed_image)
        # self.sum_pub.publish(summed_msg)

    def crop_image(self, _img):
        mid_predict_img_x = self.mid_predict_canvas_x * self.pixMetRatio
        mid_predict_img_y = _img.shape[0] - self.mid_predict_canvas_y * self.pixMetRatio
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
        _time=time.time()

        self.mid_predict_canvas_x=_midPoint.x
        self.mid_predict_canvas_y=_midPoint.y
        self.mid_predict_canvas_th=_midPoint.z

        self.recent_pts.appendleft((_midPoint.x, _midPoint.y))

        self.app_robotview.run(_midPoint, _endPoint)
        self.img = self.app_robotview.img
        ttime=Float32()
        ttime.data=float(time.time()-_time)
        self.pub_time_2.publish(ttime)

        # bridge=CvBridge()
        # virtual_map_msg=bridge.cv2_to_compressed_imgmsg(self.img)
        # self.pub_virtual_map.publish(virtual_map_msg)

    def undistort1(self, _img):
        img=self.bridge.compressed_imgmsg_to_cv2(_img)
        mtx=np.array([[390.6235286774559, 0.0, 380.6630283824523], [0.0, 391.05153994713896, 317.8133925380764], [0.0, 0.0, 1.0]])
        dst=np.array([0.006484855348017496, -0.019610851268822574, 0.0009123195786112891, -0.00424460183636266])
        # cv2.undistort(img, mt, dst, None, mtx)
        homo1= np.array([[-1.67130692e-01,  5.99743615e+00, -5.29293784e+02],
            [-2.40710331e+00,  4.76090267e+00,  1.55119117e+03],
            [-2.21043846e-04,  7.30990701e-03,  1.00000000e+00]])
        return cv2.warpPerspective( cv2.undistort(img, mtx, dst,None, mtx) , homo1, (1280,1280))


    def undistort2(self, _img):
        img=self.bridge.compressed_imgmsg_to_cv2(_img)
        mtx=np.array([[394.88382527431764, 0.0, 423.8584744100752], [0.0, 396.53524728115286, 359.4130870937376], [0.0, 0.0, 1.0]])
        dst=np.array([0.012819138964185638, -0.02474738869007743, 0.005214333503327388, -0.0028437261374924037])
        homo2= np.array([[-2.36547415e+00,  4.44589419e+00,  1.65240597e+03],
            [-1.11902669e-02,  2.88055561e+00,  2.03902843e+03],
            [-5.36747061e-06,  6.70728023e-03,  1.00000000e+00]])
        return cv2.warpPerspective( cv2.undistort(img, mtx, dst,None, mtx) , homo2, (1280,1280))

    def undistort3(self, _img):
        img=self.bridge.compressed_imgmsg_to_cv2(_img)
        mtx=np.array([[384.740566000322766, 0, 416.6703696819],[0, 386.64723334, 297.593178440],[0,0,1]])
        dst=np.array([-0.0048592546, -0.02278286, 0.00255238134, -0.002026589])
        homo3= np.array([[ 2.55132452e-01,  9.82372337e+00,  4.09600642e+03],
            [ 6.45201391e+00,  1.30885948e+01, -1.66201249e+03],
            [ 3.88669729e-04,  2.00259308e-02,  1.00000000e+00]])
        return cv2.warpPerspective( cv2.undistort(img, mtx, dst,None, mtx) , homo3, (1280,1280))


    def undistort4(self, _img):
        img=self.bridge.compressed_imgmsg_to_cv2(_img)
        mtx=np.array([[387.43952147965115, 0.0, 412.56131546398876], [0.0, 389.1761911600528, 259.39190229663814], [0.0, 0.0, 1.0]])
        dst=np.array([0.005292760390926921, -0.025832001932141472, 0.0005161396135159652, -0.00047231070184728226])
        homo4= np.array([[ 2.57420243e+00,  5.85803823e+00, -4.05003547e+02],
            [-1.15034759e-01,  7.22474987e+00, -7.29546146e+02],
            [-1.92621119e-04,  8.88963498e-03,  1.00000000e+00]])
        return cv2.warpPerspective( cv2.undistort(img, mtx, dst,None, mtx) , homo4, (1280,1280))

    def undistort_left(self, _img):
        img=self.bridge.compressed_imgmsg_to_cv2(_img)
        mtx=np.array([[496.88077412085187, 0.0, 486.19161191113693], [0.0, 497.77308359203073, 348.482250144119], [0.0, 0.0, 1.0]])
        dst=np.array([-0.27524035766660704, 0.055346669640229516, 0.002041430748143387, -0.0012188333190676689])

    def undistort_right(self, _img):
        img=self.bridge.compressed_imgmsg_to_cv2(_img)
        mtx=np.array([[494.0169295185964, 0.0, 483.6710483879246], [0.0, 495.87509303786857, 336.69262125267153], [0.0, 0.0, 1.0]])
        dst=np.array([-0.26693726936305806, 0.05239559897759021, 0.0024912074565555443, -0.0015904998174301696])
if __name__=='__main__':
    num_pts_delete = 150 #num_of_waypoints_to_delete_in_virtualmap_after_compensation
    VisualCompensation(num_pts_delete)
