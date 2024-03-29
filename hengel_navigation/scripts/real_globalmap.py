#!/usr/bin/env python

import rospy
from Tkinter import *
import PIL.Image
import PIL.ImageTk
import time
import os
import sys
from time import sleep
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
import math
import numpy as np
import cv2
from cv_bridge import CvBridge
from matplotlib import pyplot as plt
from around_view import AroundImage
import logging


package_base_path = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "../.."))

class RealGlobalMap():
    def __init__(self, _arr_path):
        self.around_subscribed = False
        self.arr_path = _arr_path
        self.initialize()

        self.package_base_path = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "../.."))

        ############## DEBUG ###########################
        # self.image_debug()
        ################################################

        # self.MapPublisher()

    def initialize(self):
        #rospy.init_node('real_globalmap', anonymous=True)

        self.root = Tk()

        self.photo = []
        self.photo = np.ndarray(self.photo)

        self.last_letter_img = []
        self.last_letter_img = np.ndarray(self.last_letter_img)

        self.last2_letter_img = []
        self.last2_letter_img = np.ndarray(self.last2_letter_img)

        self.size_x = 1000
        self.size_y = 1000
        self.scale_factor =282/0.74  #[pixel/m]

        self.r = rospy.Rate(50)
        # self.aroundview_subscriber=rospy.Subscriber('/around_img', Image, self.callback_view)
        # print("initialized")
        self.aroundImage = AroundImage()

    def run(self, letter_number, _position):
        self.photo = self.aroundImage.takeAroundImage()
        self.x = _position[0]
        self.y = _position[1]
        self.th = _position[2]

        T = np.eye(2,3, dtype=np.float32)
        warp_matrix = np.eye(2, 3, dtype=np.float32)
        if letter_number > 0:
            rospy.loginfo("letter # >0, start transformation")
            # 1. Crop second last letter
            self.last2_letter_img = self.crop_letter(letter_number, 2)

            # 2. Calculate the offset
            sz = self.last2_letter_img.shape
            #########################
            number_of_iterations = 5000
            termination_eps = 1e-6
            #########################
            criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
                        number_of_iterations, termination_eps)
            try:
                #(cc, warp_matrix) = cv2.findTransformECC(self.last_letter_img, self.last2_letter_img, warp_matrix, cv2.MOTION_EUCLIDEAN, criteria)
                ##warp_matirix: virtual to actual transformation in image world
                #s_th=-warp_matrix[0][1]
                #c_th=warp_matrix[0][0]
                #tx=warp_matrix[0][2]
                #ty=warp_matrix[1][2]
                #print("warp_matrix", warp_matrix)

       		##frame center
		#center_x= (0.75+(letter_number-1)*0.85)
                #center_y=(0.75)

		##frame origin
		#fx= (0.75-0.44+(letter_number-1)*0.85)
                #fy=(0.75-0.61)


		#tx_real = (fx*(1-c_th)+s_th*fy+ty/self.scale_factor)*0.58
		#ty_real = (fy*(1-c_th)-s_th*fx+tx/self.scale_factor)*0.58

		###Calculate center in real world
                #center_m=[center_x+s_th*tx/(2*(1-c_th)*self.scale_factor), center_y-s_th*ty/(2*(1-c_th)*self.scale_factor)]
                #center_m[0]=center_m[0]
                #center_m[1]=center_m[1]

		##Compute warp_matrix in real world
		#T=[]
                #T.append([c_th, s_th, (-c_th*center_m[0]-s_th*center_m[1]+center_m[0])*0.58])
                #T.append([-s_th, c_th, (s_th*center_m[0]-c_th*center_m[1]+center_m[1])*0.58])
                #T.append([0,0,1])
		##T=[[c_th, -s_th, tx_real],[s_th, c_th, ty_real],[0,0,1]]
                ##T: virtual to actual transformation in real world
                #T= np.linalg.inv(T)


                #####################180607 MODIFY##########################
                #cv2.imwrite(package_base_path +"/hengel_navigation/viewpoint_img/cropped_2_" +time.strftime("%y%m%d_%H%M%S") + '.jpg',self.last2_letter_img)
                #cv2.imwrite(package_base_path + "/hengel_navigation/viewpoint_img/cropped_1_" +time.strftime("%y%m%d_%H%M%S") + '.jpg',self.last_letter_img)
                cv2.imwrite("/home/hengel/ecc/0611/cropped_2_" +time.strftime("%y%m%d_%H%M%S") + '.jpg',self.last2_letter_img)
                cv2.imwrite("/home/hengel/ecc/0611/cropped_1_" +time.strftime("%y%m%d_%H%M%S") + '.jpg',self.last_letter_img)
                (cc, self.warp_matrix) = cv2.findTransformECC(self.last_letter_img, self.last2_letter_img, warp_matrix, cv2.MOTION_EUCLIDEAN, criteria)
                #warp_matirix: virtual to actual transformation in image world
                s_th=-warp_matrix[0][1]
                c_th=warp_matrix[0][0]
                tx=warp_matrix[0][2]
                ty=warp_matrix[1][2]
                print("warp_matrix", warp_matrix)

		#frame origin
		fx=0.75-0.44+(letter_number-1)*0.85
                fy=0.75-0.61
                print("fx, fy:", fx, fy)

                fx2=fx+ty/self.scale_factor
                fy2=fy+tx/self.scale_factor
                print("fx2, fy2:", fx2, fy2)

		tx_real =-c_th*fx-s_th*fy+fx2
		ty_real =s_th*fx-c_th*fy+fy2
                print("tx_real, ty_real:", tx_real, ty_real)

                tx_real=tx_real*0.58
                ty_real=ty_real*0.58

		#Compute warp_matrix in real world
		T=[[c_th, s_th, tx_real],[-s_th, c_th, ty_real]]
                #T: virtual to actual transformation in real world
            except:
                print("ECC transform error")

        # 3. Update last letter
        #while not self.around_subscribed:
        #    self.wait_for_aroundimg()
        self.last_letter_img = self.crop_letter(letter_number, 1)
        sz=self.last_letter_img.shape
        if letter_number >0:
            T=np.array(T)
            self.last_letter_img = self.warpImage(self.last_letter_img, T)
            #self.last_letter_img = cv2.warpAffinedddd(self.last_letter_img, T, (sz[1], sz[0]), flags=cv2.INTER_LINEAR+cv2.WARP_INVERSE_MAP)
        print("T:", T)
        return T

    def crop_letter(self, letter_number, ind):
        ###################
        x_padding = 10
        y_padding = 10
        ###################
	if ind==1:
            crop_img = self.photo[500:852,108:596]
	else:
            crop_img = self.photo[126:478,108:596]
        ################ FOR DEBUGGING #####################
        # threshold_img1 = cv2.threshold(crop_img, 50, 255, cv2.THRESH_BINARY)
        # threshold_img2 = cv2.threshold(crop_img, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY,11,2)
        # threshold_img3 = cv2.threshold(crop_img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11,2)

        # titles = ['Original Image', 'Global Thresholding(v=50)', 'Adaptive Mean Thresholding', 'Adaptive Gaussian Thresholding']
        # imgs=[crop_img, threshold_img1, threshold_img2, threshold_img3]

        # for i in xrange(4):
        #     plt.subplot(2,2,i+1), plt.imshow(imgs[i], 'gray')
        #     plt.title(titles[i])
        #     plt.xticks([]), plt.yticks([])
        # plt.show()
        #####################################################

        return cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)

    def warpImage(self, image, warp_mat):
        rows, cols = image.shape[:2]
        diagonal = (int)(math.sqrt(rows * rows + cols * cols))

        offsetX = int((diagonal - cols) / 2)
        offsetY = int((diagonal - rows) / 2)

        background = np.zeros((diagonal, diagonal), np.uint8)
        for i in range(diagonal):
            for j in range(diagonal):
                if i in range(offsetX, rows + offsetX) and j in range(
                        offsetY, offsetY + cols):
                    background[i][j] += image[i - offsetX][j - offsetY]
                else:
                    background[i][j] = 255

        rot_image = cv2.warpAffine(
            background, warp_mat, (diagonal, diagonal), borderValue=(256, 256, 256),
            flags=cv2.INTER_LINEAR)
        return rot_image[offsetX:(rows+offsetX), offsetY:(cols+offsetY)]

    def MapPublisher(self):
        x_px = self.scale_factor * self.x
        y_px = self.scale_factor * self.y

        #Rotate image (size: diagonal * diagonal)
        rows, cols = self.photo.shape[:2]
        diagonal = (int)(math.sqrt(rows * rows + cols * cols))

        offsetX = int((diagonal - cols) / 2)
        offsetY = int((diagonal - rows) / 2)

        background = np.zeros((diagonal, diagonal, 3), np.uint8)
        for i in range(diagonal):
            for j in range(diagonal):
                if i in range(offsetX, rows + offsetX) and j in range(
                        offsetY, offsetY + cols):
                    background[i][j] += self.photo[i - offsetX][j - offsetY]
                else:
                    background[i][j] = [255, 255, 255]

        R = cv2.getRotationMatrix2D((diagonal / 2, diagonal / 2),
                                    np.rad2deg(self.th) + 180, 1)

        rot_photo = cv2.warpAffine(
            background, R, (diagonal, diagonal), borderValue=(256, 256, 256))
        photo_PIL = PIL.Image.fromarray(rot_photo)

        cv2.waitKey(10000)
        new_Image = PIL.Image.new("RGB", (self.size_x, self.size_y),
                                  (256, 256, 256))
        new_Image.paste(photo_PIL, (x_px - diagonal / 2, y_px - diagonal / 2))
        # new_Image.show()

    ################## DEBUG #########################
    # def image_debug(self):
    #     self.photo=cv2.imread(self.package_base_path+"/hengel_visual_odometry/scripts/abc.jpg", cv2.IMREAD_COLOR)
    #     # self.photo=cv2.cvtColor(self.photo, cv2.COLOR_RGB2BGR)
    #     cv2.imshow("minion", self.photo)
    #     self.photo=cv2.resize(self.photo, (200,200))
    ##################################################

    def callback_view(self, _img):
        self.around_subscribed = True
        bridge = CvBridge()
        self.photo = bridge.imgmsg_to_cv2(_img, "rgb8")
        # self.photo=cv2.cvtColor(self.photo, cv2.COLOR_RGB2BGR)
    def wait_for_aroundimg(self):
        loop_cnt = 0
        if loop_cnt == 500:
            return
        else:
            loop_cnt = loop_cnt + 1
            self.r.sleep()


# if __name__ == "__main__":
#     try:
#         RealGlobalMap()
#     except Exception as e:
#         print(e)
#         rospy.loginfo("shutdown program")
