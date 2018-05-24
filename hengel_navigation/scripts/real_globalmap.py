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
from math import sqrt
import numpy as np
import cv2
from cv_bridge import CvBridge
from matplotlib import pyplot as plt
from around_view import AroundImage


class RealGlobalMap():
    def __init__(self, _arr_path):
        self.around_subscribed = False
        self.arr_path = _arr_path
        self.initialize()

        self.package_base_path = os.path.abspath(os.path.join(os.path.dirname(__file__),"../.."))

        ############## DEBUG ###########################
        # self.image_debug()
        ################################################

        # self.MapPublisher()

    def initialize(self):
        #rospy.init_node('real_globalmap', anonymous=True)

        self.root=Tk()

        self.photo=[]
        self.photo=np.ndarray(self.photo)

        self.last_letter_img=[]
        self.last_letter_img=np.ndarray(self.last_letter_img)

        self.last2_letter_img=[]
        self.last2_letter_img=np.ndarray(self.last2_letter_img)

        self.size_x=1000
        self.size_y=1000
        self.scale_factor= 5.5 #[pixel/cm]

        self.r=rospy.Rate(50)

        # self.aroundview_subscriber=rospy.Subscriber('/around_img', Image, self.callback_view)
        # print("initialized")
        self.aroundImage = AroundImage()

    def run(self, letter_number, _position):
        self.photo = self.aroundImage.takeAroundImage()

        self.x = _position[0]
        self.y = _position[1]
        self.th = _position[2]

        data=[0,0,0]
        if letter_number >1 :
            # 1. Crop second last letter
            self.last2_letter_img = self.crop_letter(letter_number, 2)

            # 2. Calculate the offset
            sz= self.last2_letter_img.shape
            warp_matrix= np.eye(3, 3, dtype=np.float32)
            #########################
            number_of_iterations = 5000
            termination_eps = 1e-6
            #########################
            criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, number_of_iterations,  termination_eps)
            try:
                (cc, warp_matrix) = cv2.findTransformECC (self.im2_gray,warp_matrix, warp_mode, criteria)
            except:
                print("ECC transform error")
                return data

            offset_th = math.atan(warp_matrix[0][0], -warp_matrix[0][1])

            #Calculate x, y offset
            xx = sz[0]/2-warp_matrix[0][2]
            yy = sz[1]/2-warp_matrix[1][2]
            x2= cos(offset_th)*xx - sin(offset_th)*yy
            y2= sin(offset_th)*xx + cos(offset_th)*yy

            offset_x = x2-sz[0]/2
            offset_y = y2-sz[1]/2

            #Change to the world coordinate
            data=[ offset_y/self.scale_factor, offset_x/self.scale_factor, offset_th/self.scale_factor]

        # 3. Update last letter
        while not self.around_subscribed:
            self.wait_for_aroundimg()
        self.last_letter_img = self.crop_letter(letter_number, 1)
        print(data)
        return data

    def crop_letter(self, letter_number, ind):
        ###################
        x_padding= 10
        y_padding = 10
        ###################
        last_letter_x = [i[0] for i in self.arr_path[letter_number-ind]]
        last_letter_y = [i[1] for i in self.arr_path[letter_number-ind]]

        x_min = min(last_letter_x)
        x_max = max(last_letter_x)
        y_min = min(last_letter_y)
        y_max = max(last_letter_y)

        crop_img= self.photo[int(max(y_min-y_padding,0)):int((y_max+y_padding)),int( max(x_min-x_padding,0)): int((x_max+x_padding))]

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

    def MapPublisher(self):
        x_px= self.scale_factor*self.x
        y_px= self.scale_factor*self.y

        #Rotate image (size: diagonal * diagonal)
        rows, cols = self.photo.shape[:2]
        diagonal = (int)(sqrt(rows*rows+cols*cols))

        offsetX = int((diagonal - cols)/2)
        offsetY = int((diagonal - rows)/2)

        background = np.zeros((diagonal, diagonal,3), np.uint8)
        for i in range(diagonal):
            for j in range(diagonal):
                if i in range(offsetX,rows+offsetX) and j in range(offsetY,offsetY+cols):
                    background[i][j]+=self.photo[i-offsetX][j-offsetY]
                else:
                    background[i][j] = [255,255,255]

        R= cv2.getRotationMatrix2D((diagonal/2, diagonal/2), np.rad2deg(self.th)+180, 1)

        rot_photo = cv2.warpAffine(background, R, (diagonal, diagonal), borderValue=(256,256,256))
        photo_PIL=PIL.Image.fromarray(rot_photo)

        cv2.waitKey(10000)
        new_Image=PIL.Image.new("RGB", (self.size_x, self.size_y), (256,256,256))
        new_Image.paste(photo_PIL, (x_px-diagonal/2, y_px-diagonal/2))
        # new_Image.show()

    ################## DEBUG #########################
    # def image_debug(self):
    #     self.photo=cv2.imread(self.package_base_path+"/hengel_visual_odometry/scripts/abc.jpg", cv2.IMREAD_COLOR)
    #     # self.photo=cv2.cvtColor(self.photo, cv2.COLOR_RGB2BGR)
    #     cv2.imshow("minion", self.photo)
    #     self.photo=cv2.resize(self.photo, (200,200))
    ##################################################

    def callback_view(self, _img):
        self.around_subscribed=True
        bridge=CvBridge()
        self.photo= bridge.imgmsg_to_cv2(_img, "rgb8")
        # self.photo=cv2.cvtColor(self.photo, cv2.COLOR_RGB2BGR)
    def wait_for_aroundimg(self):
        loop_cnt = 0
        if loop_cnt==500:
            return
        else:
            loop_cnt=loop_cnt+1
            self.r.sleep()

# if __name__ == "__main__":
#     try:
#         RealGlobalMap()
#     except Exception as e:
#         print(e)
#         rospy.loginfo("shutdown program")

