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

size_x=1000
size_y=1000
scale_factor= 3 #[pixel/cm]

class RealGlobalMap():
    def __init__(self):
        self.root=Tk()

        self.photo=[]
        self.photo=np.ndarray(self.photo)

        self.x=0
        self.y=0
        self.th=0

        self.c=Canvas(self.root, height=size_x, width=size_y, bg="white")
        self.root.title("global map")
        _str=str(size_x)+"x"+str(size_y)
        self.root.geometry(_str)

        self.around_view_subscriber= rospy.Subscriber('/around_img', Image, self.callback_view)
        self.position_subscriber=rospy.Subscriber('/current_position', Point, self.callback_position)
        self.heading_subscriber = rospy.Subscriber('/current_heading', Float32, self.callback_heading)

        ############## DEBUG ###########################
        # self.photo=PIL.ImageTk.PhotoImage(_map)
        # self.c.create_image(0,0,anchor="nw", image=self.photo)
        self.image_debug()
        ################################################

        x_px= scale_factor*self.x
        y_px= scale_factor*self.y

        #Rotate image
        rows, cols = self.photo.shape[:2]
        diagonal = (int)(sqrt(rows*rows+cols*cols))

        offsetX = (diagonal - cols)/2
        offsetY = (diagonal - rows)/2

        background = np.zeros((diagonal, diagonal,3), np.uint8)
        # background = [i*256 for i in background]

        # background = [256,256,256]
        # background= np.ndarray(background)
        # background= [[background]*diagonal]*diagonal
        # for i in range(rows):
        #     for j in range(cols):
        #         background[offsetX+i][offsetY+j]+=self.photo[i][j]
        for i in range(diagonal):
            for j in range(diagonal):
                if i in range(offsetX,rows+offsetX) and j in range(offsetY,offsetY+cols):
                    background[i][j]+=self.photo[i-offsetX][j-offsetY]
                else:
                    background[i][j] = [255,255,255]
                    print("changed")
        R= cv2.getRotationMatrix2D((diagonal/2, diagonal/2), np.rad2deg(self.th)+180, 1)

        rot_photo = cv2.warpAffine(background, R, (diagonal, diagonal), borderValue=(256,256,256))
        photo_PIL=PIL.Image.fromarray(rot_photo)

        cv2.waitKey(10000)
        new_Image=PIL.Image.new("RGB", (size_x, size_y), (256,256,256))
        new_Image.paste(photo_PIL, (x_px, y_px))
        new_Image.show()

        rospy.init_node('real_globalmap', anonymous=True)

    ################## DEBUG #########################
    def image_debug(self):
        self.photo=cv2.imread("./abc.jpg", cv2.IMREAD_COLOR)
        self.photo=cv2.cvtColor(self.photo, cv2.COLOR_RGB2BGR)
        self.photo=cv2.resize(self.photo, (200,200))
    ##################################################
    def callback_view(self, _img):
        bridge=CvBridge()
        self.photo= bridge.imgmsg_to_cv2(_img, "rgb8")
        self.photo=cv2.cvtColor(self.photo, cv2.COLOR_RGB2BGR)

    def callback_position(self, _pnt):
        self.x=_pnt.x
        self.y=_pnt.y
    
    def callback_heading(self, _heading):
        self.th=_heading.data



if __name__ == "__main__":
    try:        
        RealGlobalMap()
    except Exception as e:
        print(e)
        rospy.loginfo("shutdown program")

