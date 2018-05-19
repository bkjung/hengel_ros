#!/usr/bin/env python

import rospy
from Tkinter import *
# from PIL import Image, ImageTk
import PIL.Image
import PIL.ImageTk
import os
import time
import sys
from time import sleep
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from math import cos, sin
import numpy as np
from cv_bridge import CvBridge
import cv2
import skimage.io as ski_io

size_x=1200
size_y=1200
robot_size= 15 #[cm]
scale_factor= 3 #[pixel/cm]

class PredictGlobalMap():
    def __init__(self):
        self.root=Tk()
        self.c=Canvas(self.root, height=size_x, width=size_y, bg= "white")
        self.root.title("global map")
        _str=str(size_x)+"x"+str(size_y)
        self.root.geometry(_str)

        # c.pack(expand=YES, fill=BOTH)
        # c.grid(row=1, column=0)
        # f=Frame(self.root)
        # f.grid(row=0, column=0, sticky="n")

        self.x=50
        self.y=50
        self.th=0

        self.photo=[]
        self.photo=np.ndarray(self.photo)

        # self.map_subscriber = rospy.Subscriber('/current_global_map', Image, callback_map)
        # self.position_subscriber = rospy.Subscriber('/current_position', Point, callback_position)
        # self.heading_subscriber = rospy.Subscriber('/current_heading', Float32, callback_heading)
        self.global_map_publisher = rospy.Publisher('/predict_global_map', Image, queue_size=10)
        self.robot_view_publisher = rospy.Publisher('/predict_robot_view', Image, queue_size=10)

        ############## DEBUG ###########################
        # self.photo=PIL.ImageTk.PhotoImage(_map)
        # self.c.create_image(0,0,anchor="nw", image=self.photo)
        self.image_debug()
        ################################################

        self.photo_height, self.photo_width=self.photo.shape[:2]

        photo_PIL=PIL.Image.fromarray(self.photo)
        photo_PIL=PIL.ImageTk.PhotoImage(photo_PIL)
        self.c.create_image(0,0,anchor="nw", image=photo_PIL)

        x_px= scale_factor*self.x
        y_px= scale_factor*self.y
        r_px= scale_factor*robot_size
        self.c.create_oval(x_px-r_px, y_px-r_px, x_px+r_px, y_px+r_px, width=2, fill='')

        #draw an area of robot view
        self.c.create_polygon(int(x_px-75.5*scale_factor*cos(self.th)), -int(-y_px+scale_factor*75.5*sin(self.th)), 
                    int(x_px-58*scale_factor*cos(self.th)), -int(-y_px+scale_factor*58*sin(self.th)),
                    int(x_px-29*scale_factor*cos(self.th)+25*scale_factor*sin(self.th)), -int(-y_px+scale_factor*29*sin(self.th)+25*scale_factor*cos(self.th)),
                    int(x_px+29*scale_factor*cos(self.th)+25*scale_factor*sin(self.th)), -int(-y_px-scale_factor*29*sin(self.th)+25*scale_factor*cos(self.th)),
                    int(x_px+58*scale_factor*cos(self.th)), -int(-y_px-scale_factor*58*sin(self.th)),
                    int(x_px+75.5*scale_factor*cos(self.th)), -int(-y_px-scale_factor*75.5*sin(self.th)),
                    int(x_px+75.5*scale_factor*cos(self.th)+111*scale_factor*sin(self.th)), -int(-y_px-scale_factor*75.5*sin(self.th)+111*scale_factor*cos(self.th)),
                    int(x_px-75.5*scale_factor*cos(self.th)+111*scale_factor*sin(self.th)), -int(-y_px+scale_factor*75.5*sin(self.th)+111*scale_factor*cos(self.th)),
                    outline='red', fill='', width=3)
        self.c.pack()

        #tkinter canvas to imgmsg & publish

        self.c.postscript(file="tmp_canvas.eps", colormode="color", width=self.photo_width, height=self.photo_height, pagewidth=self.photo_width-1, pageheight=self.photo_width-1)
        data=ski_io.imread("tmp_canvas.eps")

        bridge=CvBridge()
        globalmap_msg=bridge.cv2_to_imgmsg(data, "rgb8")
        self.global_map_publisher.publish(globalmap_msg)
        self.crop_image()
        self.root.mainloop()

    ################## DEBUG #########################
    def image_debug(self):
        print("image_debug")
        self.photo=cv2.imread("./abc.jpg", cv2.IMREAD_COLOR)
        self.photo=cv2.cvtColor(self.photo, cv2.COLOR_RGB2BGR)
    ##################################################

    def crop_image(self):
        x_px= scale_factor*self.x
        y_px= scale_factor*self.y
        r_px= scale_factor*robot_size
        mask = np.zeros((self.photo_height, self.photo_width), dtype=np.uint8)
        pts=np.array([[[int(x_px-75.5*scale_factor*cos(th)), -int(-y_px+75.5*scale_factor*sin(th))],
                    [int(x_px-58*scale_factor*cos(th)), -int(-y_px+58*sin(th))],
                    [int(x_px-29*scale_factor*cos(th)+25*scale_factor*sin(th)), -int(-y_px+29*scale_factor*sin(th)+25*scale_factor*cos(th))],
                    [int(x_px+29*scale_factor*cos(th)+25*scale_factor*sin(th)), -int(-y_px-29*scale_factor*sin(th)+25*scale_factor*cos(th))],
                    [int(x_px+58*scale_factor*cos(th)), -int(-y_px-58*scale_factor*sin(th))],
                    [int(x_px+75.5*scale_factor*cos(th)), -int(-y_px-75.5*scale_factor*sin(th))],
                    [int(x_px+75.5*scale_factor*cos(th)+111*scale_factor*sin(th)), -int(-y_px-75.5*scale_factor*sin(th)+111*scale_factor*cos(th))],
                    [int(x_px-75.5*scale_factor*cos(th)+111*scale_factor*sin(th)), -int(-y_px+75.5*scale_factor*sin(th)+111*scale_factor*cos(th))]]])
        cv2.fillPoly(mask, pts, (255))
        res= cv2.bitwise_and(self.photo, self.photo, mask=mask)

        rect= cv2.boundingRect(pts)
        cropped=res[rect[1]: rect[1] + rect[3], rect[0]: rect[0] + rect[2]]]

    def callback_position(self, _position):
        self.x= _position.x
        self.y= _position.y


    def callback_heading(self, _heading):
        self.th=_heading.data

    def callback_map(self, _map):
        bridge=CvBridge()
        self.photo = bridge.imgmsg_to_cv2(_map, "rgb8")
        # self.photo = cv2.cvtColor(self.photo, cv2.COLOR_RGB2BGR)
        



if __name__ == "__main__":
    try:        
        rospy.init_node('hengel_global_map')
        PredictGlobalMap()
    except Exception as e:
        print(e)
        rospy.loginfo("shutdown program")
