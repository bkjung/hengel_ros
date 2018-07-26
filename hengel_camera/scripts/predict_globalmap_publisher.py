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
from sensor_msgs.msg import Image, CompressedImage
from math import cos, sin, pi
import numpy as np
from cv_bridge import CvBridge
import cv2
import skimage.io as ski_io

size_x = 1200
size_y = 1200
robot_size = 15  #[cm]
scale_factor = 3  #[pixel/cm]
th=0

class PredictGlobalMap():
    def __init__(self, _img):
        rospy.init_node('hengel_global_map')

        self.pixMetRatio=100
        self.mapImg=_img

        self.x = 0
        self.y = 0
        self.th = 0
        
        self.global_map_publisher = rospy.Publisher(
            '/predict_global_map', Image, queue_size=10)
        self.robot_view_publisher = rospy.Publisher(
            '/predict_robot_view', CompressedImage, queue_size=10)

        rospy.Subscriber('/midpoint', Point, self.callback_point)
        rospy.spin()

    ################## DEBUG #########################
    def image_debug(self):
        try:
            self.root = Tk()
            self.c = Canvas(self.root, height=size_x, width=size_y, bg="white")
            self.root.title("global map")
            _str = str(size_x) + "x" + str(size_y)
            self.root.geometry(_str)

            self.photo = cv2.imread("./abc.jpg", cv2.IMREAD_COLOR)
            self.photo = cv2.cvtColor(self.photo, cv2.COLOR_RGB2BGR)
            print(type(self.photo))
            photo_PIL = PIL.Image.fromarray(self.photo)
            photo_PIL_Img = PIL.ImageTk.PhotoImage(photo_PIL)

            self.c.create_image(0, 0, anchor="nw", image=photo_PIL_Img)
            print("DEBUG-2")
            self.c.create_polygon(0,0,100,0,100,100,0,100, outline='red', fill='', width=3)
            self.c.pack()
            self.c.create_image(0, 0, anchor="nw", image=photo_PIL_Img)
            self.root.mainloop()

        except Exception as e:
            print("abc.jpg not loaded")
            print(e)
    ##################################################

    def callback_point(self, _pnt):
        map_cv=self.mapImg
        x_px=_pnt.x*self.pixMetRatio
        y=self.mapImg.shape[2] - _pnt.y*self.pixMetRatio
        th=pnt.z
        print("time:", time.time())

        if map_cv is not None:
            self.root = Toplevel()
            self.c = Canvas(self.root, height=size_x, width=size_y, bg="white")
            self.root.title("global map")
            _str = str(size_x) + "x" + str(size_y)
            self.root.geometry(_str)

            self.photo_height, self.photo_width = map_cv.shape[:2]
            
            photo_PIL = PIL.Image.fromarray(map_cv)
            photo_PIL = PIL.ImageTk.PhotoImage(photo_PIL)

            map_size=100

            imagesprite=self.c.create_image(0,0, anchor="nw", image=photo_PIL)
            self.c.create_polygon(
                x_px+map_size/sqrt(2)*cos(th+pi/4),
                y_px+map_size/sqrt(2)*sin(th+pi/4),
                x_px-map_size/sqrt(2)*cos(pi/4-th),
                y_px+map_size/sqrt(2)*sin(pi/4-th),
                x_px-map_size/sqrt(2)*cos(pi/4+th),
                y_px-map_size/sqrt(2)*sin(pi/4+th),
                x_px+map_size/sqrt(2)*cos(pi/4-th),
                y_px-map_size/sqrt(2)*sin(pi/4-th),
                outline='red',
                fill='',
                width=3)
            self.c.pack()

            #tkinter canvas to imgmsg & publish
            self.c.postscript(
                file="tmp_canvas.eps",
                colormode="color",
                width=self.photo_width,
                height=self.photo_height,
                pagewidth=self.photo_width - 1,
                pageheight=self.photo_height - 1
                )
            self.root.destroy()
            data = ski_io.imread("tmp_canvas.eps")

            bridge = CvBridge()
            globalmap_msg = bridge.cv2_to_imgmsg(data, "rgb8")
            self.global_map_publisher.publish(globalmap_msg)
            print("publish-1, time:", time.time())
            self.crop_image()
            # self.root.mainloop()

        else:
            pass        


    def crop_image(self):
        x_px = scale_factor * self.x
        y_px = scale_factor * self.y
        r_px = scale_factor * robot_size
        mask = np.zeros((self.photo_height, self.photo_width), dtype=np.uint8)
        pts = np.array([[[
            int(x_px - 75.5 * scale_factor * cos(th)),
            -int(-y_px + 75.5 * scale_factor * sin(th))
        ], [
            int(x_px - 58 * scale_factor * cos(th)), -int(-y_px + 58 * sin(th))
        ], [
            int(x_px - 29 * scale_factor * cos(th) +
                25 * scale_factor * sin(th)),
            -int(-y_px + 29 * scale_factor * sin(th) +
                 25 * scale_factor * cos(th))
        ], [
            int(x_px + 29 * scale_factor * cos(th) +
                25 * scale_factor * sin(th)),
            -int(-y_px - 29 * scale_factor * sin(th) +
                 25 * scale_factor * cos(th))
        ], [
            int(x_px + 58 * scale_factor * cos(th)),
            -int(-y_px - 58 * scale_factor * sin(th))
        ], [
            int(x_px + 75.5 * scale_factor * cos(th)),
            -int(-y_px - 75.5 * scale_factor * sin(th))
        ], [
            int(x_px + 75.5 * scale_factor * cos(th) +
                111 * scale_factor * sin(th)),
            -int(-y_px - 75.5 * scale_factor * sin(th) +
                 111 * scale_factor * cos(th))
        ], [
            int(x_px - 75.5 * scale_factor * cos(th) +
                111 * scale_factor * sin(th)),
            -int(-y_px + 75.5 * scale_factor * sin(th) +
                 111 * scale_factor * cos(th))
        ]]])
        cv2.fillPoly(mask, pts, (255))
        res = cv2.bitwise_and(self.photo, self.photo, mask=mask)
        bridge=CvBridge()
        imgMsg=bridge.cv2_to_compressed_imgmsg(res)
        self.robot_view_publisher.publish(imgMsg)


        # rect = cv2.boundingRect(pts)
        # cropped = res[rect[1]:rect[1] + rect[3], rect[0]:rect[0] + rect[2]]