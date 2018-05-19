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
from hengel_navigation/GlobalFeedback.srv import *

size_x=1000
size_y=1000
scale_factor= 3 #[pixel/cm]

package_base_path = os.path.abspath(os.path.join(os.path.dirname(__file__),"../.."))

class RealGlobalMap():
    def __init__(self):
        rospy.init_node('real_globalmap', anonymous=True)

        s = rospy.Service('/global_feedback', GlobalFeedback, handle_globalfeedback)

        self.root=Tk()

        self.photo=[]
        self.photo=np.ndarray(self.photo)

        self.x=0
        self.y=0
        self.th=0

        path_file=cv2.FileStorage(package_base_path+"/hengel_path_manager/waypnts/Path.xml", cv2.FILE_STORAGE_READ)
        self.waypnts_arr= path_file.getNode("arr_path")
        path_file.release()

        self.c=Canvas(self.root, height=size_x, width=size_y, bg="white")
        self.root.title("global map")
        _str=str(size_x)+"x"+str(size_y)
        self.root.geometry(_str)

        # self.around_view_subscriber= rospy.Subscriber('/around_img', Image, self.callback_view)
        # self.position_subscriber=rospy.Subscriber('/current_position', Point, self.callback_position)
        # self.heading_subscriber = rospy.Subscriber('/current_heading', Float32, self.callback_heading)

        ############## DEBUG ###########################
        # self.photo=PIL.ImageTk.PhotoImage(_map)
        # self.c.create_image(0,0,anchor="nw", image=self.photo)
        self.image_debug()
        ################################################
        
        self.MapPublisher()

    def handle_globalfeedback(self,req):
        letter_number = req.letter_number
        
        ######### ADD CODES ###############



        return [delta_x, delta_y, delta_th]


    def MapPublisher(self):
        x_px= scale_factor*self.x
        y_px= scale_factor*self.y

        #Rotate image (size: diagonal * diagonal)
        rows, cols = self.photo.shape[:2]
        diagonal = (int)(sqrt(rows*rows+cols*cols))

        offsetX = (diagonal - cols)/2
        offsetY = (diagonal - rows)/2

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
        new_Image=PIL.Image.new("RGB", (size_x, size_y), (256,256,256))
        new_Image.paste(photo_PIL, (x_px-diagonal/2, y_px-diagonal/2))
        new_Image.show()


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

