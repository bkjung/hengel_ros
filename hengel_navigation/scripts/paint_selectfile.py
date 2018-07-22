#!/usr/bin/env python

# import rospy
from geometry_msgs.msg import Twist, Point, Quaternion, PoseStamped
from std_msgs.msg import Float32
import tf
from math import radians, copysign, sqrt, pow, pi, atan2, sin, ceil, floor, cos
from tf.transformations import euler_from_quaternion
import numpy as np
import sys
from PIL import Image
import time
import os
from navigation_control import NavigationControl
import cv2

from Tkinter import *
import tkFileDialog

#CANVAS_SIDE_LENGTH = 5.0
CANVAS_SIDE_LENGTH = 1.0
#CANVAS_SIDE_LENGTH = 1.5 * 0.58
#CANVAS_SIDE_LENGTH = 0.5 * 0.58
#PADDING_LENGTH = 0.0
#PADDING_LENGTH = -0.65
#PADDING_LENGTH = -0.30
#VIEWPOINT_DISTANCE = 0.3

package_base_path = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "../.."))
home_path = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "../../../../.."))        
os.system("mkdir -p " + package_base_path +
        "/hengel_path_manager/output_pathmap")


class PaintSelectfile():
    def __init__(self):
        print("Length of Canvas Side = " + str(CANVAS_SIDE_LENGTH))
        # print("Length of Padding = " + str(PADDING_LENGTH))
        # print("Distance of Viewpoint = " + str(VIEWPOINT_DISTANCE))
        self.arr_path = []
        self.arr_intensity = []
        self.start_point_list = []
        self.end_point_list = []
        self.isIntensityControl = False
        self.isStartEndIndexed = False

        while True:
            word = raw_input("[1] Position control, [2] RPM control, [3] Position control & Spray intensity control\n Type 1 or 2 or 3:")
            if int(word)==1:
                self.isPositionControl=True
                while True:
                    word_2 = raw_input("Start / End Point Indexed???\n 1=Yes, 2=No\n Type: ")
                    if int(word_2)==1:
                        self.isStartEndIndexed=True
                        break
                    elif int(word_2)==2:
                        break
                break

            elif int(word)==2:
                self.isPositionControl=False
                break
            elif int(word)==3:
                self.isPositionControl=True
                self.isIntensityControl=True
                break

        if self.isPositionControl:
            while True:
                word1=raw_input("Type the offset of the applicator (0.20 for test flag bot): ")

                self.D=float(word1)
                if self.D==0:
                    print("Offset cannot be zero")
                else:
                    break

            while True:
                word=raw_input("Type the option for interval \n[1] Same as file input \n[2] Equal interval manipulation \nType: ")
                self.option_interval=int(word)
                if self.option_interval == 1 or self.option_interval == 2:
                    break
                
            if self.option_interval == 2:
                while True:
                    word=raw_input("Type the interval(>=0.001) of waypoints: ")

                    self.interval=float(word)
                    if self.interval<0.001:
                        print("Type value(>=0.001)")
                    else:
                        break

        self.get_path()
        print("path creation completed")
        print("-----------------------------------------------")
        # print(self.arr_path)
        for i in range(len(self.arr_path)):
            for j in range(len(self.arr_path[i])):
                for k in range(len(self.arr_path[i][j])):
		    pass
                    #print(str(self.arr_path[i][j][k][0])+" "+str(self.arr_path[i][j][k][1]))
        print("-----------------------------------------------")
        self.run()

    def get_path(self):
        letter_path = []
        subletter_path=[]
        x_last = -self.D
        y_last = 0.0
        cnt_points = 0
        subletter_path.append([x_last, y_last])
        cnt_points += 1

        flag_start = True
        dist = 0

        root = Tk()
        path_str = tkFileDialog.askopenfilename(parent=root,initialdir=home_path,title='Please select a path file to play')
        root.quit()
        if os.path.isfile(path_str):
            with open(path_str, "r") as file_path:
                for idx, line in enumerate(file_path):
                    _str = line.split()
                    if not len(_str) == 0:
                        x_curr=(float(_str[0])*CANVAS_SIDE_LENGTH)*(-1.0)
                        y_curr=(1-float(_str[1]))*CANVAS_SIDE_LENGTH

                        if self.option_interval==2:
                            dist=sqrt(pow(x_last-x_curr,2)+pow(y_last-y_curr,2))
                            if dist>self.interval:
                                div=int(ceil(dist/self.interval))
                                for k in range(div):
                                    x=x_last+(k+1)/float(div)*(x_curr-x_last)
                                    y=y_last+(k+1)/float(div)*(y_curr-y_last)
                                    subletter_path.append([x,y])
                                    cnt_points += 1
                                x_last=x
                                y_last=y
                                dist = 0
                            else:
                                continue

                        elif self.option_interval==1:
                            subletter_path.append([x_curr, y_curr])
                            cnt_points += 1

                        else:
                            print("THIS SHOULD NOT HAPPENNNNNNNNNNNNN!!!!")
                            sys.exit(":(:(:(:(:(:(:(")

                        if self.isIntensityControl:
                            if len(_str)>2:
                                self.arr_intensity.append(float(_str[2]))
                            else:
                                print("Spray intenstiy field empty!!!!!")

                        else:
                            if self.isStartEndIndexed:
                                if len(_str)>2:
                                    if int(float(_str[2]))==0:         #if the input waypoint is marked as end point
                                        self.end_point_list.append(cnt_points-1)
                                    elif int(float(_str[2]))==1:         #if the input waypoint is marked as start point
                                        self.start_point_list.append(cnt_points-1)
                                elif len(_str)==2:
                                    if flag_start==True:
                                        self.start_point_list.append(cnt_points-1)  #if the input waypoint is the initial one.
                                        flag_start  = False

                letter_path.append(subletter_path)
        self.arr_path.append(letter_path)


    def run(self):
        NavigationControl(self.arr_path, self.arr_intensity, self.start_point_list, self.end_point_list, self.isPositionControl, self.isIntensityControl, self.D)


if __name__ == '__main__':
    try:
        PaintLetter()

        print("End of Main Function")

    except Exception as e:
        print(e)
        # rospy.loginfo("shutdown program.")
