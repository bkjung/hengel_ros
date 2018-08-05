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
# CANVAS_SIDE_LENGTH = 1.0
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
        while True:
            word = raw_input("Do you want to scale up input waypoints? (NO:1, YES:2)\n Type: ")
            self.option_scale_up = int(word)
            if self.option_scale_up==1 or self.option_scale_up==2:
                break

        if self.option_scale_up==2:
            while True:
                word = raw_input("What is the LENGTH AND HEIGHT OF CANVAS SIDE in selected file?\n Type: ")
                self.CANVAS_SIDE_LENGTH = float(word.split()[0])
                self.CANVAS_SIDE_HEIGHT = float(word.split()[1])
                break
            word = raw_input("What is the ratio you want to multiply waypoints? (length, height)\n Type: ")
            self.scale_up_factor_length = float(word.split()[0])
            self.scale_up_factor_height = float(word.split()[1])
            self.CANVAS_SIDE_LENGTH = self.CANVAS_SIDE_LENGTH*self.scale_up_factor_length
            self.CANVAS_SIDE_HEIGHT = self.CANVAS_SIDE_HEIGHT*self.scale_up_factor_height

        if self.option_scale_up==1:
            while True:
                word = raw_input("What is the LENGTH AND HEIGHT OF CANVAS SIDE in selected file?\n Type: ")
                self.CANVAS_SIDE_LENGTH = float(word.split()[0])
                self.CANVAS_SIDE_HEIGHT = float(word.split()[1])
                break

        print("Length, Height of Canvas Side = " + str(self.CANVAS_SIDE_LENGTH)+" "+str(self.CANVAS_SIDE_HEIGHT))
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

        #while True:
        #    word=raw_input("Type the option for interval \n[1] Same as file input \n[2] Equal interval manipulation \nType: ")
        #    self.option_interval=int(word)
        #    if self.option_interval == 1 or self.option_interval == 2:
        #        break
        #
        #if self.option_interval == 2:
        while True:
            word=raw_input("Type the interval(>=0.001) of waypoints: ")

            self.interval=float(word)
            if self.interval<0.001:
                print("Type value(>=0.001)")
            else:
                break
        #        for k in range(len(self.arr_path[i][j])):
        #            pass
                    #print(str(self.arr_path[i][j][k][0])+" "+str(self.arr_path[i][j][k][1]))
        self.get_path()
        print("-----------------------------------------------")


        self.run()

    def get_path(self):
        letter_path = []
        subletter_path=[]
        #x_last = -self.D
        x_last = self.D
        y_last = 0.0
        self.cnt_points = 0
        #subletter_path.append([x_last, y_last])

        flag_start = True
        dist = 0

        #root = Tk()
        #path_str = tkFileDialog.askopenfilename(parent=root,initialdir=home_path,title='Please select a path file to play')
        path_str=raw_input("Type Waypoint FILE PATH: ")

        #root.quit()
        if os.path.isfile(path_str):
            with open(path_str, "r") as file_path:
                for idx, line in enumerate(file_path):
                    _str = line.split()
                    if not len(_str) == 0:
                        # x_curr=(float(_str[0])*CANVAS_SIDE_LENGTH)*(-1.0)
                        # y_curr=(4.0-float(_str[1]))*CANVAS_SIDE_LENGTH
                        if self.option_scale_up==1:
                            x_curr=float(_str[0])*(-1)
                            y_curr=(self.CANVAS_SIDE_HEIGHT-float(_str[1]))
                            #x_curr=(float(_str[0]))
                            #y_curr=(self.CANVAS_SIDE_HEIGHT-float(_str[1]))*(-1.0)
                            # y_curr=float(_str[1])
                        else:
                            #x_curr=(float(_str[0])*(-1.0))*self.scale_up_factor_length
                            #y_curr=self.CANVAS_SIDE_HEIGHT-float(_str[1])*self.scale_up_factor_height
                            x_curr=(float(_str[0])*(-1))*self.scale_up_factor_length
                            y_curr=self.CANVAS_SIDE_HEIGHT-float(_str[1])*self.scale_up_factor_height
                            # y_curr=float(_str[1])


                        #if self.option_interval==2:
                        dist=sqrt(pow(x_last-x_curr,2)+pow(y_last-y_curr,2))
                        if dist>self.interval*2.0:
                            div=int(ceil(dist/self.interval))
                            for k in range(div):
                                x=x_last+(k+1)/float(div)*(x_curr-x_last)
                                y=y_last+(k+1)/float(div)*(y_curr-y_last)
                                subletter_path.append([x,y])
                                if self.isIntensityControl:
                                    if flag_start==True:
                                        self.arr_intensity.append(255.0)
                                    else:
                                        if len(_str)>2:
                                            self.arr_intensity.append(float(_str[2]))
                                        else:
                                            print("Spray intenstiy field empty!!!!!")
                                            sys.exit("Spray intenstiy field empty!!!!!")
                                self.cnt_points += 1
                            x_last=x
                            y_last=y
                        else:
                            subletter_path.append([x_curr, y_curr])

                            if self.isIntensityControl:
                                if flag_start==True:
                                    self.arr_intensity.append(255.0)
                                else:
                                    if len(_str)>2:
                                        self.arr_intensity.append(float(_str[2]))
                                    else:
                                        print("Spray intenstiy field empty!!!!!")
                                        sys.exit("Spray intenstiy field empty!!!!!")

                            self.cnt_points += 1
                            x_last=x_curr
                            y_last=y_curr
                            #continue

                        #elif self.option_interval==1:
                        #    subletter_path.append([x_curr, y_curr])
                        #    self.cnt_points += 1

                        #else:
                        #    print("THIS SHOULD NOT HAPPENNNNNNNNNNNNN!!!!")
                        #    sys.exit(":(:(:(:(:(:(:(")

                        if self.isIntensityControl:
                            pass

                        else:
                            if self.isStartEndIndexed:
                                if len(_str)>2:
                                    if int(float(_str[2]))==0:         #if the input waypoint is marked as end point
                                        self.end_point_list.append(self.cnt_points-1)
                                    elif int(float(_str[2]))==1:         #if the input waypoint is marked as start point
                                        self.start_point_list.append(self.cnt_points-1)
                                elif len(_str)==2:
                                    if flag_start==True:
                                        self.start_point_list.append(self.cnt_points-1)  #if the input waypoint is the initial one.
                    flag_start=False
                letter_path.append(subletter_path)
        self.arr_path.append(letter_path)


    def run(self):
        NavigationControl(self.arr_path, self.arr_intensity, self.start_point_list, self.end_point_list, self.isPositionControl, self.isIntensityControl, self.isStartEndIndexed, self.D)


if __name__ == '__main__':
    try:
        #PaintLetter()
        app = PaintSelectfile()
        app.run()


        print("End of Main Function")

    except Exception as e:
        print(e)
        # rospy.loginfo("shutdown program.")
