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

CANVAS_SIDE_LENGTH = 5.0
#CANVAS_SIDE_LENGTH = 1.5 * 0.58
#CANVAS_SIDE_LENGTH = 0.5 * 0.58
#PADDING_LENGTH = 0.0
#PADDING_LENGTH = -0.65
PADDING_LENGTH = -0.30
VIEWPOINT_DISTANCE = 0.3

package_base_path = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "../.."))
os.system("mkdir -p " + package_base_path +
        "/hengel_path_manager/output_pathmap")
os.system("mkdir -p " + package_base_path + "/hengel_path_manager/waypnts")


class PaintLetter():
    def __init__(self):
        print("Length of Canvas Side = " + str(CANVAS_SIDE_LENGTH))
        print("Length of Padding = " + str(PADDING_LENGTH))
        print("Distance of Viewpoint = " + str(VIEWPOINT_DISTANCE))
        self.arr_path = []
        self.start_point_list = []
        self.end_point_list = []
        # self.arr_keypoint=[]
        self.word = raw_input("Type letters to draw:")

        while True:
            word = raw_input("[1] Position control, [2] RPM control\n Type 1 or 2 :")
            if int(word)==1:
                self.isPositionControl=True
                break
            elif int(word)==2:
                self.isPositionControl=False
                break

        if self.isPositionControl:
            while True:
                word1=raw_input("Type the offset of the applicator (0.20 for test flag bot): ")

                self.D=float(word1)
                if self.D==0:
                    print("Offset cannot be zero")
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
        #dir_str=package_base_path + "/hengel_path_manager/alphabet_path/li_eul_continuous.txt"
        dir_str = package_base_path + "/hengel_path_manager/alphabet_path/path_"
        letter_index = 0
        row_index = 0
        initial_letter = 1
        x_last = 0.0
        y_last = 0.0
        cnt_waypoints = 0

        for letter in self.word:
            print(letter)
            letter_path = []

            if initial_letter == 1:
                subletter_path=[]
                x_last = -self.D
                #x_last = 0.0
                y_last = 0.0
                subletter_path.append([x_last, y_last])
                cnt_waypoints += 1
                letter_path.append(subletter_path)
                self.arr_path.append(letter_path)
                letter_path = []
                initial_letter = 0

            if letter == ' ':
                pass
            elif letter == '^':
                print("new line included")
                row_index = row_index + 1
            else:
                for i in range(1, 5):
                    subletter_path = []
                    path_str = dir_str + letter.capitalize() + "_" + str(i) + ".txt"
                    #path_str = dir_str
                    if os.path.isfile(path_str):
                        with open(path_str, "r") as file_path:
                            for idx, line in enumerate(file_path):
                                _str = line.split()
                                if not len(_str) == 0:
                                    #letter_path.append([(float)(_str[0])+(float)(letter_index)-(2*(float)(letter_index)-1)*250/1632, 1.0-(float)(_str[1])])
                                    x_curr=(float(_str[0])*CANVAS_SIDE_LENGTH+float(letter_index)*(CANVAS_SIDE_LENGTH+PADDING_LENGTH))*-1.0
                                    y_curr=(1-float(_str[1]))*CANVAS_SIDE_LENGTH+row_index*CANVAS_SIDE_LENGTH

                                    dist=sqrt(pow(x_last-x_curr,2)+pow(y_last-y_curr,2))
                                    #if dist>0.001:
                                    if dist>0.003:
                                        div=int(ceil(dist/0.003))
                                        #div=int(ceil(dist/0.001))
                                        for k in range(div):
                                            x=x_last+(k+1)/float(div)*(x_curr-x_last)
                                            y=y_last+(k+1)/float(div)*(y_curr-y_last)
                                            subletter_path.append([x,y])
                                            cnt_waypoints += 1
                                        x_last=x
                                        y_last=y
                                    else:
                                        subletter_path.append([x_curr, y_curr])
                                        cnt_waypoints += 1
                                        x_last=x_curr
                                        y_last=y_curr

                                    if idx==0:
                                        self.start_point_list.append(cnt_waypoints-1)
                            self.end_point_list.append(cnt_waypoints-1)


                            letter_path.append(subletter_path)


            self.arr_path.append(letter_path)

            letter_index = letter_index + 1

    def run(self):
        NavigationControl(self.arr_path, self.start_point_list, self.end_point_list, self.isPositionControl,self.D)


if __name__ == '__main__':
    try:
        PaintLetter()

        print("End of Main Function")

    except Exception as e:
        print(e)
        # rospy.loginfo("shutdown program.")
