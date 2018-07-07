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

CANVAS_SIDE_LENGTH = 1.0
#CANVAS_SIDE_LENGTH = 1.5 * 0.58
#CANVAS_SIDE_LENGTH = 0.5 * 0.58
#PADDING_LENGTH = 0.0
PADDING_LENGTH = -0.65
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
        self.docking_point_list = []
        self.center_point_list = []
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
                word1=raw_input("Type the offset of the applicator (0.19 for test flag bot): ")

                self.D=float(word1)
                if self.D==0:
                    print("Offset cannot be zero")
                else:
                    break

        self.get_path()
        self.run()

    def get_path(self):
        #dir_str=package_base_path + "/hengel_path_manager/alphabet_path/li_eul_continuous.txt"
        dir_str = package_base_path + "/hengel_path_manager/alphabet_path/path_"
        letter_index = 0
        row_index = 0
        for letter in self.word:
            letter_path = []
            if letter == ' ':
                pass
            elif letter == '^':
                print("new line included")
                row_index = row_index + 1
            else:
                for i in range(1, 5):
                    subletter_path = []
                    if i==1:
                        subletter_path.append([-self.D,0])
                    else:
                        pass
                    path_str = dir_str + letter.capitalize() + "_" + str(i) + ".txt"
                    #path_str = dir_str
                    if os.path.isfile(path_str):
                        with open(path_str, "r") as file_path:
                            for idx, line in enumerate(file_path):
                                _str = line.split()
                                if not len(_str) == 0:
                                    #letter_path.append([(float)(_str[0])+(float)(letter_index)-(2*(float)(letter_index)-1)*250/1632, 1.0-(float)(_str[1])])
                                    x_curr=float(_str[0])*CANVAS_SIDE_LENGTH+float(letter_index)*(CANVAS_SIDE_LENGTH+PADDING_LENGTH)
                                    y_curr=(1-float(_str[1]))*CANVAS_SIDE_LENGTH+row_index*CANVAS_SIDE_LENGTH
                                    if self.isPositionControl:
                                        if len(subletter_path)==0:
                                            x_last=letter_path[-1][-1][0]
                                            y_last=letter_path[-1][-1][1]
                                        else:
                                            x_last=subletter_path[-1][0]
                                            y_last=subletter_path[-1][1]
                                        x_last=subletter_path[-1][0]
                                        y_last=subletter_path[-1][1]

                                        if len(subletter_path)!=0 or len(letter_path)!=0:
                                            dist=sqrt(pow(x_last-x_curr,2)+pow(y_last-y_curr,2))
                                            if dist>0.001:
                                                div=int(ceil(dist/0.001))
                                                for k in range(div-1):

                                                    x=x_last+(k+1)/float(div)*(x_curr-x_last)
                                                    y=y_last+(k+1)/float(div)*(y_curr-y_last)
                                                    subletter_path.append([x,y])
                                        else:
                                            pass
                                    subletter_path.append([x_curr, y_curr])
                                    if len(_str)>2:
                                        if _str[2]=="docking_line" or _str[2]=="docking_point_list":
                                            #letter_index, segment_index, waypoint_index
                                            self.docking_point_list.append([letter_index, i-1, idx])
                                    else:
                                        pass
                        letter_path.append(subletter_path)
                        self.center_point_list.append([
                            0.5 * CANVAS_SIDE_LENGTH +
                            (float)(letter_index) *
                            (CANVAS_SIDE_LENGTH + PADDING_LENGTH),
                            0.5 * CANVAS_SIDE_LENGTH + row_index * CANVAS_SIDE_LENGTH
                            ])



                        #Stop point for global view photo
            subletter_path = []
            subletter_path.append([
                CANVAS_SIDE_LENGTH + VIEWPOINT_DISTANCE +
                (float)(letter_index) * (CANVAS_SIDE_LENGTH + PADDING_LENGTH),
                (0.5) * CANVAS_SIDE_LENGTH + row_index * CANVAS_SIDE_LENGTH
                ])
            letter_path.append(subletter_path)
            self.arr_path.append(letter_path)

            #Keypoint Calculation
            # cnt_waypoints_in_lettter = len(letter_path)
            # for i in range(cnt_waypoints_in_letter):
            #     letter_path[i]...

            letter_index = letter_index + 1

    def run(self):
        NavigationControl(self.arr_path, self.docking_point_list, self.center_point_list, self.isPositionControl,self.D)


if __name__ == '__main__':
    try:
        PaintLetter()

        print("End of Main Function")

    except Exception as e:
        print(e)
        # rospy.loginfo("shutdown program.")
