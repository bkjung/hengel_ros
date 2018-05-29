#!/usr/bin/env python

# import rospy
from geometry_msgs.msg import Twist, Point, Quaternion, PoseStamped
from std_msgs.msg import Float32
import tf
from math import radians, copysign, sqrt, pow, pi, atan2, sin, floor, cos
from tf.transformations import euler_from_quaternion
import numpy as np
import sys
from PIL import Image
import time
import os
from navigation_control import NavigationControl
import cv2

CANVAS_SIDE_LENGTH = 1.5 * 0.58
#CANVAS_SIDE_LENGTH = 0.5 * 0.58
#PADDING_LENGTH = 0.0
PADDING_LENGTH = -0.65 * 0.58
VIEWPOINT_DISTANCE = 0.3 * 0.58

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
        # self.arr_keypoint=[]
        self.word = raw_input("Type letters to draw:")

        self.get_path()
        self.run()

    def get_path(self):
        dir_str = package_base_path + "/hengel_path_manager/alphabet_path/path_"
        letter_index = 0
        for letter in self.word:
            letter_path = []
            if letter == ' ':
                pass

            else:
                for i in range(1, 5):
                    subletter_path = []
                    path_str = dir_str + letter.capitalize() + "_" + str(i) + ".txt"
                    if os.path.isfile(path_str):
                        with open(path_str, "r") as file_path:
                            for idx, line in enumerate(file_path):
                                _str = line.split()
                                if not len(_str) == 0:
                                    #letter_path.append([(float)(_str[0])+(float)(letter_index)-(2*(float)(letter_index)-1)*250/1632, 1.0-(float)(_str[1])])
                                    subletter_path.append([
                                        (float)(_str[0]) * CANVAS_SIDE_LENGTH +
                                        (float)(letter_index) *
                                        (CANVAS_SIDE_LENGTH + PADDING_LENGTH),
                                        (1.0 - (float)(_str[1])) * CANVAS_SIDE_LENGTH
                                    ])
                                else:
                                    pass
                        letter_path.append(subletter_path)

            #Stop point for global view photo
            subletter_path = []
            subletter_path.append([
                CANVAS_SIDE_LENGTH + VIEWPOINT_DISTANCE +
                (float)(letter_index) * (CANVAS_SIDE_LENGTH + PADDING_LENGTH),
                (0.5) * CANVAS_SIDE_LENGTH
            ])
            letter_path.append(subletter_path)
            self.arr_path.append(letter_path)

            #Keypoint Calculation
            # cnt_waypoints_in_lettter = len(letter_path)
            # for i in range(cnt_waypoints_in_letter):
            #     letter_path[i]...

            letter_index = letter_index + 1

    def run(self):
        NavigationControl(self.arr_path)


if __name__ == '__main__':
    try:
        PaintLetter()

        print("End of Main Function")

    except Exception as e:
        print(e)
        # rospy.loginfo("shutdown program.")
