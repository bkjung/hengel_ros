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



package_base_path = os.path.abspath(os.path.join(os.path.dirname(__file__),"../.."))
os.system("mkdir -p "+package_base_path+"/hengel_path_manager/output_pathmap")
os.system("mkdir -p"+package_base_path+"/hengel_path_manager/waypnts")


class PaintLetter():
    def __init__(self):
        self.arr_path=[]
        self.draw_start_index=[]
        self.word=raw_input("Type letters to draw:")
        print("Input:", self.word)

        self.get_path()
        self.run()

    def get_path(self):
        dir_1= package_base_path+"/hengel_path_manager/alphabet_path/path_"
        dir_2=".txt"
        letter_index = 0
        for letter in self.word:
            letter_path=[]
            if letter==' ':
                pass
                #if spacing is included
            else:
                with open(dir_1+letter.capitalize()+dir_2,"r") as file_path:
                    for idx, line in enumerate(file_path):
                        _str = line.split()
                        if not len(_str)==0:
                            #letter_path.append([(float)(_str[0])+(float)(letter_index)-(2*(float)(letter_index)-1)*250/1632, 1.0-(float)(_str[1])])
                            letter_path.append([(float)(_str[0])+(float)(letter_index), 1.0-(float)(_str[1])])
                        else:
                            pass
            #count the number of letters including spacing
            letter_index = letter_index + 1

            self.arr_path.append(letter_path)

#        arr_path_file =cv2.FileStorage(package_base_path+"/hengel_path_manager/waypnts/Path.xml", cv2.FILE_STORAGE_WRITE)
#        arr_path_file.write("arr_path", self.arr_path)
#        arr_path_file.release()

    def run(self):
        # NavigationControl(self.arr_path, self.draw_start_index)
        NavigationControl(self.arr_path)


if __name__ == '__main__':
    try:
        PaintLetter()

        print("End of Main Function")

    except Exception as e:
        print(e)
        # rospy.loginfo("shutdown program.")
