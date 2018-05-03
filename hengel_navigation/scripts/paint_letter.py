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


package_base_path = os.path.abspath(os.path.join(os.path.dirname(__file__),"../.."))
os.system("mkdir -p "+package_base_path+"/hengel_path_manager/output_pathmap")



def get_path(word):
    global cnt_letter
    arr_path=[]
    dir_1= package_base_path+"/hengel_path_manager/alphabet_path/path_"
    dir_2=".txt"
    cnt_letter = 0
    for letter in word:
        if letter==' ':
            pass
            #if spacing is included
        else:
            with open(dir_1+letter.capitalize()+dir_2,"r") as file_path:
                for idx, line in enumerate(file_path):
                    _str = line.split()
                    if not len(_str)==0:
                        arr_path.append([(float)(_str[0])+(float)(cnt_letter)-(2*(float)(cnt_letter)-1)*250/1632, 1.0-(float)(_str[1])])
                    else:
                        pass

        #count the number of letters including spacing
        cnt_letter = cnt_letter + 1

    return arr_path

class PaintLetter():
    def __init__(self):
        word=raw_input("Type letters to draw:")
        print("Input:", word)
        path_letter_saved=get_path(word)

        NavigationControl(path_letter_saved, sys.argv[1])
 



if __name__ == '__main__':
    try:
        PaintLetter()
        print("End of Main Function")

    except Exception as e:
        print(e)
        # rospy.loginfo("shutdown program.")
