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
from pathmaker_ui import PathMaker
from navigation_control import NavigationControl


class PaintDrawing():
    def __init__(self):
        self.PathUI = PathMaker()
        self.run()

    def run(self):
        if self.PathUI.isFinishedCorrectly == True:
            NavigationControl(self.PathUI.path_drawing, self.PathUI.draw_start_index)
        else:
            raise Exception("Path Drawing UI did not finish correctly")
        

if __name__ == '__main__':
    try:
        PaintDrawing()

        print("End of Main Function")

    except Exception as e:
        print(e)
        # rospy.loginfo("shutdown program.")
