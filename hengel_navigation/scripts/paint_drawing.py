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

CANVAS_SIDE_LENGTH = 6.0 * 0.58
VIEWPOINT_DISTANCE = 0.3 * 0.58

class PaintDrawing():
    def __init__(self):
        self.PathUI = PathMaker()

        self.path_scaled_append = []

        self.path_scaled = []
        self.path_scaled = [[i[0]*CANVAS_SIDE_LENGTH, i[1]*CANVAS_SIDE_LENGTH] for i in self.PathUI.path_drawing]
        self.path_scaled_append.append(self.path_scaled)

        self.path_scaled = []
        self.path_scaled.append([
            CANVAS_SIDE_LENGTH + VIEWPOINT_DISTANCE,
            (0.5) * CANVAS_SIDE_LENGTH
        ])
        self.path_scaled_append.append(self.path_scaled)

        self.path_return = []
        self.path_return.append(self.path_scaled_append)

        self.run()

    def run(self):
        if self.PathUI.isFinishedCorrectly == True:
            NavigationControl(self.path_return, [], [])
        else:
            raise Exception("Path Drawing UI did not finish correctly")


if __name__ == '__main__':
    try:
        PaintDrawing()

        print("End of Main Function")

    except Exception as e:
        print(e)
        # rospy.loginfo("shutdown program.")
