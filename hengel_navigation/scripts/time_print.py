#!/usr/bin/env python
import rospy
import sys
from geometry_msgs.msg import Point
from std_msgs.msg import Float32, Time, Header
from sensor_msgs.msg import Image, CompressedImage
from markRobotView import RobotView
from math import radians, copysign, sqrt, pow, pi, atan2, sin, floor, cos, asin,ceil
from numpy.linalg import inv
import numpy as np
import time
import os
import copy
from os.path import expanduser
import message_filters
import collections
from feature_match import FeatureMatch
from matplotlib import pyplot as plt
from hengel_camera.msg import CmpImg
from cv_bridge import CvBridge
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')


class TimePrint():