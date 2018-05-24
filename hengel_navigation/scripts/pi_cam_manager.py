#!/usr/bin/env python
import rospy
import numpy as np
import cv2, sys, time, math
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import os

package_base_path = os.path.abspath(os.path.join(os.path.dirname(__file__),"../.."))
os.system("mkdir -p "+package_base_path+"/hengel_path_manager/pi_cam_keypoint_capture")

class PiCamManager():
    def __init__(self):
        self.picam_subscriber = rospy.Subscriber('/pi_floorcam/image_raw/compressed', CompressedImage, self.callback_picam)
        self.bridge=CvBridge()
        self.save_mode = False
        self.filename = ""

    def save(self, _filename):
        self.save_mode = True
        self.filename = _filename

    def callback_picam(self, _img):
        if self.save_mode:
            self.picam_photo= self.bridge.imgmsg_to_cv2(_img, "rgb8")
            cv2.imwrite(package_base_path+"/hengel_path_manager/pi_cam_keypoint_capture/"+self.filename+".jpg", cv2.imencode('jpg', picam_photo))
            self.save_mode = False
            self.filename = ""
        else:
            pass








