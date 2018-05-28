#!/usr/bin/env python
import rospy
import numpy as np
import cv2, sys, time, math
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import os
import time

package_base_path = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "../.."))


class PiCamManager():
    def __init__(self, _starttime):
        self.picam_subscriber = rospy.Subscriber(
            '/pi_floorcam/image_raw/compressed', CompressedImage,
            self.callback_picam)
        self.bridge = CvBridge()
        self.save_mode = False
        self.starttime = _starttime
        os.system("mkdir -p " + package_base_path +
                "/hengel_path_manager/pi_cam_keypoint_capture/"+self.starttime)
        self.filename = ""

    def save(self, _filename):
        self.save_mode = True
        self.filename = _filename

    def callback_picam(self, _img):
        if self.save_mode:
            np_arr = np.fromstring(_img.data, np.uint8)
            image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            image_np = cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY)
            #self.picam_photo = self.bridge.imgmsg_to_cv2(_img, "rgb8")
            cv2.imwrite(
                package_base_path +
                "/hengel_path_manager/pi_cam_keypoint_capture/" + self.starttime + str("/") + self.filename
                + ".jpg", image_np)
            self.save_mode = False
            self.filename = ""
        else:
            pass


if __name__=="__main__":
    try:
        rospy.init_node('pi_cam_manager')
        app = PiCamManager(time.strftime("%y%m%d_%H%M%S"))
        
        cnt = 0
        while(True):
            try:
                app.save(str(cnt))
                cnt = cnt+1
                time.sleep(0.02)
            except KeyboardInterrupt:
                break
    except Exception as e:
        print(e)
        rospy.loginfo("shutdown program")
