#!/usr/bin/env python
import rospy
import numpy as np
import cv2, sys, time, math
from sensor_msgs.msg import Image


class CrosspiontRotation():
    def __init__(self):
        self.photo = []
        self.photo = np.ndarray(self.photo)

        self.count_callback = 0
        self.floorcam_subscriber = rospy.Subscriber('/pi_floorcam/image_raw/compressed',
                                                    CompressedImage, callback_floorcam)

    def callback_floorcam(self, _photo):
        np_arr = np.fromstring(_img.data, np.uint8)
        image_cv = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        image_cv = cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY)


if __name__ == "__main__":
    try:
        rospy.init_node('hengel_crosspoint_rotation')
        CrosspiontRotation()
    except Exception as e:
        print(e)
        rospy.loginfo("shutdown program")
