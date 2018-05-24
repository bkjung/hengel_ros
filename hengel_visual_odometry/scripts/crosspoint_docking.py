#!/usr/bin/env python
import rospy
import numpy as np
import cv2, sys, time, math
from sensor_msgs.msg import Image


class CrosspiontDocking():
    def __init__(self):
        self.photo = []
        self.photo = np.ndarray(self.photo)

        self.count_callback = 0
        self.floorcam_subscriber = rospy.Subscriber('/pi_floorcam/image_raw',
                                                    Image, callback_floorcam)

    def callback_floorcam(self, _photo):
        bridge = CvBridge()
        self.photo = bridge.imgmsg_to_cv2(_map, "rgb8")
        self.photo = cv2.cvtColor(self.photo, cv2.COLOR_RGB2BGR)


if __name__ == "__main__":
    try:
        rospy.init_node('hengel_crosspoint_docking')
        CrosspiontDocking()
    except Exception as e:
        print(e)
        rospy.loginfo("shutdown program")
