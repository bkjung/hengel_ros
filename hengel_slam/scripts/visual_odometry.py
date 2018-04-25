#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image

class VisualOdometry():
    def __init__(self):
        while True:
            self.image_sub = rospy.Subscriber('/usb_cam/image_raw', self.callback_image_sub)



if __name__ == '__main__':
    try:
        VisualOdometry()

        print("End of Main Function)

    except Exception as e:
        print(e)
        rospy.loginfo("shutdown program.")