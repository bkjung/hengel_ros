#!/usr/bin/env python

import rospy
import numpy as np
import cv2, sys, time, math
# from bird_view import calibrate_mj
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt


class VisualOdometry():
    def __init__(self):
        #Initialize the node with rospy
        rospy.init_node('visual_odometry_node')
        ################
        rospy.on_shutdown(self.shutdown)
        #Create publisher
        self.vo_point_pub=rospy.Publisher("vo_current_point", Point, queue_size=50)
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', self.callback_image_sub)
        
        birdview_publisher.birdview_publisher(image_sub)
        


        position = Point()
        rospy.spin()

            

    def callback_image_sub(self, _image):
        bridge= CvBridge()
        return bridge.imgmsg_to_cv2(_image)


if __name__ == '__main__':
    try:
        VisualOdometry()

        print("End of Main Function")

    except Exception as e:
        print(e)
        rospy.loginfo("shutdown program.")