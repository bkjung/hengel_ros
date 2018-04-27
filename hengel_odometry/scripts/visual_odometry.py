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

        while True:
            #####
            #Step 1. Capture current k_th frame


            #Step 2. Predict current position, assuming continuous motion. (pred_p_k = p_k-1 + (p_k-1 - p_k-2) )


            #Step 3. Predict Homography Matrix H_k^- (using SVD)


            #Step 4. determine search window according to H_k^-. Find corresponding pixels F_k-1^' by lucas-kanade algorithm


            #Step 5. Compute initial estimate of homography matrix H_k^' (using RANSAC)


            #Step 6. 

            

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