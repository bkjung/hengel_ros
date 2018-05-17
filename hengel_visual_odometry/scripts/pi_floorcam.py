#!/usr/bin/env python
import rospy
import numpy as np
import cv2, sys, time, math
from sensor_msgs.msg import Image

if __name__ == '__main__':
    cap=cv2.VideoCapture(0)
    pub=rospy.Publisher('/pi_floorcam/image_raw', Image, queue_size=10)
    rospy.init_node('pi_floorcam_node', anonymous= True)
    rate=rospy.Rate(10)

    bridge=CvBridge()
    while not rospy.is_shutdown():
        ret,frame=cap.read()
        pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
        cv2.waitKey(100)
    
    cap.release()
