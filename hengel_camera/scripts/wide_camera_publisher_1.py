#!/usr/bin/env python
import rospy
import numpy as np
import cv2, sys, time, math
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

if __name__ == '__main__':
    rospy.init_node('wide_cam_node_1', anonymous=True)
    cap = cv2.VideoCapture(0)
    pub_raw=rospy.Publisher('/wide_cam_1/image_raw', Image, queue_size=3)
    pub = rospy.Publisher(
        '/wide_cam_1/compressed', CompressedImage, queue_size=3)
    rate = rospy.Rate(10)

    bridge = CvBridge()
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        msg = CompressedImage()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', frame)[1]).tostring()
        #publish new image
        pub.publish(msg)
        #pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))

        cv2.waitKey(100)    #wait for input(ms)

    cap.release()
