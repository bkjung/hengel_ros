#!/usr/bin/env python
import rospy
import numpy as np
import cv2, sys, time, math
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

if __name__ == '__main__':
    rospy.init_node('pi_floorcam_node', anonymous=True)
    cap = cv2.VideoCapture(0)
    pub = rospy.Publisher(
        '/pi_floorcam/image_raw/compressed', CompressedImage, queue_size=3)
    pub_box = rospy.Publisher(
        '/pi_floorcam_box/image_raw/compressed', CompressedImage, queue_size=3)
    rate = rospy.Rate(10)

    bridge = CvBridge()
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        msg = CompressedImage()
        msg.format = "jpeg"
        np_arr = np.array(cv2.imencode('.jpg', frame)[1])
        msg.data = np_arr.tostring()
        #publish new image
        pub.publish(msg)
        #pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))

        msg_box = CompressedImage()
        msg_box.format = "jpeg"

        for i in range(199, 300):
            np_arr[i][169] = [255, 0, 0]
            np_arr[i][329] = [255, 0, 0]
        for j in range(169, 330):
            np_arr[199][j] = [255, 0, 0]
            np_arr[299][j] = [255, 0, 0]
        
        msg_box.data = np_arr.tostring()
        pub_box.publish(msg_box)

        cv2.waitKey(100)

    cap.release()
