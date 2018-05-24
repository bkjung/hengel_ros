#!/usr/bin/env python
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

if __name__ == '__main__':
    cap = cv2.VideoCapture(1)
    pub = rospy.Publisher('/usb_cam/image_raw', Image, queue_size=10)
    rospy.init_node('raw_img_publisher', anonymous=True)
    rate = rospy.Rate(10)

    bridge = CvBridge()
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
        cv2.waitKey(100)

    cap.release()
