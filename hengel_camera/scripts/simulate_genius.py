#!/usr/bin/env python
import rospy
import numpy as np
import cv2, sys, time, math
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

if __name__ == '__main__':
    rospy.init_node('simulate_genius_node', anonymous=True)

    blank_photo = cv2.imread("./blank.png", cv2.IMREAD_COLOR)
    blank_photo = cv2.cvtColor(blank_photo, cv2.COLOR_RGB2BGR)

    pub1 = rospy.Publisher('/genius1/compressed', CompressedImage, queue_size=3)
    pub2 = rospy.Publisher('/genius2/compressed', CompressedImage, queue_size=3)
    pub3 = rospy.Publisher('/genius3/compressed', CompressedImage, queue_size=3)
    pub4 = rospy.Publisher('/genius4/compressed', CompressedImage, queue_size=3)
    rate = rospy.Rate(1)

    bridge = CvBridge()
    while not rospy.is_shutdown():
        # msg = CompressedImage()
        # msg.format = "jpeg"
        # msg.data = np.array(cv2.imencode('.jpg', frame)[1]).tostring()
        # #publish new image
        bridge=CvBridge()
        msg=bridge.cv2_to_compressed_imgmsg(blank_photo)
        pub1.publish(msg)
        pub2.publish(msg)
        pub3.publish(msg)
        pub4.publish(msg)
        #pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))

        cv2.waitKey(100)    #wait for input(ms)

    cap.release()
