#!/usr/bin/env python
import rospy
import io
import time
import cv2
import numpy as np
#from hengel_camera.msg import CmpImg
from sensor_msgs.msg import CompressedImage
#from cv_bridge import CvBridge
import cv_bridge

package="/home/turtleberry/catkin_ws/src/hengel_ros/hengel_camera/"

class Undistort():
    def __init__(self):
        rospy.init_node('undistortion', anonymous=True)
        self.pub1 = rospy.Publisher('/usb_cam1/undistort/compressed', CompressedImage, queue_size=3)
        self.pub2 = rospy.Publisher('/usb_cam2/undistort/compressed', CompressedImage, queue_size=3)
        self.pub3 = rospy.Publisher('/usb_cam3/undistort/compressed', CompressedImage, queue_size=3)
        self.pub4 = rospy.Publisher('/usb_cam4/undistort/compressed', CompressedImage, queue_size=3)

        rospy.Subscriber('/usb_cam1/image_raw/compressed', CompressedImage, self.callback_undistort1)
        rospy.Subscriber('/usb_cam2/image_raw/compressed', CompressedImage, self.callback_undistort2)
        rospy.Subscriber('/usb_cam3/image_raw/compressed', CompressedImage, self.callback_undistort3)
        rospy.Subscriber('/usb_cam4/image_raw/compressed', CompressedImage, self.callback_undistort4)

        rospy.spin()

        rate = rospy.Rate(10)

    def callback_undistort1(self,_img):
        print("SUBSCRIBE-1")
        try:
            bridge=cv_bridge.CvBridge()
            rawimg=bridge.compressed_imgmsg_to_cv2(_img, "bgr8")
        except Exception as e:
            print(e)
            return
        mtx=[[657.340773, 0, 664.549143],[ 0, 661.426938, 469.186635], [0,0,1]]
        mtx=np.array(mtx)
        dst=[-0.276379, 0.054208, 0.000882, 0.001326]
        dst=np.array(dst)
        if len(mtx)!=0 and len(dst)!=0:
            if rawimg is not None:
                undistImg=cv2.undistort(rawimg, mtx, dst, None, mtx)
                cv2.imwrite("undistort.png", undistImg)
                imgmsg=bridge.cv2_to_compressed_imgmsg(undistImg)
                self.pub1.publish(imgmsg)
                cv2.waitKey(3)
            else:
                print("Image1 is None")

    def callback_undistort2(self,_img):
        print("SUBSCRIBE-2")
        try:
            bridge=cv_bridge.CvBridge()
            rawimg=bridge.compressed_imgmsg_to_cv2(_img, "bgr8")
        except Exception as e:
            print(e)
            return
        mtx=[[669.202531, 0, 647.156226],[0, 671.493159, 463.590883], [0,0,1]]
        mtx=np.array(mtx)
        dst=[-0.287414, 0.060847, 0.000121, -0.000953]
        dst=np.array(dst)
        if len(mtx)!=0 and len(dst)!=0:
            if rawimg is not None:
                undistImg=cv2.undistort(rawimg, mtx, dst, None, mtx)
                imgmsg=bridge.cv2_to_compressed_imgmsg(undistImg)
                self.pub2.publish(imgmsg)
                # cv2.imshow('raw_img', rawimg)
                # cv2.imshow('undist_img_2', undistImg)
                cv2.waitKey(3)
            else:
                print("Image2 is None")

    def callback_undistort3(self,_img):
        print("SUBSCRIBE-3")
        try:
            bridge=cv_bridge.CvBridge()
            rawimg=bridge.compressed_imgmsg_to_cv2(_img, "bgr8")
        except Exception as e:
            print(e)
            return
        mtx=[[658.207463, 0, 663.417588], [0, 658.937363, 483.361444], [0,0,1]]
        mtx=np.array(mtx)
        dst=[-0.260287, 0.047087, -0.001265, -0.001767]
        dst=np.array(dst)
        if len(mtx)!=0 and len(dst)!=0:
            if rawimg is not None:
                undistImg=cv2.undistort(rawimg, mtx, dst, None, mtx)
                imgmsg=bridge.cv2_to_compressed_imgmsg(undistImg)
                self.pub3.publish(imgmsg)
                # cv2.imshow('raw_img', rawimg)
                cv2.imshow('undist_img_3', undistImg)
                cv2.waitKey(0)
            else:
                print("Image3 is None")

    def callback_undistort4(self,_img):
        print("SUBSCRIBE-4")
        try:
            bridge=cv_bridge.CvBridge()
            rawimg=bridge.compressed_imgmsg_to_cv2(_img, "bgr8")
        except Exception as e:
            print(e)
            return
        mtx=[[664.061879, 0, 626.968730], [0, 666.885989, 487.863042], [0,0,1]]
        mtx=np.array(mtx)
        dst=[-0.265812, 0.047736, 0.000330, 0.001299]
        dst=np.array(dst)
        if len(mtx)!=0 and len(dst)!=0:
            if rawimg is not None:
                undistImg=cv2.undistort(rawimg, mtx, dst, None, mtx)
                imgmsg=bridge.cv2_to_compressed_imgmsg(undistImg)
                self.pub4.publish(imgmsg)
                # cv2.imshow('raw_img', rawimg)
                # cv2.imshow('undist_img_4', undistImg)
                cv2.waitKey(3)
            else:
                print("Image4 is None")


if __name__=='__main__':
    Undistort()
    cv2.destroyAllWindows()
