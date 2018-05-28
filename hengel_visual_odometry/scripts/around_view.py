#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
import os
'''
CAMERA NODE RUNNING AT FULL SPEED
NO IMAGE RECORDING
'''

# Node to obtain call camera data. Separate I/O pipeline
rospy.loginfo('Init Cameras...')

cam_bottom = cv2.VideoCapture(1)
cam_middle = cv2.VideoCapture(0)
cam_top = cv2.VideoCapture(2)
cam_bottom.set(cv2.CAP_PROP_FRAME_WIDTH, 864)
cam_bottom.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cam_bottom.set(cv2.CAP_PROP_FOURCC, int(
    0x47504A4D))  # COLLECT IMAGE IN MJPG FORM, SOLVE USB HUB BANDWIDTH ISSUE
cam_middle.set(cv2.CAP_PROP_FRAME_WIDTH, 864)
cam_middle.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cam_middle.set(cv2.CAP_PROP_FOURCC, int(0x47504A4D))
cam_top.set(cv2.CAP_PROP_FRAME_WIDTH, 864)
cam_top.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cam_top.set(cv2.CAP_PROP_FOURCC, int(0x47504A4D))

# DEFINE HOMOGRAPHIES

objPts = np.zeros((3, 4, 2), dtype=np.float32)
objPts[0] = [[300,1800],[700,1800],[700,1400],[300,1400]] #bottom
objPts[1] = [[300,1400],[700,1400],[700, 1000],[300,1000]]  #middle_1
objPts[2] = [[300,1000],[700,1000],[700,600],[300,600]] #top

imgPts = np.zeros((3, 4, 2), dtype=np.float32)
imgPts[0] = [[37,462],[581,444],[452,97],[150,100]]  #bottom_1
imgPts[1] = [[172,352],[417,349],[369,238],[219,239]]  #middle
imgPts[2] = [[228,402],[390,402],[363,333],[255,333]] #top

for i in range(3):
    for j in range(4):
        objPts[i][j][0] += 0
        objPts[i][j][1] -= 500
    objPts[i] = np.array(objPts[i], np.float32)
    imgPts[i] = np.array(imgPts[i], np.float32)

homography_bottom = cv2.getPerspectiveTransform(imgPts[0], objPts[0])
homography_middle = cv2.getPerspectiveTransform(imgPts[1], objPts[1])
homography_top = cv2.getPerspectiveTransform(imgPts[2], objPts[2])

# int_file=cv2.FileStorage("../calibrate_info/Intrinsic.xml", cv2.FILE_STORAGE_READ)
# dist_file=cv2.FileStorage("../calibrate_info/Distortion.xml", cv2.FILE_STORAGE_READ)
# intrin=int_file.getNode("Intrinsic").mat()
# dist=dist_file.getNode("Distortion").mat()
# int_file.release()
# dist_file.release()


def warp_image(image, homography):
    im_out = cv2.warpPerspective(image, homography, (1000,1400))
    return im_out


# EFFICIENT TRUE/FALSE MASKING - NUMPY MASKING
# ALLOWS SIMPLE ADDITION OF PIXELS FOR IMAGE STITCHING
def find_mask(image):
    black_range1 = np.array([0, 0, 0])
    im_mask = (cv2.inRange(image, black_range1, black_range1)).astype('bool')
    im_mask_inv = (1 - im_mask).astype('bool')
    im_mask_inv = np.dstack((im_mask_inv, im_mask_inv, im_mask_inv))
    im_mask = np.dstack((im_mask, im_mask, im_mask))
    return im_mask_inv, im_mask


def imagePublisher():
    try:
        current_time = str(time.time())
        # print("current time:", current_time)
        # path = "/home/snuzero/"+current_time
        # os.mkdir(path)
        warp_pub = rospy.Publisher('around_img', Image, queue_size=1)
        rospy.init_node('around_img_publisher', anonymous=True)
        print("node initialized")
        # rate=rospy.Rate(30)#10hz
        bridge = CvBridge()

        while not rospy.is_shutdown():
            try:
                _, bottom_img  =cam_bottom.read()
                _, middle_img = cam_middle.read()
                _, top_img = cam_top.read()

                cv2.imshow("bottom", bottom_img)
                cv2.imshow("middle", middle_img)
                cv2.imshow("top", top_img)

                h, w = bottom_img.shape[:2]
                # optimalMat, roi = cv2.getOptimalNewCameraMatrix(intrin, dist, (w,h), 1, (w,h))
                # undist_bottom = cv2.undistort(bottom_img, intrin, dist, None, intrin)
                # undist_middle= cv2.undistort(middle_img,intrin, dist, None, intrin)

                init_time = time.time()
                im_bottom = warp_image(bottom_img,
                                     homography_bottom).astype('uint8')
                im_middle = warp_image(middle_img, homography_middle).astype('uint8')
                im_top = warp_image(top_img,
                                      homography_top).astype('uint8')
                # MULTIPLY WARPED IMAGE, THEN ADD TO BLANK IMAGE
                im_mask_inv, im_mask = find_mask(im_middle)
                bottom_masked = np.multiply(im_bottom,       im_mask).astype('uint8')
                middle_masked = np.multiply(im_middle, im_mask_inv).astype('uint8')
                top_masked = np.multiply(im_top, im_mask).astype('uint8')
                summed_image = bottom_masked + middle_masked + top_masked
                #summed_image = im_bottom+im_middle+im_top
                summed_image = cv2.resize(
                    summed_image, (500,700), interpolation=cv2.INTER_AREA)

                cv2.imshow('warped', summed_image)

                # SEND IMAGE AS ROS imgmsg
                summed_image = bridge.cv2_to_imgmsg(summed_image, "bgr8")
                # rospy.loginfo("images sent")
                # print("Time taken: ", time.time() -init_time)
                k = cv2.waitKey(1) & 0xFF
                if k == 27:
                    break

                warp_pub.publish(summed_image)
            except KeyboardInterrupt:
                rospy.signal_shutdown("keyboard interrupt")
                break

        cv2.destroyAllWindows()
        cam_bottom.release()
        cam_middle.release()
        cam_top.release()
    except KeyboardInterrupt:
        rospy.signal_shutdown("keyboard interrupt")


if __name__ == '__main__':
    try:
        print("start main function")
        imagePublisher()
#print('sending image')
    except rospy.ROSInterruptException:
        pass
