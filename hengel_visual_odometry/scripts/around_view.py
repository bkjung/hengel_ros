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

# define values boundaries for color
lower_yellow = np.array([15,50,100],np.uint8)
upper_yellow = np.array([40,255,255],np.uint8)
lower_white_hsv = np.array([0,0,120], np.uint8)
upper_white_hsv = np.array([255,30,255], np.uint8)

lower_white_rgb = np.array([190,190,190], np.uint8)
upper_white_rgb = np.array([255,255,255], np.uint8)


# Node to obtain call camera data. Separate I/O pipeline
rospy.loginfo('Init Cameras...')
cam_front = cv2.VideoCapture(0)
cam_left = cv2.VideoCapture(2)
# cam_right = cv2.VideoCapture(3)
cam_front.set(cv2.CAP_PROP_FRAME_WIDTH, 864)
cam_front.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cam_front.set(cv2.CAP_PROP_FOURCC, int(0x47504A4D))     # COLLECT IMAGE IN MJPG FORM, SOLVE USB HUB BANDWIDTH ISSUE
cam_left.set(cv2.CAP_PROP_FRAME_WIDTH, 864)
cam_left.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cam_left.set(cv2.CAP_PROP_FOURCC, int(0x47504A4D))
# cam_right.set(cv2.CAP_PROP_FRAME_WIDTH, 864)
# cam_right.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
# cam_right.set(cv2.CAP_PROP_FOURCC, int(0x47504A4D))

# DEFINE HOMOGRAPHIES
# homography_front = np.array([[1.20578316901691e-05, 0.000329407217584187, -0.654631511346573],
#         [0.000956007807820993, -0.00231196866646363, -0.755942976367266],
#         [4.59523223437821e-08, -7.58289486150618e-06, -0.00119706768797383]])

homography_front = np.array([[4.21260078, 5.50168937,-623.515151],
        [-0.056711486, 12.9895610,-381.409009],
        [1.47985693e-05, 7.72959730e-03, 1.0000000e+00]])
# homography_left = np.array([[0.000718031591780952, -0.000165635757963769, 0.0810412365295545],
#         [-0.000457696149644549, 0.00197605152199676, 0.996708002646204],
#         [3.29970074027985e-07, 7.21616117600935e-06, 0.000904500215204792]])
homography_left=np.array([[5.44147964e+00, 9.17160302e+00, 1.64443801e+03],[5.28461989e+00, 2.49527779e+01, -2.88454272e+03],[-3.80122568e-04, 1.55170299e-02, 1.00000e+00]])
homography_right = np.array([[0.000915698708180926, 0.000160507016324320, -0.989937462884797],
        [0.000940331110058012, -0.00341128707549284, -0.141453165043701],
        [2.16613893241818e-07, -1.07504356915543e-05, -0.00119841227368443]])

int_file=cv2.FileStorage("../calibrate_info/Intrinsic.xml", cv2.FILE_STORAGE_READ)
dist_file=cv2.FileStorage("../calibrate_info/Distortion.xml", cv2.FILE_STORAGE_READ)
intrin=int_file.getNode("Intrinsic").mat()
dist=dist_file.getNode("Distortion").mat()
int_file.release()
dist_file.release()



def warp_image(image, homography):
    im_out = cv2.warpPerspective(image, homography, (1400,1500))
    return im_out

# EFFICIENT TRUE/FALSE MASKING - NUMPY MASKING
# ALLOWS SIMPLE ADDITION OF PIXELS FOR IMAGE STITCHING
def find_mask(image):
    black_range1 = np.array([0,0,0])
    im_mask = (cv2.inRange(image, black_range1, black_range1)).astype('bool')
    im_mask_inv = (1-im_mask).astype('bool')
    im_mask_inv = np.dstack((im_mask_inv, im_mask_inv, im_mask_inv))
    im_mask= np.dstack((im_mask, im_mask, im_mask))
    return im_mask_inv, im_mask


def imagePublisher():
    try:
        current_time = str(time.time())
        print("current time:", current_time)
        # path = "/home/snuzero/"+current_time
        # os.mkdir(path)
        warp_pub = rospy.Publisher('around_img', Image, queue_size=1)
        rospy.init_node('around_img_publisher', anonymous=True)
        print("node initialized")
        #rate=rospy.Rate(30)#10hz
        bridge = CvBridge()

        while not rospy.is_shutdown():
            try:
                _, front_img = cam_front.read()
                _, left_img = cam_left.read()

                cv2.imshow("front_distort", front_img)
                cv2.imshow("left_distort", left_img)
                h,w=front_img.shape[:2]
                # optimalMat, roi = cv2.getOptimalNewCameraMatrix(intrin, dist, (w,h), 1, (w,h))
                # undist_front = cv2.undistort(front_img, intrin, dist, None, intrin)
                # undist_left= cv2.undistort(left_img,intrin, dist, None, intrin) 

                # cv2.imshow("front_undist", undist_front)
                # cv2.imshow("left_undist", undist_left)
                
                # _, right_img = cam_right.read()
                print("cam read")
                init_time = time.time()
                im_front = warp_image(front_img, homography_front).astype('uint8')
                im_left = warp_image(left_img, homography_left).astype('uint8')

                # im_right = warp_image(right_img, homography_right).astype('uint8')
                print("image warping done")
                # MULTIPLY WARPED IMAGE, THEN ADD TO BLANK IMAGE
                im_mask_inv, im_mask = find_mask(im_front)
                front_masked = np.multiply(im_front, im_mask_inv).astype('uint8')
                left_masked = np.multiply(im_left, im_mask).astype('uint8')
                # right_masked = np.multiply(im_right, im_mask).astype('uint8')
                # summed_image = front_masked + left_masked+right_masked
                summed_image=front_masked+left_masked
                summed_image=cv2.resize(summed_image, (700,750), interpolation=cv2.INTER_AREA)

                cv2.imshow('warped', summed_image)
                
                # SEND IMAGE AS ROS imgmsg
                summed_image = bridge.cv2_to_imgmsg(summed_image, "bgr8")
                rospy.loginfo("images sent")
                print("Time taken: ", time.time() -init_time)
                k = cv2.waitKey(1) & 0xFF
                if k ==27:
                    break

                warp_pub.publish(summed_image)
            except KeyboardInterrupt:
                rospy.signal_shutdown("keyboard interrupt")
                break

        cv2.destroyAllWindows()
        cam_front.release()
        cam_left.release()
        # cam_right.release()
    except KeyboardInterrupt:
        rospy.signal_shutdown("keyboard interrupt")



if __name__ == '__main__':
    try:
        print("start main function")
        imagePublisher()
	#print('sending image')
    except rospy.ROSInterruptException:
        pass