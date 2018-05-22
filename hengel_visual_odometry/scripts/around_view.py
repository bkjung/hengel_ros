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

cam_front = cv2.VideoCapture(2)
cam_left = cv2.VideoCapture(1)
cam_right = cv2.VideoCapture(0)
cam_front.set(cv2.CAP_PROP_FRAME_WIDTH, 864)
cam_front.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cam_front.set(cv2.CAP_PROP_FOURCC, int(0x47504A4D))     # COLLECT IMAGE IN MJPG FORM, SOLVE USB HUB BANDWIDTH ISSUE
cam_left.set(cv2.CAP_PROP_FRAME_WIDTH, 864)
cam_left.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cam_left.set(cv2.CAP_PROP_FOURCC, int(0x47504A4D))
cam_right.set(cv2.CAP_PROP_FRAME_WIDTH, 864)
cam_right.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cam_right.set(cv2.CAP_PROP_FOURCC, int(0x47504A4D))

# DEFINE HOMOGRAPHIES

objPts=np.zeros((3,4,2), dtype=np.float32)
objPts[0]=[[450,800],[950,800],[950,300],[450,300]]
objPts[1]=[[950,800],[1180,800],[1180,300],[950,300]]      #left_1
objPts[2]=[[220,800],[450,800],[450,300],[220,300]]

imgPts=np.zeros((3,4,2), dtype=np.float32)
imgPts[0] =[[109.3,306.3],[506,307.7],[417.8,116],[200,116.7]] #front_1
imgPts[1] =[[194.5,399.5],[311.8,317.5],[172.1,226.3],[74.1,258.3]] #left
imgPts[2] =[[284.5,275],[402.8,354.2],[521.1,212],[425.1,183.6]]

for i in range(3):
    for j in range(4):
        objPts[i][j][0]+=200
        objPts[i][j][1]-=100
    objPts[i]=np.array(objPts[i], np.float32)
    imgPts[i]=np.array(imgPts[i], np.float32)

homography_front=cv2.getPerspectiveTransform(imgPts[0], objPts[0])
homography_left= cv2.getPerspectiveTransform(imgPts[1], objPts[1])
homography_right=cv2.getPerspectiveTransform(imgPts[2], objPts[2])

# int_file=cv2.FileStorage("../calibrate_info/Intrinsic.xml", cv2.FILE_STORAGE_READ)
# dist_file=cv2.FileStorage("../calibrate_info/Distortion.xml", cv2.FILE_STORAGE_READ)
# intrin=int_file.getNode("Intrinsic").mat()
# dist=dist_file.getNode("Distortion").mat()
# int_file.release()
# dist_file.release()



def warp_image(image, homography):
    im_out = cv2.warpPerspective(image, homography, (1800,1300))
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
                _, front_img = cam_front.read()
                _, left_img = cam_left.read()
                _, right_img = cam_right.read()

                # cv2.imshow("front", front_img)
                # cv2.imshow("left", left_img)
                # cv2.imshow("right", right_img)

                h,w=front_img.shape[:2]
                # optimalMat, roi = cv2.getOptimalNewCameraMatrix(intrin, dist, (w,h), 1, (w,h))
                # undist_front = cv2.undistort(front_img, intrin, dist, None, intrin)
                # undist_left= cv2.undistort(left_img,intrin, dist, None, intrin)

                init_time = time.time()
                im_front = warp_image(front_img, homography_front).astype('uint8')
                im_left = warp_image(left_img, homography_left).astype('uint8')
                im_right = warp_image(right_img, homography_right).astype('uint8')
                # MULTIPLY WARPED IMAGE, THEN ADD TO BLANK IMAGE
                im_mask_inv, im_mask = find_mask(im_front)
                front_masked = np.multiply(im_front, im_mask_inv).astype('uint8')
                left_masked = np.multiply(im_left, im_mask).astype('uint8')
                right_masked = np.multiply(im_right, im_mask).astype('uint8')
                summed_image = front_masked + left_masked+right_masked
                #summed_image=front_masked
                summed_image=cv2.resize(summed_image, (900,650), interpolation=cv2.INTER_AREA)

                cv2.imshow('warped', summed_image)

                # SEND IMAGE AS ROS imgmsg
                summed_image = bridge.cv2_to_imgmsg(summed_image, "bgr8")
                # rospy.loginfo("images sent")
                # print("Time taken: ", time.time() -init_time)
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
        cam_right.release()
    except KeyboardInterrupt:
        rospy.signal_shutdown("keyboard interrupt")



if __name__ == '__main__':
    try:
        print("start main function")
        imagePublisher()
	#print('sending image')
    except rospy.ROSInterruptException:
        pass
