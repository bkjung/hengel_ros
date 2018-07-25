#!/usr/bin/env python
import rospy
import io
import time
#import picamera
#import picamera.array
import cv2
import numpy as numpy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

package="/home/mjlee/catkin_ws/src/hengel_ros/hengel_camera/"
if __name__=='__main__':
	rospy.init_node('genius_publisher_2', anonymous=True)
	pub = rospy.Publisher('/genius1/compressed', CompressedImage, queue_size=1)
	pub2= rospy.Publisher('/genius2/compressed', CompressedImage, queue_size=1)
	rate = rospy.Rate(10)
	
	cam=cv2.VideoCapture(0)
	#cam2=cv2.VideoCapture(1)
	cam.set(cv2.CAP_PROP_FRAME_WIDTH,960)
	cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 540)
	cam.set(cv2.CAP_PROP_FOURCC, int(0x47504A4D))
	#cam2.set(cv2.CAP_PROP_FRAME_WIDTH,960)
	#cam2.set(cv2.CAP_PROP_FRAME_HEIGHT, 540)
	#cam2.set(cv2.CAP_PROP_FOURCC, int(0x47504A4D))

	while True:
       	    if rospy.is_shutdown():
    		break
    	    #with picamera.PiCamera() as camera:
    	    #	camera.resolution=(1280,720)
    	    ## camera.framerate=Fraction(1,6)
    	    ## camera.shutter_speed=6000000
    	    ## camera.exposure_mode='off'
    	    ## camera.iso=800

    	    #	camera.start_preview()
    	    #	time.sleep(2)
    	    #	with picamera.array.PiRGBArray(camera) as stream:
    	    #		camera.capture(stream, format='bgr')
    	    #		image=stream.array

    	    #		bridge=CvBridge()
    	    #		msg=bridge.cv2_to_compressed_imgmsg(image)
    	    #		pub.publish(msg)
	    bridge=CvBridge()
	    if cam.isOpened():
	        _, img=cam.read()
	        msg1=bridge.cv2_to_compressed_imgmsg(img)
	        #cv2.imshow("cam", img)
	    #cv2.imshow("cam2",img2)
	        pub.publish(msg1)
	    #if cam2.isOpened():
	    #    _, img2=cam2.read()
	    #    msg2=bridge.cv2_to_compressed_imgmsg(img2)
	    #    pub2.publish(msg2)
	    cv2.waitKey(3)
