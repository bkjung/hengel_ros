#!/usr/bin/env python
import rospy
import io
import time
import picamera
import picamera.array
import cv2
import numpy as numpy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

package="/home/turtleberry/catkin_ws/src/hengel_ros/hengel_camera/"
if __name__=='__main__':
	rospy.init_node('wide_cam_node_1', anonymous=True)
	pub = rospy.Publisher(
			'/wide_cam_1/compressed', CompressedImage, queue_size=3)
	rate = rospy.Rate(10)

	while True:
       	    if rospy.is_shutdown():
    		break
    	    with picamera.PiCamera() as camera:
    	    	camera.resolution=(1280,720)
    	    # camera.framerate=Fraction(1,6)
    	    # camera.shutter_speed=6000000
    	    # camera.exposure_mode='off'
    	    # camera.iso=800
    
    	    	camera.start_preview()
    	    	time.sleep(2)
    	    	with picamera.array.PiRGBArray(camera) as stream:
    	    		camera.capture(stream, format='bgr')
    	    		image=stream.array
    
    	    		bridge=CvBridge()
    	    		msg=bridge.cv2_to_compressed_imgmsg(image)
    	    		pub.publish(msg)
    
    
