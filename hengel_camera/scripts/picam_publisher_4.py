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
from fractions import Fraction 

class rePublish():
    def __init__(self):
	rospy.init_node('wide_cam_node_4', anonymous=True)
	self.pub = rospy.Publisher('/wide_cam_4/compressed', CompressedImage, queue_size=3)
	rospy.Subscriber('/usb_cam/image_raw/compressed', CompressedImage, self.wideCamPublish)
	self.rate = rospy.Rate(10)
	rospy.spin()

	#self.publish()

    def publish(self):
	_time=time.time()
	while True:
       	    if rospy.is_shutdown():
    		break
    	    with picamera.PiCamera() as camera:
		time1=time.time()
    	    	camera.resolution=(1280,720)
		#camera.framerate=50
            #camera.framerate=Fraction(1,6)
    	    # camera.shutter_speed=6000000
    	    # camera.exposure_mode='off'
    	    # camera.iso=800
    
    	    	camera.start_preview()
    	    	with picamera.array.PiRGBArray(camera) as stream:
    	    		camera.capture(stream, format='bgr')
    	    		image=stream.array
    
    	    		bridge=CvBridge()
    	    		msg=bridge.cv2_to_compressed_imgmsg(image)
    	    		self.pub.publish(msg)
			print(time.time()-time1)

    def wideCamPublish(self, cmpImg):
	self.pub.publish(cmpImg)
	

package="/home/turtleberry/catkin_ws/src/hengel_ros/hengel_camera/"
if __name__=='__main__':
    rospy.init_node('wide_cam_node_4', anonymous=True)
    rePublish()

