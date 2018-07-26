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
from std_msgs.msg import Bool

package="/home/mjlee/catkin_ws/src/hengel_ros/hengel_camera/"

class CamPublisher():
	def __init__(self):
		rospy.init_node('genius_publisher2', anonymous=True)

		
		self.cam=cv2.VideoCapture(0)
		self.cam2=cv2.VideoCapture(1)
		self.cam.set(cv2.CAP_PROP_FRAME_WIDTH,960)
		self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 540)
		self.cam.set(cv2.CAP_PROP_FOURCC, int(0x47504A4D))
		self.cam2.set(cv2.CAP_PROP_FRAME_WIDTH,960)
		self.cam2.set(cv2.CAP_PROP_FRAME_HEIGHT, 540)
		self.cam2.set(cv2.CAP_PROP_FOURCC, int(0x47504A4D))

		rospy.Subscriber('/initiator', Bool, self.initiator)
		self.pub = rospy.Publisher('/genius3/compressed', CompressedImage, queue_size=10)
		self.pub2= rospy.Publisher('/genius4/compressed', CompressedImage, queue_size=10)
		self.rate = rospy.Rate(5)

		rospy.spin()


	def initiator(self, msg):
		self.publish()


	def publish(self):
		print("pi2 initiated, time: "+str(time.time()))
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
			_, img=self.cam.read()
			_, img2=self.cam2.read()

			msg1=bridge.cv2_to_compressed_imgmsg(img)
			msg2=bridge.cv2_to_compressed_imgmsg(img2)
			self.pub.publish(msg1)
			self.pub2.publish(msg2)
			self.rate.sleep()

if __name__=='__main__':
	CamPublisher()


