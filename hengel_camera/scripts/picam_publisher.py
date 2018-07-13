#!/usr/bin/env python
import io
import time
import picamera
import picamera.array
import cv2
import numpy as numpy

stream=io.BytesIO()
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

data=np.fromstring(stream.getvalue(), dtype=np.uint8)
image=cv2.imdecode(data,1)
image=image[:,:,::-1]



# from picamera import PiCamera
# import subprocess
# import cv2
# from cv_bridge import CvBridge

# package="/home/turtleberry/catkin_ws/src/hengel_ros/hengel_camera/data/"
# camera=Picamera()
# cmd = "raspistill -t 0 -o "+package+"img.jpeg -q 100"


# while True:
#     subprocess.call(cmd, shell=True)
#     img = cv2.imread(package+"img.jpeg")
#     bridge=CvBridge()
#     msg=bridge.cv2_to_compressed_imgmsg(img)

