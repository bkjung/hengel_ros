#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from sensor_msgs.msg import Image, CompressedImage
<<<<<<< HEAD
=======
from markRobotView import RobotView
>>>>>>> 7deec31f1372df487ec280ec50e51328a7e96ebe
from math import radians, copysign, sqrt, pow, pi, atan2, sin, floor, cos, asin
import numpy as np
import sys
import time
import os
import cv2
from cv_bridge import CvBridge
import message_filters
import collections
<<<<<<< HEAD
=======
from feature_match import FeatureMatch
>>>>>>> 7deec31f1372df487ec280ec50e51328a7e96ebe
from matplotlib import pyplot as plt

class VisualCompensation():
    def __init__(self, _num_pts_delete):
        self.initialize()

    def initialize(self):
        rospy.init_node('hengel_camera_compensation', anonymous=False)

<<<<<<< HEAD
        self.bridge= CvBridge()
=======
>>>>>>> 7deec31f1372df487ec280ec50e51328a7e96ebe
        rospy.Subscriber('/genius1/compressed', CompressedImage, self.undistort1)
        rospy.Subscriber('/genius2/compressed', CompressedImage, self.undistort2)
        rospy.Subscriber('/genius3/compressed', CompressedImage, self.undistort3)
        rospy.Subscriber('/genius4/compressed', CompressedImage, self.undistort4)
        rospy.Subscriber('/usb_cam3/image_raw/compressed', CompressedImage, self.undistort_left)
        rospy.Subscriber('/usb_cam4/image_raw/compressed', CompressedImage, self.undistort_right)



        ############################ DEBUG ################################
        self.pub_time_1=rospy.Publisher('/time1', Float32, queue_size=5)
        self.pub_time_2=rospy.Publisher('/time2', Float32, queue_size=5)
        self.pub1=rospy.Publisher('/genius1/undist', Image, queue_size=5 )
        self.pub2=rospy.Publisher('/genius2/undist', Image, queue_size=5 )
        self.pub3=rospy.Publisher('/genius3/undist', Image, queue_size=5 )
        self.pub4=rospy.Publisher('/genius4/undist', Image, queue_size=5 )
        self.pub_pi_l=rospy.Publisher('/pi_left/undist', Image, queue_size=5 )
        self.pub_pi_r=rospy.Publisher('/pi_right/undist', Image, queue_size=5 )



        rospy.spin()


    def undistort1(self, _img):
        print("undistort1")
        img=self.bridge.compressed_imgmsg_to_cv2(_img)
        mtx=np.array([[393.8666817683925, 0.0, 399.6813895086665], [0.0, 394.55108358870405, 259.84676565717876], [0.0, 0.0, 1.0]])
        dst=np.array([-0.0032079005049939543, -0.020856072501002923, 0.000252242294186179, -0.0021042704510431365])

        #################DEBUG#######################333
        undist=cv2.undistort(img, mtx, dst, None, mtx)
        cv2.imwrite("/home/hengel/undist_for_homography/genius1/undist_"+str(time.time())+".png",undist)

    def undistort2(self, _img):
        img=self.bridge.compressed_imgmsg_to_cv2(_img)
        mtx=np.array([[396.01900903941834, 0.0, 410.8496405295566], [0.0, 396.2406539134792, 285.8932176591904], [0.0, 0.0, 1.0]])
        dst=np.array([-0.008000340519517233, -0.016478659972026452, 7.25792172844022e-05, -0.00434319738405187])

        #################DEBUG#######################333
        undist=cv2.undistort(img, mtx, dst, None, mtx)
        cv2.imwrite("/home/hengel/undist_for_homography/genius2/undist_"+str(time.time())+".png",undist)
        # imgmsg=self.bridge.cv2_to_imgmsg(undist)
        # self.pub2.publish(imgmsg)

        # homo2= np.array([[-2.36547415e+00,  4.44589419e+00,  1.65240597e+03],
        #     [-1.11902669e-02,  2.88055561e+00,  2.03902843e+03],
        #     [-5.36747061e-06,  6.70728023e-03,  1.00000000e+00]])
        # return cv2.warpPerspective( cv2.undistort(img, mtx, dst,None, mtx) , homo2, (1280,1280))

    def undistort3(self, _img):
        img=self.bridge.compressed_imgmsg_to_cv2(_img)
        mtx=np.array([[387.8191999285985, 0.0, 392.3078288789019],[ 0.0, 382.1093651210362, 317.43368009853674], [0.0, 0.0, 1.0]])
        dst=np.array([-0.008671221810333559, -0.013546386893040543, -0.00016537575030651431, 0.002659594999360673])


        #################DEBUG#######################333
        undist=cv2.undistort(img, mtx, dst, None, mtx)
        cv2.imwrite("/home/hengel/undist_for_homography/genius3/undist_"+str(time.time())+".png",undist)
        # imgmsg=self.bridge.cv2_to_imgmsg(undist)
        # self.pub3.publish(imgmsg)

        # homo3= np.array([[ 2.55132452e-01,  9.82372337e+00,  4.09600642e+03],
        #     [ 6.45201391e+00,  1.30885948e+01, -1.66201249e+03],
        #     [ 3.88669729e-04,  2.00259308e-02,  1.00000000e+00]])
        # return cv2.warpPerspective( cv2.undistort(img, mtx, dst,None, mtx) , homo3, (1280,1280))


    def undistort4(self, _img):
        img=self.bridge.compressed_imgmsg_to_cv2(_img)
        mtx=np.array([[384.2121883964654, 0.0, 423.16727407803353], [0.0, 386.8188468139677, 359.5190506678551], [0.0, 0.0, 1.0]])
        dst=np.array([-0.0056866549555025896, -0.019460881544303938, 0.0012937686026747307, -0.0031999317338443087])


        #################DEBUG#######################333
        undist=cv2.undistort(img, mtx, dst, None, mtx)
        cv2.imwrite("/home/hengel/undist_for_homography/genius4/undist_"+str(time.time())+".png",undist)
        # imgmsg=self.bridge.cv2_to_imgmsg(undist)
        # self.pub4.publish(imgmsg)


        # homo4= np.array([[ 2.57420243e+00,  5.85803823e+00, -4.05003547e+02],
        #     [-1.15034759e-01,  7.22474987e+00, -7.29546146e+02],
        #     [-1.92621119e-04,  8.88963498e-03,  1.00000000e+00]])
        # return cv2.warpPerspective( cv2.undistort(img, mtx, dst,None, mtx) , homo4, (1280,1280))

    def undistort_left(self, _img):
        img=self.bridge.compressed_imgmsg_to_cv2(_img)
        mtx=np.array([[496.88077412085187, 0.0, 486.19161191113693], [0.0, 497.77308359203073, 348.482250144119], [0.0, 0.0, 1.0]])
        dst=np.array([-0.27524035766660704, 0.055346669640229516, 0.002041430748143387, -0.0012188333190676689])

        #################DEBUG#######################333
        undist=cv2.undistort(img, mtx, dst, None, mtx)
        cv2.imwrite("/home/hengel/undist_for_homography/pi_l/undist_"+str(time.time())+".png",undist)
        # imgmsg=self.bridge.cv2_to_imgmsg(undist)
        # self.pub_pi_l.publish(imgmsg)


    def undistort_right(self, _img):
        img=self.bridge.compressed_imgmsg_to_cv2(_img)
        mtx=np.array([[494.0169295185964, 0.0, 483.6710483879246], [0.0, 495.87509303786857, 336.69262125267153], [0.0, 0.0, 1.0]])
        dst=np.array([-0.26693726936305806, 0.05239559897759021, 0.0024912074565555443, -0.0015904998174301696])

        #################DEBUG#######################333
        undist=cv2.undistort(img, mtx, dst, None, mtx)
        cv2.imwrite("/home/hengel/undist_for_homography/pi_r/undist_"+str(time.time())+".png",undist)
        # imgmsg=self.bridge.cv2_to_imgmsg(undist)
            # self.pub_pi_r.publish(imgmsg)

if __name__=='__main__':
    num_pts_delete = 150 #num_of_waypoints_to_delete_in_virtualmap_after_compensation
    VisualCompensation(num_pts_delete)
