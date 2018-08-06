#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from sensor_msgs.msg import Image, CompressedImage
from markRobotView import RobotView
from math import radians, copysign, sqrt, pow, pi, atan2, sin, floor, cos, asin
import numpy as np
import sys
import time
import os
import copy
from os.path import expanduser
import cv2
from cv_bridge import CvBridge
import message_filters
import collections
from feature_match import FeatureMatch
from matplotlib import pyplot as plt
from hengel_camera.msg import CmpImg

class VisualCompensation():
    def __init__(self, _num_pts_delete):
        word= raw_input("WHAT IS THE WIDTH AND HEIGHT OF CANVAS?\n Type: ")
        self.width=float(word.split()[0])
        self.height=float(word.split()[1])
        self.num_pts_delete = _num_pts_delete
        self.recent_pts = collections.deque(self.num_pts_delete*[(0.0,0.0)],self.num_pts_delete)

        self.home_path = expanduser("~")
        self.folder_path = self.home_path + "/FEATURE_MATCH/" + time.strftime("%y%m%d_%H%M%S")
        os.system("mkdir -p " + self.folder_path)

        self.initialize()


    def initialize(self):
        rospy.init_node('hengel_camera_compensation', anonymous=False)

        self.bridge=CvBridge()
        self.pixMetRatio=500

        self.virtual_map=np.full((int(self.pixMetRatio*self.height), int(self.pixMetRatio*self.width)), 255)
        self.pi_left_img=np.array([])
        self.pi_right_img=np.array([])

        self.app_robotview=RobotView(self.virtual_map) # Add the endpoint into the virtual map

        self.mid_predict_canvas_x=0
        self.mid_predict_canvas_y=0
        self.mid_predict_canvas_th=0

        self.endPoint_callback=message_filters.Subscriber('/endpoint', Point)
        self.midPoint_callback=message_filters.Subscriber('/midpoint', Point)

        self.ts=message_filters.ApproximateTimeSynchronizer([self.endPoint_callback, self.midPoint_callback], 10, 0.1, allow_headerless=True)
        self.ts.registerCallback(self.sync_virtual_callback)

        self.pub_virtual_map=rospy.Publisher('/virtual_map', CompressedImage, queue_size=3)
        self.vision_offset_publisher = rospy.Publisher('/offset_change', Point, queue_size=10)
        self.callback1=message_filters.Subscriber('/genius1/compressed', CompressedImage)
        self.callback2=message_filters.Subscriber('/genius2/compressed', CompressedImage)
        self.callback3=message_filters.Subscriber('/genius3/compressed', CompressedImage)
        self.callback4=message_filters.Subscriber('/genius4/compressed', CompressedImage)

        self.callback34=message_filters.Subscriber('/pi3_imgs/compressed', CmpImg)

        #self.callback_pi_left=message_filters.Subscriber('/usb_cam3/image_raw/compressed', CompressedImage)
        #self.callback_pi_right=message_filters.Subscriber('/usb_cam4/image_raw/compressed', CompressedImage)

        rospy.Subscriber('/usb_cam3/image_raw/compressed', CompressedImage, self.callback_left)
        rospy.Subscriber('/usb_cam4/image_raw/compressed', CompressedImage, self.callback_right)

        #self.ts=message_filters.ApproximateTimeSynchronizer([self.callback1, self.callback2, self.callback3, self.callback4, self.callback_pi_left, self.callback_pi_right ], 10, 0.1, allow_headerless=True)

        #self.ts=message_filters.ApproximateTimeSynchronizer([self.callback1, self.callback2, self.callback34], 10,0.1, allow_headerless=True)
        self.ts=message_filters.ApproximateTimeSynchronizer([self.callback1, self.callback2, self.callback3, self.callback4], 10,0.1, allow_headerless=True)
        self.ts.registerCallback(self.sync_real_callback)

        ############################ DEBUG ################################
        # self.pub1=rospy.Publisher('/genius1/undist', Image, queue_size=5 )
        # self.pub2=rospy.Publisher('/genius2/undist', Image, queue_size=5 )
        # self.pub3=rospy.Publisher('/genius3/undist', Image, queue_size=5 )
        # self.pub4=rospy.Publisher('/genius4/undist', Image, queue_size=5 )
        # self.pub_pi_l=rospy.Publisher('/pi_left/undist', Image, queue_size=5 )
        # self.pub_pi_r=rospy.Publisher('/pi_right/undist', Image, queue_size=5 )
        self.pub_sum=rospy.Publisher('/summed_image/compressed',CompressedImage, queue_size=5)



        rospy.spin()

    def callback_left(self, _img):
        print("left")
        self.pi_left_img=self.undistort_left(_img)

    def callback_right(self, _img):
        print("right")
        self.pi_right_img=self.undistort_right(_img)


#    def sync_real_callback(self, _img1, _img2, _img3, _img4, _img_left, _img_right):
    def sync_real_callback(self, _img1, _img2, _img3, _img4):
    #def sync_real_callback(self, _img1, _img2, _img34):
        _time=time.time()
        print("sync real")
        img1 = self.undistort1(_img1)
        img2 = self.undistort2(_img2)
        img3 = self.undistort3(_img3)
        img4 = self.undistort4(_img4)
        print("img3: "+str(_img3.header.stamp)+", img4: "+str(_img4.header.stamp))
        #img_left=self.undistort_left(_img_left)
        #img_right=self.undistort_right(_img_right)
        while len(self.pi_left_img)==0 or len(self.pi_right_img)==0:
            time.sleep(100)

        img_left=copy.deepcopy(self.pi_left_img)
        img_right=copy.deepcopy(self.pi_right_img)

        im_mask_inv1, im_mask1=self.find_mask(img1)
        im_mask_inv3, im_mask3=self.find_mask(img3)
        _, im_mask2=self.find_mask(img2)
        _, im_mask4=self.find_mask(img4)
        _, im_mask_l=self.find_mask(img_left)
        _, im_mask_r=self.find_mask(img_right)

        img_white=np.full((1280, 1280), 255)

        im_mask13=cv2.bitwise_and(np.array(im_mask1), np.array(im_mask3))
        im_mask24=cv2.bitwise_and(np.array(im_mask2), np.array(im_mask4))
        im_mask1234=cv2.bitwise_and(im_mask13, im_mask24)

        img_white_masked=np.multiply(np.multiply(np.multiply(img_white, im_mask1234), im_mask_l), im_mask_r)
        img2_masked=np.multiply(np.multiply(img2, im_mask13), im_mask4)
        img4_masked=np.multiply(np.multiply(img4, im_mask13), im_mask2)
        img1_masked=np.multiply(img1, im_mask_inv1)
        img3_masked=np.multiply(img3, im_mask_inv3)
        img_left_masked=np.multiply(np.multiply(img_left, im_mask1234), im_mask_r)
        img_right_masked=np.multiply(np.multiply(img_right, im_mask1234), im_mask_l)
        # cv2.imwrite("/home/hengel/left_masked.png", img_left_masked)
        # cv2.imwrite("/home/hengel/left.png", img2)

        summed_image=img1_masked+img2_masked+img3_masked+img4_masked+img_white_masked+img_left_masked+img_right_masked
        summed_msg=self.bridge.cv2_to_compressed_imgmsg(summed_image)

        self.pub_sum.publish(summed_msg)
        print("summed_image time: "+str(time.time()-_time))

        # self.crop_image(summed_image)

        #################
        try:
            fm = FeatureMatch(self.folder_path)
            # print("img1: "+str(self.virtual_map.shape)+", img2: "+str(summed_image.shape))
            if self.virtual_map is None or summed_image is None:
                print("IMAGE EMPTY")
                raise Exeption("Image Empty")
            else:
                # print("img1 "+str(self.virtual_map.dtype)+" "+"img2 "+str(summed_image.dtype))
                _img1 = np.uint8(self.virtual_map)
                _img2 = np.uint8(summed_image)

                # _img1 = cv2.cvtColor(_img1, cv2.COLOR_BGR2GRAY)
                # _img2 = cv2.cvtColor(_img2, cv2.COLOR_BGR2GRAY)



                M = fm.SIFT_FLANN_matching(_img1, _img2)
                if fm.status == True:
                    self.vision_offset_publisher.publish(Point(fm.delta_x, fm.delta_y, fm.delta_theta))
                    self.app_robotview.remove_points_during_vision_compensation(self.recent_pts)
                    self.virtual_map = self.app_robotview.img

                    #Initialize Queue
                    self.recent_pts = collections.deque(self.num_pts_delete*[(0.0,0.0)],self.num_pts_delete)
        except Exception as e:
            print(e)
            sys.exit("Feature Match error")

        #################

        print("Cam Input -> Visual Calc / Total Time: "+str(time.time()-_time))

        # bridge=CvBridge()
        # summed_msg=bridge.cv2_to_compressed_imgmsg(summed_image)
        # self.sum_pub.publish(summed_msg)

    def crop_image(self, _img):
        mid_predict_img_x = self.mid_predict_canvas_x * self.pixMetRatio
        mid_predict_img_y = _img.shape[0] - self.mid_predict_canvas_y * self.pixMetRatio
        mid_predict_img_th = self.mid_predict_canvas_th
        half_map_size = 125

        imgPts=np.array([[0,0], [0, 1280], [1280, 1280], [1280,0]])
        obsPts=np.array([[mid_predict_img_x-half_map_size*sin(mid_predict_img_th), mid_predict_img_y-cos(mid_predict_img_th)],
                        [mid_predict_img_x+half_map_size]])

    def find_mask(self, img):
        print(img.shape)
        _time=time.time()
        black_range1=np.array([0])
        im_mask=(cv2.inRange(img, black_range1, black_range1))
        # im_mask=np.dstack((im_mask, im_mask, im_mask))
        im_mask_inv=(1-im_mask)
        return im_mask_inv, im_mask

    def sync_virtual_callback(self, _endPoint, _midPoint):
        # print("sync virtual")
        _time=time.time()

        self.mid_predict_canvas_x=_midPoint.x
        self.mid_predict_canvas_y=_midPoint.y
        self.mid_predict_canvas_th=_midPoint.z

        self.recent_pts.appendleft((_midPoint.x, _midPoint.y))

        # self.virtual_map = self.app_robotview.run(_midPoint, _endPoint)
        self.app_robotview.run(_midPoint, _endPoint)
        self.virtual_map = self.app_robotview.img
        ttime=Float32()
        ttime.data=float(time.time()-_time)


        #PUBLISHING VIRTUAL MAP, but currently the msg cannot be viewed at rqt (supposedly because of msgtype mismatch)
        # virtual_map_msg=self.bridge.cv2_to_compressed_imgmsg(self.virtual_map)
        # self.pub_virtual_map.publish(virtual_map_msg)

    def undistort1(self, _img):
        img=self.bridge.compressed_imgmsg_to_cv2(_img)
        img=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        mtx=np.array([[393.8666817683925, 0.0, 399.6813895086665], [0.0, 394.55108358870405, 259.84676565717876], [0.0, 0.0, 1.0]])
        dst=np.array([-0.0032079005049939543, -0.020856072501002923, 0.000252242294186179, -0.0021042704510431365])

        # #################DEBUG#######################333
        # undist=cv2.undistort(img, mtx, dst, None, mtx)
        # cv2.imwrite("/home/bkjung/undist_for_homography/genius1/undist_"+str(time.time())+".png",undist)

        # imgmsg=self.bridge.cv2_to_imgmsg(undist)
        # self.pub1.publish(imgmsg)

        homo1= np.array([[-1.99133283e+00,  3.96335542e+00,  1.43529996e+03],
            [ 1.26013904e-02,  3.20665968e+00,  1.58076675e+03],
            [-5.52616807e-05,  6.56931832e-03,  1.00000000e+00]])
        return cv2.warpPerspective( cv2.undistort(img, mtx, dst,None, mtx) , homo1, (1280,1280))





    def undistort2(self, _img):
        img=self.bridge.compressed_imgmsg_to_cv2(_img)
        img=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        mtx=np.array([[396.01900903941834, 0.0, 410.8496405295566], [0.0, 396.2406539134792, 285.8932176591904], [0.0, 0.0, 1.0]])
        dst=np.array([-0.008000340519517233, -0.016478659972026452, 7.25792172844022e-05, -0.00434319738405187])

        # #################DEBUG#######################333
        # undist=cv2.undistort(img, mtx, dst, None, mtx)
        # cv2.imwrite("/home/bkjung/undist_for_homography/genius2/undist_"+str(time.time())+".png",undist)
        # imgmsg=self.bridge.cv2_to_imgmsg(undist)
        # self.pub2.publish(imgmsg)

        homo2= np.array([[-1.74979870e-01,  2.54748382e+00,  1.50601223e+03],
            [ 1.58754865e+00,  3.68531885e+00, -7.91871465e+01],
            [-2.64765872e-04,  5.74538321e-03,  1.00000000e+00]])
        return cv2.warpPerspective( cv2.undistort(img, mtx, dst,None, mtx) , homo2, (1280,1280))

    def undistort3(self, _img):
        img=self.bridge.compressed_imgmsg_to_cv2(_img)
        img=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        mtx=np.array([[387.8191999285985, 0.0, 392.3078288789019],[ 0.0, 382.1093651210362, 317.43368009853674], [0.0, 0.0, 1.0]])
        dst=np.array([-0.008671221810333559, -0.013546386893040543, -0.00016537575030651431, 0.002659594999360673])


        # #################DEBUG#######################333
        # undist=cv2.undistort(img, mtx, dst, None, mtx)
        # cv2.imwrite("/home/bkjung/undist_for_homography/genius3/undist_"+str(time.time())+".png",undist)
        # imgmsg=self.bridge.cv2_to_imgmsg(undist)
        # self.pub3.publish(imgmsg)

        homo3= np.array([[ 2.04906162e+00,  3.92623252e+00, -1.90107542e+02],
            [ 1.64193237e-02,  5.25578879e+00, -4.40829402e+02],
            [ 1.60923552e-05,  6.47353214e-03,  1.00000000e+00]])


        return cv2.warpPerspective( cv2.undistort(img, mtx, dst,None, mtx) , homo3, (1280,1280))


    def undistort4(self, _img):
        img=self.bridge.compressed_imgmsg_to_cv2(_img)
        img=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        mtx=np.array([[384.2121883964654, 0.0, 423.16727407803353], [0.0, 386.8188468139677, 359.5190506678551], [0.0, 0.0, 1.0]])
        dst=np.array([-0.0056866549555025896, -0.019460881544303938, 0.0012937686026747307, -0.0031999317338443087])


        #################DEBUG#######################333
        # undist=cv2.undistort(img, mtx, dst, None, mtx)
        # cv2.imwrite("/home/bkjung/undist_for_homography/genius4/undist_"+str(time.time())+".png",undist)
        # imgmsg=self.bridge.cv2_to_imgmsg(undist)
        # self.pub4.publish(imgmsg)


        homo4= np.array([[ 2.28317132e-01,  1.40100840e+01, -2.33867485e+03],
            [-5.15991902e+00,  1.17501321e+01,  3.00914222e+03],
            [ 5.65609486e-04,  1.83615756e-02,  1.00000000e+00]])
        return cv2.warpPerspective( cv2.undistort(img, mtx, dst,None, mtx) , homo4, (1280,1280))

    def undistort_left(self, _img):
        img=self.bridge.compressed_imgmsg_to_cv2(_img)
        img=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        mtx=np.array([[496.88077412085187, 0.0, 486.19161191113693], [0.0, 497.77308359203073, 348.482250144119], [0.0, 0.0, 1.0]])
        dst=np.array([-0.27524035766660704, 0.055346669640229516, 0.002041430748143387, -0.0012188333190676689])

        #################DEBUG#######################333
        # undist=cv2.undistort(img, mtx, dst, None, mtx)
        # cv2.imwrite("/home/bkjung/undist_for_homography/pi_l/undist_"+str(time.time())+".png",undist)
        # imgmsg=self.bridge.cv2_to_imgmsg(undist)
        # self.pub_pi_l.publish(imgmsg)

        homo4= np.array([[-8.70731094e-02,  1.47816011e+00,  4.02882375e+02],
            [-3.60449157e-01,  1.38082612e+00,  7.28709159e+02],
            [-1.63990036e-04,  2.28324521e-03,  1.00000000e+00]])
        return cv2.warpPerspective( cv2.undistort(img, mtx, dst,None, mtx) , homo4, (1280,1280))


    def undistort_right(self, _img):
        img=self.bridge.compressed_imgmsg_to_cv2(_img)
        img=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        mtx=np.array([[494.0169295185964, 0.0, 483.6710483879246], [0.0, 495.87509303786857, 336.69262125267153], [0.0, 0.0, 1.0]])
        dst=np.array([-0.26693726936305806, 0.05239559897759021, 0.0024912074565555443, -0.0015904998174301696])

        #################DEBUG#######################333
        # undist=cv2.undistort(img, mtx, dst, None, mtx)
        # cv2.imwrite("/home/bkjung/undist_for_homography/pi_r/undist_"+str(time.time())+".png",undist)
        # imgmsg=self.bridge.cv2_to_imgmsg(undist)
        # self.pub_pi_r.publish(imgmsg)

        homo4= np.array([[-1.85451252e-03,  1.48625255e+00,  4.00372407e+02],
            [-2.81151513e-01,  1.55607938e+00,  7.91845451e+02],
            [-2.73379929e-05,  2.29141415e-03,  1.00000000e+00]])
        return cv2.warpPerspective( cv2.undistort(img, mtx, dst,None, mtx) , homo4, (1280,1280))

if __name__=='__main__':
    num_pts_delete = 150 #num_of_waypoints_to_delete_in_virtualmap_after_compensation
    VisualCompensation(num_pts_delete)
