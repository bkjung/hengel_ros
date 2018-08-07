#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from sensor_msgs.msg import Image, CompressedImage
from markRobotView import RobotView
from math import radians, copysign, sqrt, pow, pi, atan2, sin, floor, cos, asin,ceil
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
        while True:
            word= raw_input("WHAT IS THE WIDTH AND HEIGHT OF CANVAS?\n Type: ")
            self.width=float(word.split()[0])
            self.height=float(word.split()[1])
            break
        self.num_pts_delete = _num_pts_delete
        self.recent_pts = collections.deque(self.num_pts_delete*[(0.0,0.0)],self.num_pts_delete)

        self.home_path = expanduser("~")
        self.folder_path = self.home_path + "/FEATURE_MATCH/" + time.strftime("%y%m%d_%H%M%S")
        os.system("mkdir -p " + self.folder_path)

        self.initialize()


    def initialize(self):
        rospy.init_node('hengel_camera_compensation', anonymous=False)

        self.isNavigationStarted = False
        self.bridge=CvBridge()
        self.pixMetRatio=500

        self.cropped_virtual_map=np.full((1280,1280),255).astype('uint8')
        self.virtual_map=np.full((int(self.pixMetRatio*self.height), int(self.pixMetRatio*self.width)), 255)
        self.app_robotview=RobotView(self.virtual_map) # Add the endpoint into the virtual map


        self.pi_left_img=np.array([])
        self.pi_right_img=np.array([])

        

        self.mid_predict_canvas_x=0
        self.mid_predict_canvas_y=0
        self.mid_predict_canvas_th=0

        self.mid_real_photo_x=640+55.77116996/2
        self.mid_real_photo_y=640

        self.endPoint_callback=message_filters.Subscriber('/endpoint', Point)
        self.midPoint_callback=message_filters.Subscriber('/midpoint', Point)

        self.ts=message_filters.ApproximateTimeSynchronizer([self.endPoint_callback, self.midPoint_callback], 10, 0.1, allow_headerless=True)
        self.ts.registerCallback(self.sync_virtual_callback)

        self.pub_virtual_map=rospy.Publisher('/virtual_map', CompressedImage, queue_size=3)
        self.callback1=message_filters.Subscriber('/genius1/compressed', CompressedImage)
        self.callback2=message_filters.Subscriber('/genius2/compressed', CompressedImage)
        self.callback3=message_filters.Subscriber('/genius3/compressed', CompressedImage)
        self.callback4=message_filters.Subscriber('/genius4/compressed', CompressedImage)



        self.ts=message_filters.ApproximateTimeSynchronizer([self.callback1, self.callback2, self.callback3, self.callback4], 10,0.1, allow_headerless=False)
        rospy.Subscriber('/usb_cam3/image_raw/compressed', CompressedImage, self.callback_left)
        rospy.Subscriber('/usb_cam4/image_raw/compressed', CompressedImage, self.callback_right)


        # self.ts=message_filters.ApproximateTimeSynchronizer([self.callback1, self.callback2, self.callback3, self.callback4, self.callback_pi_left, self.callback_pi_right], 10,0.1, allow_headerless=False)
        # self.callback_pi_left=message_filters.Subscriber('/usb_cam3/image_raw/compressed', CompressedImage)
        # self.callback_pi_right=message_filters.Subscriber('/usb_cam4/image_raw/compressed', CompressedImage)



        self.ts.registerCallback(self.sync_real_callback)


        self.pub_offset=rospy.Publisher('/offset_change', Point, queue_size=5)
        ############################ DEBUG ################################
        # self.pub_pi_l=rospy.Publisher('/pi_left/undist', Image, queue_size=5 )
        # self.pub_pi_r=rospy.Publisher('/pi_right/undist', Image, queue_size=5 )
        self.pub_sum=rospy.Publisher('/summed_image/compressed',CompressedImage, queue_size=5)
    

        rospy.spin()


#################################################################
    def callback_left(self, _img):
        # print("left")
        self.pi_left_img=self.undistort_left(_img)

    def callback_right(self, _img):
        # print("right")
        self.pi_right_img=self.undistort_right(_img)

    def sync_real_callback(self, _img1, _img2, _img3, _img4):
        if self.isNavigationStarted:     
            _time=time.time()
            print("sync real")
            img1 = self.undistort1(_img1)
            img2 = self.undistort2(_img2)
            img3 = self.undistort3(_img3)
            img4 = self.undistort4(_img4)

            # while len(self.pi_left_img)==0 or len(self.pi_right_img)==0:
            #     print("empty pi_left or pi_right")
            #     time.sleep(100)

            # img_left=copy.deepcopy(self.pi_left_img)
            # img_right=copy.deepcopy(self.pi_right_img)

            im_mask_inv1, im_mask1=self.find_mask(img1)
            im_mask_inv3, im_mask3=self.find_mask(img3)
            _, im_mask2=self.find_mask(img2)
            _, im_mask4=self.find_mask(img4)
            # _, im_mask_l=self.find_mask(img_left)
            # _, im_mask_r=self.find_mask(img_right)

            # img_white=np.full((1280, 1280), 255)

            im_mask13=cv2.bitwise_and(np.array(im_mask1), np.array(im_mask3))
            im_mask24=cv2.bitwise_and(np.array(im_mask2), np.array(im_mask4))
            im_mask1234=cv2.bitwise_and(im_mask13, im_mask24)

            # print("im_mask13.dtype : "+str(im_mask13.dtype))
            # print("im_mask24.dtype : "+str(im_mask24.dtype))
            # print("im_mask1234.dtype : "+str(im_mask1234.dtype))

            # img_white_masked=np.multiply(np.multiply(np.multiply(img_white, im_mask1234), im_mask_l), im_mask_r).astype('uint8')
            img2_masked=np.multiply(np.multiply(img2, im_mask13), im_mask4).astype('uint8')
            img4_masked=np.multiply(np.multiply(img4, im_mask13), im_mask2).astype('uint8')
            img1_masked=np.multiply(img1, im_mask_inv1).astype('uint8')
            img3_masked=np.multiply(img3, im_mask_inv3).astype('uint8')
            # img_left_masked=np.multiply(np.multiply(img_left, im_mask1234), im_mask_r).astype('uint8')
            # img_right_masked=np.multiply(np.multiply(img_right, im_mask1234), im_mask_l).astype('uint8')
            
            # summed_image=img1_masked+img2_masked+img3_masked+img4_masked+img_white_masked+img_left_masked+img_right_masked
            # summed_image=img1_masked+img2_masked+img3_masked+img4_masked+img_left_masked+img_right_masked
            summed_image=img1_masked+img2_masked+img3_masked+img4_masked
            summed_msg=self.bridge.cv2_to_compressed_imgmsg(summed_image)

            self.pub_sum.publish(summed_msg)
            print("summed_image time: "+str(time.time()-_time))


            homography_virtual_map=self.crop_image(self.virtual_map) #background is black

            print("DEBUG111")

            # im_mask, im_mask_inv = self.find_mask(homography_virtual_map)

            # im_white=np.full((1280,1280),255)
            # im_white_masked=np.uint8(np.multiply(im_white, im_mask))
            # homography_virtual_map_masked=np.uint8(np.multiply(homography_virtual_map, im_mask_inv))
            # self.cropped_virtual_map=im_white_masked+homography_virtual_map_masked
            self.cropped_virtual_map=homography_virtual_map.astype('uint8')

            print("DEBUG222")

            #################
            try:
                fm = FeatureMatch(self.folder_path)
                # print("img1: "+str(self.virtual_map.shape)+", img2: "+str(summed_image.shape))
                # if self.cropped_virtual_map is None or summed_image is None:
                if self.cropped_virtual_map is None or summed_image is None:
                    print("IMAGE EMPTY")
                    raise Exeption("Image Empty")
                else:

                    
                    M = fm.SIFT_FLANN_matching(self.cropped_virtual_map, summed_image)
                    # M = fm.SIFT_FLANN_matching(summed_image, self.cropped_virtual_map)
                    if fm.status == True:
                        # self.vision_offset_publisher.publish(Point(fm.delta_x, fm.delta_y, fm.delta_theta))
                        # self.app_robotview.remove_points_during_vision_compensation(self.recent_pts)
                        # self.virtual_map = self.app_robotview.img

                        #Initialize Queue
                        self.recent_pts = collections.deque(self.num_pts_delete*[(0.0,0.0)],self.num_pts_delete)

                        self.relocalization(M)


            except Exception as e:
                print(e)
                sys.exit("Feature Match error")

            #################

            print("Cam Input -> Visual Calc / Total Time: "+str(time.time()-_time))

            # bridge=CvBridge()
            # summed_msg=bridge.cv2_to_compressed_imgmsg(summed_image)
            # self.sum_pub.publish(summed_msg)

        else:
            print("Navigation not started yet")
#################################################################


    def sync_virtual_callback(self, _endPoint, _midPoint):
        if not self.isNavigationStarted:
            self.isNavigationStarted = True
        # print("sync virtual")
        _time=time.time()

        self.mid_predict_canvas_x=_midPoint.x
        self.mid_predict_canvas_y=_midPoint.y
        self.mid_predict_canvas_th=_midPoint.z

        self.recent_pts.appendleft((_midPoint.x, _midPoint.y))

        # self.virtual_map = self.app_robotview.run(_midPoint, _endPoint)
        self.app_robotview.run(_midPoint, _endPoint)
        self.virtual_map = self.app_robotview.img

        self.mid_predict_img_x=-self.mid_predict_canvas_x *self.pixMetRatio
        self.mid_predict_img_y=self.virtual_map.shape[0]-self.mid_predict_canvas_y*self.pixMetRatio
        self.mid_predict_img_th=-self.mid_predict_canvas_th

        
        ttime=Float32()
        ttime.data=float(time.time()-_time)

        #PUBLISHING VIRTUAL MAP, but currently the msg cannot be viewed at rqt (supposedly because of msgtype mismatch)
        # virtual_map_msg=self.bridge.cv2_to_compressed_imgmsg(self.virtual_map)
        # self.pub_virtual_map.publish(virtual_map_msg)


    def relocalization(self, homography):
        mid_real_virtual_x, mid_real_virtual_y, _= np.matmul(homography, [self.mid_real_photo_x, self.mid_real_photo_y, 1])
        del_x_virtual=mid_real_virtual_x-self.mid_real_photo_x
        del_y_virtual=mid_real_virtual_y-self.mid_real_photo_y
        del_th_virtual=-atan2(homography[0][1],homography[0][0])

        rotation=np.array([[cos(self.mid_predict_canvas_th), -sin(self.mid_predict_canvas_th)],
                            [sin(self.mid_predict_canvas_th), cos(self.mid_predict_canvas_th)]])
        del_x_canvas, del_y_canvas = np.matmul(rotation, [-del_x_virtual, -del_y_virtual])
        # *(-1) in del_x_virtual for calibration of x waypoint coordinate
        # *(-1) in del_y_virtual for calibration of image coordiate to canvas coordinate
        
        offset=Point()
        offset.x=del_x_canvas
        offset.y=del_y_canvas
        offset.z=del_th_virtual

        print(offset)

        self.pub_offset.publish(offset)
        

    def crop_image(self, _img):
        _time=time.time()
        padding=int(ceil(640*sqrt(2)))
        img_padding=np.full((_img.shape[0]+padding*2, _img.shape[1]+padding*2),0).astype('uint8')

        img_test=img_padding[padding:padding+_img.shape[0],padding:padding+_img.shape[1]]


        img_padding[padding:padding+_img.shape[0],padding:padding+_img.shape[1]]=_img

        cv2.imwrite("/home/bkjung/img_padding.png", img_padding)
        half_map_size_diagonal = 1280/sqrt(2)
        midpnt_offset=55.77116996/2 # in virtual map coordiate

        #middle point of the cropped image in virtual map coordinate
        x_mid_crop=self.mid_predict_img_x-midpnt_offset*cos(self.mid_predict_img_th)
        y_mid_crop=self.mid_predict_img_y+midpnt_offset*sin(self.mid_predict_img_th)

        imgPts=[[x_mid_crop-half_map_size_diagonal*cos(pi/4-self.mid_predict_img_th), y_mid_crop-half_map_size_diagonal*sin(pi/4-self.mid_predict_img_th)],
                    [x_mid_crop-half_map_size_diagonal*cos(pi/4+self.mid_predict_img_th), y_mid_crop+half_map_size_diagonal*sin(pi/4+self.mid_predict_img_th)],
                    [x_mid_crop+half_map_size_diagonal*cos(pi/4-self.mid_predict_img_th), y_mid_crop+half_map_size_diagonal*sin(pi/4-self.mid_predict_img_th)],
                    [x_mid_crop+half_map_size_diagonal*cos(pi/4+self.mid_predict_img_th), y_mid_crop-half_map_size_diagonal*sin(pi/4+self.mid_predict_img_th)]]
        
        imgPts_padding=[[a[0]+padding, a[1]+padding] for a in imgPts]
        # print("points: "+str(imgPts_padding))
        
        imgPts_padding=np.array(imgPts_padding)

        objPts=np.array([[0,0], [0, 1280], [1280, 1280], [1280,0]])
        homography, _=cv2.findHomography(imgPts_padding, objPts)

        img_padding=np.uint8(img_padding)
        return cv2.warpPerspective(img_padding, homography,(1280,1280))

    def find_mask(self, img):
        # print(img.shape)
        _time=time.time()
        black_range1=np.array([0])
        im_mask=(cv2.inRange(img, black_range1, black_range1))
        # im_mask=np.dstack((im_mask, im_mask, im_mask))
        im_mask_inv=(1-im_mask)
        return im_mask_inv, im_mask

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

        # homo1= np.array([[-1.99133283e+00,  3.96335542e+00,  1.43529996e+03],
        #     [ 1.26013904e-02,  3.20665968e+00,  1.58076675e+03],
        #     [-5.52616807e-05,  6.56931832e-03,  1.00000000e+00]])
        homo1=np.array([[ 1.92059741e+00,  4.44537102e+00, -1.55299796e+02],
            [-8.33364081e-02,  5.20206669e+00, -3.00766611e+02],
            [-5.52618496e-05,  6.56931686e-03,  1.00000000e+00]])
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

        # homo2= np.array([[-1.74979870e-01,  2.54748382e+00,  1.50601223e+03],
        #     [ 1.58754865e+00,  3.68531885e+00, -7.91871465e+01],
        #     [-2.64765872e-04,  5.74538321e-03,  1.00000000e+00]])
        homo2= np.array([[-1.63920447e-01,  4.80660669e+00, -2.26012234e+02],
            [-1.92644897e+00,  3.66877166e+00,  1.35918715e+03],
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

        # homo3= np.array([[ 2.04906162e+00,  3.92623252e+00, -1.90107542e+02],
        #     [ 1.64193237e-02,  5.25578879e+00, -4.40829402e+02],
        #     [ 1.60923552e-05,  6.47353214e-03,  1.00000000e+00]])
        homo3= np.array([[-2.00983918e+00,  4.28781278e+00,  1.46124710e+03],
            [-2.92334083e-03,  2.97243939e+00,  1.71000087e+03],
            [ 7.54059686e-06,  6.36915820e-03,  1.00000000e+00]])


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


        # homo4= np.array([[ 2.28317132e-01,  1.40100840e+01, -2.33867485e+03],
        #     [-5.15991902e+00,  1.17501321e+01,  3.00914222e+03],
        #     [ 5.65609486e-04,  1.83615756e-02,  1.00000000e+00]])
        homo4= np.array([[ 4.95663167e-01,  9.49273943e+00,  3.61867640e+03],
            [ 5.88390241e+00,  1.17526923e+01, -1.72914355e+03],
            [ 5.65609665e-04,  1.83615875e-02,  1.00000000e+00]])
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

        # homo5= np.array([[-8.70731094e-02,  1.47816011e+00,  4.02882375e+02],
        #     [-3.60449157e-01,  1.38082612e+00,  7.28709159e+02],
        #     [-1.63990036e-04,  2.28324521e-03,  1.00000000e+00]])
        homo5= np.array([[-1.22834137e-01,  1.44439375e+00,  8.77117625e+02],
            [ 1.50541911e-01,  1.54172775e+00,  5.51290841e+02],
            [-1.63990036e-04,  2.28324521e-03,  1.00000000e+00]])
        return cv2.warpPerspective( cv2.undistort(img, mtx, dst,None, mtx) , homo5, (1280,1280))


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

        # homo4= np.array([[-1.85451252e-03,  1.48625255e+00,  4.00372407e+02],
        #     [-2.81151513e-01,  1.55607938e+00,  7.91845451e+02],
        #     [-2.73379929e-05,  2.29141415e-03,  1.00000000e+00]])
        homo5= np.array([[-3.31381222e-02,  1.44675753e+00,  8.79627593e+02],
            [ 2.46158878e-01,  1.37693071e+00,  4.88154549e+02],
            [-2.73379975e-05,  2.29141411e-03,  1.00000000e+00]])
        return cv2.warpPerspective( cv2.undistort(img, mtx, dst,None, mtx) , homo5, (1280,1280))

if __name__=='__main__':
    num_pts_delete = 150 #num_of_waypoints_to_delete_in_virtualmap_after_compensation
    VisualCompensation(num_pts_delete)
