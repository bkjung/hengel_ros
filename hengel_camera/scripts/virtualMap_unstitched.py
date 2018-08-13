#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Float32, Time, Header
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

# def ros_time_to_float(msg_time):
#     # print(type(msg_time))
#     # print(type(msg_time.data))
#     # print(type(msg_time.secs))
#     print(msg_time)
#     print(msg_time.data)
#     return msg_time.sec+msg_time.nsec*0.000000001

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
        rospy.init_node('hengel_camera_compensation_unstitched', anonymous=False)

        self.isNavigationStarted = False
        self.bridge=CvBridge()
        self.pixMetRatio=500

        self.cropped_virtual_map=np.full((1280,1280),255).astype('uint8')
        self.virtual_map=np.full((int(self.pixMetRatio*self.height), int(self.pixMetRatio*self.width)), 255)
        self.app_robotview=RobotView(self.virtual_map) # Add the endpoint into the virtual map

        self.pi_left_img=np.array([])
        self.pi_right_img=np.array([])

        self.isProcessingVirtualmapTime = False
        self.mid_predict_canvas_x=collections.deque(300*[0.0],300)
        self.mid_predict_canvas_y=collections.deque(300*[0.0],300)
        self.mid_predict_canvas_th=collections.deque(300*[0.0],300)
        self.mid_predict_canvas_time=collections.deque(300*[0.0],300)

        self.mid_real_photo_x=640+55.77116996/2
        self.mid_real_photo_y=640

        self.endPoint_callback=message_filters.Subscriber('/endpoint', Point)
        self.midPoint_callback=message_filters.Subscriber('/midpoint', Point)
        self.midPoint_time_callback=message_filters.Subscriber('/midpoint_time', Time)


        self.threshold1=100
        self.threshold2=100
        self.threshold3=100
        self.threshold4=100

        self.is_first1=True
        self.is_first2=True
        self.is_first3=True
        self.is_first4=True
        

        self.ts=message_filters.ApproximateTimeSynchronizer([self.endPoint_callback, self.midPoint_callback, self.midPoint_time_callback], 10, 0.1, allow_headerless=True)
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
            print("sync real")
            _time = time.time()
            image_time = (_img1.header.stamp.to_nsec()+_img2.header.stamp.to_nsec()+_img3.header.stamp.to_nsec()+_img4.header.stamp.to_nsec())/4.0

            min_diff = 999999999999999.9
            min_index = -1
            self.isProcessingVirtualmapTime = True
            for i in range(len(self.mid_predict_canvas_time)):
                curr_diff = abs(self.mid_predict_canvas_time[i]-image_time)
                if (curr_diff < min_diff):
                    min_diff = curr_diff
                    min_index = i

            self.current_mid_predict_canvas_x = self.mid_predict_canvas_x[min_index]
            self.current_mid_predict_canvas_y = self.mid_predict_canvas_y[min_index]
            self.current_mid_predict_canvas_th = self.mid_predict_canvas_th[min_index]
            self.isProcessingVirtualmapTime = False

            self.mid_predict_img_x=-self.current_mid_predict_canvas_x *self.pixMetRatio
            self.mid_predict_img_y=self.virtual_map.shape[0]-self.current_mid_predict_canvas_y*self.pixMetRatio
            self.mid_predict_img_th=-self.current_mid_predict_canvas_th

            # print("Processing Virtualmap Sync Time: "+str(time.time()-_time))

            img1 = self.undistort1(_img1)
            img2 = self.undistort2(_img2)
            img3 = self.undistort3(_img3)
            img4 = self.undistort4(_img4)

            im_mask_inv1, im_mask1=self.find_mask(self.img_for_mask1)
            im_mask_inv2, im_mask2=self.find_mask(self.img_for_mask2)
            im_mask_inv3, im_mask3=self.find_mask(self.img_for_mask3)
            im_mask_inv4, im_mask4=self.find_mask(self.img_for_mask4)

            # img_white=np.full((1280, 1280), 145)

            homography_virtual_map=self.crop_image(self.virtual_map) #background is black
            im_mask_inv, im_mask = self.find_mask(homography_virtual_map)

            virtual_map_converted1 = np.multiply(homography_virtual_map, im_mask_inv1)
            virtual_map_converted2 = np.multiply(homography_virtual_map, im_mask_inv2)
            virtual_map_converted3 = np.multiply(homography_virtual_map, im_mask_inv3)
            virtual_map_converted4 = np.multiply(homography_virtual_map, im_mask_inv4)

            virtual_map1 = cv2.bitwise_not(virtual_map_converted1)
            virtual_map2 = cv2.bitwise_not(virtual_map_converted2)
            virtual_map3 = cv2.bitwise_not(virtual_map_converted3)
            virtual_map4 = cv2.bitwise_not(virtual_map_converted4)

            print("sum & crop image time: "+str(time.time()-_time))
         
##################################################################################

            try:
                print("debug0")
                fm = FeatureMatch(self.folder_path)
                # print("img1: "+str(self.virtual_map.shape)+", img2: "+str(summed_image.shape))
                # if self.cropped_virtual_map is None or summed_image is None:
                if self.cropped_virtual_map is None or img1 is None or img1 is None or img1 is None or img1 is None:
                    print("IMAGE EMPTY")
                    raise Exception("Image Empty")
                else:
                    # M = fm.SIFT_FLANN_matching(self.cropped_virtual_map, summed_image)

                    # M = fm.ORB_BF_matching(summed_image, self.cropped_virtual_map)
                    # M = fm.IMAGE_ALIGNMENT_ecc(summed_image, self.cropped_virtual_map)
                    print("debug1")
                    M1 = fm.SURF_BF_matching(img1, virtual_map1)
                    M2 = fm.SURF_BF_matching(img2, virtual_map2)
                    M3 = fm.SURF_BF_matching(img3, virtual_map3)
                    M4 = fm.SURF_BF_matching(img4, virtual_map4)



                    # if fm.status == True:
                    #     # self.vision_offset_publisher.publish(Point(fm.delta_x, fm.delta_y, fm.delta_theta))
                    #     # self.app_robotview.remove_points_during_vision_compensation(self.recent_pts)
                    #     # self.virtual_map = self.app_robotview.img

                    #     #Initialize Queue
                    #     # self.recent_pts = collections.deque(self.num_pts_delete*[(0.0,0.0)],self.num_pts_delete)

                    #     __time=time.time()
                    #     self.relocalization(M)
                    #     print("relocation time: "+str(time.time()-__time))

            except Exception as e:
                print(e)
                sys.exit("Feature Match error - debug1")

            #################

            print("Total Time (visual feedback): "+str(time.time()-_time))

            # bridge=CvBridge()
            # summed_msg=bridge.cv2_to_compressed_imgmsg(summed_image)
            # self.sum_pub.publish(summed_msg)

        else:
            print("Navigation not started yet")
#################################################################

    def sync_virtual_callback(self, _endPoint, _midPoint, _midPointTime):
        if not self.isNavigationStarted:
            self.isNavigationStarted = True
        # print("sync virtual")
        # _time=time.time()

        if not self.isProcessingVirtualmapTime:
            self.mid_predict_canvas_x.appendleft(_midPoint.x)
            self.mid_predict_canvas_y.appendleft(_midPoint.y)
            self.mid_predict_canvas_th.appendleft(_midPoint.z)
            self.mid_predict_canvas_time.appendleft(_midPointTime.data.to_nsec())

            self.recent_pts.appendleft((_midPoint.x, _midPoint.y))

            # self.virtual_map = self.app_robotview.run(_midPoint, _endPoint)
            self.app_robotview.run(_midPoint, _endPoint)
            self.virtual_map = self.app_robotview.img

            # ttime=Float32()
            # ttime.data=float(time.time()-_time)

            #PUBLISHING VIRTUAL MAP, but currently the msg cannot be viewed at rqt (supposedly because of msgtype mismatch)
            # virtual_map_msg=self.bridge.cv2_to_compressed_imgmsg(self.virtual_map)
            # self.pub_virtual_map.publish(virtual_map_msg)


    def relocalization(self, homography):
        print("debug1")
        mid_real_virtual_x, mid_real_virtual_y, _= np.matmul(homography, [self.mid_real_photo_x, self.mid_real_photo_y, 1])
        print("debug2")
        print(mid_real_virtual_x)
        print(self.mid_real_photo_x)
        del_x_virtual=mid_real_virtual_x-self.mid_real_photo_x
        print("debug3")
        del_y_virtual=mid_real_virtual_y-self.mid_real_photo_y
        print("debug4")
        del_th_virtual=-atan2(homography[0][1],homography[0][0])
        print("debug5")
        rotation=np.array([[cos(self.current_mid_predict_canvas_th), -sin(self.current_mid_predict_canvas_th)],
                            [sin(self.current_mid_predict_canvas_th), cos(self.current_mid_predict_canvas_th)]])
        print("debug6")
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
        img_not=cv2.bitwise_not(_img)
        padding=int(ceil(640*sqrt(2)))
        img_padding=np.full((_img.shape[0]+padding*2, _img.shape[1]+padding*2),0).astype('uint8')

        img_test=img_padding[padding:padding+_img.shape[0],padding:padding+_img.shape[1]]


        img_padding[padding:padding+_img.shape[0],padding:padding+_img.shape[1]]=img_not

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

    # def find_mask(self, img):
    #     # print(img.shape)
    #     _time=time.time()
    #     black_range1=np.array([0])
    #     im_mask=(cv2.inRange(img, black_range1, black_range1))
    #     # im_mask=np.dstack((im_mask, im_mask, im_mask))
    #     im_mask_inv=(1-im_mask)
    #     return im_mask_inv, im_mask
    def find_mask(self, img):
        # print(img.shape)
        _time=time.time()
        black_range1=np.array([0])
        black_range2=np.array([0])
        im_mask=(cv2.inRange(img, black_range1, black_range2)).astype('bool')
        # im_mask=np.dstack((im_mask, im_mask, im_mask))
        im_mask_inv=(1-im_mask).astype('bool')
        return im_mask_inv, im_mask 

    def find_mask_virtual(self, img):
        # print(img.shape)
        _time=time.time()
        black_range1=np.array([0])
        black_range2=np.array([250])
        im_mask=(cv2.inRange(img, black_range1, black_range2)).astype('bool')
        # im_mask=np.dstack((im_mask, im_mask, im_mask))
        im_mask_inv=(1-im_mask).astype('bool')
        return im_mask_inv, im_mask 

    def image_processing(self, _img):
        print("IMAGE PROCESSING")
        cv2.imwrite("/home/bkjung/before.png",_img)
        for i in xrange(len(_img)):
            for j in xrange(len(_img[i])):
                if _img[i][j]==0 and i>1 and j>1 and i<_img.shape[1]-1 and j<_img.shape[0]-1:
                    
                    a=[]
                    a.append(_img[i-1][j])  #P1
                    a.append(_img[i][j+1])  #P2
                    a.append(_img[i+1][j])  #P3
                    a.append(_img[i][j-1])  #P4
                    a.append(_img[i-1][j+1])    #P5
                    a.append(_img[i+1][j+1])    #P6
                    a.append(_img[i+1][j-1])    #P7
                    a.append(_img[i-1][j-1])    #P8
                    if sum(a[:8])>= 255*7:
                        _img[i][j]=255
                        print("SALT_POINT: "+str(i)+", "+str(j))
                    else:
                        pass
                else:
                    pass
        cv2.imwrite("/home/bkjung/after.png", _img)
        print("done")
        return _img


    def undistort1(self, _img):
        img=self.bridge.compressed_imgmsg_to_cv2(_img)
        img=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        mtx=np.array([[393.8666817683925, 0.0, 399.6813895086665], [0.0, 394.55108358870405, 259.84676565717876], [0.0, 0.0, 1.0]])
        dst=np.array([-0.0032079005049939543, -0.020856072501002923, 0.000252242294186179, -0.0021042704510431365])


        homo1=np.array([[ 1.92059741e+00,  4.44537102e+00, -1.55299796e+02],
            [-8.33364081e-02,  5.20206669e+00, -3.00766611e+02],
            [-5.52618496e-05,  6.56931686e-03,  1.00000000e+00]])

        undist_img_binary= cv2.threshold(cv2.undistort(img ,mtx,dst ,None, mtx), self.threshold1, 255, cv2.THRESH_BINARY)[1]
        if self.is_first1==True:
            self.img_for_mask1=cv2.warpPerspective(cv2.undistort(img ,mtx, dst, None, mtx), homo1, (1280, 1280))

        return cv2.bitwise_not(cv2.warpPerspective( cv2.bitwise_not(undist_img_binary) , homo1, (1280,1280)))





    def undistort2(self, _img):
        img=self.bridge.compressed_imgmsg_to_cv2(_img)
        img=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        mtx=np.array([[396.01900903941834, 0.0, 410.8496405295566], [0.0, 396.2406539134792, 285.8932176591904], [0.0, 0.0, 1.0]])
        dst=np.array([-0.008000340519517233, -0.016478659972026452, 7.25792172844022e-05, -0.00434319738405187])


        homo2= np.array([[-1.63920447e-01,  4.80660669e+00, -2.26012234e+02],
            [-1.92644897e+00,  3.66877166e+00,  1.35918715e+03],
            [-2.64765872e-04,  5.74538321e-03,  1.00000000e+00]])

        undist_img_binary= cv2.threshold(cv2.undistort(img ,mtx,dst ,None, mtx), self.threshold2, 255, cv2.THRESH_BINARY)[1]

        if self.is_first2==True:
            self.img_for_mask2=cv2.warpPerspective(cv2.undistort(img ,mtx, dst, None, mtx), homo2, (1280, 1280))
            self.is_first2=False
        return cv2.bitwise_not( cv2.warpPerspective( cv2.bitwise_not(undist_img_binary) , homo2, (1280,1280)))


    def undistort3(self, _img):
        img=self.bridge.compressed_imgmsg_to_cv2(_img)
        img=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        mtx=np.array([[387.8191999285985, 0.0, 392.3078288789019],[ 0.0, 382.1093651210362, 317.43368009853674], [0.0, 0.0, 1.0]])
        dst=np.array([-0.008671221810333559, -0.013546386893040543, -0.00016537575030651431, 0.002659594999360673])

        homo3= np.array([[-2.00983918e+00,  4.28781278e+00,  1.46124710e+03],
            [-2.92334083e-03,  2.97243939e+00,  1.71000087e+03],
            [ 7.54059686e-06,  6.36915820e-03,  1.00000000e+00]])

        undist_img_binary= cv2.threshold(cv2.undistort(img ,mtx,dst ,None, mtx), self.threshold3, 255, cv2.THRESH_BINARY)[1]

        if self.is_first3==True:
            self.img_for_mask3=cv2.warpPerspective(cv2.undistort(img ,mtx, dst, None, mtx), homo3, (1280, 1280))
            self.is_first3=False
        
        return cv2.bitwise_not(cv2.warpPerspective( cv2.bitwise_not(undist_img_binary) , homo3, (1280,1280)))


    def undistort4(self, _img):
        img=self.bridge.compressed_imgmsg_to_cv2(_img)
        img=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        mtx=np.array([[384.2121883964654, 0.0, 423.16727407803353], [0.0, 386.8188468139677, 359.5190506678551], [0.0, 0.0, 1.0]])
        dst=np.array([-0.0056866549555025896, -0.019460881544303938, 0.0012937686026747307, -0.0031999317338443087])

        homo4= np.array([[ 4.95663167e-01,  9.49273943e+00,  3.61867640e+03],
            [ 5.88390241e+00,  1.17526923e+01, -1.72914355e+03],
            [ 5.65609665e-04,  1.83615875e-02,  1.00000000e+00]])

        undist_img_binary= cv2.threshold(cv2.undistort(img ,mtx,dst ,None, mtx), self.threshold4, 255, cv2.THRESH_BINARY)[1]

        if self.is_first4==True:
            self.img_for_mask4=cv2.warpPerspective(cv2.undistort(img ,mtx, dst, None, mtx), homo4, (1280, 1280))
            self.is_first4=False
        
        return cv2.bitwise_not(cv2.warpPerspective( cv2.bitwise_not(undist_img_binary) , homo4, (1280,1280)))

    def undistort_left(self, _img):
        img=self.bridge.compressed_imgmsg_to_cv2(_img)
        img=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        mtx=np.array([[496.88077412085187, 0.0, 486.19161191113693], [0.0, 497.77308359203073, 348.482250144119], [0.0, 0.0, 1.0]])
        dst=np.array([-0.27524035766660704, 0.055346669640229516, 0.002041430748143387, -0.0012188333190676689])

        homo5= np.array([[-1.22834137e-01,  1.44439375e+00,  8.77117625e+02],
            [ 1.50541911e-01,  1.54172775e+00,  5.51290841e+02],
            [-1.63990036e-04,  2.28324521e-03,  1.00000000e+00]])
        return cv2.warpPerspective( cv2.bitwise_not(cv2.undistort(img, mtx, dst,None, mtx) , homo5, (1280,1280)))


    def undistort_right(self, _img):
        img=self.bridge.compressed_imgmsg_to_cv2(_img)
        img=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        mtx=np.array([[494.0169295185964, 0.0, 483.6710483879246], [0.0, 495.87509303786857, 336.69262125267153], [0.0, 0.0, 1.0]])
        dst=np.array([-0.26693726936305806, 0.05239559897759021, 0.0024912074565555443, -0.0015904998174301696])


        homo5= np.array([[-3.31381222e-02,  1.44675753e+00,  8.79627593e+02],
            [ 2.46158878e-01,  1.37693071e+00,  4.88154549e+02],
            [-2.73379975e-05,  2.29141411e-03,  1.00000000e+00]])
        return cv2.warpPerspective(cv2.bitwise_not( cv2.undistort(img, mtx, dst,None, mtx) , homo5, (1280,1280)))

if __name__=='__main__':
    num_pts_delete = 150 #num_of_waypoints_to_delete_in_virtualmap_after_compensation
    VisualCompensation(num_pts_delete)