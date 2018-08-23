#!/usr/bin/env python
import rospy
import sys
from geometry_msgs.msg import Point
from std_msgs.msg import Float32, Time, Header
from sensor_msgs.msg import Image, CompressedImage
from markRobotView import RobotView
from math import radians, copysign, sqrt, pow, pi, atan2, sin, floor, cos, asin,ceil
from numpy.linalg import inv
import numpy as np
import time
import os
import copy
from os.path import expanduser
import message_filters
import collections
from feature_match import FeatureMatch
from matplotlib import pyplot as plt
from hengel_camera.msg import CmpImg
from cv_bridge import CvBridge
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')

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
        rospy.init_node('hengel_camera_compensation', anonymous=False)

        self.isNavigationStarted = False
        self.bridge=CvBridge()
        self.pixMetRatio=250

        self.cropped_virtual_map=np.full((1280,1280),255).astype('uint8')
        self.virtual_map=np.full((int(self.pixMetRatio*self.height), int(self.pixMetRatio*self.width)), 255)
        self.app_robotview=RobotView(self.virtual_map) # Add the endpoint into the virtual map

        self.isProcessingVirtualmapTime = False
        self.mid_predict_canvas_x=collections.deque(300*[0.0],300)
        self.mid_predict_canvas_y=collections.deque(300*[0.0],300)
        self.mid_predict_canvas_th=collections.deque(300*[0.0],300)
        self.mid_predict_canvas_time=collections.deque(300*[0.0],300)

        self.mid_real_photo_x=640+55.77116996/2
        self.mid_real_photo_y=640

        self.summed_image = None
        self.summed_image_prev = None

        self.ts_2=message_filters.ApproximateTimeSynchronizer([self.callback1, self.callback2, self.callback3, self.callback4], 10,0.1, allow_headerless=False)
        
        rospy.Subscriber('/usb_cam3/image_raw/compressed', CompressedImage, self.callback_left)
        rospy.Subscriber('/usb_cam4/image_raw/compressed', CompressedImage, self.callback_right)


        # self.ts_2=message_filters.ApproximateTimeSynchronizer([self.callback1, self.callback2, self.callback3, self.callback4, self.callback_pi_left, self.callback_pi_right], 10,0.1, allow_headerless=False)
        # self.callback_pi_left=message_filters.Subscriber('/usb_cam3/image_raw/compressed', CompressedImage)
        # self.callback_pi_right=message_filters.Subscriber('/usb_cam4/image_raw/compressed', CompressedImage)

        self.is_first1=True
        self.is_first2=True
        self.is_first3=True
        self.is_first4=True

        self.threshold1=100
        self.threshold2=100
        self.threshold3=100
        self.threshold4=100        

        self.ts_2.registerCallback(self.sync_real_callback)


        self.pub_offset=rospy.Publisher('/offset_change', Point, queue_size=5)
        ############################ DEBUG ################################
        # self.pub_pi_l=rospy.Publisher('/pi_left/undist', Image, queue_size=5 )
        # self.pub_pi_r=rospy.Publisher('/pi_right/undist', Image, queue_size=5 )
        self.pub_sum=rospy.Publisher('/summed_image/compressed',CompressedImage, queue_size=5)


        self.success_try = 0
        self.total_try = 0

        self.sum_compensation_distance = 0.0


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

            # while len(self.pi_left_img)==0 or len(self.pi_right_img)==0:
            #     print("empty pi_left or pi_right")
            #     time.sleep(100)

            # img_left=copy.deepcopy(self.pi_left_img)
            # img_right=copy.deepcopy(self.pi_right_img)
####  1  #############################################################################
            # im_mask_inv1, im_mask1=self.find_mask(img1)
            # im_mask_inv3, im_mask3=self.find_mask(img3)
            # im_mask_inv2, im_mask2=self.find_mask(img2)
            # im_mask_inv4, im_mask4=self.find_mask(img4)
            # # _, im_mask_l=self.find_mask(img_left)
            # # _, im_mask_r=self.find_mask(img_right)

            # img_white=np.full((1280, 1280), 255)

            # im_mask13=cv2.bitwise_and(np.array(im_mask1).astype('uint8'), np.array(im_mask3).astype('uint8'))

            # im_mask_inv_13=cv2.bitwise_and(np.array(im_mask_inv1).astype('uint8'), np.array(im_mask_inv3).astype('uint8'))
            # im_mask_inv_24=cv2.bitwise_and(np.array(im_mask_inv2).astype('uint8'), np.array(im_mask_inv4).astype('uint8'))
            # im_mask_inv1234=cv2.bitwise_and(im_mask_inv_13, im_mask_inv_24)

            # img_white_masked=np.multiply(img_white, im_mask_inv1234)
            # img2_masked=np.multiply(np.multiply(img2, im_mask13), im_mask_inv2)
            # img4_masked=np.multiply(np.multiply(img4, im_mask13), im_mask_inv4)
            # img1_masked=np.multiply(img1, im_mask_inv1)
            # img3_masked=np.multiply(img3, im_mask_inv3)
            # # img_left_masked=np.multiply(np.multiply(img_left, im_mask1234), im_mask_r).astype('uint8')
            # # img_right_masked=np.multiply(np.multiply(img_right, im_mask1234), im_mask_l).astype('uint8')
            # summed_image=(img1_masked+img2_masked+img3_masked+img4_masked+img_white_masked).astype('uint8')
            # summed_image=(img1_masked+img2_masked+img3_masked+img4_masked).astype('uint8')
            # summed_msg=self.bridge.cv2_to_compressed_imgmsg(summed_image)

            # self.pub_sum.publish(summed_msg)
            # print("summed_image time: "+str(time.time()-_time))


            # homography_virtual_map=self.crop_image(self.virtual_map) #background is black
            # im_mask_inv, im_mask = self.find_mask(homography_virtual_map)

            # im_white=np.full((1280,1280),255).astype('uint8')
            # im_white_masked=np.multiply(im_white, np.array(im_mask)).astype('uint8')
            # homography_virtual_map_masked=np.multiply(homography_virtual_map, im_mask_inv).astype('uint8')
            # self.cropped_virtual_map=homography_virtual_map_masked+im_white_masked

######  2  ############################################################################
            # im_mask_inv1, im_mask1=self.find_mask(img1)
            # im_mask_inv3, im_mask3=self.find_mask(img3)
            # im_mask_inv2, im_mask2=self.find_mask(img2)
            # im_mask_inv4, im_mask4=self.find_mask(img4)
            # # _, im_mask_l=self.find_mask(img_left)
            # # _, im_mask_r=self.find_mask(img_right)

            # img_white=np.full((1280, 1280), 255)

            # im_mask13=cv2.bitwise_and(im_mask1, im_mask3)
            # im_mask_inv_13=cv2.bitwise_and(im_mask_inv1, im_mask_inv3)
            # im_mask_inv_24=cv2.bitwise_and(im_mask_inv2, im_mask_inv4)
            # im_mask_inv1234=cv2.bitwise_and(im_mask_inv_13, im_mask_inv_24) #mask & mask_inv data type: numpy.ndarray
            # print("inv1234: "+str(im_mask_inv1234))

            # img_white_masked=np.multiply(img_white, im_mask1234)
            # # img2_masked=np.multiply(np.multiply(img2, im_mask_inv_13), np.array(im_mask_inv4))
            # # img4_masked=np.multiply(np.multiply(img4, im_mask_inv_13), np.array(im_mask_inv2))
            # img2_masked=np.multiply(img2, im_mask13)
            # img4_masked=np.multiply(img4, im_mask13)
            # img1_masked=np.multiply(img1, im_mask_inv1)
            # img3_masked=np.multiply(img3, im_mask_inv3)
            # # img_left_masked=np.multiply(np.multiply(img_left, im_mask1234), im_mask_r).astype('uint8')
            # # img_right_masked=np.multiply(np.multiply(img_right, im_mask1234), im_mask_l).astype('uint8')


            # # summed_image=(img1+img2_masked+img3+img4_masked+img_white_masked).astype('uint8')
            # summed_image=(img2_masked+img4_masked+img_white_masked).astype('uint8')
            # # summed_image=(img1_masked+img2_masked+img3_masked+img4_masked).astype('uint8')
            # summed_msg=self.bridge.cv2_to_compressed_imgmsg(summed_image)

            # self.pub_sum.publish(summed_msg)
            # print("summed_image time: "+str(time.time()-_time))


            # homography_virtual_map=self.crop_image(self.virtual_map) #background is black
            # im_mask_inv, im_mask = self.find_mask(homography_virtual_map)
            # print("inv"+str(im_mask_inv))

            # im_white=np.full((1280,1280),255)
            # im_white_masked=np.multiply(im_white, np.array(im_mask))
            # homography_virtual_map_masked=np.multiply(homography_virtual_map, np.array(im_mask_inv))
            # self.cropped_virtual_map=(homography_virtual_map_masked+im_white_masked).astype('uint8')

##### 3  #############################################################################
            # im_mask_inv1, im_mask1=self.find_mask(img1)
            # im_mask_inv3, im_mask3=self.find_mask(img3)
            # im_mask_inv2, im_mask2=self.find_mask(img2)
            # im_mask_inv4, im_mask4=self.find_mask(img4)

            im_mask13=cv2.bitwise_and(np.array(self.im_mask1).astype('uint8'), np.array(self.im_mask3).astype('uint8'))

            img2_masked=np.multiply(np.multiply(img2, im_mask13), self.im_mask4).astype('uint8')
            img4_masked=np.multiply(np.multiply(img4, im_mask13), self.im_mask2).astype('uint8')

    	    # summed_image=img_white_masked
            summed_image=img1+img2_masked+img3+img4_masked
            summed_image=cv2.bitwise_not(summed_image)

            summed_image[531:571,497:577]=156
            summed_image[535:570,597:706]=150
            summed_image[620:651, 510:547]=176
            summed_image[723:744, 617: 708]=135
            summed_image[608:659, 778:887]=153
            summed_image[659:692, 866:935]=158
            summed_image[631:650, 900:980]=165
            summed_image[589:597, 565:569]=155
            summed_image[526:599 , 558:725]=150



            # print("summed_image time: "+str(time.time()-_time))

            # for i in range(len(summed_image)):
            #     for j in range(len(summed_image[i])):
            #         if summed_image[i][j] >= 90:
            #             summed_image[i][j] = 255
            #         else:
            #             summed_image[i][j] = 0
            # summed_image = (summed_image <80) * summed_image
            summed_image= cv2.threshold(summed_image, 70, 255, cv2.THRESH_BINARY)[1]
            # summed_image= cv2.threshold(summed_image, 90, 255, cv2.THRESH_BINARY)[1]
            summed_image= cv2.threshold(summed_image, 110, 255, cv2.THRESH_BINARY)[1]
            # print(summed_image)
            # summed_image=self.image_processing(summed_image)
            # print(summed_image.shape)

            self.summed_image = summed_image

            homography_virtual_map=self.crop_image(self.virtual_map) #background is black
            # im_mask_inv, im_mask = self.find_mask(homography_virtual_map)

            # im_white=np.full((1280,1280),255).astype('uint8')
            # im_white_masked=np.multiply(im_white, np.array(im_mask)).astype('uint8')
            # homography_virtual_map_masked=np.multiply(homography_virtual_map, im_mask_inv).astype('uint8')

            # self.cropped_virtual_map=im_white_masked+homography_virtual_map_masked
            # self.cropped_virtual_map=im_white_masked+homography_virtual_map
            self.cropped_virtual_map=cv2.bitwise_not(homography_virtual_map)

            print("sum & crop image time: "+str(time.time()-_time))

            self.total_try += 1

            print("Compensation Success Count = %d/%d" %(self.success_try, self.total_try))
            print("SUM of Compensation Distance = %f" %(self.sum_compensation_distance))

##################################################################################

            try:
                fm = FeatureMatch(self.folder_path)
                # print("img1: "+str(self.virtual_map.shape)+", img2: "+str(summed_image.shape))
                # if self.cropped_virtual_map is None or summed_image is None:
                if self.cropped_virtual_map is None or summed_image is None:
                    print("IMAGE EMPTY")
                    raise Exception("Image Empty")
                else:
                    # M = fm.SIFT_FLANN_matching(self.cropped_virtual_map, summed_image)

                    # M = fm.ORB_BF_matching(summed_image, self.cropped_virtual_map)
                    M=fm.SIFT_BF_matching(summed_image, self.cropped_virtual_map)
                    # M = fm.SIFT_FLANN_matching(summed_image, self.cropped_virtual_map)
                    # M = fm.IMAGE_ALIGNMENT_ecc(summed_image, self.cropped_virtual_map)
                    # M=fm.SURF_BF_matching(summed_image, self.cropped_virtual_map)


                    if fm.status == True:
                        # self.vision_offset_publisher.publish(Point(fm.delta_x, fm.delta_y, fm.delta_theta))
                        # self.app_robotview.remove_points_during_vision_compensation(self.recent_pts)
                        # self.virtual_map = self.app_robotview.img

                        #Initialize Queue
                        # self.recent_pts = collections.deque(self.num_pts_delete*[(0.0,0.0)],self.num_pts_delete)

                        __time=time.time()
                        self.relocalization(M)
                        print("relocation time: "+str(time.time()-__time))

            except Exception as e:
                print(e)
                sys.exit("Feature Match error - debug1")

            #################

            print("Total Time (visual feedback): "+str(time.time()-_time))

            # bridge=CvBridge()
            # summed_msg=bridge.cv2_to_compressed_imgmsg(summed_image)
            # self.sum_pub.publish(summed_msg)

            # self.summed_image_prev = self.summed_image

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
        
        mid_real_virtual_x, mid_real_virtual_y, _= np.matmul(homography, [self.mid_real_photo_x, self.mid_real_photo_y, 1])

        del_x_virtual=mid_real_virtual_x-self.mid_real_photo_x
        
        del_y_virtual=mid_real_virtual_y-self.mid_real_photo_y
        del_th_virtual=-atan2(homography[0][1],homography[0][0])
        rotation=np.array([[cos(self.current_mid_predict_canvas_th), -sin(self.current_mid_predict_canvas_th)],
                            [sin(self.current_mid_predict_canvas_th), cos(self.current_mid_predict_canvas_th)]])
        del_x_canvas, del_y_canvas = np.matmul(rotation, [-del_x_virtual, -del_y_virtual])
        # *(-1) in del_x_virtual for calibration of x waypoint coordinate
        # *(-1) in del_y_virtual for calibration of image coordiate to canvas coordinate

        print("virtual photo mid: %d, %d / real photo midpnt: %d, %d" %(mid_real_virtual_x, mid_real_virtual_y, self.mid_real_photo_x, self.mid_real_photo_y))

        offset=Point()
        offset.x=del_x_canvas/self.pixMetRatio
        offset.y=del_y_canvas/self.pixMetRatio
        offset.z=del_th_virtual

        print(offset)
        print(homography)

        self.success_try += 1
        self.sum_compensation_distance += sqrt(offset.x*offset.x+offset.y*offset.y)

        self.pub_offset.publish(offset)


    def crop_image(self, _img):
        _time=time.time()
        img_not=cv2.bitwise_not(_img)
        padding=int(ceil(640*sqrt(2)))
        img_padding=np.full((_img.shape[0]+padding*2, _img.shape[1]+padding*2),0).astype('uint8')

        img_test=img_padding[padding:padding+_img.shape[0],padding:padding+_img.shape[1]]


        img_padding[padding:padding+_img.shape[0],padding:padding+_img.shape[1]]= img_not

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
        im_mask=(cv2.inRange(img, black_range1, black_range1)).astype('bool')
        # im_mask=np.dstack((im_mask, im_mask, im_mask))
        im_mask_inv=(1-im_mask).astype('bool')
        return im_mask_inv, im_mask

    def image_processing(self, _img):
        print("IMAGE PROCESSING")
        cv2.imwrite("/home/bkjung/before.png",_img)
        for i in range(len(_img)):
            for j in range(len(_img[i])):
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
                        # print("SALT_POINT: "+str(i)+", "+str(j))
                    else:
                        pass
                else:
                    pass
        cv2.imwrite("/home/bkjung/after.png", _img)
        print("done")
        return _img


    def undistort1(self, _img):
        homo1=np.array([[ -1.19105303e+00 ,  1.68060322e+00  , 1.12892282e+03],
 [ -1.81606541e-02 ,  7.67289427e-01 ,  1.18937721e+03],
 [ -3.84572928e-05  , 2.63351073e-03 ,  1.00000000e+00]])

        undist_img_binary= cv2.threshold( _img, self.threshold1, 255, cv2.THRESH_BINARY)[1]
        if self.is_first1 == True:
            img_for_mask = cv2.warpPerspective(cv2.undistort(img ,mtx, dst, None, mtx), homo1, (1280, 1280))
            self.im_mask_inv1, self.im_mask1 = self.find_mask(img_for_mask)
            self.is_first1=False
        
        return cv2.warpPerspective( cv2.bitwise_not(undist_img_binary) , homo1, (1280,1280))



    def undistort2(self, _img):
        img=self.bridge.compressed_imgmsg_to_cv2(_img)
        img=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        mtx=np.array([[382.750581, 0, 422.843185], [0, 385.64829129, 290.20197850], [0.0, 0.0, 1.0]])
        dst=np.array([-0.018077383, -0.0130221045547, 0.0003464289655, 0.00581105231096])

        homo2= np.array([[ -1.48776708e-01  , 5.50706654e+00 , -3.60421668e+02]
 [ -2.28522629e+00 ,  4.17778419e+00 ,  1.53166978e+03]
 [ -2.57162595e-04  , 6.57751893e-03  , 1.00000000e+00]])
        
        undist_img_binary= cv2.threshold(_img, self.threshold2, 255, cv2.THRESH_BINARY)[1]
        if self.is_first2 == True:
            img_for_mask = cv2.warpPerspective(cv2.undistort(img ,mtx, dst, None, mtx), homo2, (1280, 1280))
            self.im_mask_inv2, self.im_mask2 = self.find_mask(img_for_mask)
            self.is_first2=False
                   
        return cv2.warpPerspective( cv2.bitwise_not(undist_img_binary) , homo2, (1280,1280))

    def undistort3(self, _img):
        img=self.bridge.compressed_imgmsg_to_cv2(_img)
        img=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        mtx=np.array([[387.8191999285985, 0.0, 392.3078288789019],[ 0.0, 382.1093651210362, 317.43368009853674], [0.0, 0.0, 1.0]])
        dst=np.array([-0.008671221810333559, -0.013546386893040543, -0.00016537575030651431, 0.002659594999360673])

        homo3= np.array([[  1.40641268e+00  , 2.02456544e+00  , 1.15695930e+02],
 [  5.75929657e-02  , 3.08154517e+00  ,-4.22071086e+01],
 [  6.92537809e-05  , 3.17103531e-03  , 1.00000000e+00]])

        undist_img_binary= cv2.threshold(_img, self.threshold1, 255, cv2.THRESH_BINARY)[1]
        if self.is_first3 == True:
            img_for_mask = cv2.warpPerspective(cv2.undistort(img ,mtx, dst, None, mtx), homo3, (1280, 1280))
            self.im_mask_inv3, self.im_mask3 = self.find_mask(img_for_mask)
            self.is_first3=False
        
        return cv2.warpPerspective( cv2.bitwise_not(undist_img_binary) , homo3, (1280,1280))


    def undistort4(self, _img):
        img=self.bridge.compressed_imgmsg_to_cv2(_img)
        img=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        mtx=np.array([[384.2121883964654, 0.0, 423.16727407803353], [0.0, 386.8188468139677, 359.5190506678551], [0.0, 0.0, 1.0]])
        dst=np.array([-0.0056866549555025896, -0.019460881544303938, 0.0012937686026747307, -0.0031999317338443087])

        homo4= np.array([[  1.89007466e-01  , 6.81705144e+00  , 3.46768518e+03]
 [  5.49408235e+00   ,9.98981898e+00 , -1.56094237e+03]
 [  3.10194079e-04  , 1.58077911e-02 ,  1.00000000e+00]])

        undist_img_binary= cv2.threshold(_img, self.threshold4, 255, cv2.THRESH_BINARY)[1]
        if self.is_first4 == True:
            img_for_mask = cv2.warpPerspective(cv2.undistort(img ,mtx, dst, None, mtx), homo4, (1280, 1280))
            self.im_mask_inv4, self.im_mask4 = self.find_mask(img_for_mask)
            self.is_first4=False
        

        return cv2.warpPerspective( cv2.bitwise_not(undist_img_binary) , homo4, (1280,1280))
        

    def undistort_left(self, _img):
        img=self.bridge.compressed_imgmsg_to_cv2(_img)
        img=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        mtx=np.array([[496.88077412085187, 0.0, 486.19161191113693], [0.0, 497.77308359203073, 348.482250144119], [0.0, 0.0, 1.0]])
        dst=np.array([-0.27524035766660704, 0.055346669640229516, 0.002041430748143387, -0.0012188333190676689])

        homo5= np.array([[-1.22834137e-01,  1.44439375e+00,  8.77117625e+02],
            [ 1.50541911e-01,  1.54172775e+00,  5.51290841e+02],
            [-1.63990036e-04,  2.28324521e-03,  1.00000000e+00]])
        return cv2.warpPerspective( cv2.undistort(img, mtx, dst,None, mtx) , homo5, (1280,1280))


    def undistort_right(self, _img):
        img=self.bridge.compressed_imgmsg_to_cv2(_img)
        img=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        mtx=np.array([[494.0169295185964, 0.0, 483.6710483879246], [0.0, 495.87509303786857, 336.69262125267153], [0.0, 0.0, 1.0]])
        dst=np.array([-0.26693726936305806, 0.05239559897759021, 0.0024912074565555443, -0.0015904998174301696])


        homo5= np.array([[-3.31381222e-02,  1.44675753e+00,  8.79627593e+02],
            [ 2.46158878e-01,  1.37693071e+00,  4.88154549e+02],
            [-2.73379975e-05,  2.29141411e-03,  1.00000000e+00]])
        return cv2.warpPerspective( cv2.undistort(img, mtx, dst,None, mtx) , homo5, (1280,1280))

if __name__=='__main__':
    num_pts_delete = 150 #num_of_waypoints_to_delete_in_virtualmap_after_compensation
    VisualCompensation(num_pts_delete)