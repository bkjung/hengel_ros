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
    def __init__(self, option_str):
        self.option_debug = False
        self.option_without_save = False

        if option_str=='run':
            pass
        elif option_str=='debug':
            self.option_debug = True
        elif option_str=='without_save':
            self.option_without_save = True
        else:
            print("Wrong Argument")
            sys.exit(1)

        while True:
            word= raw_input("WHAT IS THE WIDTH AND HEIGHT OF CANVAS?\n Type: ")
            self.width=float(word.split()[0])
            self.height=float(word.split()[1])
            break
        # self.num_pts_delete = _num_pts_delete
        self.recent_pts = collections.deque(150*[(0.0,0.0)],150)

        self.home_path = expanduser("~")
        self.folder_path = self.home_path + "/FEATURE_MATCH/" + time.strftime("%y%m%d_%H%M%S")
        os.system("mkdir -p " + self.folder_path)

        self.initialize()


    def initialize(self):
        rospy.init_node('hengel_camera_compensation', anonymous=False)

        self.isNavigationStarted = False
        self.bridge=CvBridge()
        #self.pixMetRatio=300
        self.pixMetRatio=250

        self.line_thickness= 0.022
        # self.line_thickness=0.1

        self.canvas_padding = self.line_thickness * self.pixMetRatio * 2        #2 means each side
        self.real_canvas_crop_padding = 0.3 * self.pixMetRatio

        self.view_padding=int(ceil(1280*sqrt(2))) #Robot may see outside of canvas

        self.cropped_virtual_map=np.full((1280,1280),255).astype('uint8')
        self.virtual_map=np.full((int(self.pixMetRatio*self.height + self.canvas_padding*2), int(self.pixMetRatio*self.width+self.canvas_padding*2)), 255)
        self.app_robotview=RobotView(self.virtual_map, self.pixMetRatio, self.line_thickness, self.canvas_padding) # Add the endpoint into the virtual map

        self.pi_left_img=np.array([])
        self.pi_right_img=np.array([])

        self.isProcessingVirtualmapTime = False
        self.mid_predict_canvas_x=collections.deque(1000*[0.0],300)
        self.mid_predict_canvas_y=collections.deque(1000*[0.0],300)
        self.mid_predict_canvas_th=collections.deque(1000*[0.0],300)
        self.mid_predict_canvas_time=collections.deque(1000*[0.0],300)
        self.end_predict_canvas_x= collections.deque(1000*[0.0],300)
        self.end_predict_canvas_y= collections.deque(1000*[0.0],300)

        # self.mid_real_photo_x=640+55.77116996/2
        self.mid_real_photo_x=640
        self.mid_real_photo_y=640

        self.endPoint_callback=message_filters.Subscriber('/endpoint', Point)
        self.midPoint_callback=message_filters.Subscriber('/midpoint', Point)
        self.midPoint_time_callback=message_filters.Subscriber('/midpoint_time', Time)

        self.ts=message_filters.ApproximateTimeSynchronizer([self.endPoint_callback, self.midPoint_callback, self.midPoint_time_callback], 10, 0.1, allow_headerless=True)
        self.ts.registerCallback(self.sync_virtual_callback)

        self.pub_virtual_map=rospy.Publisher('/virtual_map', CompressedImage, queue_size=3)

        self.calculate_homography(self.pixMetRatio)

        self.callback1=message_filters.Subscriber('/genius1/compressed', CompressedImage)
        self.callback2=message_filters.Subscriber('/genius2/compressed', CompressedImage)
        self.callback3=message_filters.Subscriber('/genius3/compressed', CompressedImage)
        self.callback4=message_filters.Subscriber('/genius4/compressed', CompressedImage)

        # self.summed_image = None
        # self.summed_image_prev = None

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

        self.threshold1=110
        self.threshold2=110
        self.threshold3=110
        self.threshold4=110

        self.ts_2.registerCallback(self.sync_real_callback)


        self.vision_offset_publisher=rospy.Publisher('/offset_change', Point, queue_size=5)
        ############################ DEBUG ################################
        # self.pub_pi_l=rospy.Publisher('/pi_left/undist', Image, queue_size=5 )
        # self.pub_pi_r=rospy.Publisher('/pi_right/undist', Image, queue_size=5 )
        self.pub_sum=rospy.Publisher('/summed_image/compressed',CompressedImage, queue_size=5)


        self.success_try = 0
        self.total_try = 0

        self.sum_compensation_distance = 0.0

        self.x_img_max=None
        self.x_img_min=None
        self.y_img_max=None
        self.y_img_min=None


        rospy.spin()


#################################################################
    def callback_left(self, _img):
        # print("left")
        self.pi_left_img=self.undistort_left(_img)

    def callback_right(self, _img):
        # print("right")
        self.pi_right_img=self.undistort_right(_img)

    def sync_real_callback(self, _img1, _img2, _img3, _img4):
        if self.option_debug:
            print("------image time------")
            print(_img1.header.stamp.to_nsec())
            print(_img2.header.stamp.to_nsec())
            print(_img3.header.stamp.to_nsec())
            print(_img4.header.stamp.to_nsec())
            print("----------------------")
            self.isNavigationStarted=True
            self.app_robotview.isPaintStarted=True

        if self.isNavigationStarted==True:
            if self.app_robotview.isPaintStarted == True:
                print("\n-----------------sync real-----------------")
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
                self.current_end_predict_canvas_x = self.end_predict_canvas_x[min_index]
                self.current_end_predict_canvas_y = self.end_predict_canvas_y[min_index]
                self.isProcessingVirtualmapTime = False

                self.mid_predict_img_x=-self.current_mid_predict_canvas_x *self.pixMetRatio + self.canvas_padding
                self.mid_predict_img_y=self.virtual_map.shape[0]-(self.current_mid_predict_canvas_y*self.pixMetRatio +self.canvas_padding)
                self.mid_predict_img_th=-self.current_mid_predict_canvas_th

                # print("Processing Virtualmap Sync Time: "+str(time.time()-_time))

                img1 = self.undistort1(_img1)
                img2 = self.undistort2(_img2)
                img3 = self.undistort3(_img4)
                img4 = self.undistort4(_img3)
                #ONLY FOR DEBUGGING!!!!!!!
                if self.option_debug:
                    bridge=CvBridge()
                    summed_msg=bridge.cv2_to_compressed_imgmsg(img1+img2+img3+img4)
                    self.pub_sum.publish(summed_msg)
                    cv2.imwrite(self.folder_path+"/"+time.strftime("%y%m%d_%H%M%S")+"_summed_1.png",img1+img2+img3+img4)

                im_mask13=cv2.bitwise_and(np.array(self.im_mask1).astype('uint8'), np.array(self.im_mask3).astype('uint8'))

                img2_masked=np.multiply(np.multiply(img2, im_mask13), self.im_mask4).astype('uint8')
                img4_masked=np.multiply(np.multiply(img4, im_mask13), self.im_mask2).astype('uint8')

                summed_image_not=img1+img2_masked+img3+img4_masked
                summed_image_copy=copy.deepcopy(summed_image)
                #ONLY FOR DEBUGGING!!!!!!!
                #if self.option_debug:
                #    bridge=CvBridge()
                #    summed_msg=bridge.cv2_to_compressed_imgmsg(summed_image_not)
                #    self.pub_sum.publish(summed_msg)

                #Cover robot area
                x1=[526, 525, 507, 494, 517, 487, 500, 489, 491, 530, 593, 564, 765, 792]
                y1=[538, 724, 643, 643, 608, 590, 565, 613, 609, 515, 528, 507, 586, 598]
                x2=[765, 527, 527, 507, 527, 527, 527, 491, 495, 593, 623, 568, 793, 810]
                y2=[748, 731, 682, 649, 643, 608, 590, 615, 610, 541, 541, 515, 648, 641]

                y1_ratio=[int((y-1-640)*self.pixMetRatio/float(400)+640) for y in y1]
                y2_ratio=[int(ceil((y+1-640)*self.pixMetRatio/float(400)+640)) for y in y2]
                x1_ratio=[int((x-1-640)*self.pixMetRatio/float(400)+640) for x in x1]
                x2_ratio=[int(ceil((x+1-640)*self.pixMetRatio/float(400)+640)) for x in x2]


                #ONLY FOR DEBUGGING!!!!!!!
                #if self.option_debug:
                #    bridge=CvBridge()
                #    summed_msg=bridge.cv2_to_compressed_imgmsg(summed_image_not)
                #    self.pub_sum.publish(summed_msg)

                # print("summed_image time: "+str(time.time()-_time))

                # for i in range(len(summed_image)):
                #     for j in range(len(summed_image[i])):
                #         if summed_image[i][j] >= 90:
                #             summed_image[i][j] = 255
                #         else:
                #             summed_image[i][j] = 0
                # summed_image = (summed_image <80) * summed_image
                # summed_image= cv2.threshold(summed_image, 90, 255, cv2.THRESH_BINARY)[1]
                # print(summed_image)
                # summed_image=self.image_processing(summed_image)
                # print(summed_image.shape)
                # summed_image= cv2.threshold(summed_image, 110, 255, cv2.THRESH_BINARY)[1]

                # self.summed_image = summed_image

                homography_virtual_map, homography =self.crop_image(self.virtual_map) #background is black
                self.cropped_virtual_map=cv2.bitwise_not(homography_virtual_map)
                virtual_map_copy=copy.deepcopy(self.cropped_virtual_map)

                for i in xrange(len(y1)):
                    summed_image_not[y1_ratio[i]:y2_ratio[i], x1_ratio[i]:x2_ratio[i]]=0
                    self.cropped_virtual_map[y1_ratio[i]:y2_ratio[i], x1_ratio[i]:x2_ratio[i]]=255

                #Cover area outside of canvas
                #Make homography
                homo_inv= inv(homography)

                ####ONLY FOR DEBUGGING####
                #Compute vertices of canvas
                virtual_map_crop_pts=[[self.x_img_min, self.y_img_min], [self.x_img_min, self.y_img_max], [self.x_img_max, self.y_img_max], [self.x_img_max, self.y_img_min]]
                virtual_map_padding=[[-self.real_canvas_crop_padding, -self.real_canvas_crop_padding],[-self.real_canvas_crop_padding,2*self.canvas_padding+self.real_canvas_crop_padding], [2*self.canvas_padding+self.real_canvas_crop_padding, 2*self.canvas_padding+self.real_canvas_crop_padding], [2*self.canvas_padding+self.real_canvas_crop_padding, -self.real_canvas_crop_padding]] #canvas padding
                # virtual_map_padding=[[-self.real_canvas_crop_padding,-self.real_canvas_crop_padding],[-self.real_canvas_crop_padding,3*self.real_canvas_crop_padding], [3*self.real_canvas_crop_padding, 3*self.real_canvas_crop_padding], [3*self.real_canvas_crop_padding, -self.real_canvas_crop_padding]] #canvas padding
                # virtual_map_padding=[[-2*self.real_canvas_crop_padding,-2*self.real_canvas_crop_padding],[-2*self.real_canvas_crop_padding,4*self.real_canvas_crop_padding], [4*self.real_canvas_crop_padding, 4*self.real_canvas_crop_padding], [4*self.real_canvas_crop_padding, -2*self.real_canvas_crop_padding]] #canvas padding

                virtual_map_crop_pts=np.matrix(virtual_map_crop_pts)+np.matrix(virtual_map_padding)
                virtual_map_crop_pts_padding=[[a.item(0)+self.view_padding, a.item(1)+self.view_padding] for a in virtual_map_crop_pts] #Add view point

                real_map_crop_pts=[]

                #Compute vertices of canvas in image coordinate
                for i in xrange(4):
                    real_map_crop_pnt= np.matmul(homography, [virtual_map_crop_pts_padding[i][0],virtual_map_crop_pts_padding[i][1],1])
                    real_map_crop_pts.append([int(real_map_crop_pnt[0]), int(real_map_crop_pnt[1])])
                real_map_crop_pts = np.array([real_map_crop_pts])

                #Make mask of the vertices
                mask=np.zeros((summed_image_not.shape[0], summed_image_not.shape[1]),dtype=np.uint8 )
                cv2.fillPoly(mask, real_map_crop_pts, (255))

                # summed_image_not=cv2.fill((1280,1280), (255)).astype('uint8')
                res=cv2.bitwise_and(summed_image_not, summed_image_not, mask=mask)
                res_copy=cv2.bitwise_and(summed_image_not_copy, summed_image_not_copy, mask=mask)
                # img_white=np.full((1280,1280),(255)).astype('uint8')
                # res=cv2.bitwise_and(img_white, img_white, mask=mask)
                summed_image= cv2.bitwise_not(res)
                summed_image_copy=cv2.bitwise_not(res_copy)

                print("sum & crop image time: "+str(time.time()-_time))

                self.total_try += 1

                print("Compensation Success Count = %d/%d" %(self.success_try, self.total_try))
                print("SUM of Compensation Distance = %f" %(self.sum_compensation_distance))

##################################################################################
                try:
                    fm = FeatureMatch(self.folder_path, self.option_without_save)
                    # print("img1: "+str(self.virtual_map.shape)+", img2: "+str(summed_image.shape))
                    # if self.cropped_virtual_map is None or summed_image is None:
                    if self.cropped_virtual_map is None or summed_image is None:
                        print("IMAGE EMPTY")
                        raise Exception("Image Empty")
                    else:
                        # M = fm.SIFT_FLANN_matching(self.cropped_virtual_map, summed_image)

                        # M = fm.ORB_BF_matching(summed_image, self.cropped_virtual_map)
                        #M=fm.SIFT_BF_matching(summed_image, self.cropped_virtual_map, summed_image_copy)
                        M=fm.SIFT_BF_matching(summed_image, self.cropped_virtual_map,summed_image_copy, virtual_map_copy)
                        # M = fm.SIFT_FLANN_matching(summed_image, self.cropped_virtual_map)
                        # M = fm.IMAGE_ALIGNMENT_ecc(summed_image, self.cropped_virtual_map)
                        # M=fm.SURF_BF_matching(summed_image, self.cropped_virtual_map)

                        if fm.status == True:
                            #self.app_robotview.remove_points_during_vision_compensation(self.recent_pts, int((time.time()-_time)/0.02))
                            self.virtual_map = self.app_robotview.img

                            #Initialize Queue
                            # self.recent_pts = collections.deque(self.num_pts_delete*[(0.0,0.0)],self.num_pts_delete)

                            __time=time.time()
                            _pnt = self.relocalization(M)
                            # self.vision_offset_publisher.publish(Point(fm.delta_x, fm.delta_y, fm.delta_theta))
                            self.vision_offset_publisher.publish(_pnt)
                            self.app_robotview.run(Point(), Point(self.current_end_predict_canvas_x, self.current_end_predict_canvas_y, 0), self.line_thickness*3)
                            virtual_map_marked= self.app_robotview.img_copy
                            if not self.option_without_save:
                                file_time = time.strftime("%y%m%d_%H%M%S")
                                cv2.imwrite(self.folder_path+"/SUMMED_IMAGE_"+file_time+".png", virtual_map_marked)
                            # print("relocation time: "+str(time.time()-__time))
                            print("Total Time (visual feedback): "+str(time.time()-_time))
                except Exception as e:
                    print(e)
                    sys.exit("Feature Match error - debug1")
            #################

                #ONLY FOR DEBUGGING!!!!!!!
                if self.option_debug:
                    cv2.imwrite(self.folder_path+"/"+time.strftime("%y%m%d_%H%M%S")+"_summed_2.png",summed_image)
                #    bridge=CvBridge()
                #    summed_msg=bridge.cv2_to_compressed_imgmsg(summed_image)
                #    self.pub_sum.publish(summed_msg)

                # self.summed_image_prev = self.summed_image
            else:
                print("Painting not started yet")



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
            self.end_predict_canvas_x.appendleft(_endPoint.x)
            self.end_predict_canvas_y.appendleft(_endPoint.y)
            self.mid_predict_canvas_time.appendleft(_midPointTime.data.to_nsec())

            self.end_predict_img_y=(self.height-_midPoint.y)*self.pixMetRatio
            self.end_predict_img_x=-_midPoint.x * self.pixMetRatio

            if self.x_img_min is None:
                self.x_img_min = self.end_predict_img_x
                self.x_img_max = self.end_predict_img_x
                self.y_img_min = self.end_predict_img_y
                self.y_img_max = self.end_predict_img_y
            else:
                self.x_img_max=max(self.x_img_max, self.end_predict_img_x)
                self.x_img_min=min(self.x_img_min, self.end_predict_img_x)
                self.y_img_max=max(self.y_img_max, self.end_predict_img_y)
                self.y_img_min=min(self.y_img_min, self.end_predict_img_y)

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

        # print("virtual photo mid: %d, %d / real photo midpnt: %d, %d" %(mid_real_virtual_x, mid_real_virtual_y, self.mid_real_photo_x, self.mid_real_photo_y))

        offset=Point()
        offset.x=del_x_canvas/self.pixMetRatio
        offset.y=del_y_canvas/self.pixMetRatio
        offset.z=del_th_virtual

        print(offset)
        # print(homography)

        self.success_try += 1

        dist= sqrt(pow(offset.x,2)+pow(offset.y,2))

        #if dist>=0.02:
        if dist>=0.1:
            offset=Point()
        else:
            #print("OFFSET less than limit (= 0.02)")
            print("OFFSET less than limit (= 0.1)")
            self.sum_compensation_distance += sqrt(offset.x*offset.x+offset.y*offset.y)
        return offset

        # self.pub_offset.publish(offset)


    def crop_image(self, _img):
        _time=time.time()
        img_not=cv2.bitwise_not(_img)
        img_padding=np.full((_img.shape[0]+self.view_padding*2, _img.shape[1]+self.view_padding*2),0).astype('uint8')

        img_padding[self.view_padding:self.view_padding+_img.shape[0],self.view_padding:self.view_padding+_img.shape[1]]= img_not

        half_map_size_diagonal = 1280/sqrt(2)

        #middle point of the cropped image in virtual map coordinate
        # midpnt_offset=55.77116996/2 # in virtual map coordiate
        # x_mid_crop=self.mid_predict_img_x-midpnt_offset*cos(self.mid_predict_img_th)
        # y_mid_crop=self.mid_predict_img_y+midpnt_offset*sin(self.mid_predict_img_th)

        # imgPts=[[x_mid_crop-half_map_size_diagonal*cos(pi/4-self.mid_predict_img_th), y_mid_crop-half_map_size_diagonal*sin(pi/4-self.mid_predict_img_th)],
        #             [x_mid_crop-half_map_size_diagonal*cos(pi/4+self.mid_predict_img_th), y_mid_crop+half_map_size_diagonal*sin(pi/4+self.mid_predict_img_th)],
        #             [x_mid_crop+half_map_size_diagonal*cos(pi/4-self.mid_predict_img_th), y_mid_crop+half_map_size_diagonal*sin(pi/4-self.mid_predict_img_th)],
        #             [x_mid_crop+half_map_size_diagonal*cos(pi/4+self.mid_predict_img_th), y_mid_crop-half_map_size_diagonal*sin(pi/4+self.mid_predict_img_th)]]

        x_mid_crop=self.mid_predict_img_x
        y_mid_crop=self.mid_predict_img_y

        imgPts=[[x_mid_crop-half_map_size_diagonal*cos(pi/4-self.mid_predict_img_th), y_mid_crop-half_map_size_diagonal*sin(pi/4-self.mid_predict_img_th)],
                    [x_mid_crop-half_map_size_diagonal*cos(pi/4+self.mid_predict_img_th), y_mid_crop+half_map_size_diagonal*sin(pi/4+self.mid_predict_img_th)],
                    [x_mid_crop+half_map_size_diagonal*cos(pi/4-self.mid_predict_img_th), y_mid_crop+half_map_size_diagonal*sin(pi/4-self.mid_predict_img_th)],
                    [x_mid_crop+half_map_size_diagonal*cos(pi/4+self.mid_predict_img_th), y_mid_crop-half_map_size_diagonal*sin(pi/4+self.mid_predict_img_th)]]

        imgPts_padding=[[a[0]+self.view_padding, a[1]+self.view_padding] for a in imgPts]
        imgPts_padding=np.array(imgPts_padding)

        objPts=np.array([[0,0], [0, 1280], [1280, 1280], [1280,0]])
        homography, _=cv2.findHomography(imgPts_padding, objPts)

        img_padding=np.uint8(img_padding)

        return cv2.warpPerspective(img_padding, homography,(1280,1280)), homography

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
        img=self.bridge.compressed_imgmsg_to_cv2(_img)
        img=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if self.option_debug:
            cv2.imwrite(self.folder_path+"/"+time.strftime("%y%m%d_%H%M%S")+"_img1.png",img)
        mtx=np.array([[393.8666817683925, 0.0, 399.6813895086665], [0.0, 394.55108358870405, 259.84676565717876], [0.0, 0.0, 1.0]])
        dst=np.array([-0.0032079005049939543, -0.020856072501002923, 0.000252242294186179, -0.0021042704510431365])

        homo= self.homography[0]

        undist_img_binary= cv2.threshold(cv2.undistort(img ,mtx,dst ,None, mtx), self.threshold1, 255, cv2.THRESH_BINARY)[1]

        if self.is_first1 == True:
            img_for_mask = cv2.warpPerspective(cv2.undistort(img ,mtx, dst, None, mtx), homo, (1280, 1280))
            self.im_mask_inv1, self.im_mask1 = self.find_mask(img_for_mask)
            self.is_first1=False

        return cv2.warpPerspective( cv2.bitwise_not(undist_img_binary) , homo, (1280,1280))




    def undistort2(self, _img):
        img=self.bridge.compressed_imgmsg_to_cv2(_img)
        img=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if self.option_debug:
            cv2.imwrite(self.folder_path+"/"+time.strftime("%y%m%d_%H%M%S")+"_img2.png",img)
        mtx=np.array([[382.750581, 0, 422.843185], [0, 385.64829129, 290.20197850], [0.0, 0.0, 1.0]])
        dst=np.array([-0.018077383, -0.0130221045547, 0.0003464289655, 0.00581105231096])

        homo = self.homography[1]

        undist_img_binary= cv2.threshold(cv2.undistort(img ,mtx,dst ,None, mtx), self.threshold2, 255, cv2.THRESH_BINARY)[1]

        if self.is_first2 == True:
            img_for_mask = cv2.warpPerspective(cv2.undistort(img ,mtx, dst, None, mtx), homo, (1280, 1280))
            self.im_mask_inv2, self.im_mask2 = self.find_mask(img_for_mask)
            self.is_first2=False

        return cv2.warpPerspective( cv2.bitwise_not(undist_img_binary) , homo, (1280,1280))

    def undistort3(self, _img):
        img=self.bridge.compressed_imgmsg_to_cv2(_img)
        img=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if self.option_debug:
            cv2.imwrite(self.folder_path+"/"+time.strftime("%y%m%d_%H%M%S")+"_img3.png",img)
        mtx=np.array([[387.8191999285985, 0.0, 392.3078288789019],[ 0.0, 382.1093651210362, 317.43368009853674], [0.0, 0.0, 1.0]])
        dst=np.array([-0.008671221810333559, -0.013546386893040543, -0.00016537575030651431, 0.002659594999360673])

        homo= self.homography[2]

        undist_img_binary= cv2.threshold(cv2.undistort(img ,mtx,dst ,None, mtx), self.threshold1, 255, cv2.THRESH_BINARY)[1]

        if self.is_first3 == True:
            img_for_mask = cv2.warpPerspective(cv2.undistort(img ,mtx, dst, None, mtx), homo, (1280, 1280))
            self.im_mask_inv3, self.im_mask3 = self.find_mask(img_for_mask)
            self.is_first3=False

        return cv2.warpPerspective( cv2.bitwise_not(undist_img_binary) , homo, (1280,1280))


    def undistort4(self, _img):
        img=self.bridge.compressed_imgmsg_to_cv2(_img)
        img=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if self.option_debug:
            cv2.imwrite(self.folder_path+"/"+time.strftime("%y%m%d_%H%M%S")+"_img4.png",img)
        mtx=np.array([[384.2121883964654, 0.0, 423.16727407803353], [0.0, 386.8188468139677, 359.5190506678551], [0.0, 0.0, 1.0]])
        dst=np.array([-0.0056866549555025896, -0.019460881544303938, 0.0012937686026747307, -0.0031999317338443087])

        homo= self.homography[3]

        undist_img_binary= cv2.threshold(cv2.undistort(img ,mtx,dst ,None, mtx), self.threshold4, 255, cv2.THRESH_BINARY)[1]

        if self.is_first4 == True:
            img_for_mask = cv2.warpPerspective(cv2.undistort(img ,mtx, dst, None, mtx), homo, (1280, 1280))
            self.im_mask_inv4, self.im_mask4 = self.find_mask(img_for_mask)
            self.is_first4=False


        return cv2.warpPerspective(cv2.bitwise_not(undist_img_binary) , homo, (1280,1280))


    # def undistort_left(self, _img):
    #     img=self.bridge.compressed_imgmsg_to_cv2(_img)
    #     img=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #     mtx=np.array([[496.88077412085187, 0.0, 486.19161191113693], [0.0, 497.77308359203073, 348.482250144119], [0.0, 0.0, 1.0]])
    #     dst=np.array([-0.27524035766660704, 0.055346669640229516, 0.002041430748143387, -0.0012188333190676689])

    #     homo5= np.array([[-1.22834137e-01,  1.44439375e+00,  8.77117625e+02],
    #         [ 1.50541911e-01,  1.54172775e+00,  5.51290841e+02],
    #         [-1.63990036e-04,  2.28324521e-03,  1.00000000e+00]])
    #     return cv2.warpPerspective( cv2.undistort(img, mtx, dst,None, mtx) , homo5, (1280,1280))


    # def undistort_right(self, _img):
    #     img=self.bridge.compressed_imgmsg_to_cv2(_img)
    #     img=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #     mtx=np.array([[494.0169295185964, 0.0, 483.6710483879246], [0.0, 495.87509303786857, 336.69262125267153], [0.0, 0.0, 1.0]])
    #     dst=np.array([-0.26693726936305806, 0.05239559897759021, 0.0024912074565555443, -0.0015904998174301696])


    #     homo5= np.array([[-3.31381222e-02,  1.44675753e+00,  8.79627593e+02],
    #         [ 2.46158878e-01,  1.37693071e+00,  4.88154549e+02],
    #         [-2.73379975e-05,  2.29141411e-03,  1.00000000e+00]])
    #     return cv2.warpPerspective( cv2.undistort(img, mtx, dst,None, mtx) , homo5, (1280,1280))

    def calculate_homography(self, _ratio):
        robotPtsArr=[]
        imgPtsArr=[]

        #genius1
        robotPtsArr.append([
            [75,-80],[55,-80],[55,-100],[35,-80],[15,-80],[65,-60],[45,-60],[25,-60],[5,-60],[15,-50],[35,-50],[55,-50],[65,-40],[45,-40],[25,-40],[45,-30],[45,-20],[10,25],[20,25],[30,25],[40,25],[50,25],[60,35],[50,35],[30,35],[30,45],[40,45],[10,55],[30,55],[40,55],[10,65],[20,65],[50,65],[30,75]
            ])
        #genius1
        imgPtsArr.append([
            [670,264],[696,330],[776,332],[728,410],[768,513],[607,295],[627,367],[652,456],[658,572],[618,509],[596,407],[578,392],[533,293],[545,366],[559,457],[503,365],[462,365],[239,534],[252,478],[261,427],[270,382],[277,343],[247,307],[236,343],[215,428],[171,428],[184,382],[87,534],[124,427],[140,382],[34,534],[58,477],[115,342],[35,426]
            ])

        #genius2
        robotPtsArr.append([
            [-5,-100],[-25,-100],[75,-100],[-65,-80],[-45,-80],[-25,-80],[-5,-80],[35,-80],[75,-80],[-65,-70],[-5,-70],[45,-70],[75,-60],[45,-60],[-5,-60],[-55,-60],[-85,-60],[-65,-50],[-5,-50],[25,-50],[45,-50],[65,-50],[75,-40],[45,-40],[25,-40],[-25,-40],[-55,-40],[-55,-30],[65,-30]
            ] )
        #genius2
        imgPtsArr.append([
            [430,285],[495,287],[162,282],[651,344],[579,344],[505,343],[431,342],[283,340],[128,339],[667,379],[432,376],[232,373],[82,414],[217,414],[433,415],[644,418],[771,418],[709,463],[434,461],[294,460],[200,460],[104,460],[19,517],[180,516],[284,516],[535,516],[684,518],[708,580],[38,582]
            ])

        #genius3
        robotPtsArr.append([[-10,65], [-30, 65], [-60,65], [-80, 65],
            [-20, 55], [-30, 55], [-50,55], [-80, 55],
            [-10,35], [-30, 35], [-50,35], [-80, 35],
            [-40, 25] , [-60, 25],
            [-40, 15], [-50, 15], [-70, 15],
            [ -35, -30], [-45, -30], [-55, -30], [-85, -30], [-105, -30],
            [ -25 ,-40], [-35, -40], [-45, -40], [-55, -40],
            [-15, -60], [-25, -60], [-45, -60], [-55, -60], [-65, -60],
            [-25, -80], [-45, -80], [-65, -80],
            [-65, -100]]
            )
        #genius3
        imgPtsArr.append([[775.3, 569], [729.3, 460.3], [676.2, 338.8], [648.8, 275],
            [702.7, 513], [683.7, 462], [650,375.3], [613.8,276],
            [604, 513.3], [590, 462.3], [567, 375.7], [542.8,276.8],
            [534.8, 416.5], [520.2, 339],
            [491, 417], [485.8, 376.5], [475.8, 307.5],
            [294.8, 441.8], [300,399.2], [304.2, 360.5], [314.2, 266.5], [318.8, 215.8],
            [241.8, 490.5], [249.8, 442.7], [257.6, 399.5], [264, 361],
            [130, 548], [146.8, 492.8], [173.8, 401], [185, 362.3], [232, 327.2],
            [51.7, 494.3], [88.7, 402.7], [118.3, 329.3],
            [41.7, 330.3]])

        #genius4
        robotPtsArr.append([[70,25], [70,55], [70, 95],
            [60,55],
            [50,25], [50,35], [50,45], [50,75], [50, 105],
            [30,35], [30, 55],
            [20, 35], [20, 45], [20,105],
            [0,35], [0,45],
            [-10, 45], [-10,55], [-10,105],
            [-20, 55], [-20, 95],
            [-30, 35], [-30, 55],
            [-50, 25], [-50, 35], [-50, 95],
            [-60,45], [-60, 55], [-60, 65], [-60, 85],
            [-70,25], [-70, 45], [-70, 95],
            [-80, 35], [-80,55],
            [-90, 55], [-90, 95]])
        #genius4
        imgPtsArr.append([[777,586.7], [704.2, 451.5], [643, 339.5],
            [664,452.2],
            [676, 587], [655.8, 535.2], [637.8, 490.4], [611.1, 419.6], [572.9,319.1],
            [563.2,535.2], [544,452.8],
            [517.8, 535.8], [511, 492], [484.2, 320.4],
            [428, 535.8], [428.3, 492.3],
            [385.3, 493], [387.5, 453.8], [395.6, 320.5],
            [348.9, 454.5], [363.6, 342.4],
            [294, 537], [310.2, 455.2],
            [184.8, 589.2], [203, 538.2], [ 272, 344.8],
            [177.2, 494.2], [193, 456.8],[208, 423.2], [231.5, 367.2],
            [88.2, 590.2], [134.5, 494.5], [210.2, 344.2],
            [68.2, 539], [115.2, 456.8],
            [74.8, 457.2], [148, 344]]
            )

        objPts = [[[(point_r[1]+9.33)*(_ratio/float(100))+640.0, point_r[0]*(_ratio/float(100))+640.0]  for point_r in robotPts]  for robotPts in robotPtsArr]

        self.homography=[]

        for i in xrange(4):
            hom, _ = cv2.findHomography(np.array(imgPtsArr[i]), np.array(objPts[i]), cv2.RANSAC, 10)
            self.homography.append(hom)
            print("%d homography appended" %(i+1))


if __name__=='__main__':
    # num_pts_delete = 150 #num_of_waypoints_to_delete_in_virtualmap_after_compensation
    # VisualCompensation(num_pts_delete)
    if len(sys.argv)==1:
        VisualCompensation('run')
    elif len(sys.argv)==2:
        if sys.argv[1]=='debug':
            VisualCompensation('debug')
        elif sys.argv[1]=='without_save':
            VisualCompensation('without_save')
    else:
        print("Wrong Argument")
