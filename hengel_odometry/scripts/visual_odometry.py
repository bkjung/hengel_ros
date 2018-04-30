#!/usr/bin/env python

import rospy
import numpy as np
import cv2, sys, time, math
# from bird_view import calibrate_mj
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt
from numpy.linalg import inv


class VisualOdometry():
    def __init__(self):
        #Initialize the node with rospy
        rospy.init_node('visual_odometry_node')
        ################
        rospy.on_shutdown(self.shutdown)
        ################
        #Create publisher
        self.vo_point_pub=rospy.Publisher("vo_current_point", Point, queue_size=50)
        init_status= True

        position = Point()
        x_state=[[0,0,0,0,0,0]]

        #3x3 matrix containing information about intrinsic, extrinsic parameters of the cam
        M=np.identity(3)
        
        #Parameters for feature detection
        feature_params = dict( maxCorners = 100,
                            qualityLevel = 0.3,
                            minDistance = 7,
                            blockSize = 7 )

        #For KF (state estimation)
        A=zeros((6,6))
        for i in range(6):
            if i%2==0:
                A[i]=1
                A[i+1]=1
            else:
                A[i]=1

        while True:
            #####
            #Step 1. Capture current k_th frame, feature detection
            self.image_cv = rospy.Subscriber('/usb_cam/image_raw', self.callback_image_sub)
            image = birdview.birdview_transform(image_cv)
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            if init_status:
                old_image = image
                old_gray = gray
                init_status=False
                break
                #############Corner detection###########
            old_corners = cv2.goodFeaturesToTrack(old_gray, mask = None, **feature_params)
                        
            #Step 2. Predict current position, assuming continuous motion. (pred_p_k = p_k-1 + (p_k-1 - p_k-2) )
            estimate_state= A * x_state[-1]
            tx= estimate_state[1]
            ty= estimate_state[3]
            theta=estimate_state[5]

            #Step 3. Predict Homography Matrix H_k^- (using SVD)
            SE=[[math.cos(theta), math.sin(theta), tx], [-math.sin(theta), math.cos(theta), ty], [0,0,1]]
            H_estimate=M*SE*inv(M)


            #Step 4. determine search window according to H_k^-. Find corresponding pixels F_k-1^' by lucas-kanade algorithm
            winsize_arr=[]

            for corner in corners:
                ######Determine Search Window##########
                winsize_arr.append((10,10))
                winsize=(10,10)

            lk_params = dict( winSize,
                            maxLevel = 2,
                            criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
            
            ############Should be changed (different window size) ###############
            new_corners, st, err = cv2.calcOpticalFlowPyrLK(old_gray, gray, old_corners, None, **lk_params)

                
            # Select good points
            good_new = new_corners[st==1]
            good_old = corners[st==1]

            data=[]
            for i,(new,old) in enumerate(zip(good_new,good_old)):
                a,b = new.ravel()
                c,d = old.ravel()
                data.append([a,b],[c,d])

            #Step 5. Compute initial estimate of homography matrix H_k^' (using RANSAC)
            RANSAC.ransac(data)

            #Step 6. Outlier rejection

            #Step 7. estimate camera's ego-motion through homography, estimate current pos., update the state
            x_state.append()

    def callback_image_sub(self, _image):
        bridge= CvBridge()
        return bridge.imgmsg_to_cv2(_image)


if __name__ == '__main__':
    try:
        VisualOdometry()
        print("End of Main Function")

    except Exception as e:
        print(e)
        rospy.loginfo("shutdown program.")