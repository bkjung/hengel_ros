#!/usr/bin/env python

import rospy
import numpy as np
import cv2, sys, time, math, sys, os
import birdview
import RANSAC
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt
from numpy.linalg import inv

class VisualOdometry():
    image=[]
    gray =[]
    def __init__(self):
        try:
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
            A=np.zeros((6,6))
            for i in range(6):
                if i%2==0:
                    A[i]=1
                    A[i+1]=1
                else:
                    A[i]=1
            cap=cv2.VideoCapture(0)
            ret, old_frame=cap.read()
            old_gray=cv2.cvtColor(old_frame, cv2.COLOR_BGR2GRAY)
            old_corners = cv2.goodFeaturesToTrack(old_gray, mask = None, **feature_params)
                
            while True:
                #####
                #Step 1. Capture current k_th frame
                ret, old_frame=cap.read()
                # rospy.Subscriber('/usb_cam/image_raw', Image, self.callback_image_sub)
                frame_gray=cv2.cvtColor(old_frame, cv2.COLOR_BGR2GRAY)
                # cv2.cvtColor(self.image, self.gray, cv2.COLOR_BGR2GRAY)
                
                    #############Corner detection###########
                #Step 4. determine search window according to H_k^-. Find corresponding pixels F_k-1^' by lucas-kanade algorithm
                winsize_arr=[]
                for corner in old_corners:
                    ######Determine Search Window##########
                    winsize_arr.append((10,10))
                    winsize=(10,10)
                lk_params = dict( winSize= winsize,
                                maxLevel = 2,
                                criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
                
                ############Should be changed (different window size) ###############
                new_corners, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, old_corners, None, **lk_params)
                
                # Select good points
                good_new = new_corners[st==1]
                good_old = old_corners[st==1]

                data=[]
                for i,(new,old) in enumerate(zip(good_new,good_old)):
                    a,b = new.ravel()
                    c,d = old.ravel()
                    data.append([[a,b],[c,d]])
                #Step 5. Compute initial estimate of homography matrix H_k^' (using RANSAC)
                H=RANSAC.ransac(data)

                #Step 6. Outlier rejection

                #Step 7. estimate camera's ego-motion through homography, estimate current pos., update the state
                T=inv(M)*H*M
                tx=T[0][2]/T[2][2]
                ty=T[1][2]/T[2][2]
                theta=math.atan(ty/tx)
                xk=x_state[-1][0]+tx
                yk=x_state[-1][2]+ty
                thk=x_state[-1][4]+theta
                x_state.append([xk, tx, yk, ty, thk, theta])
                cv2.waitKey(100)

        except Exception as e:
            print(e)

    def callback_image_sub(self, msg):
        try:
            bridge= CvBridge()
            image_cv=bridge.imgmsg_to_cv2(msg)
            self.image = birdview.birdview_transform(image_cv)
            
        except Exception as e:
            print(e)

    def shutdown(self):
        print("Shut Down!")


if __name__ == '__main__':
    try:
        VisualOdometry()
        print("End of Main Function")

    except Exception as e:
        print(e)
        rospy.loginfo("shutdown program.")
