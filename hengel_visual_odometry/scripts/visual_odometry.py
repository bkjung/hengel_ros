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

package_base_path = os.path.abspath(os.path.join(os.path.dirname(__file__),"../.."))
os.system("mkdir -p "+package_base_path+"/hengel_visual_odometry/output_path")

class VisualOdometry():
    image=[]
    gray =[]
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
        # cv_file_homo=cv2.FileStorage("../calibrate_info/"+sys.argv[1]+".xml", cv2.FILE_STORAGE_READ)
        print(sys.argv[1])
        cv_file_homo=cv2.FileStorage(package_base_path+"/hengel_visual_odometry/calibrate_info/"+sys.argv[1]+".xml", cv2.FILE_STORAGE_READ)
        H=cv_file_homo.getNode("H").mat()
        cv_file_homo.release()
        print(H)

        # M=np.identity(3)
        M=H
        
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
        cap=cv2.VideoCapture(1)

        #Initialization
        try:
            ret, old_frame=cap.read()
            # old_bird_frame=birdview.birdview_transform(old_frame)
            # old_gray=cv2.cvtColor(old_bird_frame, cv2.COLOR_BGR2GRAY)
            old_gray=cv2.cvtColor(old_frame, cv2.COLOR_BGR2GRAY)
            old_corners = cv2.goodFeaturesToTrack(old_gray, mask = None, **feature_params)
        except KeyboardInterrupt:
            rospy.signal_shutdown("keyboard interrupt")
        #Create some random color
        color=np.random.randint(0,255,(1000,3))
        #Create a mask image for drawing purposes
        mask=np.zeros_like(old_frame)
        while not rospy.is_shutdown():
            try:        
                #Step 1. Capture current k_th frame
                ret, frame=cap.read()
                # rospy.Subscriber('/usb_cam/image_raw', Image, self.callback_image_sub)
                # frame_bird=birdview.birdview_transform(frame)
                # frame_gray=cv2.cvtColor(frame_bird, cv2.COLOR_BGR2GRAY)
                frame_gray=cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                                
                    #############Corner detection###########
                #Step 4. determine search window according to H_k^-. Find corresponding pixels F_k-1^' by lucas-kanade algorithm
                
                # winsize_arr=[]
                # for corner in old_corners:
                #     ######Determine Search Window##########
                #     winsize_arr.append((10,10))
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

                    #Drawing lines
                    mask = cv2.line(mask, (a,b),(c,d), color[i].tolist(),2)
                    frame=cv2.circle(frame, (a,b), 5, color[i].tolist(), -1)

                img=cv2.add(frame, mask)
                cv2.imshow('frame', img)
                #Step 5. Compute initial estimate of homography matrix H_k^' (using RANSAC)
                H=RANSAC.ransac(data)
                if H is None:
                    continue
                #Step 6. Outlier rejection

                #Step 7. estimate camera's ego-motion through homography, estimate current pos., update the state
                T=np.matmul(np.matmul(inv(M),H),M)
                tx=T[0][2]/T[2][2]
                ty=T[1][2]/T[2][2]
                theta=math.atan2(tx,ty)
                xk=x_state[-1][0]+tx
                yk=x_state[-1][2]+ty
                thk=x_state[-1][4]+theta
                x_state.append([xk, tx, yk, ty, thk, theta])
                old_gray=frame_gray.copy()
                old_corners=good_new.reshape(-1,1,2)
                
                #Step 8. publish path
                msg=Point()
                msg.x=xk
                msg.y=yk
                msg.z=thk
                self.vo_point_pub.publish(msg)
                cv2.waitKey(100)
            except KeyboardInterrupt:
                print("Interrupted")
                rospy.signal_shutdown("KeyboardInterrupt")
                break
        print(x_state)
        cv_file=cv2.FileStorage(package_base_path+"/hengel_visual_odometry/output_path/"+time.strftime("%y%m%d_%H%M%S")+".xml", cv2.FILE_STORAGE_WRITE)
        x_state=np.array(x_state)
        cv_file.write("path_arr", x_state)
        cv_file.release()

    def callback_image_sub(self, msg):
        bridge= CvBridge()
        image_cv=bridge.imgmsg_to_cv2(msg)
        self.image = birdview.birdview_transform(image_cv)
         
    def shutdown(self):
        rospy.signal_shutdown("shud down")
        print("Shut Down!")
        


if __name__ == '__main__':
    try:
        VisualOdometry()
        print("End of Main Function")

    except Exception as e:
        print(e)
        rospy.loginfo("shutdown program.")
