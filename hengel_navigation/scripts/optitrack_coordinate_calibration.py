#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion, PoseStamped
from std_msgs.msg import Float32, Time, Int32
from sensor_msgs.msg import Image, CompressedImage
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from hengel_optigation.msg import ValveInput, OperationMode
import tf
from math import radians, copysign, sqrt, pow, pi, atan2, sin, floor, cos, asin, ceil
from tf.transformations import euler_from_quaternion
import numpy as np
import sys
import time
import os
import cv2
import cv_bridge
import logging
import matplotlib.pyplot as plt

package_base_path = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "../.."))

class Optitrack():
    def __init__(self, file_optii, file_opti):
        self.R=None
        self.pnt_navi_0=None
        self.pnt_opti_0=None
        self.isInitialized= False

        self.newFile=open(package_base_path+"/hengel_navigation/optitrack_"+time.strftime("%y%m%d_%H%M%S")+".txt", "w"))

        with open(file_navi, "r"):
            self.arr_navi=[]
            for idx, line in enumerate(file_navi):
                _str=line.split()
                self.arr_navi.append([float(_str[0]), float(_str[1]), _str[2]])
        with open(file_opti):
            self.arr_opti=[]
            for idx, line in enumerate(file_opti):
                _str=line.split()
                self.arr_opti.append([float(_str[5]), float(_str[7]), _str[0]])

    # def coordinate_calibration(self, pnt_navi_0, pnt_navi_1, pnt_opti_0, pnt_opti_1):
    def coordinate_calibration(self, index_navi, index_opti):
        self.pnt_navi_0_x=arr_navi[index_navi][0]
        self.pnt_navi_0_y=arr_navi[index_navi][1]

        self.pnt_opti_0_x=arr_opti[index_opti][0]
        self.pnt_opti_0_y=arr_opti[index_opti][1]
        
        theta_navi= atan2(arr_navi[index_navi+1][1]-self.pnt_navi_0_y, arr_navi[index_navi+1][0]-self.pnt_navi_0_x)
        theta_opti= atan2(arr_opti[index_opti+1][1]-self.pnt_opti_0_y, arr_opti[index_opti+1][0]-self.pnt_opti_0_x)

        delta_theta=theta_navi - theta_opti
        
        self.R=[[cos(delta_theta), -sin(delta_theta)],[sin(delta_theta), cos(delta_theta)]]

    def opti_to_navi(self, opti_pnt):
        x= np.matmul(R, [opti_pnt.x-self.pnt_opti_0.x, opti_pnt.y-self.pnt_opti_0.y])[0]+self.pnt_navi_0.x
        y= np.matmul(R, [opti_pnt.x-self.pnt_opti_0.x, opti_pnt.y-self.pnt_opti_0.y])[1]+self.pnt_navi_0.y

        return Point(x,y,0)


    def run(self):
        index_navi=0
        index_opti=0

        while True:
            while not self.isInitialized:
                t0=self.arr_opti[index_navi][2]
                t1=self.arr_navi[index_opti][2]

        self.pnt_navi_0=pnt_navi_0
        self.pnt_opti_0=pnt_opti_0
        
        theta_navi= atan2(pnt_navi_1.y-pnt_navi_1.x)
        theta_opti= atan2(pnt_opti_1.y-pnt_opti_1.x)

        delta_theat=theta_navi - theta_opti
        
        self.R=[[cos(delta_theta), -sin(delta_theta)],[sin(delta_theta), cos(delta_theta)]]

    def opti_to_navi(self, index_navi, index_opti):
        opti_pnt_x=self.arr_opti[index_opti][0]
        opti_pnt_y=self.arr_opti[index_opti][1]
        navi_pnt_x=self.arr_navi[index_navi][0]
        navi_pnt_y=self.arr_navi[index_navi][1]

        x= np.matmul(R, [opti_pnt_x-self.pnt_opti_0.x, opti_pnt_y-self.pnt_opti_0.y])[0]+self.pnt_navi_0.x
        y= np.matmul(R, [opti_pnt_x-self.pnt_opti_0.x, opti_pnt_y-self.pnt_opti_0.y])[1]+self.pnt_navi_0.y

        return [x,y]
    
    def isT1Bigger(self, t1, t2):
        for i in range(min(len(t1), len(t2))):
            if int(t1[i])==int(t2[i]):
                pass
            elif int(t1[i])>int(t2[i]):
                return True
            else:
                return False
        
        if len(t1)> len(t2):
            return True
        else:
            return False

    
    def run(self):
        index_navi=0
        index_opti=0

        while True:
            while not self.isInitialized:
                t_navi_0=self.arr_opti[index_navi][2]
                t_navi_1=self.arr_opti[index_navi+1][2]
                t_opti=self.arr_navi[index_opti][0]
    
                if isT1Bigger(t_navi_0, t_opti):
                    index_opti+=1
                elif isT1Bigger(t1, t_navi_1):
                    index_navi+=1
                else:
                    self.coordinate_calibration(index_navi, index_opti)
            
            t_navi_0=self.arr_opti[index_navi][2]
            t_navi_1=self.arr_opti[index_navi+1][2]
            t_opti=self.arr_navi[index_opti][0]

            if isT1Bigger(t_navi_0, t_opti):
                index_opti+=1
            elif isT1Bigger(t1, t_navi_1):
                index_navi+=1
            else:
                x, y = self.opti_to_navi(index_navi, index_opti)
                self.file.write(str(x)+"\t"+str(y)+"\n")
        

if __name__=="__main__":
    if len(sys.argv)!=3:
        print("Wrong Argument")
    elif os.path.isfile(sys.argv[1]) and os.path.isfile(sys.argv[2]):
        Optitrack(sys.argv[1], sys.argv[2])
    else:
        print("Wrong Argument")