#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion, PoseStamped
from std_msgs.msg import Float32, Time, Int32
from sensor_msgs.msg import Image, CompressedImage
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
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
home_path = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "../../../../..")
)

class Optitrack():
    def __init__(self, path_navi, path_opti):
        self.R=None
        self.pnt_navi_0=None
        self.pnt_opti_0=None
        self.isInitialized= False

        self.newFile=open(home_path+"/Dropbox/optitrack_log/"+path_opti+"_calibrated.txt", "w")

        with open(path_navi, "r") as file_navi:
            self.arr_navi=[]
            for idx, line in enumerate(file_navi):
                try:
                    _str=line.split()
                    self.arr_navi.append([float(_str[0]), float(_str[1]), _str[2]])
                except Exception as e:
                    print(e)
                    print(idx, line)
        with open(path_opti) as file_opti:
            self.arr_opti=[]
            for idx, line in enumerate(file_opti):
                _str=line.split()
                if len(_str)==9:
                    self.arr_opti.append([float(_str[5]), float(_str[7]), _str[0]])
                # self.arr_opti.append([float(_str[0]), float(_str[1]), _str[2]])

        self.run()

    # def coordinate_calibration(self, pnt_navi_0, pnt_navi_1, pnt_opti_0, pnt_opti_1):
    def coordinate_calibration(self, index_navi, index_opti):
        print("CALIBRATE")
        self.isInitialized=True
        self.pnt_navi_0_x=self.arr_navi[index_navi][0]
        self.pnt_navi_0_y=self.arr_navi[index_navi][1]

        self.pnt_opti_0_x=self.arr_opti[index_opti][0]
        self.pnt_opti_0_y=self.arr_opti[index_opti][1]

        theta_navi= atan2(self.arr_navi[index_navi+1][1]-self.pnt_navi_0_y, self.arr_navi[index_navi+1][0]-self.pnt_navi_0_x)
        theta_opti= atan2(self.arr_opti[index_opti+1][1]-self.pnt_opti_0_y, self.arr_opti[index_opti+1][0]-self.pnt_opti_0_x)

        delta_theta=theta_navi - theta_opti

        self.R=[[cos(delta_theta), -sin(delta_theta)],[sin(delta_theta), cos(delta_theta)]]


    def opti_to_navi(self, index_opti):
        opti_pnt_x=self.arr_opti[index_opti][0]
        opti_pnt_y=self.arr_opti[index_opti][1]

        x= np.matmul(self.R, [opti_pnt_x-self.pnt_opti_0_x, opti_pnt_y-self.pnt_opti_0_y])[0]+self.pnt_navi_0_x
        y= np.matmul(self.R, [opti_pnt_x-self.pnt_opti_0_x, opti_pnt_y-self.pnt_opti_0_y])[1]+self.pnt_navi_0_y

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

    def diffT1T2(self,t1,t2):
        t1_int= int(t1)
        t2_int= int(t2)
        if len(t1)<21:
            num=21-len(t1)
            t1_int=t1_int*pow(10,num)
        if len(t2)<21:
            num=21-len(t2)
            t2_int=t2_int*pow(10,num)
        return t1_int - t2_int


    def run(self):
        index_navi=0
        index_opti=0

        while index_opti+1<len(self.arr_opti) and index_navi+1<len(self.arr_navi):
            while not self.isInitialized and index_opti+1<len(self.arr_opti) and index_navi+1<len(self.arr_navi):
                t_navi_0=self.arr_navi[index_navi][2]
                t_navi_1=self.arr_navi[index_navi+1][2]
                t_opti=self.arr_opti[index_opti][2]

                if self.diffT1T2(t_navi_0, t_opti)>0:
                    print(self.diffT1T2(t_navi_0, t_opti))
                    index_opti+=1
                elif self.diffT1T2(t_opti, t_navi_1)>0:
                    index_navi+=1
                else:
                    self.coordinate_calibration(index_navi, index_opti)
                    index_opti+=1
                    index_navi+=1


            t_navi_0=self.arr_navi[index_navi][2]
            t_navi_1=self.arr_navi[index_navi+1][2]
            t_opti=self.arr_opti[index_opti][2]

            if self.diffT1T2(t_navi_0, t_opti)>0:
                index_opti+=1
            elif self.diffT1T2(t_opti, t_navi_1)>0:
                index_navi+=1
            else:
                x_opti, y_opti = self.opti_to_navi(index_opti)

                x0=self.arr_navi[index_navi][0]
                y0=self.arr_navi[index_navi][1]

                x1=self.arr_navi[index_navi+1][0]
                y1=self.arr_navi[index_navi+1][1]

                t=self.diffT1T2(t_navi_1, t_navi_0)
                tt=self.diffT1T2(t_opti, t_navi_0)

                x_navi= x0+(x1-x0)/float(t)*tt
                y_navi= y0+(y1-y0)/float(t)*tt

                self.newFile.write(str(x_navi-x_opti)+"\t"+str(y_navi-y_opti)+"\t"+str(x_opti)+"\t"+str(y_opti)+"\t"+str(t_opti)+"\t"+str(t_navi_0)+"\t"+str(t_navi_1)+"\n")
                index_opti+=1
                index_navi+=1

            #x,y=self.opti_to_navi(index_opti)
            #index_opti+=1
            #index_navi+=1
            #self.newFile.write(str(x)+"\t"+str(y)+"\n")


if __name__=="__main__":
    if len(sys.argv)!=3:
        print("Wrong Argument")
    elif os.path.isfile(sys.argv[1]) and os.path.isfile(sys.argv[2]):
        Optitrack(sys.argv[1], sys.argv[2])
    else:
        print("Wrong Argument")
