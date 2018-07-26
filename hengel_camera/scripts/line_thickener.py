#!/usr/bin/env python
import numpy as np
import os
from math import *
import cv2
import sys
from predict_globalmap_publisher import PredictGlobalMap

class MapMaker():
    def __init__(self, _path, _isIntensityControl, _isStartEndIndexed, 
    _arr_intensity, _start_point_list, _end_point_list):
        self.arr_path=_path

        self.isIntensityControl=_isIntensityControl
        self.isStartEndIndexed=_isStartEndIndexed

        self.arr_intensity=_arr_intensity
        self.start_point_list=_start_point_list
        self.end_point_list=_end_point_list

        self.sprayIntensity=255

        self.pixMetRatio=100
        self.lineThickness=0.1
        self.height=400
        self.width=100

        if _isIntensityControl:
            if _isStartEndIndexed:
                #0~255
                self.mode=1
            else:
                sys.exit("Wrong Intensity Option")
        else:
            if _isStartEndIndexed:
                #start, end index
                self.mode=2
            else:
                #continuously drawing
                self.sprayIntensity=0
        
        self.run()
            
    def run(self):   
        x_last, y_last=0,0
        # img=np.full((pixMetRatio*letterNum, pixMetRatio),255)
        img=np.full((self.height, self.width), 255)

        count, start_count, end_count=0,0,0
        for letter_path in self.arr_path:
            for subletter_path in letter_path:
                for points in subletter_path:
                    x_curr=-float(_str[0])*pixMetRatio
                    y_curr=float(_str[1])*pixMetRatio
                    if self.mode==1:
                        self.sprayIntensity=self.arr_intensity[count]
                        
                    elif self.mode==2:
                        if (self.sprayIntensity==255) and (self.start_point_list[start_count]==count) :
                            sprayIntensity=0
                        if (self.sprayIntensity==0) and (self.end_point_list[end_count]==count):
                            sprayIntensity=255              
                    count+=1

                    if self.sprayIntensity!=255:
                        img[int(x_curr)][int(y_curr)]=0

                        dist=self.lineThickness*self.pixMetRatio
                        x1=int(x_curr-dist/2)
                        x2=int(x_curr+dist/2)

                        for i in range(x1, x2+1):
                            x=x_curr-i
                            if abs(x)>dist/2:
                                img[i][int(y_curr)]=min(0, img[i][int(y_curr)])
                            else:
                                y=sqrt(dist*dist/4-x*x)
                                y1=int(y_curr-y)
                                y2=int(y_curr+y)
                                for j in range(y1, y2+1):
                                    img[i][j]=min(0, img[i][j])
        PredictGlobalMap(img)