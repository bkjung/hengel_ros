#!/usr/bin/env python
import numpy as np
import os
from math import *
import cv2
import sys
from predict_globalmap_publisher import PredictGlobalMap

class MapMaker():
    def __init__(self, _path, _isIntensityControl, _isStartEndIndexed,
    _arr_intensity, _start_point_list, _end_point_list, _length, _height):
        print("MapMaker started")
        self.arr_path=_path

        self.isIntensityControl=_isIntensityControl
        self.isStartEndIndexed=_isStartEndIndexed

        self.arr_intensity=_arr_intensity
        self.start_point_list=_start_point_list
        self.end_point_list=_end_point_list

        self.sprayIntensity=255

        self.pixMetRatio=500
        self.lineThickness=0.03
        self.width=self.pixMetRatio*_length
        self.height = self.pixMetRatio*_height

        if _isIntensityControl:
            #0~255
            self.mode=1
        else:
            if _isStartEndIndexed:
                #start, end index
                self.mode=2
            else:
                #continuously drawing
                self.sprayIntensity=0
                self.mode=3
        print("MODE: "+str(self.mode))


    def run(self):
        _time=time.time()
        x_last, y_last=0,0
        # img=np.full((pixMetRatio*letterNum, pixMetRatio),255)
        img=np.full((self.height, self.width), 255)

        count, start_count, end_count=0,0,0

        for letter_path in self.arr_path:
            for subletter_path in letter_path:
                for point in subletter_path:
                    if len(point)==2:
                        x_curr=-point[0]*self.pixMetRatio
                        y_curr=point[1]*self.pixMetRatio

                    if x_curr>self.width or x_curr<0 or y_curr<0 or y_curr>self.height:

                        sys.exit("WAYPOINTS OUT OF CANVAS")
                    if self.mode==1:
                        if count<len(self.arr_intensity):
                            self.sprayIntensity=int(self.arr_intensity[count])

                    elif self.mode==2:
                        if (self.sprayIntensity==255) and start_count<len(self.start_point_list)  and (self.start_point_list[start_count]==count) :
                            self.sprayIntensity=0
                            start_count+=1
                        if (self.sprayIntensity==0) and end_count<len(self.end_point_list)  and (self.end_point_list[end_count]==count):
                            self.sprayIntensity=255
                            end_count+=1
                    count+=1

                    if self.sprayIntensity!=255:
                        img[int(y_curr)][int(x_curr)]=self.sprayIntensity

                        dist=self.lineThickness*self.pixMetRatio
                        x1=int(x_curr-dist/2)
                        x2=int(x_curr+dist/2)

                        for i in range(x1, x2+1):
                            x=x_curr-i
                            if i>0 and i<self.width:
                                if abs(x)>dist/2:
                                    img[int(y_curr)][i]=min(self.sprayIntensity, img[int(y_curr)][i])
                                else:
                                    if i>x_curr:                                    
                                        y=sqrt(dist*dist/4- (x_curr-i)*(x_curr-i))
                                    else:
                                        y=sqrt(dist*dist/4- (x_curr-i-1)*(x_curr-i-1))
                                    y1=int(y_curr-y)
                                    y2=int(y_curr+y)
                                    for j in range(y1, y2+1):
                                        if j>0 and j<self.height:
                                            img[j][i]=min(self.sprayIntensity, img[j][i])
        print("map making time: "+str(time.time()-_time)) 
        return img
