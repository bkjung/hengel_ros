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
        x_last, y_last=0,0
        # img=np.full((pixMetRatio*letterNum, pixMetRatio),255)
        img=np.full((self.height, self.width), 255)

        print("MapMaker running")

        count, start_count, end_count=0,0,0

        for letter_path in self.arr_path:
            for subletter_path in letter_path:
                for point in subletter_path:
                    debug=-1
                    if len(point)==2:
                        x_curr=-point[0]*self.pixMetRatio
                        y_curr=point[1]*self.pixMetRatio
                        debug=-5

                    if x_curr>self.width or x_curr<0 or y_curr<0 or y_curr>self.height:

                        sys.exit("WAYPOINTS OUT OF CANVAS")
                    if self.mode==1:
                        debug=-2
                        if count<len(self.arr_intensity):
                            self.sprayIntensity=int(self.arr_intensity[count])

                    elif self.mode==2:
                        debug = -10
                        if (self.sprayIntensity==255) and start_count<len(self.start_point_list)  and (self.start_point_list[start_count]==count) :
                            self.sprayIntensity=0
                            start_count+=1
                        debug = -3
                        if (self.sprayIntensity==0) and end_count<len(self.end_point_list)  and (self.end_point_list[end_count]==count):
                            self.sprayIntensity=255
                            end_count+=1
                    count+=1

                    debug=0
                    if self.sprayIntensity!=255:
                        img[int(x_curr)][int(y_curr)]=self.sprayIntensity
                        debug=1

                        dist=self.lineThickness*self.pixMetRatio
                        x1=int(x_curr-dist/2)
                        x2=int(x_curr+dist/2)

                        for i in range(x1, x2+1):
                            debug=2
                            x=x_curr-i
                            debug=3
                            if i>0 and i<self.width:
                                debug=4
                                if abs(x)>dist/2:
                                    img[i][int(y_curr)]=min(self.sprayIntensity, img[i][int(y_curr)])
                                    debug=5
                                else:
                                    y=sqrt(dist*dist/4-x*x)
                                    y1=int(y_curr-y)
                                    y2=int(y_curr+y)
                                    debug=6
                                    for j in range(y1, y2+1):
                                        debug=7
                                        if j>0 and j<self.height:
                                            debug=8
                                            img[i][j]=min(self.sprayIntensity, img[i][j])
                                            debug=9
        debug=-19
        print(img.shape)
        cv2.imwrite('/home/hengel/globalmap.png', img)
        return img
