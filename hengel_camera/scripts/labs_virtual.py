#!/usr/bin/env python
import numpy as np
import os
from math import *
import cv2

if __name__=='__main__':
    pixMetRatio=100
    # letterNum=4
    lineThickness=0.1
    path_str="~/tmp/LABS_waypoints_precise.txt"
    # path_str="./labs_precise_2.txt"
    # path_str="./spray1.txt"
    # path_str="./spiral.txt"

    x_last, y_last=0,0
    draw=False
    height=400
    width=100
    img=np.full((height, width), 255)
    if os.path.isfile(path_str):
        print(path_str)
        with open(path_str, "r") as file_path:
            for idex, line in enumerate(file_path):
                _str=line.split()
                if not len(_str)==0:
                    x_curr=-float(_str[0])*pixMetRatio
                    y_curr=float(_str[1])*pixMetRatio

                    if len(_str)==3:
                        if float(_str[2])==1:
                            draw=True
                            # print("true")
                        elif float(_str[2])==0:
                            draw=False
                            # print("false")
                    # spray=float(_str[2])

                    if draw and img[int(x_curr)][int(y_curr)]==255:
                        img[int(x_curr)][int(y_curr)]=0

                        # exception - slope is infinite
                        if (y_curr != y_last):
                            slope=-(x_curr-x_last)/(y_curr-y_last)
                        else:
                            slope=999999999999
                        print("\nx_curr:", x_curr, "y_curr:", y_curr, "slope:", slope)

                        dist=lineThickness*pixMetRatio

                        x1=x_curr+dist/2*(1/sqrt(slope*slope+1))
                        x2=x_curr-dist/2*(1/sqrt(slope*slope+1))
                        y1=y_curr+dist/2*(1/sqrt(slope*slope+1))*slope
                        y2=y_curr-dist/2*(1/sqrt(slope*slope+1))*slope

                        x_dist=abs(int(x1)-int(x_curr))

                        print(x_dist)
                        
                        if slope==999999999999:
                            print("slope:", slope, "dist:", int(dist/2))
                            for i in xrange(int(dist/2)):
                                print(i)
                                img[int(x_curr)][int(y_curr)-i-1]=0
                                img[int(x_curr)][int(y_curr)+i+1]=0

                        elif slope==0:
                            print("slope:", slope,"dist:", int(dist/2))
                            for i in xrange(int(dist/2)):
                                img[int(x_curr)-i-1][int(y_curr)]=0
                                img[int(x_curr)+i+1][int(y_curr)]=0
                        elif abs(slope)<1:
                            print("case1")
                            x_pnt=int(x_curr)
                            print("while loop-1: "+str(abs(int(y1)-int(y_curr))))
                            for k in range(abs(int(y1)-int(y_curr))+1):
                                x=x_curr+(k+int(y_curr)-y_curr)/slope
                                print(abs(int(x)-x_pnt))
                                for j in xrange(min(abs(int(x)-x_pnt)+1, int(dist/2))):
                                    if (int(y_curr)-k>0) and (int(y_curr)+k<width) and (x_pnt+j < height) and x_pnt+j<x1:
                                        if slope>0:
                                            img[x_pnt+j][int(y_curr)+k]=0
                                        else:
                                            img[x_pnt-j][int(y_curr)+k]=0
                                x_pnt=int(x)
                                print("x_pnt:", x_pnt)

                            x_pnt=int(x_curr)
                            print("while loop-2: "+str(abs(int(y_curr)-int(y2))))
                            for k in range(abs(int(y_curr)-int(y2))+1):
                                x=x_curr+(int(y_curr)-y_curr-k)/slope
                                for j in xrange(min(abs(int(x)-x_pnt)+1, int(dist/2))):
                                    if (x_pnt-j>0) and (int(y_curr)+k<width) and (int(y_curr)-k+1)>0 and x_pnt-j+1>x2:
                                        if slope>0:
                                            img[x_pnt-j][int(y_curr)-k]=0
                                        else:
                                            img[x_pnt+j][int(y_curr)-k]=0
                                x_pnt=int(x)
                        else:
                            print("case2")
                            y_pnt=int(y_curr)
                            print("while loop-1: "+str(abs(int(x1)-int(x_curr))))
                            for k in range(abs(int(x1)-int(x_curr))+1):
                                y=y_curr+slope*(k+int(x_curr)-x_curr)
                                print(abs(int(y)-y_pnt), "int y: ",int(y), "y_pnt: ", y_pnt)
                                for j in xrange(min(abs(int(y)-y_pnt)+1, int(dist/2))):
                                    if( (y_pnt-j)>0 and (int(x_curr)+k) < height and (y_pnt+j)<width):
                                        if slope>0:
                                            img[int(x_curr)+k][y_pnt+j]=0
                                        else:
                                            img[int(x_curr)+k][y_pnt-j]=0
                                y_pnt=int(y)

                            y_pnt=int(y_curr)
                            print("while loop-2: "+str(abs(int(x_curr)-int(x2))))
                            for k in range(1, abs(int(x_curr)-int(x2))+1):
                                y=y_curr+slope*(-k+int(x_curr)-x_curr)
                                for j in xrange(min(abs(int(y)-y_pnt)+1, int(dist/2))):
                                    if((int(x_curr)-k)>0 and (int(y_curr)-j)>0 and (int(y_curr)+j)<height):
                                        if slope>0:
                                            img[int(x_curr)-k][int(y_curr)-j]=0
                                        else:
                                            img[int(x_curr)-k][int(y_curr)+j]=0
                                        l+=1

                                k+=1
                                print("y_pnt"+str(int(y)))
                                y_pnt=int(y)

                        x_last=x_curr
                        y_last=y_curr
        cv2.imshow("labs", img)
        cv2.imwrite("labs1.png", img)
        cv2.waitKey(0)
    else:
        print("Path is wrong")
                    
        