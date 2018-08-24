#!/usr/bin/env python
import rospy
import os
import time
import sys
from time import sleep
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from sensor_msgs.msg import Image, CompressedImage
from math import cos, sin, pi, sqrt
import numpy as np
import copy
import collections
from cv_bridge import CvBridge
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')

class RobotView():
    def __init__(self, _img, ratio, thickness):
    #def __init__(self):
        self.pixMetRatio=ratio
        # self.lineThickness=0.01   #original
        self.lineThickness= thickness

        self.img=_img

        self.pub1=rospy.Publisher('/markedImg', CompressedImage, queue_size=3)
        self.pub2=rospy.Publisher('/notMarkedImg', CompressedImage, queue_size=3)
        self.pub3=rospy.Publisher('/time', Float32, queue_size=5)

    def cvtCanvasToImgCoord(self, _canvasPnt):
        imgPnt=Point()
        imgPnt.x= -_canvasPnt.x * self.pixMetRatio
        imgPnt.y= self.img.shape[0]-_canvasPnt.y*self.pixMetRatio
        imgPnt.z= -_canvasPnt.z

        return imgPnt

    def run(self, _midpoint, _endpoint):
        self.mid_x = self.cvtCanvasToImgCoord(_midpoint).x
        self.mid_y = self.cvtCanvasToImgCoord(_midpoint).y
        self.th = self.cvtCanvasToImgCoord(_midpoint).z

        self.end_x= self.cvtCanvasToImgCoord(_endpoint).x
        self.end_y= self.cvtCanvasToImgCoord(_endpoint).y

        self.spray_intensity=_endpoint.z

        if self.spray_intensity ==0 :
            self.spray_intensity=1
        # print("end_x: "+str( self.end_x)+", end_y: "+str(self.end_y)+", spray: "+str(self.spray_intensity))

        # print(self.end_x, self.img.shape[1], self.end_y, self.img.shape[0])

        if self.end_x>=0 and self.end_x<self.img.shape[1] and self.end_y>=0 and self.end_y<self.img.shape[0]:

            # print("add run")
            self.add_endpoint()
            #self.img[int(self.end_y)][int(self.end_x)]=self.spray_intensity

    def add_endpoint(self):
        _time=time.time()
        if self.spray_intensity!=255:
            print("====Endpoint Append: %d, %d " %(self.end_x, self.end_y))
            self.img[int(self.end_y)][int(self.end_x)]=self.spray_intensity
            dist=self.lineThickness*self.pixMetRatio
            x1=int(self.end_x-dist/2)
            x2=int(self.end_x+dist/2)

            for i in range(x1, x2+1):
                x=self.end_x-i
                if i>=0 and i<self.img.shape[1]:
                    if abs(x)>dist/2:
                        self.img[int(self.end_y)][i]=min(self.spray_intensity, self.img[int(self.end_y)][i])
                    else:
                        if i>self.end_x:
                            y=sqrt(dist*dist/4-(self.end_x-i)*(self.end_x-i))
                        else:
                            y=sqrt(dist*dist/4-(self.end_x-i-1)*(self.end_x-i-1))
                        y1=int(self.end_y-y)
                        y2=int(self.end_y+y)
                        for j in range(y1, y2+1):
                            if j>0 and j<self.img.shape[0]:
                                self.img[j][i]=min(self.spray_intensity, self.img[j][i])
                        
                    print("====endpoint added")
                else:
                    print("====Endpoint append- RANGE ERROR!!!!")
        # ttime=Float32()
        # ttime.data=float(time.time()-_time)
        # self.pub3.publish(ttime)
        # print("map making time: "+str(time.time()-_time))

    def remove_points_during_vision_compensation(self, _recent_pts, _num_remove_pts):
        #Make all pixels in recent_pts to white (255)
        print("remove %d points during visual compenstaion" %(_num_remove_pts))

        #This should be carefully selected !!!!!!!!!!!!!!!!!!!
        dist=self.lineThickness*self.pixMetRatio

        for ind in range(_num_remove_pts):
            point_x = self.cvtCanvasToImgCoord(_recent_pts[ind]).x
            point_y = self.cvtCanvasToImgCoord(_recent_pts[ind]).y

            if point_x != 0.0 and point_y != 0.0:
                x1=int(point_x+dist/2)
                x2=int(point_x+dist/2)

                for i in range(x1, x2+1):
                    x=point_x-i
                    if i>0 and i<self.img.shape[1]:
                        if abs(x)>dist/2:
                            self.img[int(point_y)][i]=255
                        else:
                            if i>point_x:
                                y=sqrt(dist*dist/4-(point_x-i)*(point_x-i))
                            else:
                                y=sqrt(dist*dist/4-(point_x-i-1)*(point_x-i-1))
                            y1=int(point_y-y)
                            y2=int(point_y+y)
                            for j in range(y1, y2+1):
                                if j>0 and j<self.img.shape[0]:
                                    self.img[j][i]=255


    def viewMarker(self):
        map_size=1280

        square=np.array([
            [self.mid_x+map_size/sqrt(2)*cos(self.th+pi/4), self.mid_y+map_size/sqrt(2)*sin(self.th+pi/4)],
            [self.mid_x-map_size/sqrt(2)*cos(-self.th+pi/4), self.mid_y+map_size/sqrt(2)*sin(-self.th+pi/4)],
            [self.mid_x-map_size/sqrt(2)*cos(self.th+pi/4), self.mid_y-map_size/sqrt(2)*sin(self.th+pi/4)],
            [self.mid_x+map_size/sqrt(2)*cos(-self.th+pi/4), self.mid_y-map_size/sqrt(2)*sin(-self.th+pi/4)],
            ], np.int32)

        self.markedImg=copy.deepcopy(self.img)

        cv2.polylines(self.markedImg, [square],True,0 ,5)
        bridge=CvBridge()
        imgMsg1=bridge.cv2_to_compressed_imgmsg(self.markedImg)
        self.pub1.publish(imgMsg1)




#if __name__=='__main__':
#    RobotView()
