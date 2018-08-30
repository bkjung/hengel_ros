#!/usr/bin/env python
import rospy
import os
import time
import sys
from time import sleep
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from sensor_msgs.msg import Image, CompressedImage
from math import cos, sin, pi, sqrt, ceil
import numpy as np
import copy
import collections
from cv_bridge import CvBridge
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')

class RobotView():
    def __init__(self, _img, ratio, thickness, canvas_padding):
    #def __init__(self):
        self.pixMetRatio=ratio
        # self.lineThickness=0.01   #original
        self.lineThickness= thickness
        self.canvas_padding= canvas_padding
        self.interval= 100

        self.img=copy.deepcopy(_img)
        self.img_copy=copy.deepcopy(_img)

        self.pub1=rospy.Publisher('/markedImg', CompressedImage, queue_size=3)
        self.pub2=rospy.Publisher('/notMarkedImg', CompressedImage, queue_size=3)
        self.pub3=rospy.Publisher('/time', Float32, queue_size=5)

        self.isPaintStarted = False

        self.prev_end_point= None
        self.isDrawing= False

    def cvtCanvasToImgCoord(self, _canvasPnt):
        imgPnt=Point()
        imgPnt.x= -_canvasPnt.x * self.pixMetRatio + self.canvas_padding
        imgPnt.y= self.img.shape[0]-(_canvasPnt.y*self.pixMetRatio +self.canvas_padding)
        imgPnt.z= -_canvasPnt.z

        return imgPnt

    def run(self, _midpoint, _endpoint, change_thickness=-1):
        #If change_thickness == -1(default), add endpoint in virtualMap
        #If change_thickness == -2, add square at modified endpoint
        #Else, add circle at the endpoint (to mark where relocalization executed)
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
            if change_thickness==-1:
                self.add_endpoint(self.lineThickness)
            elif change_thickness==-2:
                print("CHANGE_THICKNESS: %d" %(change_thickness))
                self.add_square(self.end_x, self.end_y)
            else:
                self.add_endpoint(change_thickness, False)
            #self.img[int(self.end_y)][int(self.end_x)]=self.spray_intensity

    def draw_endpoint(self, end_x, end_y, intensity, lineThickness, isSmallCircle):
        #if isVirtualMapChanging is False, add big circle to mark current endpoint
        self.img[int(end_y)][int(end_x)]=intensity
        self.img_copy[int(end_y)][int(end_x)]=intensity
        dist=lineThickness*self.pixMetRatio
        x1=int(end_x-dist/2)
        x2=int(end_x+dist/2)

        for i in range(x1, x2+1):
            x=end_x-i
            if i>=0 and i<self.img.shape[1]:
                if abs(x)>dist/2:
                    if isSmallCircle: #add endpoint
                        self.img[int(end_y)][i]=min(intensity, self.img[int(end_y)][i])
                        self.img_copy[int(end_y)][i]=min(intensity, self.img[int(end_y)][i])
                    else: #add big circle
                        self.img_copy[int(end_y)][i]=min(intensity, self.img[int(end_y)][i])

                else:
                    if i>end_x:
                        y=sqrt(dist*dist/4-(end_x-i)*(end_x-i))
                    else:
                        y=sqrt(dist*dist/4-(end_x-i-1)*(end_x-i-1))
                    y1=int(end_y-y)
                    y2=int(end_y+y)
                    for j in range(y1, y2+1):
                        if j>0 and j<self.img.shape[0]:
                            if isSmallCircle:
                                self.img[j][i]=min(intensity, self.img[j][i])
                                self.img_copy[j][j]=min(intensity, self.img[j][i])
                            else:
                                self.img_copy[j][i]=min(intensity, self.img[j][i])

            else:
                print("canvas size: %d, %d" %(self.img.shape[1], self.img.shape[0]))
                print("point: %d, %d" %(point_x, point_y))

    def add_endpoint(self, lineThickness, isVirtualMapChanging=True):
        #if isVirtualMapChanging is False, add big circle to mark current endpoint
        _time=time.time()
        if self.spray_intensity!=255:
            self.isPaintStarted=True #Initiate sync_real
            #if (self.prev_end_point is not None) and self.isDrawing and isVirtualMapChanging:  #Add points between prev & current endpoint
            #    queue=[]
            #    dist=sqrt(pow(self.prev_end_point[0]-self.end_x,2)+pow(self.prev_end_point[1]-self.end_y,2))
            #    div=int(ceil(dist/self.interval))
            #    for k in range(div+1):
            #        x=self.prev_end_point[0]+(k+1)/float(div)*(self.end_x-self.prev_end_point[0])
            #        y=self.prev_end_point[1]+(k+1)/float(div)*(self.end_y-self.prev_end_point[1])
            #        self.draw_endpoint(x,y, self.spray_intensity, lineThickness, isVirtualMapChanging)
            #else:
            #    self.draw_endpoint(self.end_x, self.end_y, self.spray_intensity, lineThickness, isVirtualMapChanging)
            self.draw_endpoint(self.end_x, self.end_y, self.spray_intensity, lineThickness, isVirtualMapChanging)
            self.isDrawing=True
            self.prev_end_point= [self.end_x, self.end_y]
        else:
            self.isDrawing=False

        # ttime=Float32()
        # ttime.data=float(time.time()-_time)
        # self.pub3.publish(ttime)
        # print("map making time: "+str(time.time()-_time))

    def add_square(self, _end_x, _end_y):
        print("ADD_SQUARE, point: %d, %d / IMG SIZE: %d, %d" %(_end_x, _end_y, self.img_copy.shape[1], self.img_copy.shape[0]))
        # size=int(1.5*self.lineThickness)
        half_size=3

        marked_pixel_cnt = 0
        for i in range(2*half_size):
            x_diff=[-half_size, half_size, -half_size+i, -half_size+i ]
            y_diff=[-half_size+i, -half_size+i, -half_size, half_size ]
            for j in range(len(x_diff)):
                x=_end_x+x_diff[j]
                y=_end_y+y_diff[j]
                if x>0 and x<self.img_copy.shape[1] and y>0 and y<self.img_copy.shape[0]:
                    self.img_copy[y][x]=0
                    marked_pixel_cnt += 1
        print("marked %d pixels" % marked_pixel_cnt)


    def remove_points_during_vision_compensation(self, _recent_pts, _num_remove_pts):
        #Make all pixels in recent_pts to white (255)
        print("remove %d points during visual compenstaion" %(_num_remove_pts))

        #This should be carefully selected !!!!!!!!!!!!!!!!!!!
        dist=self.lineThickness*self.pixMetRatio

        for ind in range(_num_remove_pts):
            point_x = self.cvtCanvasToImgCoord(Point(_recent_pts[ind][0], _recent_pts[ind][1], 0)).x
            point_y = self.cvtCanvasToImgCoord(Point(_recent_pts[ind][0], _recent_pts[ind][1], 0)).y

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
                                    print("removed (%d, %d)" %(j, i))
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
