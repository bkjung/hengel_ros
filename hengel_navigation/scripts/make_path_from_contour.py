#!/usr/bin/env python 
import time 
import os 
import sys 
import math 
import numpy as np 
import cv2 
import copy 
import random 
import matplotlib.pyplot as plt 

class MakePath(): 
    def __init__(self, img): 
        self.BW=np.ndarray([]) 
        self.path=[] 
        self.makePathFromContour(img) 
         
    def makePathFromContour(self, img): 
        self.height, self.width = img.shape 
        print("self.height : "+str(self.height)) 
        print("self.width : "+str(self.width)) 
        path_img_arr = [] 
        path_img = np.zeros((self.height,self.width,1), np.uint8) 
        current_path = [] 

        self.BW=copy.deepcopy(img) 
        self.height, self.width= self.BW.shape 

        for i in range(len(img)): 
            for j in range(len(img[i])): 
                if self.BW[i][j] < 127: #staring point of single path 
                    current_path = self.contour_trace(i, j) 
                    self.path.append(current_path) 

    def contour_trace(self, row,col): 
        C = np.full((self.height, self.width),255) 
        k=1 
        loop=0 
        sp_x = [] 
        sp_x.append((row,col)) 

        x_dir=[-1, 0, 1, -1, 0, 1, 1, -1] 
        y_dir=[-1, -1, -1, 1, 1, 1, 0, 0] 

        while True: 
            # matlab 
            # prev_size=size(sp_x,1) 
            prev_size = len(sp_x) 
            while True: 
                loop=loop+1 
                for i in xrange(8): 
                    if (sp_x[loop-1][0]+x_dir[i])>=0 and (sp_x[loop-1][1]+y_dir[i])>=0 and (sp_x[loop-1][0]+x_dir[i])<self.height and (sp_x[loop-1][1]+y_dir[i])<self.width: 
                        if self.BW[(sp_x[loop-1][0]+x_dir[i])][(sp_x[loop-1][1]+y_dir[i])]==0: 
                            k=k+1 
                            sp_x.append((sp_x[loop-1][0]+x_dir[i], sp_x[loop-1][1]+y_dir[i])) 
                            # BW[(sp_x[loop-1][0]-1)][(sp_x[loop-1][1]-1)]=1 
                            self.BW[(sp_x[loop-1][0]+x_dir[i])][(sp_x[loop-1][1]+y_dir[i])]=255 
                if loop>=prev_size: 
                    break 
            if prev_size==len(sp_x): 
                break 

        for loop in range(len(sp_x)): 
            C[sp_x[loop][0]][sp_x[loop][1]]=0 
             
        return sp_x 


if __name__ == "__main__": 
    img=cv2.imread("LA.jpg", cv2.IMREAD_COLOR) 
    if img is not None: 
        img_gray=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) 
        img_bw = cv2.threshold(img_gray, 127, 255, cv2.THRESH_BINARY)[1] 
         
        mp=MakePath(img_bw) 
        path=mp.path 

        height, width = img_bw.shape 
        total_path_image = np.full((height, width),255) 


        for cnt_subpath in range(len(path)): 
            print("No. "+str(cnt_subpath)+" length: "+str(len(path[cnt_subpath]))) 
            for loop in range(len(path[cnt_subpath])): 
                # if len(path[cnt_subpath])!=1: 
                total_path_image[path[cnt_subpath][loop][0]][path[cnt_subpath][loop][1]]=0 
                print(str(path[cnt_subpath][loop][0])+" "+str(path[cnt_subpath][loop][1])) 



        plt.imshow(total_path_image, cmap='gray') 
        plt.show() 

    else: 
        print("Image is empty")