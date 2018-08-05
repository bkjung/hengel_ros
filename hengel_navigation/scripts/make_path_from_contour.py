#!/usr/bin/env python
import time
import os
import sys
from math import cos, sin
from mpmath import acot
import numpy as np
import cv2
import copy
import random
import matplotlib.pyplot as plt

# def wols(x,y,w):
#     # Weighted orthogonal least squares fit of line a*x+b*y+c=0 to a set of 2D points with coordiantes given by x and y and weights w
#     n = sum(w)
#     meanx = sum(w *elmul* x) / n
#     meany = sum(w *elmul* y) / n
#     x = x - meanx
#     y = y - meany
#     y2x2 = sum(w *elmul* (y **elpow** 2 - x **elpow** 2))
#     xy = sum(w *elmul* x *elmul* y)
#     alpha = 0.5 * acot(0.5 * y2x2 / xy) + pi / 2 * (y2x2 > 0)
#     #if y2x2 > 0, alpha = alpha + pi/2; end
#     a = sin(alpha)
#     b = cos(alpha)
#     c = -(a * meanx + b * meany)

#     return a, b, c


# def project_point_on_line(m1, b1, x1, y1):
#     m2 = -1.0/ m1
#     b2 = -m2 * x1 + y1
#     x2 = (b2 - b1) /eldiv/ (m1 - m2)
#     y2 = m2 *elmul* x2 + b2

#     return x2, y2


# def smooth_contours(X, Y, Radius):
#     Xs = zeros((len(X), 1))
#     Ys = zeros((len(Y), 1))

#     # copy out-of-bound points as they are
#     Xs(mslice[1:Radius]).lvalue = X(mslice[1:Radius])
#     Ys(mslice[1:Radius]).lvalue = Y(mslice[1:Radius])
#     Xs(mslice[len(X) - Radius:end]).lvalue = X(mslice[len(X) - Radius:end])
#     Ys(mslice[len(X) - Radius:end]).lvalue = Y(mslice[len(X) - Radius:end])

#     # obtain the bounding box
#     maxX = max(max(X))
#     minX = min(min(X))
#     maxY = max(max(Y))
#     minY = min(min(Y))

#     # smooth now
#     for i in mslice[Radius + 1:len(X) - Radius]:
#         ind = (mslice[i - Radius:i + Radius])
#         xLocal = X(ind)
#         yLocal = Y(ind)

#         # local regression line
#         #p=polyfit(xLocal,yLocal,1);
#         [a, b, c] = wols(xLocal, yLocal, gausswin(len(xLocal), 5))
#         p(1).lvalue = -a / b
#         p(2).lvalue = -c / b

#         # project point on local regression line
#         [x2, y2] = project_point_on_line(p(1), p(2), X(i), Y(i))

#         # check erronous smoothing
#         # points should stay inside the bounding box
#         if (x2 >= minX and y2 > minY and x2 <= maxX and y2 <= maxY):
#             Xs(i).lvalue = x2
#             Ys(i).lvalue = y2
#         else:
#             Xs(i).lvalue = X(i)
#             Ys(i).lvalue = Y(i)

#     return Xs, Ys


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
        # C = np.full((self.height, self.width),255)
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

        # for loop in range(len(sp_x)):
        #     C[sp_x[loop][0]][sp_x[loop][1]]=0

        return sp_x


if __name__ == "__main__":
    #img=cv2.imread("/home/bkjung/Dropbox/Making_Robot_Path/previous/Adidas_logo_large.png", cv2.IMREAD_COLOR)
    #img=cv2.imread("/home/bkjung/g4218.png", cv2.IMREAD_COLOR)
    # img=cv2.imread("/home/bkjung/g4218.bmp")
    input_file_name = "ambidex_vector"
    input_file_type = "bmp"
    img=cv2.imread("/home/bkjung/"+input_file_name+"."+input_file_type)
    if img is not None:
        img_gray=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img_bw = cv2.threshold(img_gray, 127, 255, cv2.THRESH_BINARY)[1]
        # img_bw = cv2.threshold(img_gray, 40, 255, cv2.THRESH_BINARY)[1]
        # plt.imshow(img_gray)
        # plt.show()
        # plt.imshow(img_bw)
        # plt.show()

        mp=MakePath(img_bw)
        path=mp.path

        height, width = img_bw.shape
        total_path_image = np.full((height, width),0)


        current_time = time.strftime("%y%m%d_%H%M%S")
        with open("/home/bkjung/contour_path_"+input_file_name+"_"+current_time+".txt", "w") as f:       
            for cnt_subpath in range(len(path)):
                print("No. "+str(cnt_subpath)+" length: "+str(len(path[cnt_subpath])))

                for loop in range(len(path[cnt_subpath])):
                    # if len(path[cnt_subpath])!=1:
                    # if cnt_subpath==0:

                    total_path_image[path[cnt_subpath][loop][0]][path[cnt_subpath][loop][1]]=255
                        # For MATLAB, add 1 to index
                    x_in_canvas = float(path[cnt_subpath][loop][0])/float(height)
                    y_in_canvas = float(path[cnt_subpath][loop][1])/float(width)

                    if loop==0:
                        f.write("%f  %f  %d\n" % (x_in_canvas, y_in_canvas, 1))
                    elif loop==len(path[cnt_subpath])-1:
                        f.write("%f  %f  %d\n" % (x_in_canvas, y_in_canvas, 0))
                    else:
                        f.write("%f  %f\n" % (x_in_canvas, y_in_canvas))
                    # elif cnt_subpath==1:
                    #     print(str(path[cnt_subpath][loop][0])+" "+str(path[cnt_subpath][loop][1]))

                    # print("len(input_1)=%d" %len(input_1))
                    # print("len(input_2)=%d" %len(input_2))
                    # Ys, Xs = smooth_contours(input_1, input_2, 50)                    


        plt.imshow(total_path_image, cmap='gray')
        plt.show()

    else:
        print("Image is empty")


