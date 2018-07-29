#!/usr/bin/env python
from math import radians, copysign, sqrt, pow, pi, atan2, sin, ceil, floor, cos
import numpy as np
import sys
from PIL import Image
import time
import os
import cv2

from Tkinter import *
import PIL.Image
import PIL.ImageTk
import tkFileDialog

package_base_path = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "../.."))
home_path = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "../../../../.."))
os.system("mkdir -p " + package_base_path +
        "/hengel_path_manager/output_pathmap")


class CreateImageWithWaypoints():
    def __init__(self):
        word = raw_input("What is the max range of x, y in input file?\n Type: ")
        self.input_size_x = float(word.split()[0])
        self.input_size_y = float(word.split()[1])
        word = raw_input("What is the size of canvas x, y in output image?\n Type: ")
        self.canvas_size_x = int(word.split()[0])
        self.canvas_size_y = int(word.split()[1])
        print("Length, Height of Output Image = %d, %d" %(self.canvas_size_x, self.canvas_size_y))

        self.img=np.zeros((self.canvas_size_y, self.canvas_size_x) )

    def get_path(self):
        root = Tk()
        path_str = tkFileDialog.askopenfilename(parent=root,initialdir=home_path,title='Please select a path file to create image')
        root.quit()

        # x_dir=[0, -1,  0,  1, -1, 0, 1, 1, -1]
        # y_dir=[0, -1, -1, -1,  1, 1, 1, 0,  0]
        x_dir=[0, -1,  0,  1, -1, 0, 1, 1, -1, -2, -1,  0,  1,  2, -2,  2, -2, 2, -2,  2, -2, -1, 0, 1, 2]
        y_dir=[0, -1, -1, -1,  1, 1, 1, 0,  0, -2, -2, -2, -2, -2, -1, -1,  0, 0,  1,  1,  2,  2, 2, 2, 2]


        if os.path.isfile(path_str):
            with open(path_str, "r") as file_path:
                for idx, line in enumerate(file_path):
                    _str = line.split()
                    if not len(_str) > 2:
                        x_input=float(_str[0])
                        y_input=float(_str[1])

                        y_converted = int(y_input*self.canvas_size_y/self.input_size_y) 
                        x_converted = int(x_input*self.canvas_size_x/self.input_size_x)

                        for i in xrange(len(x_dir)):
                            y_curr = y_converted + y_dir[i]
                            x_curr = x_converted + x_dir[i]
                            if y_curr < self.canvas_size_y and y_curr >=0 and x_curr < self.canvas_size_x and x_curr >=0:
                                self.img[y_curr][x_curr] = 255

        

if __name__ == '__main__':
    try:
        app = CreateImageWithWaypoints()
        app.get_path()
        
        cv2.imwrite("/home/bkjung/image_from_waypoints_"+str(time.time())+".png", app.img)

        print("End of Main Function")

    except Exception as e:
        print(e)
