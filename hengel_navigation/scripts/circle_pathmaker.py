#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
from math import cos, sin, sqrt, pow, floor, pi
import time
from PIL import Image
import PIL.Image
import cv2

class ConcenctricPath():
    def __init__(self):
        pass

    def run(self):
        CANVAS_SIZE = 4.0
        #WAYPOINT_INTERVAL = 0.005
        WAYPOINT_INTERVAL = 0.0075
        THETA_INTERVAL = 25

        # r = np.arange(0, 1.0/2, 0.0000001)
        r = np.ones(20*2512)
        r = r*2.0
        list_size = len(r)
        theta = np.arange(0, 2*pi*20, 2*pi/2512)

        x_prev = 0
        y_prev = 0
        x=[]
        y=[]
        for i in range(list_size):
            x_new = r[i]*cos(theta[i])
            y_new = -r[i]*sin(theta[i])
            if sqrt(pow((x_new-x_prev),2)+pow((y_new-y_prev),2))>=WAYPOINT_INTERVAL:
                x.append(x_new+CANVAS_SIZE/2.0)
                y.append(y_new+CANVAS_SIZE/2.0)
                x_prev = x_new
                y_prev = y_new


        # plt.scatter(x,y, s=0.1)
        # # plt.show()
        # plt.draw()
        # plt.pause(0.1)


        # for i in range(len(x)):
            # print("%f  %f" % (x[i], y[i]))

        current_time = time.strftime("%y%m%d_%H%M%S")
        with open("/home/hengel/circle_"+current_time+".txt", "w") as f:
            for i in range(len(x)):
                f.write("%f  %f  %f\n" % (x[i], y[i], 25.0-25.0*cos(2*pi*i/1256)))


if __name__ == '__main__':
    try:
        app = ConcenctricPath()
        app.run()

        print("End of Main Function")

    except Exception as e:
        print(e)



