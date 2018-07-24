#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
from math import cos, sin, sqrt, pow, floor
import time
from PIL import Image
import PIL.Image
import cv2

class ConcenctricPath():
    def __init__(self):
        pass

    def run(self):
        CANVAS_SIZE = 4.0
        WAYPOINT_INTERVAL = 0.005
        THETA_INTERVAL = 25

        # r = np.arange(0, 1.0/2, 0.0000001)
        r = np.arange(0, CANVAS_SIZE/2, 0.0000001)
        list_size = len(r)
        theta = 2 * np.pi * r * THETA_INTERVAL
        # ax = plt.subplot(111, projection='polar')
        # ax.plot(theta, r)
        # ax.set_rmax(2)
        # # ax.set_rticks([0.5, 1, 1.5, 2])  # less radial ticks
        # ax.set_rticks([2])  # less radial ticks
        # ax.set_rlabel_position(-22.5)  # get radial labels away from plotted line
        # ax.grid(True)

        # ax.set_title("A line plot on a polar axis", va='bottom')
        # plt.show()


        x_prev = 0
        y_prev = 0
        x=[]
        y=[]
        for i in range(list_size):
            x_new = r[i]*cos(theta[i])
            y_new = r[i]*sin(theta[i])
            if sqrt(pow((x_new-x_prev),2)+pow((y_new-y_prev),2))>=WAYPOINT_INTERVAL:
                x.append(x_new+CANVAS_SIZE/2.0)
                y.append(y_new+CANVAS_SIZE/2.0)
                x_prev = x_new
                y_prev = y_new


        # plt.scatter(x,y, s=0.1)
        # # plt.show()
        # plt.draw()
        # plt.pause(0.1)


        img = cv2.imread('/home/hengel/Dropbox/Path_Workspace/ambidex_gray.jpg',0)
        size = img.shape[0], img.shape[1]
        m = np.zeros(size, dtype=np.uint8)
        m.fill(255)


        # for i in range(len(x)):
            # print("%f  %f" % (x[i], y[i]))

        current_time = time.strftime("%y%m%d_%H%M%S")
        with open("/home/hengel/"+current_time+"_"+str(WAYPOINT_INTERVAL)+"_"+str(THETA_INTERVAL)+"_concentric.txt", "w") as f:
            for i in range(len(x)):
                x_in_canvas = int(floor(size[0]*x[i]/CANVAS_SIZE))
                y_in_canvas = int(floor(size[1]*y[i]/CANVAS_SIZE))

                x_in_canvas = int(size[0]) if x_in_canvas>int(size[0]) else x_in_canvas
                y_in_canvas = int(size[1]) if y_in_canvas>int(size[1]) else y_in_canvas

                pixel_value = img[x_in_canvas][y_in_canvas]
                f.write("%f  %f  %f\n" % (x[i], y[i], pixel_value))
                m[x_in_canvas][y_in_canvas] = pixel_value

        cv2.imwrite("/home/hengel/"+current_time+"_"+str(WAYPOINT_INTERVAL)+"_"+str(THETA_INTERVAL)+"_concentric_paint.jpg", m)
        cv2.imshow("image", m);
        cv2.waitKey()

        # plt.show()


if __name__ == '__main__':
    try:
        app = ConcenctricPath()
        app.run()

        print("End of Main Function")

    except Exception as e:
        print(e)


