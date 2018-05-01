#!/usr/bin/env python

import rospy
from Tkinter import *
import tkFileDialog
from PIL import Image, ImageTk
import os
import sys
from time import sleep
from geometry_msgs.msg import Point
from std_msgs.msg import Float32

path=[[-1,-1]]
circle_array=[]
length_side=1000
size_x=1200
size_y=1280
radius=2

package_base_path = os.path.abspath(os.path.join(os.path.dirname(__file__),"../.."))

def draw_circle(x,y):
    global c
    circle=c.create_oval(x-radius, y-radius, x+radius, y+radius, width=2, fill='black')

# def draw_line(start_point, goal_point):
#     global c
#     c.create_line(start_point[0]*size_x, start_point[1]*size_y, goal_point[0]*size_x, goal_point[1]*size_y)

def draw_line(start_point, goal_point):
    global c
    c.create_line(start_point.x*size_x+100.0, start_point.y*size_y+100.0, goal_point.x*size_x+100.0, goal_point.y*size_y+100.0)

def command_clear():
    global c
    c.delete("all")
    return

# def command_play():
#     global root
#     for idx in range(len(circle_array)):
#         # draw_circle(float(circle_array[idx][0])*size_x, float(circle_array[idx][1])*size_y)
#         if idx < len(circle_array)-1:
#             draw_line(circle_array[idx], circle_array[idx+1])
#         root.update()
#         sleep(0.1)
#     return

class HengelEstimator():
    def __init__(self):
        global root

        self.current_position = Point()
        self.current_heading = Float32()

        rospy.init_node('hengel_image_estimator')

        self.position_subscriber = rospy.Subscriber('/current_position', Point, self.callback_position)
        self.heading_subscriber = rospy.Subscriber('/current_heading', Float32, self.callback_heading)

        self.prev_position = self.current_position
        self.prev_heading = self.current_heading
        sleep(0.2)

        while(True):
            draw_line(self.prev_position, self.current_position)
            self.prev_position = self.current_position
            self.prev_heading = self.current_heading
            print(self.current_position, self.current_heading)
            root.update()
            sleep(0.2)

    def callback_position(self, _position):
        self.current_position = _position

    def callback_heading(self, _heading):
        self.current_heading = _heading


def command_quit():
    global root
    root.quit()


root = Tk()
c=Canvas(root, height=size_x, width=size_x, bg="white")
if __name__ =="__main__":
    try:
        root.title("Hengel Cam Image Estimator")

        _str=str(size_x)+"x"+str(size_y)
        root.geometry(_str)

        c.pack(expand=YES, fill=BOTH)
        c.grid(row=1, column=0)

        f=Frame(root)
        f.grid(row=0, column=0, sticky="n")

        button_clear=Button(f, text="Clear", command=command_clear)
        button_clear.grid(row=0, column=0)

        button_quit=Button(f, text="Quit", command=command_quit)
        button_quit.grid(row=0, column=1)

        HengelEstimator()

        # c.bind("<B1-Motion>", callback)
        root.mainloop()
    
    except Exception as e:
        print(e)
        rospy.loginfo("shutdown program.")

        