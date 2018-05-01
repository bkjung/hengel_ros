#!/usr/bin/env python
import Tkinter
# import win32gui
from math import pow, sqrt
from Tkinter import *
import tkFileDialog
from PIL import Image, ImageTk
import os
import time
import sys
from time import sleep

path=[[-1,-1]]
circle_array=[]
size_x=500
size_y=540
radius=2

package_base_path = os.path.abspath(os.path.join(os.path.dirname(__file__),"../.."))

def draw_circle(x,y):
    global c
    circle=c.create_oval(x-radius, y-radius, x+radius, y+radius, width=2, fill='black')

def draw_line(start_point, goal_point):
    global c
    c.create_line(start_point[0]*size_x, start_point[1]*size_y, goal_point[0]*size_x, goal_point[1]*size_y)

def command_open():
    global root
    file_path = tkFileDialog.askopenfilename(parent=root,initialdir=package_base_path+"/hengel_path_manager/path_made",title='Please select a path file to play')
    with open(file_path, "r") as opened_file:
        for idx, line in enumerate(opened_file):
            _str = line.split()
            if not len(_str)==0:
                circle_array.append([float(_str[0]), float(_str[1])])
            else:
                pass
            # sleep(0.1)
    return

def command_clear():
    global c
    c.delete("all")
    return

def command_play():
    global root
    for idx in range(len(circle_array)):
        # draw_circle(float(circle_array[idx][0])*size_x, float(circle_array[idx][1])*size_y)
        if idx < len(circle_array)-1:
            draw_line(circle_array[idx], circle_array[idx+1])
        root.update()
        sleep(0.1)
    return

def command_quit():
    global root
    root.quit()


root = Tk()
c=Canvas(root, height=500, width=500, bg="white")
if __name__ =="__main__":
    root.title("Path player")

    _str=str(size_x)+"x"+str(size_y)
    root.geometry(_str)

    c.pack(expand=YES, fill=BOTH)
    c.grid(row=1, column=0)

    f=Frame(root)
    f.grid(row=0, column=0, sticky="n")

    button_open=Button(f, text="Open", command=command_open)
    button_open.grid(row=0, column=0)

    button_open=Button(f, text="Play", command=command_play)
    button_open.grid(row=0, column=1)

    button_clear=Button(f, text="Clear", command=command_clear)
    button_clear.grid(row=0, column=2)

    button_quit=Button(f, text="Quit", command=command_quit)
    button_quit.grid(row=0, column=3)

    # c.bind("<B1-Motion>", callback)
    root.mainloop()

    