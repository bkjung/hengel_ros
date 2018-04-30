#!/usr/bin/env python
import Tkinter
# import win32gui
from math import pow, sqrt
from Tkinter import *
from PIL import Image, ImageTk
import os
import time
import sys
from time import sleep

path=[[-1,-1]]
size_x=500
size_y=540
radius=10

def draw(x,y):
    global c
    circle=c.create_oval(x-radius, y-radius, x+radius, y+radius, width=2, fill='black')


def play():
    with open("./txtfile/"+sys.argv[1],'r') as file_path:
        for idx, line in enumerate(file_path):
            _str = line.split()
            if not len(_str)==0:
                draw(float(_str[0])*size_x, float(_str[1])*size_y)
            else:
                pass
            # sleep(0.1)
    return

def quit():
    global root
    root.quit()


root = Tk()
c=Canvas(root, height=500, width=500, bg="white")
if __name__ =="__main__":
    root.title("Path player of " + sys.argv[1])

    _str=str(size_x)+"x"+str(size_y)
    root.geometry(_str)

    c.pack(expand=YES, fill=BOTH)
    c.grid(row=1, column=0)

    f=Frame(root)
    f.grid(row=0, column=0, sticky="n")

    button_save=Button(f, text="Play", command=play)
    button_save.grid(row=0, column=0)

    button_quit=Button(f, text="Quit", command=quit)
    button_quit.grid(row=0, column=1)

    # c.bind("<B1-Motion>", callback)
    root.mainloop()

    