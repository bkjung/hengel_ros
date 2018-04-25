#!/usr/bin/env python
import Tkinter
# import win32gui
from math import pow, sqrt
from Tkinter import *
from PIL import Image, ImageTk
import os

path=[[-1,-1]]
circle_array=[]
size_x=500
size_y=540
dist=0.02415
radius=10
global isDrawmode
isDrawmode = False

def callback(event):
    if isDrawmode:
        circle=c.create_oval(event.x-radius, event.y-radius, event.x+radius, event.y+radius, width=2, fill='black')
        circle_array.append(circle)
        write(float(event.x)/size_x, float(event.y)/size_y)

def write(x,y):
    last_x, last_y=path[-1]
    if sqrt(pow(last_x-x,2)+pow(last_y-y,2))>dist:
        path.append([x,y])
        print(x,y)

def drawing():
    global isDrawmode
    if button_draw.config('relief')[-1] == 'sunken':
        button_draw.config(relief="raised")
        isDrawmode=False
    else:
        button_draw.config(relief="sunken")
        isDrawmode=True
    return

def cancel():
    c.delete(circle_array[-1])
    return

def quit():
    f=open("./txtfile/path.txt",'w')
    for i in range(1,len(path)):
        data=str(path[i][0])+"  "+str(path[i][1])+"\n"
        f.write(data)
    f.close()
    return

if __name__ =="__main__":
    root = Tk()
    root.title("Path")
    os.system("mkdir -p txtfile")
    # img=Image.open("./alphabet_imgs/"+filename+".jpeg")
    # img.thumbnail(size, Image.ANTIALIAS)
    # photo=ImageTk.PhotoImage(img)

    # l=Label(root, image=photo)
    # l.pack()
    _str=str(size_x)+"x"+str(size_y)
    root.geometry(_str)

    c=Canvas(root, height=500, width=500, bg="white")
    c.pack(expand=YES, fill=BOTH)
    c.grid(row=1, column=0)

    f=Frame(root)
    f.grid(row=0, column=0, sticky="n")

    button_draw=Button(f, text="Draw", relief="raised", command=drawing)
    button_draw.grid(row=0, column=0)

    button_cancel=Button(f, text="Cancel", command=cancel)
    button_cancel.grid(row=0, column=1)

    button_quit=Button(f, text="Save", command=quit)
    button_quit.grid(row=0, column=2)

    # label_entry=Entry(f, width=10)
    # label_entry.grid(row=0, column=2)

    c.bind("<B1-Motion>", callback)
    root.mainloop()

    