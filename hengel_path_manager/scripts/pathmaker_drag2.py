#!/usr/bin/env python
import Tkinter
# import win32gui
from math import pow, sqrt
from Tkinter import *
from PIL import Image, ImageTk

path=[[-1,-1]]
size_x=1000
size=(size_x,size_x)
dist=0.02415

def callback(event):
    # while True:
        # frame.bind("<BI-Motion>", callback_bi)
        # print(event.x, event.y)
    # if(past_  _y=event.y
    write(float(event.x)/size_x, float(event.y)/size_x)

# def print_widget_under_mouse(root):
#     x,y=root.winfo_pointerxy()
#     widget=root.winfo_containing(x,y)
#     print("widget: ", widget)
#     root.after(1000, print_widget_under_mouse, root)

def write(x,y):
    last_x, last_y=path[-1]
    if sqrt(pow(last_x-x,2)+pow(last_y-y,2))>dist:
        path.append([x,y])
        print(x,y)

if __name__ =="__main__":
    alpha=raw_input("alphabet?")
    filename="alphabet_"+alpha.capitalize()

    root = Tk()
    img=Image.open("./alphabet_imgs/"+filename+".jpeg")
    img.thumbnail(size, Image.ANTIALIAS)
    photo=ImageTk.PhotoImage(img)

    l=Label(root, image=photo)
    l.pack()
    try:
        l.bind("<B1-Motion>", callback)
        root.mainloop()
    except(KeyboardInterrupt):
        f=open("./path_made/path_"+alpha.capitalize()+".txt",'w')
        for i in range(1,len(path)):
            data=str(path[i][0])+"  "+str(path[i][1])+"\n"
            f.write(data)
        f.close()
   
    