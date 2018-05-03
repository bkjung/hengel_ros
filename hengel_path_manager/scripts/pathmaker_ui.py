#!/usr/bin/env python
# import Tkinter
# import win32gui
from math import pow, sqrt
from Tkinter import *
from PIL import Image, ImageTk
import os
import time
import rospy

length_side=1000
size_x=1000
size_y=1080

dist=0.02415
radius=2


package_base_path = os.path.abspath(os.path.join(os.path.dirname(__file__),"../.."))
os.system("mkdir -p "+package_base_path+"/hengel_path_manager/path_made")



class PathMaker():
    def __init__(self):
        self.root = Tk()

        # rospy.init_node('hengel_image_drawing_input', anonymous=False, disable_signals=True)

        self.root.title("Path")

        self.file_path=""
        # img=Image.open("./alphabet_imgs/"+filename+".jpeg")
        # img.thumbnail(size, Image.ANTIALIAS)
        # photo=ImageTk.PhotoImage(img)

        # l=Label(root, image=photo)
        # l.pack()cancel
        _str=str(size_x)+"x"+str(size_y)
        self.root.geometry(_str)
        self.isDrawmode = False
        self.path_drawing=[[-1,-1]]
        self.cnt_waypoint=0
        self.draw_start_index=[]
        self.isFinishedCorrectly = False

        self.c=Canvas(self.root, height=size_x, width=size_x, bg="white")
        self.c.pack(expand=YES, fill=BOTH)
        self.c.grid(row=1, column=0)

        self.frame=Frame(self.root)
        self.frame.grid(row=0, column=0, sticky="n")

        self.button_draw=Button(self.frame, text="Draw", relief="raised", command=self.command_drawing)
        self.button_draw.grid(row=0, column=0)

        self.button_save=Button(self.frame, text="Save", command=self.command_save)
        self.button_save.grid(row=0, column=1)

        self.button_quit=Button(self.frame, text="Quit", command=self.command_quit)
        self.button_quit.grid(row=0, column=2)

        # label_entry=Entry(self.frame, width=10)
        # label_entry.grid(row=0, column=2)

        self.c.bind("<Button-1>", self.callback_mouse_clicked)
        self.c.bind("<B1-Motion>", self.callback_move_with_button_held_down)
        self.root.mainloop()    #this will run unti UI closes

    def callback_mouse_clicked(self, event):
        self.draw_start_index.append(self.cnt_waypoint)

    def callback_move_with_button_held_down(self, event):
        if self.isDrawmode:
            circle=self.c.create_oval(event.x-radius, event.y-radius, event.x+radius, event.y+radius, width=2, fill='black')
            self.path_append(float(event.x)/length_side, float(event.y)/length_side)
            self.cnt_waypoint = self.cnt_waypoint + 1

    def path_append(self, x,y):
        last_x, last_y=self.path_drawing[-1]
        if sqrt(pow(last_x-x,2)+pow(last_y-y,2))>dist:
            self.path_drawing.append([x,y])
            print(x,y)

    def command_drawing(self):
        if self.button_draw.config('relief')[-1] == 'sunken':
            self.button_draw.config(relief="raised")
            self.isDrawmode=False
        else:
            self.button_draw.config(relief="sunken")
            self.isDrawmode=True
        return

    def command_save(self):
        self.file_path = package_base_path+"/hengel_path_manager/path_made/"+time.strftime("%y%m%d_%H%M%S")+".txt"
        f=open(self.file_path,'w')
        for i in range(1,len(self.path_drawing)):
            data=str(self.path_drawing[i][0])+"  "+str(self.path_drawing[i][1])+"\n"
            f.write(data)
        f.close()
        print("path saved at " + self.file_path)
        self.isFinishedCorrectly = True
        return
 
    def command_quit(self):
        self.root.quit()


if __name__ =="__main__":
    try:
        PathUI = PathMaker()
        print("End of Main Function")

    except Exception as e:
        print(e)
        rospy.loginfo("shutdown program.")
        sys.exit()
