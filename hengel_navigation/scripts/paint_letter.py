#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion, PoseStamped
from std_msgs.msg import Float32
import tf
from math import radians, copysign, sqrt, pow, pi, atan2, sin, floor, cos
from tf.transformations import euler_from_quaternion
import numpy as np
import sys
from PIL import Image
import time
import os


waypoint_increment = 1
waypoints_length = 0
waypoints=[]
cnt_letter = 0

cnt_path_points = 0
path_points = []


package_base_path = os.path.abspath(os.path.join(os.path.dirname(__file__),"../.."))
os.system("mkdir -p "+package_base_path+"/hengel_path_manager/output_pathmap")


def normalize_rad(input_angle):
    if input_angle>pi:
        return input_angle-2*pi
    elif input_angle<=-pi:
        return input_angle+2*pi
    else:
        return input_angle

def get_path(word):
    global cnt_letter
    arr_path=[]
    dir_1= package_base_path+"/hengel_path_manager/alphabet_path/path_"
    dir_2=".txt"
    cnt_letter = 0
    for letter in word:
        if letter==' ':
            pass
            #if spacing is included
        else:
            with open(dir_1+letter.capitalize()+dir_2,"r") as file_path:
                for idx, line in enumerate(file_path):
                    _str = line.split()
                    if not len(_str)==0:
                        arr_path.append([(float)(_str[0])+(float)(cnt_letter)-(2*(float)(cnt_letter)-1)*250/1632, 1.0-(float)(_str[1])])
                    else:
                        pass

        #count the number of letters including spacing
        cnt_letter = cnt_letter + 1

    return arr_path

class PaintLetter():
    def __init__(self, arr_path):
        global waypoints_length
        global waypoints

        rospy.init_node('hengel_paint_letter', anonymous=False, disable_signals=True)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        position = Point()
        move_cmd = Twist()
        
        r = rospy.Rate(50)


        (position, rotation) = self.get_odom()
        print("x, y, rotation", position.x, position.y, rotation)



        # go through path array

        waypoints_length = len(arr_path)
        for idx in range(waypoints_length):
            waypoints.append([arr_path[idx][0]-arr_path[0][0], arr_path[idx][1]-arr_path[0][1]])

        print("size of waypoints = ", len(waypoints))

        waypoint_index = 1  #starting from index No. 1 (c.f. No.0 is at the origin(0,0))

        while waypoint_index < waypoints_length:
            current_waypoint = [waypoints[waypoint_index][0], waypoints[waypoint_index][1]]
            # if goal_z > 180 or goal_z < -180:
            #     print("you input worng z range.")
            #     self.shutdown()
            # goal_z = np.deg2rad(goal_z)
            # (position,rotation) = self.get_odom()

            goal_distance = sqrt(pow(current_waypoint[0] - position.x, 2) + pow(current_waypoint[1] - position.y, 2))
            distance = goal_distance

            while distance > 0.03:
                try:
                    ################3print ("distance= ", '%.3f' % distance)

                    ################3print("goal_position", '%.3f' % current_waypoint[0], '%.3f' % current_waypoint[1], "current_position", '%.3f' % position.x, '%.3f' % position.y)
                    # alpha=atan2(goal_x-x_start, goal_y-y_start)-rotation
                    # alpha=normalize_rad( normalize_rad(atan2(current_waypoint[1]-position.y, current_waypoint[0]-position.x))-rotation )
                    alpha=normalize_rad(atan2(current_waypoint[1]-position.y, current_waypoint[0]-position.x)-rotation)

                    # print("goal_angle", '%.3f' % np.rad2deg(alpha),"current_angle", '%.3f' % np.rad2deg(rotation))
                    # print("goal_angle", '%.3f' % alpha,"current_angle", '%.3f' % rotation)
                    print("heading error", '%.3f' % np.rad2deg(alpha))

                    if abs(alpha)> thres1: #abs?
                        if alpha>0 or alpha<-pi:
                            move_cmd.linear.x=0
                            move_cmd.angular.z=ang_vel_1

                        else:
                            move_cmd.linear.x=0
                            move_cmd.angular.z=-ang_vel_1

                    elif abs(alpha)>thres2:
                        if alpha>0 or alpha<-pi:
                            move_cmd.linear.x=0
                            move_cmd.angular.z=ang_vel_2

                        else:
                            move_cmd.linear.x=0
                            move_cmd.angular.z=-ang_vel_2
                    elif abs(alpha)>thres3:
                        if alpha>0 or alpha<-pi:
                            move_cmd.linear.x=0
                            move_cmd.angular.z=ang_vel_3
                        else:
                            move_cmd.linear.x=0
                            move_cmd.angular.z=-ang_vel_3

                    else:
                        x=distance*sin(alpha)
                        curv=2*x/pow(distance,2)

                        if distance<0.08:
                            #lin_vel_scaled=lin_vel/4.0
                            lin_vel_scaled=lin_vel/2.0
                        else:
                            lin_vel_scaled=lin_vel

                        move_cmd.linear.x=lin_vel_scaled
                        move_cmd.angular.z=curv*lin_vel_scaled

                    #print("DEBUG. alpha=", np.rad2deg(alpha), " angular.z=", move_cmd.angular.z)
                    self.cmd_vel.publish(move_cmd)

                    (position, rotation) = self.get_odom()


                    global cnt_path_points
                    global path_points

                    cnt_path_points = cnt_path_points + 1
                    path_points.append([position.x, position.y])

                    distance = sqrt(pow((current_waypoint[0] - position.x), 2) + pow((current_waypoint[1] - position.y), 2))

                    r.sleep()
                except KeyboardInterrupt:
                    print("Got KeyboardInterrupt")
                    rospy.signal_shutdown("KeboardInterrupt")
                    break

            print("Now at Waypoint No.", waypoint_index)
            waypoint_index = waypoint_index + waypoint_increment

            if rospy.is_shutdown():
                break

        rospy.loginfo("Stopping the robot at the final destination")
        self.generate_pathmap()

        print("DEBUG-publish0")
        self.cmd_vel.publish(Twist())


    def generate_pathmap(self):
        scale = 10
        pixel_size = 100 #1m*1m canvas of 1cm accuracy points (including boundary points)
        img = Image.new("RGB", ((100+pixel_size*cnt_letter)*scale, (100+pixel_size)*scale), (255, 255, 255))

        print("cnt_path_points = ", cnt_path_points)

        for i in range(cnt_path_points):
            # print(path_points[i][0], path_points[i][1])
            x = 0.99 if path_points[i][0]==1.0 else path_points[i][0]
            y = 0.99 if (1.0-path_points[i][1])==1.0 else (1.0-path_points[i][1])
            x = (int)(floor(x*100))
            y = (int)(floor(y*100))

            x=x+50
            y=y+50

            for k in range(scale):
                for t in range(scale):
                    img.putpixel((x*scale + t, y*scale + k), (0, 0, 0))

        image_save_path = package_base_path+"/hengel_path_manager/output_pathmap/"+time.strftime("%y%m%d_%H%M%S")+".png"
        print("Pathmap image saved at "+image_save_path)
        img.save(image_save_path, "PNG")


    def shutdown(self):
        print("DEBUG-publish1")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        word=raw_input("Type a word:")
        print("word:", word)
        path=get_path(word)
        print("path loaded")
        PaintLetter(path)

        print("End of Main Function")

    except Exception as e:
        print(e)
        rospy.loginfo("shutdown program.")
