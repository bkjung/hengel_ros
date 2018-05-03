#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion, PoseStamped
from std_msgs.msg import Float32
from OperationMode.msg import *
from ValveInput.msg import *
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

thres1=np.deg2rad(30)
thres2=np.deg2rad(15)
#thres3=np.deg2rad(8)
thres3=np.deg2rad(4)
#thres3=np.deg2rad(12)

#at turtlebot3
# ang_vel_1=0.35
# ang_vel_2=0.2
# ang_vel_3=0.04
# lin_vel=0.06

#at hengel bot
# ang_vel_1=0.08
# ang_vel_2=0.04
# ang_vel_3=0.02
ang_vel_1=0.15
ang_vel_2=0.1
ang_vel_3=0.06
lin_vel=0.07
#lin_vel=0.09


package_base_path = os.path.abspath(os.path.join(os.path.dirname(__file__),"../.."))
os.system("mkdir -p "+package_base_path+"/hengel_path_manager/output_pathmap")


def normalize_rad(input_angle):
    if input_angle>pi:
        return input_angle-2*pi
    elif input_angle<=-pi:
        return input_angle+2*pi
    else:
        return input_angle

class NavigationControl():
    def __init__(self, arr_path):
        global waypoints_length
        global waypoints

        rospy.init_node('hengel_navigation_control', anonymous=False, disable_signals=True)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.position_subscriber = rospy.Sublisher('/current_position', Point, self.callback_position) 
        self.heading_subscriber = rospy.Sublisher('/current_heading', Float32, self.callback_heading)

        self.point = Point()
        self.heading = Float32()
        self.move_cmd = Twist()
        
        r = rospy.Rate(50)        


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
                            self.move_cmd.linear.x=0
                            self.move_cmd.angular.z=ang_vel_1

                        else:
                            self.move_cmd.linear.x=0
                            self.move_cmd.angular.z=-ang_vel_1

                    elif abs(alpha)>thres2:
                        if alpha>0 or alpha<-pi:
                            self.move_cmd.linear.x=0
                            self.move_cmd.angular.z=ang_vel_2

                        else:
                            self.move_cmd.linear.x=0
                            self.move_cmd.angular.z=-ang_vel_2
                    elif abs(alpha)>thres3:
                        if alpha>0 or alpha<-pi:
                            self.move_cmd.linear.x=0
                            self.move_cmd.angular.z=ang_vel_3
                        else:
                            self.move_cmd.linear.x=0
                            self.move_cmd.angular.z=-ang_vel_3

                    else:
                        x=distance*sin(alpha)
                        curv=2*x/pow(distance,2)

                        if distance<0.08:
                            #lin_vel_scaled=lin_vel/4.0
                            lin_vel_scaled=lin_vel/2.0
                        else:
                            lin_vel_scaled=lin_vel

                        self.move_cmd.linear.x=lin_vel_scaled
                        self.move_cmd.angular.z=curv*lin_vel_scaled

                    #print("DEBUG. alpha=", np.rad2deg(alpha), " angular.z=", self.move_cmd.angular.z)
                    self.cmd_vel.publish(self.move_cmd)

                    (position, rotation) = self.get_odom()
                    heading.data=rotation
                    self.position_publisher.publish(position)
                    self.heading_publisher.publish(rotation)

                    global cnt_path_points
                    global path_points

                    cnt_path_points = cnt_path_points + 1
                    path_points.append([position.x, position.y])

                    distance = sqrt(pow((current_waypoint[0] - position.x), 2) + pow((current_waypoint[1] - position.y), 2))

                    r.sleep()
                except KeyboardInterrupt:
                    print("Got KeyboardInterrupt")
                    rospy.signal_shutdown("KeyboardInterrupt")
                    break

            print("Now at Waypoint No.", waypoint_index)
            waypoint_index = waypoint_index + waypoint_increment

            if rospy.is_shutdown():
                break

        rospy.loginfo("Stopping the robot at the final destination")
        self.generate_pathmap()

        print("DEBUG-publish0")
        self.cmd_vel.publish(Twist())

    def callback_position(self, _data):
        self.point.x = data.x
        self.point.y = data.y
        

    def callback_heading(self, _data):
        self.heading.data = _data.data


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
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)


# if __name__ == '__main__':
#     try:
#         path=#########
#         NavigationControl(path)
#         print("End of Main Function")

#     except Exception as e:
#         print(e)
#         rospy.loginfo("shutdown program.")
#         sys.exit()
