#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion, PoseStamped
from std_msgs.msg import Float32
from hengel_navigation.msg import ValveInput, OperationMode
import tf
from math import radians, copysign, sqrt, pow, pi, atan2, sin, floor, cos
from tf.transformations import euler_from_quaternion
import numpy as np
import sys
from PIL import Image
import time
import os

#VALVE_OPEN = 1023
#VALVE_OPEN = 800
VALVE_OPEN=900
VALVE_CLOSE = 512

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
    def __init__(self, _arr_path, _draw_start_index):
        self.arr_path = _arr_path
        self.draw_start_index  = _draw_start_index

        self.initial_setting()
        self.run()


    def initial_setting(self):
        self.point = Point()
        self.heading = Float32()
        self.move_cmd = Twist()

        self.valve_operation_mode = OperationMode()
        self.valve_operation_mode.mode=1
        self.valve_angle_input = ValveInput()

        self.valve_status = VALVE_OPEN

        self.r = rospy.Rate(50) #50hz

        self.waypoint_increment = 1
        self.waypoints_length = 0
        self.waypoint_index = 1  #starting from index No. 1 (c.f. No.0 is at the origin(0,0))
        #self.waypoint_index = 0  # No.0 is at the origin(0,0)
        self.waypoints=[]
        self.current_waypoint = []
        self.cnt_letter = 0

        self.cnt_path_points = 0
        self.path_points = []

        self.thres1=np.deg2rad(30)
        self.thres2=np.deg2rad(15)
        self.thres3=np.deg2rad(4)

        self.ang_vel_1=0.15
        self.ang_vel_2=0.1
        self.ang_vel_3=0.06
        self.lin_vel=0.07

        self.loop_cnt=0
        self.isGoodToGo=False

        #rospy.init_node('hengel_navigation_control', anonymous=False, disable_signals=True)
        rospy.on_shutdown(self.shutdown)

        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.valve_angle_publisher = rospy.Publisher('/valve_input', ValveInput, queue_size=5)
        self.valve_operation_mode_publisher = rospy.Publisher('/operation_mode', OperationMode, queue_size=5)

        self.position_subscriber = rospy.Subscriber('/current_position', Point, self.callback_position)
        self.heading_subscriber = rospy.Subscriber('/current_heading', Float32, self.callback_heading)

    def run(self):
        # go through path array
        self.waypoints_length = len(self.arr_path)
        for idx in range(self.waypoints_length):
            self.waypoints.append([self.arr_path[idx][0]-self.arr_path[0][0], self.arr_path[idx][1]-self.arr_path[0][1]])

        print("size of waypoints = ", len(self.waypoints))

        while self.waypoint_index < self.waypoints_length:
            print("current waypoint index: "+str(self.waypoint_index))
            self.current_waypoint = [self.waypoints[self.waypoint_index][0], self.waypoints[self.waypoint_index][1]]

            goal_distance = sqrt(pow(self.current_waypoint[0] - self.point.x, 2) + pow(self.current_waypoint[1] - self.point.y, 2))
            distance = goal_distance

            while distance > 0.03:
                if rospy.is_shutdown():
                    break
                try:
                    #wait for 2sec to initialize position and heading input
                    if self.isGoodToGo==False:
                        if self.loop_cnt==100:
                            self.isGoodToGo=True
                        else:
                            self.loop_cnt=self.loop_cnt+1
                    else:
                        self.valve_operation_mode_publisher.publish(self.valve_operation_mode)
                        self.valve_angle_input.goal_position = self.valve_status
                        self.valve_angle_publisher.publish(self.valve_angle_input)

                        print("CURRENT: "+str(self.point.x)+", "+str(self.point.y)+"  NEXT: "+str(self.current_waypoint[0])+", "+str(self.current_waypoint[1]))

                        alpha=normalize_rad(atan2(self.current_waypoint[1]-self.point.y, self.current_waypoint[0]-self.point.x)-self.heading.data)

                        print("heading error: %0.3f" % np.rad2deg(alpha))

                        if abs(alpha)> self.thres1: #abs?
                            # if alpha>0 or alpha<-pi:
                            if alpha>0:
                                self.move_cmd.linear.x=0
                                self.move_cmd.angular.z=self.ang_vel_1
                            else:
                                self.move_cmd.linear.x=0
                                self.move_cmd.angular.z=-self.ang_vel_1

                        elif abs(alpha)>self.thres2:
                            # if alpha>0 or alpha<-pi:
                            if alpha>0:
                                self.move_cmd.linear.x=0
                                self.move_cmd.angular.z=self.ang_vel_2
                            else:
                                self.move_cmd.linear.x=0
                                self.move_cmd.angular.z=-self.ang_vel_2

                        elif abs(alpha)>self.thres3:
                            # if alpha>0 or alpha<-pi:
                            if alpha>0:
                                self.move_cmd.linear.x=0
                                self.move_cmd.angular.z=self.ang_vel_3
                            else:
                                self.move_cmd.linear.x=0
                                self.move_cmd.angular.z=-self.ang_vel_3

                        else:
                            x=distance*sin(alpha)
                            curv=2*x/pow(distance,2)

                            if distance<0.08:
                                lin_vel_scaled=self.lin_vel/2.0
                            else:
                                lin_vel_scaled=self.lin_vel

                            self.move_cmd.linear.x=lin_vel_scaled
                            self.move_cmd.angular.z=curv*lin_vel_scaled

                        self.cmd_vel.publish(self.move_cmd)

                        self.cnt_path_points = self.cnt_path_points + 1
                        self.path_points.append([self.point.x, self.point.y])

                        distance = sqrt(pow((self.current_waypoint[0] - self.point.x), 2) + pow((self.current_waypoint[1] - self.point.y), 2))

                    self.r.sleep()

                except KeyboardInterrupt:
                    print("Got KeyboardInterrupt")
                    rospy.signal_shutdown("KeyboardInterrupt")
                    break

            #print("Now at Waypoint No.", self.waypoint_index)
            self.control_valve()

            self.waypoint_index = self.waypoint_index + self.waypoint_increment

        #Wait for 2 seconds to close valve
        self.quit_valve()

        rospy.loginfo("Stopping the robot at the final destination")
        self.cmd_vel.publish(Twist())

        self.generate_pathmap()

    def callback_position(self, _data):
        self.point.x = _data.x
        self.point.y = _data.y

    def callback_heading(self, _data):
        self.heading.data = normalize_rad(_data.data)

    def generate_pathmap(self):
        scale = 10
        pixel_size = 100 #1m*1m canvas of 1cm accuracy points (including boundary points)
        img = Image.new("RGB", ((100+pixel_size*self.cnt_letter)*scale, (100+pixel_size)*scale), (255, 255, 255))

        print("cnt_path_points = ", self.cnt_path_points)

        for i in range(self.cnt_path_points):
            # print(self.path_points[i][0], self.path_points[i][1])
            x = 0.99 if self.path_points[i][0]==1.0 else self.path_points[i][0]
            y = 0.99 if (1.0-self.path_points[i][1])==1.0 else (1.0-self.path_points[i][1])
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

    def control_valve(self):
        for i in range(len(self.draw_start_index)):
            if self.waypoint_index < self.draw_start_index[i] and (self.waypoint_index + self.waypoint_increment) >= self.draw_start_index[i]:
                self.valve_status = VALVE_CLOSE
                return

        self.valve_status = VALVE_OPEN


    def quit_valve(self):
        for ind_quit in range(100):
            self.valve_angle_input.goal_position = VALVE_CLOSE
            self.valve_angle_publisher.publish(self.valve_angle_input)

            ind_quit = ind_quit+1
            self.r.sleep()

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
