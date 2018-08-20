#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion, PoseStamped
from std_msgs.msg import Float32, Time
from sensor_msgs.msg import Image, CompressedImage
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from hengel_navigation.msg import ValveInput, OperationMode
import tf
from math import radians, copysign, sqrt, pow, pi, atan2, sin, floor, cos, asin
from tf.transformations import euler_from_quaternion
import numpy as np
import sys
import time
import os
import cv2
import cv_bridge
import logging
import matplotlib.pyplot as plt
# import skimage.io as ski_io

#import PIL.Image
#import PIL.ImageTk

MOTOR_RESOLUTION = 3260
SPRAY_OFF = 1024

scale_factor = 3  #[pixel/cm]
robot_size = 15  #[cm]; diameter

package_base_path = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "../.."))
home_path = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "../../../../..")
)
os.system("mkdir -p " + package_base_path +
        "/hengel_path_manager/output_pathmap")

Kp = 5.0  # speed proportional gain



def pid_control(target, current):
    a = Kp * (target - current)

    return a

def normalize_rad(input_angle):
    if input_angle > pi:
        return input_angle - 2 * pi
    elif input_angle <= -pi:
        return input_angle + 2 * pi
    else:
        return input_angle


def angle_difference(angle1, angle2):
    return normalize_rad(normalize_rad(angle1) - normalize_rad(angle2))


class NavigationControl():
    def __init__(self, _arr_path, _arr_intensity, _start_point_list, _end_point_list, _isPositionControl, _isIntensityControl, _isStartEndIndexed, _D):
        while True:
            word = raw_input(
                    "There are options for motor profile change smoothing buffer.\n[1] Enable by delta_theta \n[2] Enable by waypoint  \n[3] Disable \nType :"
                    )
            self.motor_buffer_option = int(word)
            if self.motor_buffer_option == 1 or self.motor_buffer_option ==2 or self.motor_buffer_option ==3:
                break

        self.arr_path = _arr_path
        self.arr_intensity = _arr_intensity
        self.start_point_list = _start_point_list
        self.end_point_list = _end_point_list
        self.isPositionControl = _isPositionControl
        self.isIntensityControl = _isIntensityControl
        self.isStartEndIndexed = _isStartEndIndexed
        self.D=_D
        self.img=np.full((2000,2000), 255)

        self.offset_accepted = False

        self.initial_setting()

        while True:
            word = raw_input(
                    "There are options for real navigation or simulation.\n[1] Real Navigation (GO!!!!!!!!) \n[2] Simulated Result save :) \nType 1 or 2 :"
                    )
            self.simulation_option = int(word)
            if self.simulation_option== 1 or self.simulation_option==2 :
                break

        if self.isPositionControl:
            while True:
                word = raw_input(
                        # "There are 3 options for spray intensity.\n[1] Input from waypoint file \n[2] Constant 740 \n[3] Sinusoidal Fluctuation \nType :"
                        "There are 2 options for spray intensity.\n[1] Input from waypoint file (or StartEnd indexed) \n[2] Constant 740 \nType :"
                        )
                self.intensity_option = int(word)
                # if self.intensity_option==1 or self.intensity_option==2 or self.intensity_option==3:
                if self.intensity_option==1 or self.intensity_option==2:
                    break

            while True:
                word = raw_input(
                        "There are 2 options for cam_image save.\n[1] Do NOT stop & save \n[2] DO stop & save periodically \nType :"
                        )
                self.option_cam_save = int(word)
                if self.option_cam_save==1 or self.option_cam_save==2:
                    break

            if self.option_cam_save==2:
                word = raw_input(
                        "CAM SAVE Period (No. Waypoints) \nType :"
                        )
                self.cam_save_period_waypoints = int(word)
                word = raw_input(
                        "CAM SAVE Point (x,y, theta) \nType x: "
                        )
                self.cam_save_x = float(word)
                word = raw_input(
                        "CAM SAVE Point (x,y, theta) \nType y: "
                        )
                self.cam_save_y = float(word)
                word = raw_input(
                        "CAM SAVE Point (x,y, theta) \nType theta(deg): "
                        )
                self.cam_save_theta_deg = float(word)

            if self.simulation_option==1:
                self.runOffset()
            else:
                self.saveSimulation()
        else:
            print("This option cannot be executed. SORRY :(")

    def initial_setting(self):
        self.program_start_time = time.strftime("%y%m%d_%H%M%S")

        #self.R = 0.115/2 #radius of wheel
        #self.L = 0.33/2 #half of distance btw two wheels

        #test flag bot
        #self.R = 0.11/2 #radius of wheel
        self.R = 0.1249/2 #radius of wheel
        #self.R = 0.12475/2 #radius of wheel    #CORRECT ONE
        #self.R = 0.1237/2 #radius of wheel
        self.L = 0.3544/2 #half of distance btw two wheels
        #self.L = 0.347/2 #half of distance btw two wheels
        #self.L = 0.357/2 #half of distance btw two wheels

        self.point = Point()
        self.point_encoder = Point()
        self.pen_distance_per_loop = Float32()
        self.endPoint= Point()
        self.heading = Float32()
        self.heading.data = pi
        self.heading_encoder = Float32()
        self.move_cmd = Twist()

        self.valve_operation_mode = OperationMode()
        self.valve_operation_mode.mode = 1
        self.valve_angle_input = ValveInput()

        self.valve_status = SPRAY_OFF

        self.dt = 0.02  # [s]
        #self.dt = 0.01  # [s]
        self.freq = 1.0/self.dt
        self.r = rospy.Rate(1.0/self.dt)

        self.dt_sim= 0.00000001  # [s]
        self.r_sim = rospy.Rate(1.0/self.dt)

        #It SHOULD BE 1 for current code.
        #If it's not 1, then self.check_whether_moving_to_next_start() should be modified
        #self.waypoint_increment = 1

        self.letter_index = 0
        self.segment_index = 0
        self.waypoint_index_in_current_segment = 0

        self.next_letter_index = 0
        self.next_segment_index = 0
        self.next_waypoint_index_in_current_segment = 0

        self.waypoints = []
        self.current_waypoint = []
        self.next_waypoint = []
        self.cnt_letter = 0
        self.cnt_total_waypoints = 0
        self.cnt_segments_in_current_letter = 0
        self.cnt_waypoints_in_current_segment = 0
        self.cnt_waypoints = -1

        self.traj = Marker()
        self.traj.header.frame_id = '/odom'
        self.traj.header.stamp = rospy.get_rostime()
        self.traj.ns = "hengel_traj"
        self.traj.action = Marker.ADD
        self.traj.pose.orientation.w = 1.0
        self.traj.type = Marker.LINE_STRIP
        self.traj.scale.x = 0.01 # line width
        self.traj.color.r = 1.0
        self.traj.color.b = 0.0
        self.traj.color.a = 1.0

        # if(WRITE_MULTIPLE_SHAPES):
        #     traj.id = shapeCount;
        # else:
        #     traj.id = 0; #overwrite any existing shapes
        #     traj.lifetime.secs = 1; #timeout for display
        self.traj.id = 0; #overwrite any existing shapes
        self.traj.lifetime.secs = 1; #timeout for display

        self.traj.points = []

        self.traj.id = 0; #overwrite any existing shapes
        self.traj.lifetime.secs = 1; #timeout for display

        self.traj.points = []



        self.traj_painting = Marker()
        self.traj_painting.header.frame_id = '/odom'
        self.traj_painting.header.stamp = rospy.get_rostime()
        self.traj_painting.ns = "painting_traj"
        self.traj_painting.action = Marker.ADD
        self.traj_painting.pose.orientation.w = 1.0
        self.traj_painting.type = Marker.LINE_STRIP

        self.traj_painting.scale.x = 0.01 # line width
        self.traj_painting.color.r = 0.0
        self.traj_painting.color.b = 1.0
        self.traj_painting.color.a = 1.0


        self.traj_painting.id = 0; #overwrite any existing shapes

        self.traj_painting.lifetime.secs = 1; #timeout for display

        self.traj_painting.points = []

        self.traj_painting.id = 0; #overwrite any existing shapes
        self.traj_painting.lifetime.secs = 1; #timeout for display

        self.traj_painting.points = []



        self.traj_encoder = Marker()
        self.traj_encoder.header.frame_id = '/odom'
        self.traj_encoder.header.stamp = rospy.get_rostime()
        self.traj_encoder.ns = "hengel_traj_encoder"
        self.traj_encoder.action = Marker.ADD
        self.traj_encoder.pose.orientation.w = 1.0
        self.traj_encoder.type = Marker.LINE_STRIP
        self.traj_encoder.scale.x = 0.01 # line width
        self.traj_encoder.color.r = 1.0
        self.traj_encoder.color.b = 1.0
        self.traj_encoder.color.a = 1.0
        self.traj_encoder.id = 0; #overwrite any existing shapes
        self.traj_encoder.lifetime.secs = 1; #timeout for display
        self.traj_encoder.points = []
        self.traj_encoder.id = 0; #overwrite any existing shapes
        self.traj_encoder.lifetime.secs = 1; #timeout for display
        self.traj_encoder.points = []

        self.thres1 = np.deg2rad(30)
        self.thres2 = np.deg2rad(15)
        self.thres3 = np.deg2rad(4)

        self.ang_vel_1 = 0.15
        self.ang_vel_2 = 0.1
        self.ang_vel_3 = 0.06
        #self.lin_vel = 0.07
        self.lin_vel = 0.10

        self.target_speed = 0.0
        self.current_speed = 0.0

        self.is_moving_between_segments = True

        self.loop_cnt_pathmap = 0

        self.map_img = []
        self.map_img = np.ndarray(self.map_img)

        #rospy.init_node('hengel_navigation_control', anonymous=False, disable_signals=True)
        rospy.on_shutdown(self.shutdown_everything)

        self.valve_angle_publisher = rospy.Publisher(
                '/valve_input', ValveInput, queue_size=5)
        self.valve_operation_mode_publisher = rospy.Publisher(
                '/operation_mode', OperationMode, queue_size=5)
        self.valve_operation_mode_publisher.publish(
                self.valve_operation_mode)
        self.valve_angle_input.goal_position = self.valve_status
        self.valve_angle_publisher.publish(
                self.valve_angle_input)

        self.vision_offset_subscriber = rospy.Subscriber(
                '/offset_change', Point, self.callback_vision_offset)

        self.pub_markers = rospy.Publisher('/robot_trajectory_visualization', Marker, queue_size=5)
        self.pub_markers_painting = rospy.Publisher('/painting_visualization', Marker, queue_size=5)
        self.pub_markers_encoder = rospy.Publisher('/robot_trajectory_encoder_visualization', Marker, queue_size=5)

        self.pub_delta_theta_1 = rospy.Publisher('/delta_theta_1', Float32, queue_size=5)
        self.pub_delta_theta_2 = rospy.Publisher('/delta_theta_2', Float32, queue_size=5)

        self.pub_distance = rospy.Publisher('/distance', Float32, queue_size=5)
        self.pub_endpoint = rospy.Publisher('/endpoint', Point, queue_size=5)
        self.pub_midpoint = rospy.Publisher('/midpoint', Point, queue_size=5)
        self.pub_midpoint_time = rospy.Publisher('/midpoint_time', Time, queue_size=5)

        print("Initialize Done")

    def runOffset(self):
        print("--runOffset begin--")
        self.shutdown_for_seconds(5.0)
        # go through path array
        rospy.loginfo("number of letters = " + str(len(self.arr_path)))
        for idx_letter in range(len(self.arr_path)):
            rospy.loginfo("number of letter segments in letter no." + str(idx_letter) +
                    " = " + str(len(self.arr_path[idx_letter])))
            for idx_segment in range(len(self.arr_path[idx_letter])):
                #waypoints_in_segment = []
                for idx_waypoint in range(len(self.arr_path[idx_letter][idx_segment])):
                    self.cnt_total_waypoints = self.cnt_total_waypoints + 1
                #self.waypoints.append(waypoints_in_segment)


        print("Total Number of Waypoints : "+str(self.cnt_total_waypoints))

        self.cnt_letter = len(self.arr_path)
        self.th1=0
        self.th2=0
        # hskim added
        arr_delOmega = []

        pubDelta1=0         #previously published delta_1
        pubDelta2=0         #previously published delta_2
        pubIter=0

        cnt_delta_buffer = 0

        while self.letter_index < self.cnt_letter:
            if rospy.is_shutdown():
                break
            self.cnt_segments_in_current_letter = len(
                    self.arr_path[self.letter_index])
            while self.segment_index < self.cnt_segments_in_current_letter:
                if rospy.is_shutdown():
                    break
                self.cnt_waypoints_in_current_segment = len(
                        self.arr_path[self.letter_index][self.segment_index])
                while self.waypoint_index_in_current_segment < self.cnt_waypoints_in_current_segment:
                    if rospy.is_shutdown():
                        break
                    # rospy.loginfo("\n\nwaypoint index : " +
                    #         str(self.waypoint_index_in_current_segment) +
                    #         " in segment no. " + str(self.segment_index) +
                    #         " in letter no. " + str(self.letter_index))
                    self.current_waypoint = [
                        self.arr_path[self.letter_index][self.segment_index][
                            self.waypoint_index_in_current_segment][0],
                        self.arr_path[self.letter_index][self.segment_index][
                            self.waypoint_index_in_current_segment][1]
                        ]
                    self.cnt_waypoints += 1

                    if self.offset_accepted == True:
                        new_point_x=self.point.x + self.offset_x
                        new_point_y=self.point.y + self.offset_y
                        new_point_theta = self.heading.data + self.offset_theta

                        new_endpoint_x=new_point_x-self.D*cos(self.heading.data)
                        new_endpoint_y=new_point_y-self.D*sin(self.heading.data)
                        new_endpoint_z=0.0

                        dist=sqrt(pow(new_endpoint_x-self.current_waypoint[0],2)+pow(new_endpoint_y-self.current_waypoint[1],2))

                        # if the acquired offset is too large, dismiss it.
                        if dist>0.5:
                            print("VISUAL OFFSET is too LARGE!!! (Dismissing the calculated offset)")
                            pass

                        elif dist>0.001*2.0:
                            div=int(ceil(dist/0.001))
                            print("VISUAL OFFSET inserted %d waypoints" % (div))

                            self.point.x=new_point_x
                            self.point.y=new_point_y
                            self.heading.data = new_point_theta

                            for k in range(div-1):
                                x=new_point_x+(k+1)/float(div)*(self.current_waypoint[0]-new_point_x)
                                y=new_point_y+(k+1)/float(div)*(self.current_waypoint[1]-new_point_y)
                                self.robotNavigationLoop([x,y])

                        else:
                            print("VISUAL OFFSET inserted 1 waypoint")
                            self.point.x=new_point_x
                            self.point.y=new_point_y
                            self.heading.data = new_point_theta

                        self.offset_accepted = False

                    else:
                        pass

                    if self.isStartEndIndexed:
                        if (self.cnt_waypoints) in self.start_point_list:
                            self.is_moving_between_segments = False
                        elif (self.cnt_waypoints) in self.end_point_list:
                            self.is_moving_between_segments = True
                        else:
                            pass

                    if self.option_cam_save == 2:
                        if self.cnt_waypoints!=0 and self.cnt_waypoints%self.cam_save_period_waypoints==0:
                            self.go_to_point_and_come_back(self.cam_save_x, self.cam_save_y, self.cam_save_theta_deg)

                    # Motion Control

                    self.robotNavigationLoop(self.current_waypoint)

                    while True:
                        if rospy.is_shutdown():
                            break
                        try:
                            if self.intensity_option==1:
                                if not self.isStartEndIndexed:
                                    #For this option, is_moving_between_segments does not work!!!!
                                    input_pixel_value = int(self.arr_intensity[self.cnt_waypoints])
                                    input_pixel_value_graphic = int(self.arr_intensity[self.cnt_waypoints])
                                    if input_pixel_value >=0 and input_pixel_value<256:   #if the input is alright, then

                                        #cut off value larger than 230 to 230.
                                        input_pixel_value = 230 if input_pixel_value>230 else input_pixel_value
                                        spray_input = 660.0+(1024.0-660.0)*(float(input_pixel_value)/230.0)
                                        # self.spray_intensity_publisher.publish(spray_input)
                                        self.valve_angle_input.goal_position = int(spray_input)
                                        self.valve_angle_publisher.publish(self.valve_angle_input)
                                else:
                                    if self.is_moving_between_segments==True:
                                        # self.spray_intensity_publisher.publish(1024.0)
                                        self.valve_angle_input.goal_position = 1024
                                        self.valve_angle_publisher.publish(self.valve_angle_input)
                                    else:
                                        #self.spray_intensity_publisher.publish(660.0)
                                        # self.spray_intensity_publisher.publish(740.0)
                                        self.valve_angle_input.goal_position = 660
                                        self.valve_angle_publisher.publish(self.valve_angle_input)


                            elif self.intensity_option==2:
                                input_pixel_value_graphic=0
                                self.valve_angle_input.goal_position = 740
                                self.valve_angle_publisher.publish(self.valve_angle_input)

                            self.endPoint.x=self.point.x-self.D*cos(self.heading.data)
                            self.endPoint.y=self.point.y-self.D*sin(self.heading.data)
                            self.endPoint.z=float(input_pixel_value_graphic)
                            self.point.z=self.heading.data
                            self.pub_midpoint.publish(self.point)
                            self.pub_endpoint.publish(self.endPoint)

                            #print(str(self.cnt_waypoints)+"  "+str(self.endPoint.x)+"  "+str(self.endPoint.y))
                            print(str(self.endPoint.x)+"  "+str(self.endPoint.y))

                            #print("distance: ", distance)
                            #print("waypoint: ", self.current_waypoint)
                            #print("endpoint: ", self.endPoint)

                            th = self.heading.data
                            delX= self.current_waypoint[0]-self.endPoint.x
                            delY= self.current_waypoint[1]-self.endPoint.y

                            # hskim added
                            a = delX*sin(th) - delY*cos(th)
                            b = delX*cos(th) + delY*sin(th)
                            c = b - self.D
                            if arr_delOmega:
                                self.D = self.calculate_optimal_D(delX, delY, th, self.D, arr_delOmega[-1][0], arr_delOmega[-1][1])
                                # self.D = 0.233

                            delOmega = asin(a/self.D)
                            delS = self.D*cos(delOmega) + c
                            delOmega1 = (1/self.R)*(delS + self.L*delOmega)
                            delOmega2 = (1/self.R)*(delS - self.L*delOmega)
                            arr_delOmega.append([delOmega1, delOmega2])

                            # delOmega1= (1/self.R)*(delS+2*self.L*delOmega)
                            # delOmega2= (1/self.R)*(delS-2*self.L*delOmega)

                            if self.motor_buffer_option == 1:       #Motor Smoothing Buffer Enabled
                                pass
                            #    if abs(delOmega1 - pubDelta1) >= 0.01 and abs(delOmega2 - pubDelta2) >= 0.01:
                            #        pubIter = max(floor(abs(delOmega1 - pubDelta1)/0.01), floor(abs(delOmega2 - pubDelta2)/0.01))
                            #        cnt_delta_buffer += pubIter
                            #        # print("---------ITERATION(0/%d)--------- " % (pubIter))
                            #    elif abs(delOmega1 - pubDelta1) >= 0.01:
                            #        pubIter = floor(abs(delOmega1 - pubDelta1)/0.01)
                            #        # print("---------ITERATION(0/%d)--------- " % (pubIter))
                            #        cnt_delta_buffer += pubIter
                            #    elif abs(delOmega2 - pubDelta2) >= 0.01:
                            #        pubIter = floor(abs(delOmega2 - pubDelta2)/0.01)
                            #        # print("---------ITERATION(0/%d)--------- " % (pubIter))
                            #        cnt_delta_buffer += pubIter
                            #    else:
                            #        pubIter = 1

                            #    for iteration in range(int(pubIter)):
                            #        control_input_1 = pubDelta1 + (float)(delOmega1-pubDelta1)/pubIter*(iteration+1)
                            #        control_input_2 = pubDelta2 + (float)(delOmega2-pubDelta2)/pubIter*(iteration+1)
                            #        self.pub_delta_theta_1.publish(control_input_1)
                            #        self.pub_delta_theta_2.publish(control_input_2)
                            #        if pubIter != 1:
                            #            # print("---------ITERATION(%d/%d)--------- " % (iteration+1,pubIter))
                            #            pass


                            #        self.r.sleep()

                            #    pubDelta1 = delOmega1
                            #    pubDelta2 = delOmega2

                            #    delXrobotLocal, delYrobotLocal = self.calculate_robot_local_delta_from_omega(delOmega1, delOmega2)
                            #    delXrobotGlobal, delYrobotGlobal = np.matmul([[cos(self.heading.data), -sin(self.heading.data)],[sin(self.heading.data), cos(self.heading.data)]], [delXrobotLocal, delYrobotLocal])
                            #    self.point.x=self.point.x+delXrobotGlobal
                            #    self.point.y=self.point.y+delYrobotGlobal
                            #    self.heading.data=self.heading.data+self.R*(delOmega1-delOmega2)/(2*self.L)
                            #    self.pen_distance_per_loop=sqrt(
                            #        pow(delXrobotGlobal, 2) +
                            #        pow(delYrobotGlobal, 2)
                            #        )
                            #    print(str(delOmega1)+"  "+str(delOmega2)+"  "+str(self.pen_distance_per_loop))

                            #    break

                            elif self.motor_buffer_option == 2:
                                pass
                            #    if self.next_letter_index != -1:  # if current waypoint is not the end
                            #        if abs(delOmega1 - pubDelta1) >= 0.01 and abs(delOmega2 - pubDelta2) >= 0.01:
                            #            pubIter = max(floor(abs(delOmega1 - pubDelta1)/0.01), floor(abs(delOmega2 - pubDelta2)/0.01))
                            #            cnt_delta_buffer += pubIter
                            #            # print("---------ITERATION(0/%d)--------- " % (pubIter))
                            #        elif abs(delOmega1 - pubDelta1) >= 0.01:
                            #            pubIter = floor(abs(delOmega1 - pubDelta1)/0.01)
                            #            # print("---------ITERATION(0/%d)--------- " % (pubIter))
                            #            cnt_delta_buffer += pubIter
                            #        elif abs(delOmega2 - pubDelta2) >= 0.01:
                            #            pubIter = floor(abs(delOmega2 - pubDelta2)/0.01)
                            #            # print("---------ITERATION(0/%d)--------- " % (pubIter))
                            #            cnt_delta_buffer += pubIter
                            #        else:
                            #            pubDelta1 = delOmega1
                            #            pubDelta2 = delOmega2
                            #            self.pub_delta_theta_1.publish(pubDelta1)
                            #            self.pub_delta_theta_2.publish(pubDelta2)

                            #            delXrobotLocal, delYrobotLocal = self.calculate_robot_local_delta_from_omega(delOmega1, delOmega2)
                            #            delXrobotGlobal, delYrobotGlobal=np.matmul([[cos(self.heading.data), -sin(self.heading.data)],[sin(self.heading.data), cos(self.heading.data)]], [delXrobotLocal, delYrobotLocal])
                            #            self.point.x=self.point.x+delXrobotGlobal
                            #            self.point.y=self.point.y+delYrobotGlobal
                            #            self.heading.data=self.heading.data+self.R*(delOmega1-delOmega2)/(2*self.L)
                            #            self.pen_distance_per_loop=sqrt(
                            #                    pow(delXrobotGlobal, 2) +
                            #                    pow(delYrobotGlobal, 2)
                            #                    )
                            #            print(str(pubDelta1)+"  "+str(pubDelta2)+"  "+str(self.pen_distance_per_loop))

                            #            self.r.sleep()

                            #            break

                            #        delX_original= self.current_waypoint[0]-self.endPoint.x
                            #        delY_original= self.current_waypoint[1]-self.endPoint.y
                            #        delX=delX_original/pubIter
                            #        delY=delY_original/pubIter
                            #        for iteration in range(int(pubIter)):
                            #            self.endPoint.x=self.point.x-self.D*cos(self.heading.data)
                            #            self.endPoint.y=self.point.y-self.D*sin(self.heading.data)

                            #            self.next_waypoint = [
                            #                    self.arr_path[self.next_letter_index][self.next_segment_index][
                            #                        self.next_waypoint_index_in_current_segment][0],
                            #                    self.arr_path[self.next_letter_index][self.next_segment_index][
                            #                        self.next_waypoint_index_in_current_segment][1]
                            #                    ]
                            #            # I realized that next_waypoint is freaking unnecessary FUCK

                            #            th = self.heading.data


                            #            delOmega= asin((delX*sin(th)-delY*cos(th))/(self.D))
                            #            delS= self.D*cos(delOmega)-self.D+delX*cos(th)+delY*sin(th)

                            #            delOmega1= (1/self.R)*(delS+2*self.L*delOmega) * 0.75
                            #            delOmega2= (1/self.R)*(delS-2*self.L*delOmega) * 0.75

                            #            self.pub_delta_theta_1.publish(delOmega1)
                            #            self.pub_delta_theta_2.publish(delOmega2)

                            #            # if pubIter != 1:
                            #            #     print("THIS SHOULD NOT HAPPEN :) IF THIS SHOWS UP, YOU'RE FUCKED UP")

                            #            delXrobotLocal, delYrobotLocal = self.calculate_robot_local_delta_from_omega(delOmega1, delOmega2)
                            #            delXrobotGlobal, delYrobotGlobal = np.matmul([[cos(self.heading.data), -sin(self.heading.data)],[sin(self.heading.data), cos(self.heading.data)]], [delXrobotLocal, delYrobotLocal])
                            #            self.point.x=self.point.x+delXrobotGlobal
                            #            self.point.y=self.point.y+delYrobotGlobal
                            #            self.heading.data=self.heading.data+self.R*(delOmega1-delOmega2)/(2*self.L)
                            #            self.pen_distance_per_loop=sqrt(
                            #                pow(delXrobotGlobal, 2) +
                            #                pow(delYrobotGlobal, 2)
                            #                )
                            #            print(str(delOmega1)+"  "+str(delOmega1)+"  "+str(self.pen_distance_per_loop))
                            #            pubDelta1 = delOmega1
                            #            pubDelta2 = delOmega2
                            #            self.r.sleep()

                            #        break

                            #    #if current waypoint is the end
                            #    else:
                            #        pubDelta1 = delOmega1
                            #        pubDelta2 = delOmega2
                            #        self.pub_delta_theta_1.publish(pubDelta1)
                            #        self.pub_delta_theta_2.publish(pubDelta2)

                            #        delXrobotLocal, delYrobotLocal = self.calculate_robot_local_delta_from_omega(delOmega1, delOmega2)
                            #        delXrobotGlobal, delYrobotGlobal=np.matmul([[cos(self.heading.data), -sin(self.heading.data)],[sin(self.heading.data), cos(self.heading.data)]], [delXrobotLocal, delYrobotLocal])
                            #        self.point.x=self.point.x+delXrobotGlobal
                            #        self.point.y=self.point.y+delYrobotGlobal
                            #        self.heading.data=self.heading.data+self.R*(delOmega1-delOmega2)/(2*self.L)
                            #        self.pen_distance_per_loop=sqrt(
                            #                pow(delXrobotGlobal, 2) +
                            #                pow(delYrobotGlobal, 2)
                            #                )
                            #        print(str(pubDelta1)+"  "+str(pubDelta2)+"  "+str(self.pen_distance_per_loop))

                            #        self.r.sleep()

                            #        break


                            #Motor Smoothing Buffer Disabled
                            elif self.motor_buffer_option == 3:
                                pubDelta1 = delOmega1
                                pubDelta2 = delOmega2
                                self.pub_delta_theta_1.publish(pubDelta1)
                                self.pub_delta_theta_2.publish(pubDelta2)
                                # delXrobotLocal, delYrobotLocal = self.calculate_robot_local_delta_from_omega(pubDelta1, pubDelta2)
                                # delXrobotGlobal, delYrobotGlobal=np.matmul([[cos(self.heading.data), -sin(self.heading.data)],[sin(self.heading.data), cos(self.heading.data)]], [delXrobotLocal, delYrobotLocal])
                                delXrobotGlobal = self.R*(delOmega1 + delOmega2)*cos(self.heading.data)/2
                                delYrobotGlobal = self.R*(delOmega1 + delOmega2)*sin(self.heading.data)/2
                                self.point.x=self.point.x+delXrobotGlobal
                                self.point.y=self.point.y+delYrobotGlobal
                                self.heading.data=self.heading.data+self.R*(delOmega1-delOmega2)/(2*self.L)
                                self.pen_distance_per_loop=sqrt(
                                    pow(delXrobotGlobal, 2) +
                                    pow(delYrobotGlobal, 2)
                                    )
                                #print(str(delOmega1)+"  "+str(delOmega2)+"  "+str(self.pen_distance_per_loop))

                                self.r.sleep()

                                break


                        except KeyboardInterrupt:
                            print("Got KeyboardInterrupt")
                            # self.cmd_vel.publish(Twist())

                            rospy.signal_shutdown("KeyboardInterrupt")
                            break


                    #Arrived at the waypoint
                    #rospy.loginfo("CURRENT: " + str(self.point.x) + ", " +
                    #        str(self.point.y) + " \t\t WAYPOINT: " +
                    #        str(self.current_waypoint[0]) + ", " +
                    #        str(self.current_waypoint[1]))

                    self.waypoint_index_in_current_segment = self.waypoint_index_in_current_segment + 1
                self.segment_index = self.segment_index + 1
                self.waypoint_index_in_current_segment = 0

            #End of current letter.

            #it's time for next letter
            self.letter_index = self.letter_index + 1
            self.segment_index = 0
            self.waypoint_index_in_current_segment = 0


        self.shutdown_for_seconds(10.0)
        rospy.loginfo("Stopping the robot at the final destination")
        print("Total Stiff Delta_Theta Change BUFFER = %d" % (cnt_delta_buffer))
        #Wait for 1 second to close valve
        # self.shutdown_everything()
        #turn to view letters at the final global map view point
        #self.look_opposite_side()
        #stop the robot
        # self.cmd_vel.publish(Twist())

    def saveSimulation(self):
        print("--save simulation begin--")
        self.shutdown_for_seconds(5.0)
        # go through path array
        rospy.loginfo("number of letters = " + str(len(self.arr_path)))
        for idx_letter in range(len(self.arr_path)):
            rospy.loginfo("number of letter segments in letter no." + str(idx_letter) +
                    " = " + str(len(self.arr_path[idx_letter])))
            for idx_segment in range(len(self.arr_path[idx_letter])):
                #waypoints_in_segment = []
                for idx_waypoint in range(len(self.arr_path[idx_letter][idx_segment])):
                    self.cnt_total_waypoints = self.cnt_total_waypoints + 1
                #self.waypoints.append(waypoints_in_segment)


        print("Total Number of Waypoints : "+str(self.cnt_total_waypoints))

        self.cnt_letter = len(self.arr_path)
        self.th1=0
        self.th2=0

        pubDelta1=0         #previously published delta_1
        pubDelta2=0         #previously published delta_2
        pubIter=0

        cnt_delta_buffer = 0

        arr_endPoint = []
        arr_robotPoint = []
        arr_leftWheel = []
        arr_rightWheel = []
        #  hskim added
        arr_delOmega = []
        arr_deldelOmega = []
        # arr_deldelOmega_x = []
        # arr_deldelOmega_y = []
        arr_deldelOmega_error = []
        arr_D = []
        arr_delD = []
        # arr_delX = []
        # arr_delY = []

        while self.letter_index < self.cnt_letter:
            if rospy.is_shutdown():
                break
            self.cnt_segments_in_current_letter = len(
                    self.arr_path[self.letter_index])
            while self.segment_index < self.cnt_segments_in_current_letter:
                if rospy.is_shutdown():
                    break
                self.cnt_waypoints_in_current_segment = len(
                        self.arr_path[self.letter_index][self.segment_index])
                while self.waypoint_index_in_current_segment < self.cnt_waypoints_in_current_segment:
                    if rospy.is_shutdown():
                        break
                    # rospy.loginfo("\n\nwaypoint index : " +
                    #         str(self.waypoint_index_in_current_segment) +
                    #         " in segment no. " + str(self.segment_index) +
                    #         " in letter no. " + str(self.letter_index))

                    self.current_waypoint = [
                            self.arr_path[self.letter_index][self.segment_index][
                                self.waypoint_index_in_current_segment][0],
                            self.arr_path[self.letter_index][self.segment_index][
                                self.waypoint_index_in_current_segment][1]
                            ]
                    self.cnt_waypoints += 1

                    if self.isStartEndIndexed:
                        if (self.cnt_waypoints) in self.start_point_list:
                            self.is_moving_between_segments = False
                        elif (self.cnt_waypoints) in self.end_point_list:
                            self.is_moving_between_segments = True
                        else:
                            pass

                    # Motion Control
                    while True:
                        if rospy.is_shutdown():
                            break
                        try:
                            if self.intensity_option==1:
                                if not self.isStartEndIndexed:
                                    #For this option, is_moving_between_segments does not work!!!!
                                    original_input_pixel_value = int(self.arr_intensity[self.cnt_waypoints])
                                    input_pixel_value = original_input_pixel_value
                                    input_pixel_value_graphic = int(self.arr_intensity[self.cnt_waypoints])
                                    if input_pixel_value >=0 and input_pixel_value<256:   #if the input is alright, then

                                        #cut off value larger than 230 to 230.
                                        input_pixel_value = 230 if input_pixel_value>230 else input_pixel_value
                                        spray_input = 660.0+(1024.0-660.0)*(float(input_pixel_value)/230.0)
                                        # self.spray_intensity_publisher.publish(spray_input)
                                        self.valve_angle_input.goal_position = int(spray_input)
                                        self.valve_angle_publisher.publish(self.valve_angle_input)
                                else:
                                    if self.is_moving_between_segments==True:
                                        # self.spray_intensity_publisher.publish(1024.0)
                                        input_pixel_value_graphic = 255
                                        self.valve_angle_input.goal_position = 1024
                                        self.valve_angle_publisher.publish(self.valve_angle_input)
                                    else:
                                        #self.spray_intensity_publisher.publish(660.0)
                                        # self.spray_intensity_publisher.publish(740.0)
                                        input_pixel_value_graphic = 0
                                        self.valve_angle_input.goal_position = 660
                                        self.valve_angle_publisher.publish(self.valve_angle_input)


                            elif self.intensity_option==2:
                                input_pixel_value_graphic=0
                                self.valve_angle_input.goal_position = 740
                                self.valve_angle_publisher.publish(self.valve_angle_input)

                            # start


                            self.endPoint.x=self.point.x-self.D*cos(self.heading.data)
                            self.endPoint.y=self.point.y-self.D*sin(self.heading.data)

                            leftWheel = Point()
                            leftWheel.x=self.point.x-self.L*sin(self.heading.data)
                            leftWheel.y=self.point.y+self.L*cos(self.heading.data)

                            rightWheel = Point()
                            rightWheel.x=self.point.x+self.L*sin(self.heading.data)
                            rightWheel.y=self.point.y-self.L*cos(self.heading.data)


                            arr_robotPoint.append((self.point.x, self.point.y))
                            arr_leftWheel.append((leftWheel.x, leftWheel.y))
                            arr_rightWheel.append((rightWheel.x, rightWheel.y))

                            # if original_input_pixel_value<=254:
                            arr_endPoint.append((self.endPoint.x, self.endPoint.y))


                            # print(str(self.endPoint.x)+"  "+str(self.endPoint.y)+"  "+str(self.point.x)+"  "+str(self.point.y)+"  "+str(leftWheel.x)+"  "+str(leftWheel.y)+"  "+str(rightWheel.x)+"  "+str(rightWheel.y))

                            #print("distance: ", distance)
                            #print("waypoint: ", self.current_waypoint)
                            #print("endpoint: ", self.endPoint)

                            th = self.heading.data
                            delX= self.current_waypoint[0]-self.endPoint.x
                            delY= self.current_waypoint[1]-self.endPoint.y
                            # arr_delX.append(delX)
                            # arr_delY.append(delY)

                            # D(k) should add
                            #####################################################
                            a = delX*sin(th) - delY*cos(th)
                            b = delX*cos(th) + delY*sin(th)
                            c = b - self.D
                            if arr_delOmega:
                                self.D = self.calculate_optimal_D(delX, delY, th, self.D, arr_delOmega[-1][0], arr_delOmega[-1][1])
                                # self.D = 0.25
                            else:
                                print("initial self.D is " + str(self.D))
                                # self.D = 0.233
                            arr_D.append(self.D)
                            #####################################################
                            # delOmega= asin((delX*sin(th)-delY*cos(th))/(self.D))
                            # delS= self.D*cos(delOmega)-self.D+delX*cos(th)+delY*sin(th)
                            # delOmega1= (1/self.R)*(delS+self.L*delOmega) * 1.0
                            # delOmega2= (1/self.R)*(delS-self.L*delOmega) * 1.0
                            delOmega = asin(a/self.D)
                            delS = self.D*cos(delOmega) + c
                            delOmega1 = (1/self.R)*(delS + self.L*delOmega)
                            delOmega2 = (1/self.R)*(delS - self.L*delOmega)

                            if not arr_delOmega:
                                arr_delOmega.append([delOmega1, delOmega2])
                                print("initial delOmega is " + str(delOmega1) + " and " + str(delOmega2))
                            else:
                                deldelOmega1 = delOmega1 - arr_delOmega[-1][0]
                                deldelOmega2 = delOmega2 - arr_delOmega[-1][1]
                                arr_deldelOmega.append([deldelOmega1, deldelOmega2])
                                arr_deldelOmega_error.append(sqrt(pow(deldelOmega1,2) + pow(deldelOmega2,2)))
                                arr_delOmega.append([delOmega1, delOmega2])

                            #Motor Smoothing Buffer Disabled
                            pubDelta1 = delOmega1
                            pubDelta2 = delOmega2
                            # self.pub_delta_theta_1.publish(pubDelta1)
                            # self.pub_delta_theta_2.publish(pubDelta2)
                            # delXrobotLocal, delYrobotLocal = self.calculate_robot_local_delta_from_omega(delOmega1, delOmega2)
                            # delXrobotGlobal, delYrobotGlobal=np.matmul([[cos(self.heading.data), -sin(self.heading.data)],[sin(self.heading.data), cos(self.heading.data)]], [delXrobotLocal, delYrobotLocal])
                            delXrobotGlobal = self.R*(delOmega1 + delOmega2)*cos(self.heading.data)/2
                            delYrobotGlobal = self.R*(delOmega1 + delOmega2)*sin(self.heading.data)/2
                            # position update
                            self.point.x=self.point.x+delXrobotGlobal
                            self.point.y=self.point.y+delYrobotGlobal
                            self.heading.data=self.heading.data+self.R*(delOmega1-delOmega2)/(2*self.L)
                            self.pen_distance_per_loop=sqrt(
                                pow(delXrobotGlobal, 2) +
                                pow(delYrobotGlobal, 2)
                                )
                            # print(str(delOmega1)+"  "+str(delOmega2)+"  "+str(self.pen_distance_per_loop))


                            # self.plot_arr(arr_robotPoint, 'r')
                            # self.plot_arr(arr_endPoint, 'g')
                            # self.plot_arr(arr_leftWheel, 'b')
                            # self.plot_arr(arr_rightWheel, 'k')

                            # plt.axis([-0.5, 4.5, -0.5, 4.5])
                            # plt.draw()
                            # plt.pause(0.00000001)

                            self.r_sim.sleep()

                            break

                        except KeyboardInterrupt:
                            print("Got KeyboardInterrupt")
                            # self.cmd_vel.publish(Twist())

                            rospy.signal_shutdown("KeyboardInterrupt")
                            break

                    self.waypoint_index_in_current_segment = self.waypoint_index_in_current_segment + 1
                self.segment_index = self.segment_index + 1
                self.waypoint_index_in_current_segment = 0

            #End of current letter.

            #it's time for next letter
            self.letter_index = self.letter_index + 1
            self.segment_index = 0
            self.waypoint_index_in_current_segment = 0


        # plt.plot(item[0] for item in arr_robotPoint, item[1] for item in arr_robotPoint, option)
        # self.plot_arr(arr_robotPoint, 'r')
        # self.plot_arr(arr_endPoint, 'g')
        # self.plot_arr(arr_leftWheel, 'b')
        # self.plot_arr(arr_rightWheel, 'k')

        # plt.axis([-0.5, 6.0, -0.5, 6.0])
        # plt.savefig(home_path+"/simulation_"+self.program_start_time+".png")

        # hskim added
        # print(self.D)

        for i in arr_deldelOmega_error:
            if i > 1:
                print(arr_deldelOmega_error.index(i))
                print(i)

        # print("max = " + str(max(arr_deldelOmega_error)))
        # print("avg = " + str(sum(arr_deldelOmega_error) / len(arr_deldelOmega_error)))
        # print("arr_delOmega[0] = " + str(arr_delOmega[0]))
        # print("arr_delOmega[1] = " + str(arr_delOmega[1]))
        # print("arr_delOmega[2] = " + str(arr_delOmega[2]))
        # print("arr_deldelOmega_error[0] = " + str(arr_deldelOmega_error[0]))
        # print("arr_deldelOmega_error[1] = " + str(arr_deldelOmega_error[1]))
        # print("arr_deldelOmega_error[2] = " + str(arr_deldelOmega_error[2]))
        # print("arr_D[0] = " + str(arr_D[0]))
        # print("arr_D[1] = " + str(arr_D[1]))
        # print("arr_D[2] = " + str(arr_D[2]))
        # print("arr_delX[0] = " + str(arr_delX[0]))
        # print("arr_delY[0] = " + str(arr_delY[0]))
        # print("arr_delX[1] = " + str(arr_delX[1]))
        # print("arr_delY[1] = " + str(arr_delY[1]))
        # print("arr_delX[2] = " + str(arr_delX[2]))
        # print("arr_delY[2] = " + str(arr_delY[2]))

        for i in range(len(arr_D)):
            if i != len(arr_D)-1:
                arr_delD.append(arr_D[i+1] - arr_D[i])

        # plt.plot([a[0] for a in arr_deldelOmega], 'r')
        # plt.plot([a[1] for a in arr_deldelOmega], 'b')
        # plt.axis([0, 6000, -0.1, 0.1])
        # plt.plot(arr_deldelOmega_error, 'r')
        # plt.plot(arr_D, "b")
        print("max arr_delD = " + str(max(arr_delD)))
        print("min arr_delD = " + str(min(arr_delD)))
        plt.plot(arr_delD, "k")
        # plt.axis([0, 6000, -0.1, 0.4])
        plt.axis([0, 6000, -0.1, 0.1])

        # self.plot_arr([a[0] for a in arr_deldelOmega], 'r')
        # self.plot_arr([a[1] for a in arr_deldelOmega], 'b')
        plt.show()

        self.shutdown_for_seconds(2.0)
        rospy.loginfo("Stopping the robot at the final destination")
        # print("Total Stiff Delta_Theta Change BUFFER = %d" % (cnt_delta_buffer))
        #Wait for 1 second to close valve
        # self.quit_valve()

    def robotNavigationLoop(self, current_destination):
        if rospy.is_shutdown():
            print("rospy.is_shutdown()")
            self.shutdown_for_seconds(2.0)
            sys.exit("Terminate Program after shutdown everything for 2.0 sec")
        try:
            if self.intensity_option==1:
                if not self.isStartEndIndexed:
                    #For this option, is_moving_between_segments does not work!!!!
                    input_pixel_value = int(self.arr_intensity[self.cnt_waypoints])
                    input_pixel_value_graphic = int(self.arr_intensity[self.cnt_waypoints])
                    if input_pixel_value >=0 and input_pixel_value<256:   #if the input is alright, then

                        #cut off value larger than 230 to 230.
                        input_pixel_value = 230 if input_pixel_value>230 else input_pixel_value
                        spray_input = 660.0+(1024.0-660.0)*(float(input_pixel_value)/230.0)
                        # self.spray_intensity_publisher.publish(spray_input)
                        self.valve_angle_input.goal_position = int(spray_input)
                        self.valve_angle_publisher.publish(self.valve_angle_input)
                else:
                    if self.is_moving_between_segments==True:
                        # self.spray_intensity_publisher.publish(1024.0)
                        input_pixel_value_graphic = 255
                        self.valve_angle_input.goal_position = 1024
                        self.valve_angle_publisher.publish(self.valve_angle_input)
                    else:
                        #self.spray_intensity_publisher.publish(660.0)
                        # self.spray_intensity_publisher.publish(740.0)
                        input_pixel_value_graphic = 0
                        self.valve_angle_input.goal_position = 660
                        self.valve_angle_publisher.publish(self.valve_angle_input)


            elif self.intensity_option==2:
                input_pixel_value_graphic=0
                self.valve_angle_input.goal_position = 740
                self.valve_angle_publisher.publish(self.valve_angle_input)

            self.endPoint.x=self.point.x-self.D*cos(self.heading.data)
            self.endPoint.y=self.point.y-self.D*sin(self.heading.data)
            self.endPoint.z=float(input_pixel_value_graphic)
            self.point.z=self.heading.data
            self.pub_midpoint.publish(self.point)
            self.pub_endpoint.publish(self.endPoint)
            msg_time = Time()
            msg_time.data = rospy.Time.now()
            self.pub_midpoint_time.publish(msg_time)

            #print(str(self.cnt_waypoints)+"  "+str(self.endPoint.x)+"  "+str(self.endPoint.y))
            print(str(self.endPoint.x)+"  "+str(self.endPoint.y))
            # print(str(self.point.x)+"  "+str(self.point.y)+"  "+str(self.heading.data*180.0/pi))

            #print("distance: ", distance)
            #print("waypoint: ", current_destination)
            #print("endpoint: ", self.endPoint)

            th = self.heading.data
            delX= current_destination[0]-self.endPoint.x
            delY= current_destination[1]-self.endPoint.y

            delOmega= asin((delX*sin(th)-delY*cos(th))/(self.D))
            delS= self.D*cos(delOmega)-self.D+delX*cos(th)+delY*sin(th)

            # delOmega1= (1/self.R)*(delS+2*self.L*delOmega) * 0.75
            # delOmega2= (1/self.R)*(delS-2*self.L*delOmega) * 0.75
            delOmega1= (1/self.R)*(delS+self.L*delOmega)
            delOmega2= (1/self.R)*(delS-self.L*delOmega)

            if self.motor_buffer_option == 1:       #Motor Smoothing Buffer Enabled
                pass
                '''
                if abs(delOmega1 - pubDelta1) >= 0.01 and abs(delOmega2 - pubDelta2) >= 0.01:
                    pubIter = max(floor(abs(delOmega1 - pubDelta1)/0.01), floor(abs(delOmega2 - pubDelta2)/0.01))
                    cnt_delta_buffer += pubIter
                    # print("---------ITERATION(0/%d)--------- " % (pubIter))
                elif abs(delOmega1 - pubDelta1) >= 0.01:
                    pubIter = floor(abs(delOmega1 - pubDelta1)/0.01)
                    # print("---------ITERATION(0/%d)--------- " % (pubIter))
                    cnt_delta_buffer += pubIter
                elif abs(delOmega2 - pubDelta2) >= 0.01:
                    pubIter = floor(abs(delOmega2 - pubDelta2)/0.01)
                    # print("---------ITERATION(0/%d)--------- " % (pubIter))
                    cnt_delta_buffer += pubIter
                else:
                    pubIter = 1

                for iteration in range(int(pubIter)):
                    control_input_1 = pubDelta1 + (float)(delOmega1-pubDelta1)/pubIter*(iteration+1)
                    control_input_2 = pubDelta2 + (float)(delOmega2-pubDelta2)/pubIter*(iteration+1)
                    self.pub_delta_theta_1.publish(control_input_1)
                    self.pub_delta_theta_2.publish(control_input_2)
                    if pubIter != 1:
                        # print("---------ITERATION(%d/%d)--------- " % (iteration+1,pubIter))
                        pass


                    self.r.sleep()

                pubDelta1 = delOmega1
                pubDelta2 = delOmega2

                delXrobotLocal, delYrobotLocal = self.calculate_robot_local_delta_from_omega(delOmega1, delOmega2)
                delXrobotGlobal, delYrobotGlobal = np.matmul([[cos(self.heading.data), -sin(self.heading.data)],[sin(self.heading.data), cos(self.heading.data)]], [delXrobotLocal, delYrobotLocal])
                self.point.x=self.point.x+delXrobotGlobal
                self.point.y=self.point.y+delYrobotGlobal
                self.heading.data=self.heading.data+self.R*(delOmega1-delOmega2)/(2*self.L)
                self.pen_distance_per_loop=sqrt(
                    pow(delXrobotGlobal, 2) +
                    pow(delYrobotGlobal, 2)
                    )
                print(str(delOmega1)+"  "+str(delOmega2)+"  "+str(self.pen_distance_per_loop))


                break
                '''

            elif self.motor_buffer_option == 2:
                pass
                '''
                if self.next_letter_index != -1:  # if current waypoint is not the end
                    if abs(delOmega1 - pubDelta1) >= 0.01 and abs(delOmega2 - pubDelta2) >= 0.01:
                        pubIter = max(floor(abs(delOmega1 - pubDelta1)/0.01), floor(abs(delOmega2 - pubDelta2)/0.01))
                        cnt_delta_buffer += pubIter
                        # print("---------ITERATION(0/%d)--------- " % (pubIter))
                    elif abs(delOmega1 - pubDelta1) >= 0.01:
                        pubIter = floor(abs(delOmega1 - pubDelta1)/0.01)
                        # print("---------ITERATION(0/%d)--------- " % (pubIter))
                        cnt_delta_buffer += pubIter
                    elif abs(delOmega2 - pubDelta2) >= 0.01:
                        pubIter = floor(abs(delOmega2 - pubDelta2)/0.01)
                        # print("---------ITERATION(0/%d)--------- " % (pubIter))
                        cnt_delta_buffer += pubIter
                    else:
                        pubDelta1 = delOmega1
                        pubDelta2 = delOmega2
                        self.pub_delta_theta_1.publish(pubDelta1)
                        self.pub_delta_theta_2.publish(pubDelta2)

                        delXrobotLocal, delYrobotLocal = self.calculate_robot_local_delta_from_omega(delOmega1, delOmega2)
                        delXrobotGlobal, delYrobotGlobal=np.matmul([[cos(self.heading.data), -sin(self.heading.data)],[sin(self.heading.data), cos(self.heading.data)]], [delXrobotLocal, delYrobotLocal])
                        self.point.x=self.point.x+delXrobotGlobal
                        self.point.y=self.point.y+delYrobotGlobal
                        self.heading.data=self.heading.data+self.R*(delOmega1-delOmega2)/(2*self.L)
                        self.pen_distance_per_loop=sqrt(
                                pow(delXrobotGlobal, 2) +
                                pow(delYrobotGlobal, 2)
                                )
                        print(str(pubDelta1)+"  "+str(pubDelta2)+"  "+str(self.pen_distance_per_loop))

                        self.r.sleep()

                        break

                    delX_original= self.current_waypoint[0]-self.endPoint.x
                    delY_original= self.current_waypoint[1]-self.endPoint.y
                    delX=delX_original/pubIter
                    delY=delY_original/pubIter
                    for iteration in range(int(pubIter)):
                        self.endPoint.x=self.point.x-self.D*cos(self.heading.data)
                        self.endPoint.y=self.point.y-self.D*sin(self.heading.data)

                        self.next_waypoint = [
                                self.arr_path[self.next_letter_index][self.next_segment_index][
                                    self.next_waypoint_index_in_current_segment][0],
                                self.arr_path[self.next_letter_index][self.next_segment_index][
                                    self.next_waypoint_index_in_current_segment][1]
                                ]
                        # I realized that next_waypoint is freaking unnecessary FUCK

                        th = self.heading.data


                        delOmega= asin((delX*sin(th)-delY*cos(th))/(self.D))
                        delS= self.D*cos(delOmega)-self.D+delX*cos(th)+delY*sin(th)

                        delOmega1= (1/self.R)*(delS+2*self.L*delOmega) * 0.75
                        delOmega2= (1/self.R)*(delS-2*self.L*delOmega) * 0.75

                        self.pub_delta_theta_1.publish(delOmega1)
                        self.pub_delta_theta_2.publish(delOmega2)

                        # if pubIter != 1:
                        #     print("THIS SHOULD NOT HAPPEN :) IF THIS SHOWS UP, YOU'RE FUCKED UP")

                        delXrobotLocal, delYrobotLocal = self.calculate_robot_local_delta_from_omega(delOmega1, delOmega2)
                        delXrobotGlobal, delYrobotGlobal = np.matmul([[cos(self.heading.data), -sin(self.heading.data)],[sin(self.heading.data), cos(self.heading.data)]], [delXrobotLocal, delYrobotLocal])
                        self.point.x=self.point.x+delXrobotGlobal
                        self.point.y=self.point.y+delYrobotGlobal
                        self.heading.data=self.heading.data+self.R*(delOmega1-delOmega2)/(2*self.L)
                        self.pen_distance_per_loop=sqrt(
                            pow(delXrobotGlobal, 2) +
                            pow(delYrobotGlobal, 2)
                            )
                        print(str(delOmega1)+"  "+str(delOmega1)+"  "+str(self.pen_distance_per_loop))
                        pubDelta1 = delOmega1
                        pubDelta2 = delOmega2
                        self.r.sleep()

                    break

                #if current waypoint is the end
                else:
                    pubDelta1 = delOmega1
                    pubDelta2 = delOmega2
                    self.pub_delta_theta_1.publish(pubDelta1)
                    self.pub_delta_theta_2.publish(pubDelta2)

                    delXrobotLocal, delYrobotLocal = self.calculate_robot_local_delta_from_omega(delOmega1, delOmega2)
                    delXrobotGlobal, delYrobotGlobal=np.matmul([[cos(self.heading.data), -sin(self.heading.data)],[sin(self.heading.data), cos(self.heading.data)]], [delXrobotLocal, delYrobotLocal])
                    self.point.x=self.point.x+delXrobotGlobal
                    self.point.y=self.point.y+delYrobotGlobal
                    self.heading.data=self.heading.data+self.R*(delOmega1-delOmega2)/(2*self.L)
                    self.pen_distance_per_loop=sqrt(
                            pow(delXrobotGlobal, 2) +
                            pow(delYrobotGlobal, 2)
                            )
                    print(str(pubDelta1)+"  "+str(pubDelta2)+"  "+str(self.pen_distance_per_loop))

                    self.r.sleep()

                    break
                '''

            #Motor Smoothing Buffer Disabled
            elif self.motor_buffer_option == 3:
                pubDelta1 = delOmega1
                pubDelta2 = delOmega2
                self.pub_delta_theta_1.publish(pubDelta1)
                self.pub_delta_theta_2.publish(pubDelta2)
                delXrobotLocal, delYrobotLocal = self.calculate_robot_local_delta_from_omega(pubDelta1, pubDelta2)
                delXrobotGlobal, delYrobotGlobal=np.matmul([[cos(self.heading.data), -sin(self.heading.data)],[sin(self.heading.data), cos(self.heading.data)]], [delXrobotLocal, delYrobotLocal])
                self.point.x=self.point.x+delXrobotGlobal
                self.point.y=self.point.y+delYrobotGlobal
                self.heading.data=self.heading.data+self.R*(delOmega1-delOmega2)/(2*self.L)
                self.pen_distance_per_loop=sqrt(
                    pow(delXrobotGlobal, 2) +
                    pow(delYrobotGlobal, 2)
                    )
                #print(str(delOmega1)+"  "+str(delOmega2)+"  "+str(self.pen_distance_per_loop))

                self.r.sleep()

        except KeyboardInterrupt:
            print("Got KeyboardInterrupt")
            # self.cmd_vel.publish(Twist())

            rospy.signal_shutdown("KeyboardInterrupt")


    # def calculate_optimal_D(self, _delX, _delY, _th, _prev_D, _prev_delOmega1, _prev_delOmega2):
    #     arr_error = []
    #     _a = _delX*sin(_th) - _delY*cos(_th)
    #     _b = _delX*cos(_th) + _delY*sin(_th)
    #     for i in range(MOTOR_RESOLUTION):
    #         _D = 0.2 + i*0.1/MOTOR_RESOLUTION
    #         # _D = 0.2 + i*0.5/MOTOR_RESOLUTION
    #         # if exception needed
    #         _delOmega = asin(_a/_D)
    #         _delS = sqrt(_D*_D-_a*_a) - _prev_D + _b
    #         _delOmega1 = (1/self.R)*(_delS + self.L*_delOmega)
    #         _delOmega2 = (1/self.R)*(_delS - self.L*_delOmega)
    #         _error = pow(_delOmega1 - _prev_delOmega1, 2) + pow(_delOmega2 - _prev_delOmega2, 2)
    #         arr_error.append(_error)
    #     return 0.2 + arr_error.index(min(arr_error))*0.1/MOTOR_RESOLUTION
    #     # return 0.2 + arr_error.index(min(arr_error))*0.5/MOTOR_RESOLUTION

    def derivative_cost_function(self, x, _a, _b, _c, _e, _f):
        value1 = ((1/self.R)*(sqrt(pow(x,2) - pow(_a,2)) + _c + self.L*asin(_a/x)) - _e)*(1/self.R)*(pow(x,2)-self.L*_a)/(x*sqrt(pow(x,2) - pow(_a,2)))
        value2 = ((1/self.R)*(sqrt(pow(x,2) - pow(_a,2)) + _c - self.L*asin(_a/x)) - _f)*(1/self.R)*(pow(x,2)+self.L*_a)/(x*sqrt(pow(x,2) - pow(_a,2)))
        value = value1 + value2
        return value

    def cost_function(self, x, _a, _b, _c, _e, _f):
        _delOmega = asin(_a/x)
        _delS = sqrt(x*x-_a*_a) + _c
        _delOmega1 = (1/self.R)*(_delS + self.L*_delOmega)
        _delOmega2 = (1/self.R)*(_delS - self.L*_delOmega)
        value = pow(_delOmega1 - _e, 2) + pow(_delOmega2 - _f, 2)
        return value

    def calculate_optimal_D(self, _delX, _delY, _th, _prev_D, _prev_delOmega1, _prev_delOmega2):
        arr_error = []
        _a = _delX*sin(_th) - _delY*cos(_th)
        _b = _delX*cos(_th) + _delY*sin(_th)
        x1 = 0.2
        x2 = 0.3
        x3 = 0
        error = 0
        y1 = self.derivative_cost_function(x1, _a, _b, _b-_prev_D, _prev_delOmega1, _prev_delOmega2)
        y2 = self.derivative_cost_function(x2, _a, _b, _b-_prev_D, _prev_delOmega1, _prev_delOmega2)
        for i in range(100):
            x3 = x2 - y2*(x2-x1)/(y2-y1)
            error = abs(x3-x2)
            if error < 0.001:
                break
            x1 = x2
            x2 = x3
            y1 = self.derivative_cost_function(x1, _a, _b, _b-_prev_D, _prev_delOmega1, _prev_delOmega2)
            y2 = self.derivative_cost_function(x2, _a, _b, _b-_prev_D, _prev_delOmega1, _prev_delOmega2)
        f_x3 = self.cost_function(x3, _a, _b, _b-_prev_D, _prev_delOmega1, _prev_delOmega2)
        f_low = self.cost_function(0.2, _a, _b, _b-_prev_D, _prev_delOmega1, _prev_delOmega2)
        f_high = self.cost_function(0.3, _a, _b, _b-_prev_D, _prev_delOmega1, _prev_delOmega2)
        if f_x3 <= f_low and f_x3 <= f_high:
            if x3 >= 0.2 and x3 <= 0.3:
                return x3
            else:
                if f_low <= f_high:
                    return 0.2
                else:
                    return 0.3
        else:
            if f_low <= f_high:
                return 0.2
            else:
                return 0.3

    def control_motors(self, _loop_cnt, delta1, delta2):

        loop_cnt = 0
        while True:
            if rospy.is_shutdown():
                break
            try:
                if loop_cnt>=_loop_cnt:
                    self.pub_delta_theta_1.publish(0.0)
                    self.pub_delta_theta_2.publish(0.0)
                    self.r.sleep()
                    break
                self.pub_delta_theta_1.publish(delta1)
                self.pub_delta_theta_2.publish(delta2)
                self.r.sleep()
                loop_cnt = loop_cnt+1
            except KeyboardInterrupt:
                print("Got KeyboardInterrupt")
                # self.cmd_vel.publish(Twist())

                rospy.signal_shutdown("KeyboardInterrupt")
                break

    def go_to_point_and_come_back(self, _x, _y, _theta_deg):
        self.pub_delta_theta_1.publish(0.0)
        self.pub_delta_theta_2.publish(0.0)
        ### initial point (self.point.x, self.point.y, self.heading.data) to goal point (_x, _y, _theta)
        _theta = _theta_deg*pi/180.0

        theta_profile = []

        ###Step 1. Rotate and Align heading to goal
        heading_to_goal = atan2((_y-self.point.y)/(_x-self.point.x))
        goal_diff_angle = angle_difference(heading_to_goal,self.heading.data)
        self.control_motors_by_direct_input([self.L*3.141592*goal_diff_angle/(self.R*0.04), -0.04, +0.04]) #Turn left

        ###Step 2. Go straight to goal
        distance = sqrt(pow(self.point.x-_x,2)+pow(self.point.y-_y,2))
        self.control_motors_by_direct_input([distance/0.04*self.freq, +0.04, +0.04])

        ###Step 3. Rotate to cam view as _theta
        cam_diff_angle = angle_difference(_theta, heading_to_goal)
        self.control_motors_by_direct_input([self.L*3.141592*cam_diff_angle/(self.R*0.04), -0.04, +0.04]) #Turn left

        ###########################
        self.save_topview_image()
        ###########################

        ###Step 4. Rotate and Align heading to initial point
        heading_to_initial = atan2((self.point.y-_y)/(self.point.x-_x))
        initial_diff_angle = angle_difference(heading_to_initial, _theta)
        self.control_motors_by_direct_input([self.L*3.141592*initial_diff_angle/(self.R*0.04), -0.04, +0.04]) #Turn left

        ###Step 5. Go straight to initial point
        self.control_motors_by_direct_input([distance/0.04*self.freq, +0.04, +0.04])

        ###Step 6. Rotate to initial heading self.heading.data
        reset_diff_angle = angle_difference(self.heading.data, heading_to_initial)
        self.control_motors_by_direct_input([self.L*3.141592*initial_diff_angle/(self.R*0.04), -0.04, +0.04]) #Turn left

    def save_topview_image(self):
        pass

    def plot_arr(self, arr, option):
        arr_1 = list(-item[0] for item in arr)
        arr_2 = list(item[1] for item in arr)
        plt.plot(arr_1, arr_2, option)

    def visualize_traj(self):
        self.pub_markers.publish(self.point)
        self.pub_markers_painting.publish(self.endPoint)

    def visualize_traj_encoder(self, _data):
        self.traj_encoder.points.append(Point(_data.x, _data.y, 0.0))
        self.pub_markers_encoder.publish(self.traj_encoder)

    def calculate_robot_local_delta_from_omega(self, del1, del2):
        delX = 0.0
        delY = 0.0
        if del1 != del2:
            delY=-self.L*(del1+del2)/(del1-del2)*(1-cos(self.R*(del2-del1)/(2*self.L)))
            delX=-self.L*(del1+del2)/(del1-del2)*sin(self.R*(del2-del1)/(2*self.L))
        else:
            delX=self.R*del1
            delY=0
        return delX, delY

    def shutdown_everything(self):
        # self.cmd_vel.publish(Twist())
        self.pub_delta_theta_1.publish(0.0)
        self.pub_delta_theta_2.publish(0.0)
        # self.spray_intensity_publisher.publish(1024.0)
        self.valve_angle_input.goal_position = 1024
        self.valve_angle_publisher.publish(self.valve_angle_input)
        self.r.sleep()

    def shutdown_for_seconds(self, _input):
        cnt_loop = (int)(_input / self.dt)
        for i in range(cnt_loop):
            self.shutdown_everything()

    def callback_vision_offset(self, _data):
        print("Callback Vision OFFSET Received")

        #offset_accpeted callback can be accepted, only when there is no accepted offset handling right now.
        if self.offset_accepted == False:
            self.offset_accepted = True
            self.offset_x = _data.x
            self.offset_y = _data.y
            self.offset_theta = _data.z

            # self.point.x=self.point.x + offset_x
            # self.point.y=self.point.y + offset_y
            # self.heading.data = self.heading.data + offset_theta
