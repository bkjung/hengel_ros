#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion, PoseStamped
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from hengel_navigation.msg import ValveInput, OperationMode
import tf
from math import radians, copysign, sqrt, pow, pi, atan2, sin, floor, cos, asin
from tf.transformations import euler_from_quaternion
import numpy as np
import sys
#from PIL import Image
#import PIL.Image
import time
import os
import cv2
import cv_bridge
#from real_globalmap import RealGlobalMap
#from pi_cam_manager import PiCamManager
#from crosspoint_docking import CrosspointDocking
import logging

#2480 is too large, so that it hits the ground and the valve_control while loop does not end.
#MARKER_DOWN = 2480
#MARKER_DOWN = 2460
#MARKER_DOWN = 2430

#Due to wheel height change
#MARKER_DOWN = 3790
#MARKER_DOWN = 3700
MARKER_DOWN = 3900
#MARKER_UP = 3200
MARKER_UP = 3270

scale_factor = 3  #[pixel/cm]
robot_size = 15  #[cm]; diameter

package_base_path = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "../.."))
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
    # def __init__(self, _arr_path, _draw_start_index):
    #     self.arr_path = _arr_path
    #     self.draw_start_index  = _draw_start_index

    #     self.initial_setting()
    #     self.run()

    def __init__(self, _arr_path, _docking_point_list, _center_point_list, _isPositionControl,_D):
        word = raw_input(
                    "There are two options for motor profile change smoothing buffer.\n[1] Enable Buffer.\n[2] Disable Buffer. \nType 1 or 2 :"
                    )
            self.motor_buffer_option = int(word)
            if self.motor_buffer_option == 1 or self.motor_buffer_option ==2 :
                break

        self.isPositionControl = _isPositionControl
        self.arr_path = _arr_path
        self.D=_D
        self.docking_point_list = _docking_point_list
        self.center_point_list = _center_point_list
        self.initial_setting()

        if self.isPositionControl:
            self.runOffset()
        else:
            self.run()

    def initial_setting(self):
        self.program_start_time = time.strftime("%y%m%d_%H%M%S")
        #logging.basicConfig(filename='~/Dropbox/intern_share/experiment_data/Global_Alignment/log/'+self.program_start_time+'.txt', level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')
        #logging.basicConfig(filename='~/Dropbox/intern_share/experiment_data/Global_Alignment/log/'+self.program_start_time+'.txt', level=logging.DEBUG)
        #logging.debug('----Initial Center Point----')
        #logging.debug(self.center_point_list)
        print('----Initial Center Point----')
        print(self.center_point_list)

        #self.R = 0.115/2 #radius of wheel
        #self.L = 0.33/2 #half of distance btw two wheels

        #test flag bot
        #self.R = 0.11/2 #radius of wheel
        self.R = 0.12475/2 #radius of wheel
        self.L = 0.3537/2 #half of distance btw two wheels

        self.point = Point()
        self.point_encoder = Point()
        self.endPoint= Point()
        self.heading = Float32()
        self.heading_encoder = Float32()
        self.move_cmd = Twist()

        self.valve_operation_mode = OperationMode()
        self.valve_operation_mode.mode = 1
        self.valve_angle_input = ValveInput()

        self.valve_status = MARKER_UP

        self.dt = 0.02  # [s]
        self.r = rospy.Rate(1.0/self.dt)

        #It SHOULD BE 1 for current code.
        #If it's not 1, then self.check_whether_moving_to_next_start() should be modified
        #self.waypoint_increment = 1

        self.letter_index = 0
        self.segment_index = 0
        self.waypoint_index_in_current_segment = 0
        self.waypoints = []
        self.current_waypoint = []
        self.cnt_letter = 0
        self.cnt_total_waypoints = 0
        self.cnt_segments_in_current_letter = 0
        self.cnt_waypoints_in_current_segment = 0

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

        self.is_moving_between_letters = False

        self.loop_cnt_pathmap = 0

        self.map_img = []
        self.map_img = np.ndarray(self.map_img)

        #rospy.init_node('hengel_navigation_control', anonymous=False, disable_signals=True)
        rospy.on_shutdown(self.shutdown)

        # self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        self.valve_angle_publisher = rospy.Publisher(
                '/valve_input', ValveInput, queue_size=5)
        self.valve_operation_mode_publisher = rospy.Publisher(
                '/operation_mode', OperationMode, queue_size=5)
        self.valve_operation_mode_publisher.publish(
                self.valve_operation_mode)
        self.valve_angle_input.goal_position = self.valve_status
        self.valve_angle_publisher.publish(
                self.valve_angle_input)

        #self.crop_map_publisher = rospy.Publisher('/cropped_predict_map', PIL.Image, queue_size=5)
        self.offset_change_x_publisher = rospy.Publisher(
                '/offset_change_x', Float32, queue_size=5)
        self.offset_change_y_publisher = rospy.Publisher(
                '/offset_change_y', Float32, queue_size=5)
        self.offset_change_theta_publisher = rospy.Publisher(
                '/offset_change_theta', Float32, queue_size=5)


        # Stop Subscribing position & heading data

        # self.position_subscriber = rospy.Subscriber('/current_position', Point,
                # self.callback_position)
        # self.heading_subscriber = rospy.Subscriber('/current_heading', Float32,
                # self.callback_heading)

        self.pub_markers = rospy.Publisher('/robot_trajectory_visualization', Marker, queue_size=5)
        self.pub_markers_painting = rospy.Publisher('/painting_visualization', Marker, queue_size=5)
        self.pub_markers_encoder = rospy.Publisher('/robot_trajectory_encoder_visualization', Marker, queue_size=5)

        self.pub_delta_theta_1 = rospy.Publisher('/delta_theta_1', Float32, queue_size=5)
        self.pub_delta_theta_2 = rospy.Publisher('/delta_theta_2', Float32, queue_size=5)

        self.pub_distance = rospy.Publisher('/distance', Float32, queue_size=5)
        self.pub_endpoint = rospy.Publisher('/endpoint', Point, queue_size=5)

        print("Initialize Done")



    def runOffset(self):
        print("--runOffset begin--")
        self.wait_for_seconds(5.0)
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

                    if self.waypoint_index_in_current_segment == 0:
                        #print("moving to FIRST waypoint")
                        #rospy.loginfo("moving to FIRST waypoint in segment")
                        self.is_moving_between_letters = True
                    elif self.global_option==1 and self.segment_index == self.cnt_segments_in_current_letter - 1:
                        #print("moving to GLOBAL VIEW POINT")
                        #rospy.loginfo("moving to GLOBAL VIEW POINT")
                        self.is_moving_between_letters = True
                    else:
                        self.is_moving_between_letters = False



                    docking_buffer_cnt = 0

                    # pubDelta1=0
                    # pubDelta2=0
                    # pubIter=0

                    # Motion Control
                    while True:
                        if rospy.is_shutdown():
                            break
                        try:
                            self.endPoint.x=self.point.x-self.D*cos(self.heading.data)
                            self.endPoint.y=self.point.y-self.D*sin(self.heading.data)
                            #self.endPoint.x=self.point.x+self.D*cos(self.heading.data)
                            #self.endPoint.y=self.point.y+self.D*sin(self.heading.data)

                            distance = sqrt(
                                    pow(self.current_waypoint[0] - self.endPoint.x, 2) +
                                    pow(self.current_waypoint[1] - self.endPoint.y, 2))

                            #print("distance: ", distance)
                            #print("waypoint: ", self.current_waypoint)
                            #print("endpoint: ", self.endPoint)
                            self.pub_distance.publish(distance)
                            self.pub_endpoint.publish(self.endPoint)

                            # if distance < 0.01:
                            # if distance < 0.002:
                            #     break

                            #self.valve_status = MARKER_DOWN
                            th = self.heading.data
                            delX= self.current_waypoint[0]-self.endPoint.x
                            delY= self.current_waypoint[1]-self.endPoint.y

                            # print("delx: "+str(delX)+", dely: "+str(delY))

                            delOmega= asin((delX*sin(th)-delY*cos(th))/(self.D))
                            delS= self.D*cos(delOmega)-self.D+delX*cos(th)+delY*sin(th)

                            #delOmega1= (1/self.R)*(delS+2*self.L*delOmega) * 0.6
                            #delOmega2= (1/self.R)*(delS-2*self.L*delOmega) * 0.6
                            delOmega1= (1/self.R)*(delS+2*self.L*delOmega) * 0.75
                            delOmega2= (1/self.R)*(delS-2*self.L*delOmega) * 0.75
                            #delOmega1= (1/self.R)*(delS+2*self.L*delOmega) * 1.0
                            #delOmega2= (1/self.R)*(delS-2*self.L*delOmega) * 1.0

                            #print("delOmega=%f" % (delOmega))
                            #print("delOmega1=%f" % (delOmega1))
                            #print("delOmega2=%f" % (delOmega2))


                            #if abs(delOmega1)>0.04 or abs(delOmega2)>0.04:
                            #    if abs(delOmega1)>abs(delOmega2): #abs(delOmega1) should not be zero, according to this inequality
                            #        delOmega2=copysign(0.04*delOmega2/delOmega1, delOmega2)
                            #        delOmega1=copysign(0.04, delOmega1)
                            #    else:   #abs(delOmega2) should not be zero, according to this inequality
                            #        delOmega1=copysign(delOmega1/delOmega2, delOmega1)
                            #        delOmega2=copysign(0.04, delOmega2)



                            if delOmega1 != delOmega2:
                                delYrobotLocal=-self.L*(delOmega1+delOmega2)/(delOmega1-delOmega2)*(1-cos(self.R*(delOmega2-delOmega1)/(2*self.L)))
                                delXrobotLocal=-self.L*(delOmega1+delOmega2)/(delOmega1-delOmega2)*sin(self.R*(delOmega2-delOmega1)/(2*self.L))
                            else:
                                delXrobotLocal=self.R*delOmega1
                                delYrobotLocal=0

                            delXrobotGlobal, delYrobotGlobal=np.matmul([[cos(self.heading.data), -sin(self.heading.data)],[sin(self.heading.data), cos(self.heading.data)]], [delXrobotLocal, delYrobotLocal])
                            self.point.x=self.point.x+delXrobotGlobal
                            self.point.y=self.point.y+delYrobotGlobal
                            self.heading.data=self.heading.data+self.R*(delOmega1-delOmega2)/(2*self.L)


                            if self.motor_buffer_option == 1:
                                if abs(delOmega1 - pubDelta1) > 0.01 and abs(delOmega2 - pubDelta2) > 0.01:
                                    pubIter = max(floor(abs(delOmega1 - pubDelta1)/0.01), floor(abs(delOmega2 - pubDelta2)/0.01))
                                    cnt_delta_buffer += pubIter
                                    print("---------ITERATION(0/%d)--------- " % (pubIter))
                                elif abs(delOmega1 - pubDelta1) > 0.01:
                                    pubIter = floor(abs(delOmega1 - pubDelta1)/0.01)
                                    print("---------ITERATION(0/%d)--------- " % (pubIter))
                                    cnt_delta_buffer += pubIter
                                elif abs(delOmega2 - pubDelta2) > 0.01:
                                    pubIter = floor(abs(delOmega2 - pubDelta2)/0.01)
                                    print("---------ITERATION(0/%d)--------- " % (pubIter))
                                    cnt_delta_buffer += pubIter
                                else:
                                    pubIter = 1

                                for iteration in range(int(pubIter)):
                                    control_input_1 = pubDelta1 + (float)(delOmega1-pubDelta1)/pubIter*(iteration+1)
                                    control_input_2 = pubDelta2 + (float)(delOmega2-pubDelta2)/pubIter*(iteration+1)
                                    self.pub_delta_theta_1.publish(control_input_1)
                                    self.pub_delta_theta_2.publish(control_input_2)

                                    print(str(control_input_1)+"  "+str(control_input_2))
                                    if pubIter != 1:
                                        print("---------ITERATION(%d/%d)--------- " % (iteration+1,pubIter))
                                    self.r.sleep()

                                pubDelta1 = delOmega1
                                pubDelta2 = delOmega2

                                break

                            else:
                                pubDelta1 = delOmega1
                                pubDelta2 = delOmega2

                                self.pub_delta_theta_1.publish(pubDelta1)
                                self.pub_delta_theta_2.publish(pubDelta2)
                                print(str(pubDelta1)+"  "+str(pubDelta2))
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


        self.wait_for_seconds(2.0)
        rospy.loginfo("Stopping the robot at the final destination")
        print("Total Stiff Delta_Theta Change BUFFER = %d" % (cnt_delta_buffer))
        #Wait for 1 second to close valve
        self.quit_valve()
        #turn to view letters at the final global map view point
        #self.look_opposite_side()
        #stop the robot
        # self.cmd_vel.publish(Twist())

    def vel_update(self, a):
        self.current_speed = self.current_speed + a*self.dt

    def callback_position(self, _data):
        self.point.x = _data.x
        self.point.y = _data.y

    def callback_heading(self, _data):
        self.heading.data = _data.data

    def visualize_traj(self):
        self.pub_markers.publish(self.point)
        self.pub_markers_painting.publish(self.endPoint)


    def visualize_traj_encoder(self, _data):
        self.traj_encoder.points.append(Point(_data.x, _data.y, 0.0))
        self.pub_markers_encoder.publish(self.traj_encoder)





    # def generate_pathmap(self):
    #     scale = 10
    #     pixel_size = 100  #1m*1m canvas of 1cm accuracy points (including boundary points)
    #     # img = PIL.Image.new("RGB", ((100+pixel_size*self.cnt_letter)*scale, (100+pixel_size)*scale), (255, 255, 255))
    #     pil_image = PIL.Image.new("RGB",
    #                               ((pixel_size * self.cnt_letter) * scale,
    #                                (pixel_size) * scale), (255, 255, 255))

    #     print("loop_cnt_pathmap = ", self.loop_cnt_pathmap)
    #     rospy.loginfo("loop_cnt_pathmap = ", self.loop_cnt_pathmap)

    #     for i in range(self.loop_cnt_pathmap):
    #         # print(self.path_points[i][0], self.path_points[i][1])
    #         if self.path_points[i][0] < 0 or self.path_points[i][0] > 1.0 * self.cnt_letter:
    #             continue
    #         if (1.0 - self.path_points[i][1]) < 0 or (
    #                 1.0 - self.path_points[i][1]) > 1.0:
    #             continue

    #         x = 0.99 if self.path_points[i][0] == 1.0 else self.path_points[i][
    #             0]
    #         y = 0.99 if (1.0 - self.path_points[i][1]) == 1.0 else (
    #             1.0 - self.path_points[i][1])
    #         x = (int)(floor(x * pixel_size))
    #         y = (int)(floor(y * pixel_size))

    #         # x=x+50
    #         # y=y+50

    #         for k in range(scale):
    #             for t in range(scale):
    #                 pil_image.putpixel((x * scale + t, y * scale + k),
    #                                    (0, 0, 0))

    #     image_save_path = package_base_path + "/hengel_path_manager/output_pathmap/" + self.program_start_time + ".png"
    #     print("Pathmap image saved at " + image_save_path)
    #     rospy.loginfo("Pathmap image saved at " + image_save_path)
    #     pil_image.save(image_save_path, "PNG")
    #     self.map_img = np.ndarray(pil_image)
    #     #self.crop_image()
    #     # Convert RGB to BGR
    #     #cv2.cvtColor(open_cv_image, cv2.cv.CV_BGR2RGB)

    #     bridge = CvBridge()
    #     img_msg = bridge.cv2_to_imgmsg(open_cv_image, "rgb8")

    #     # global map

    def crop_image(self):
        x_px = scale_factor * self.point.x
        y_px = scale_factor * self.point.y
        r_px = scale_factor * robot_size
        th = self.heading.data

        height, width = self.map_img.shape[:2]

        mask = np.zeros((self.height, self.width), dtype=np.uint8)
        pts = np.array([[[
            int(x_px - 75.5 * scale_factor * cos(th)),
            -int(-y_px + 75.5 * scale_factor * sin(th))
            ], [
                int(x_px - 58 * scale_factor * cos(th)), -int(-y_px + 58 * sin(th))
                ], [
                    int(x_px - 29 * scale_factor * cos(th) +
                        25 * scale_factor * sin(th)),
                    -int(-y_px + 29 * scale_factor * sin(th) +
                        25 * scale_factor * cos(th))
                    ], [
                        int(x_px + 29 * scale_factor * cos(th) +
                            25 * scale_factor * sin(th)),
                        -int(-y_px - 29 * scale_factor * sin(th) +
                            25 * scale_factor * cos(th))
                        ], [
                            int(x_px + 58 * scale_factor * cos(th)),
                            -int(-y_px - 58 * scale_factor * sin(th))
                            ], [
                                int(x_px + 75.5 * scale_factor * cos(th)),
                                -int(-y_px - 75.5 * scale_factor * sin(th))
                                ], [
                                    int(x_px + 75.5 * scale_factor * cos(th) +
                                        111 * scale_factor * sin(th)),
                                    -int(-y_px - 75.5 * scale_factor * sin(th) +
                                        111 * scale_factor * cos(th))
                                    ], [
                                        int(x_px - 75.5 * scale_factor * cos(th) +
                                            111 * scale_factor * sin(th)),
                                        -int(-y_px + 75.5 * scale_factor * sin(th) +
                                            111 * scale_factor * cos(th))
                                        ]]])
        cv2.fillPoly(mask, pts, (255))
        res = cv2.bitwise_and(self.map_img, self.map_img, mask=mask)

        rect = cv2.boundingRect(pts)
        cropped = res[rect[1]:rect[1] + rect[3], rect[0]:rect[0] + rect[2]]

        bridge = CvBridge()
        crop_msg = bridge.cv2_to_imgmsg(cropped, "rgb8")
        self.crop_map_publisher.publish(crop_msg)

    def quit_valve(self):
        for ind_quit in range(50):
            self.valve_angle_input.goal_position = MARKER_UP
            self.valve_angle_publisher.publish(self.valve_angle_input)

            ind_quit = ind_quit + 1
            self.r.sleep()

    def shutdown(self):
        # self.cmd_vel.publish(Twist())
        self.pub_delta_theta_1.publish(0.0)
        self.pub_delta_theta_2.publish(0.0)
        rospy.sleep(1)

    def look_opposite_side(self):
        while (True):
            #alpha=angle_difference( pi, self.heading.data )
            alpha = angle_difference(self.heading.data, pi)
            #print("global point turning, angle = " + str(alpha))
            #if abs(alpha)> self.thres3: #abs?
            if abs(alpha) < 3.13:  #abs?
                # if alpha>0 or alpha<-pi:
                if alpha > 0:
                    self.move_cmd.linear.x = 0
                    self.move_cmd.angular.z = self.ang_vel_3
                else:
                    self.move_cmd.linear.x = 0
                    self.move_cmd.angular.z = -self.ang_vel_3
            else:
                self.move_cmd.linear.x = 0
                self.move_cmd.angular.z = 0
                # self.cmd_vel.publish(self.move_cmd)
                self.r.sleep()
                break
            # self.cmd_vel.publish(self.move_cmd)
            self.r.sleep()

    def wait_for_seconds(self, _input):
        cnt_loop = (int)(_input / self.dt)
        for i in range(cnt_loop):
            # self.cmd_vel.publish(Twist())
            self.pub_delta_theta_1.publish(0.0)
            self.pub_delta_theta_2.publish(0.0)
            self.r.sleep()

#    def real_globalmap_run(self):
#        try:
#            position = [self.point.x, self.point.y, self.heading.data]
#            #print("input for real_globmap_run = " + str(position))
#            return self.real_globalmap.run(self.letter_index, position)
#
#        #realign the frame position, according to calculated offset from global map
#            #self.offset_change_x_publisher.publish(offset[0])   #add offset_x by offset[0]
#            #self.offset_change_y_publisher.publish(offset[1])   #add offset_y by offset[1]
#            #self.offset_change_theta_publisher.publish(offset[2])   #add offset_theta by offset[2]
#
#        except rospy.ServiceException, e:
#            print("Service call failed")
#            rospy.loginfo("Service call failed")
