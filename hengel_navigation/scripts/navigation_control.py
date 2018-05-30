#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion, PoseStamped
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from hengel_navigation.msg import ValveInput, OperationMode
import tf
from math import radians, copysign, sqrt, pow, pi, atan2, sin, floor, cos
from tf.transformations import euler_from_quaternion
import numpy as np
import sys
#from PIL import Image
#import PIL.Image
import time
import os
import cv2
import cv_bridge
from real_globalmap import RealGlobalMap
from pi_cam_manager import PiCamManager
from crosspoint_docking import CrosspointDocking


#2480 is too large, so that it hits the ground and the valve_control while loop does not end.
#MARKER_DOWN = 2480
#MARKER_DOWN = 2460
#MARKER_DOWN = 2430

#Due to wheel height change
MARKER_DOWN = 2450

MARKER_UP = 2000

scale_factor = 3  #[pixel/cm]
robot_size = 15  #[cm]; diameter

package_base_path = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "../.."))
os.system("mkdir -p " + package_base_path +
          "/hengel_path_manager/output_pathmap")


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

    def __init__(self, _arr_path, _docking_point_list):
        word = raw_input(
            "There are two options for pi cam save.\n[1] Save Pi Cam - Floor Image.\n[2] Do not Save. \nType 1 or 2 :"
        )
        self.pi_cam_save_option = int(word)

        self.arr_path = _arr_path
        self.docking_point_list = _docking_point_list
        self.initial_setting()
        self.run()

    def initial_setting(self):
        self.program_start_time = time.strftime("%y%m%d_%H%M%S")

        self.point = Point()
        self.heading = Float32()
        self.move_cmd = Twist()

        self.valve_operation_mode = OperationMode()
        self.valve_operation_mode.mode = 1
        self.valve_angle_input = ValveInput()

        self.valve_status = MARKER_UP

        self.r = rospy.Rate(50)  #50hz

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


        self.path_points = []

        self.thres1 = np.deg2rad(30)
        self.thres2 = np.deg2rad(15)
        self.thres3 = np.deg2rad(4)

        self.ang_vel_1 = 0.15
        self.ang_vel_2 = 0.1
        self.ang_vel_3 = 0.06
        self.lin_vel = 0.07

        self.is_moving_between_letters = False

        self.loop_cnt_pathmap = 0

        self.map_img = []
        self.map_img = np.ndarray(self.map_img)

        #rospy.init_node('hengel_navigation_control', anonymous=False, disable_signals=True)
        rospy.on_shutdown(self.shutdown)

        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

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
            '/offset_x', Float32, queue_size=5)
        self.offset_change_y_publisher = rospy.Publisher(
            '/offset_y', Float32, queue_size=5)
        self.offset_change_theta_publisher = rospy.Publisher(
            '/offset_theta', Float32, queue_size=5)

        self.position_subscriber = rospy.Subscriber('/current_position', Point,
                                                    self.callback_position)
        self.heading_subscriber = rospy.Subscriber('/current_heading', Float32,
                                                   self.callback_heading)

        self.real_globalmap = RealGlobalMap(self.arr_path)
        self.pi_cam_manager = PiCamManager(self.program_start_time)
        self.crosspoint_docking = CrosspointDocking(self.program_start_time)
        self.crosspoint_docking.start()

    def run(self):
        self.wait_for_seconds(2)
        # go through path array
        print("number of letters = " + str(len(self.arr_path)))
        rospy.loginfo("number of letters = " + str(len(self.arr_path)))
        for idx_letter in range(len(self.arr_path)):
            print("number of letter segments in letter no." + str(idx_letter) +
                  " = " + str(len(self.arr_path[idx_letter])))
            rospy.loginfo("number of letter segments in letter no." + str(idx_letter) +
                  " = " + str(len(self.arr_path[idx_letter])))
            for idx_segment in range(len(self.arr_path[idx_letter])):
                waypoints_in_segment = []
                for idx_waypoint in range(len(self.arr_path[idx_letter][idx_segment])):
                    # waypoints_in_segment.append([self.arr_path[idx_letter][idx_waypoint][0]-self.arr_path[0][0][0], self.arr_path[idx_letter][idx_waypoint][1]-self.arr_path[0][0][1]])
                    waypoints_in_segment.append([
                        self.arr_path[idx_letter][idx_segment][idx_waypoint][0],
                        self.arr_path[idx_letter][idx_segment][idx_waypoint][1]
                    ])
                    self.cnt_total_waypoints = self.cnt_total_waypoints + 1
                self.waypoints.append(waypoints_in_segment)

        self.cnt_letter = len(self.arr_path)

        #self.real_globalmap_run()

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
                    isDockingPoint = False

                    if rospy.is_shutdown():
                        break
                    print("\n\nwaypoint index : " +
                        str(self.waypoint_index_in_current_segment) +
                        " in segment no. " + str(self.segment_index) +
                        " in letter no. " + str(self.letter_index))
                    rospy.loginfo("\n\nwaypoint index : " +
                        str(self.waypoint_index_in_current_segment) +
                        " in segment no. " + str(self.segment_index) +
                        " in letter no. " + str(self.letter_index))

                    for docking_point in self.docking_point_list:
                        if docking_point[0]==self.letter_index and
                            docking_point[1]==self.segment_index and
                            docking_point[2]==self.waypoint_index_in_current_segment:
                            print("MOVING TO DOCKING POINT")
                            isDockingPoint = True
                            break


                    self.current_waypoint = [
                        self.arr_path[self.letter_index][self.segment_index][
                            self.waypoint_index_in_current_segment][0],
                        self.arr_path[self.letter_index][self.segment_index][
                            self.waypoint_index_in_current_segment][1]
                    ]

                    if self.waypoint_index_in_current_segment == 0:
                        print("moving to FIRST waypoint")
                        rospy.loginfo("moving to FIRST waypoint in segment")
                        self.is_moving_between_letters = True
                    elif self.segment_index == self.cnt_segments_in_current_letter - 1:
                        print("moving to GLOBAL VIEW POINT")
                        rospy.loginfo("moving to GLOBAL VIEW POINT")
                        self.is_moving_between_letters = True
                    else:
                        self.is_moving_between_letters = False

                    distance = sqrt(
                        pow(self.current_waypoint[0] - self.point.x, 2) +
                        pow(self.current_waypoint[1] - self.point.y, 2))
                    
                    docking_buffer_cnt = 0
                    # Motion Control
                    while True:
                        if rospy.is_shutdown():
                            break
                        try:
                            distance = sqrt(
                                pow(self.current_waypoint[0] - self.point.x, 2) +
                                pow(self.current_waypoint[1] - self.point.y, 2))

                            if isDockingPoint:
                                if self.crosspoint_docking.check():
                                    #docked
                                    break
                                elif distance < 0.03 * 0.58:
                                    self.current_waypoint[0] = self.current_waypoint[0]+(self.current_waypoint[0]-self.point.x)*0.03*0.58/distance
                                    self.current_waypoint[1] = self.current_waypoint[1]+(self.current_waypoint[1]-self.point.y)*0.03*0.58/distance
                            
                            elif distance < 0.03 * 0.58:
                                self.cmd_vel.publish(Twist())
                                break

                            alpha = angle_difference(
                                atan2(self.current_waypoint[1] - self.point.y,
                                    self.current_waypoint[0] - self.point.x),
                                self.heading.data)

                            #print("CURRENT: "+str(self.point.x)+", "+str(self.point.y)+"  NEXT: "+str(self.current_waypoint[0])+", "+str(self.current_waypoint[1]))
                            #print("heading error: %0.3f" % np.rad2deg(alpha))

                            if abs(alpha) <= self.thres3:
                                self.valve_status = MARKER_DOWN
                                x = distance * sin(alpha)
                                curv = 2 * x / pow(distance, 2)

                                if distance < 0.02:
                                    lin_vel_scaled = self.lin_vel / 5.0
                                elif distance < 0.04:
                                    lin_vel_scaled = self.lin_vel / 4.0
                                elif distance < 0.06:
                                    lin_vel_scaled = self.lin_vel / 3.0
                                elif distance < 0.08:
                                    lin_vel_scaled = self.lin_vel / 2.0
                                else:
                                    lin_vel_scaled = self.lin_vel

                                self.move_cmd.linear.x = lin_vel_scaled
                                self.move_cmd.angular.z = curv * lin_vel_scaled

                            else:
                                #rotate to align in threshold
                                #self.valve_status = MARKER_UP

                                if abs(alpha) > self.thres1:  #abs?
                                    # if alpha>0 or alpha<-pi:
                                    if alpha > 0:
                                        self.move_cmd.linear.x = 0
                                        self.move_cmd.angular.z = self.ang_vel_1
                                    else:
                                        self.move_cmd.linear.x = 0
                                        self.move_cmd.angular.z = -self.ang_vel_1

                                elif abs(alpha) > self.thres2:
                                    # if alpha>0 or alpha<-pi:
                                    if alpha > 0:
                                        self.move_cmd.linear.x = 0
                                        self.move_cmd.angular.z = self.ang_vel_2
                                    else:
                                        self.move_cmd.linear.x = 0
                                        self.move_cmd.angular.z = -self.ang_vel_2

                                elif abs(alpha) > self.thres3:
                                    # if alpha>0 or alpha<-pi:
                                    if alpha > 0:
                                        self.move_cmd.linear.x = 0
                                        self.move_cmd.angular.z = self.ang_vel_3
                                    else:
                                        self.move_cmd.linear.x = 0
                                        self.move_cmd.angular.z = -self.ang_vel_3

                            if self.is_moving_between_letters:
                                self.valve_status = MARKER_UP
                            else:
                                pass
                            self.valve_angle_input.goal_position = self.valve_status
                            self.valve_angle_publisher.publish(
                                self.valve_angle_input)

                            self.cmd_vel.publish(self.move_cmd)

                            #                        self.loop_cnt_pathmap = self.loop_cnt_pathmap + 1
                            #                            if self.loop_cnt_pathmap==25:
                            #                                self.loop_cnt_pathmap = 0
                            #                                self.path_points.append([self.point.x, self.point.y])
                            #                                self.generate_pathmap()
                            #
                            self.r.sleep()

                        except KeyboardInterrupt:
                            print("Got KeyboardInterrupt")
                            rospy.signal_shutdown("KeyboardInterrupt")
                            break

                    #Arrived at the waypoint
                    print("CURRENT: " + str(self.point.x) + ", " +
                        str(self.point.y) + " \t\t WAYPOINT: " +
                        str(self.current_waypoint[0]) + ", " +
                        str(self.current_waypoint[1]))
                    rospy.loginfo("CURRENT: " + str(self.point.x) + ", " +
                        str(self.point.y) + " \t\t WAYPOINT: " +
                        str(self.current_waypoint[0]) + ", " +
                        str(self.current_waypoint[1]))

                    #stop the robot
                    self.cmd_vel.publish(Twist())

                    if self.pi_cam_save_option==1:
                        #take picam floor photo
                        self.wait_for_seconds(0.5)
                        self.pi_cam_manager.save("picam_letter-" +
                            str(self.letter_index) + "_segment-" +
                            str(self.segment_index) + "_waypoint-" +
                            str(self.waypoint_index_in_current_segment))
                        print("Pi Cam Saved")
                        rospy.loginfo("Pi Cam Saved")
                    else:
                        pass

                    self.waypoint_index_in_current_segment = self.waypoint_index_in_current_segment + 1
                self.segment_index = self.segment_index + 1
                self.waypoint_index_in_current_segment = 0

            #End of current letter.
            #This is global map view point
            print("At the global map view point")
            rospy.loginfo("At the global map view point")
            #turn to view letters
            self.look_opposite_side()

            print("real globalmap run!!")
            rospy.loginfo("real globalmap run!!")
            self.real_globalmap_run()

            print("wait for 5 sec")
            rospy.loginfo("wait for 5 sec")
            time.sleep(5)

            #it's time for next letter
            self.letter_index = self.letter_index + 1
            self.segment_index = 0
            self.waypoint_index_in_current_segment = 0

        rospy.loginfo("Stopping the robot at the final destination")
        #Wait for 1 second to close valve
        self.quit_valve()
        #turn to view letters at the final global map view point
        #self.look_opposite_side()
        #stop the robot
        self.cmd_vel.publish(Twist())

    def callback_position(self, _data):
        self.point.x = _data.x
        self.point.y = _data.y

    def callback_heading(self, _data):
        self.heading.data = _data.data

    def generate_pathmap(self):
        scale = 10
        pixel_size = 100  #1m*1m canvas of 1cm accuracy points (including boundary points)
        # img = PIL.Image.new("RGB", ((100+pixel_size*self.cnt_letter)*scale, (100+pixel_size)*scale), (255, 255, 255))
        pil_image = PIL.Image.new("RGB",
                                  ((pixel_size * self.cnt_letter) * scale,
                                   (pixel_size) * scale), (255, 255, 255))

        print("loop_cnt_pathmap = ", self.loop_cnt_pathmap)
        rospy.loginfo("loop_cnt_pathmap = ", self.loop_cnt_pathmap)

        for i in range(self.loop_cnt_pathmap):
            # print(self.path_points[i][0], self.path_points[i][1])
            if self.path_points[i][0] < 0 or self.path_points[i][0] > 1.0 * self.cnt_letter:
                continue
            if (1.0 - self.path_points[i][1]) < 0 or (
                    1.0 - self.path_points[i][1]) > 1.0:
                continue

            x = 0.99 if self.path_points[i][0] == 1.0 else self.path_points[i][
                0]
            y = 0.99 if (1.0 - self.path_points[i][1]) == 1.0 else (
                1.0 - self.path_points[i][1])
            x = (int)(floor(x * pixel_size))
            y = (int)(floor(y * pixel_size))

            # x=x+50
            # y=y+50

            for k in range(scale):
                for t in range(scale):
                    pil_image.putpixel((x * scale + t, y * scale + k),
                                       (0, 0, 0))

        image_save_path = package_base_path + "/hengel_path_manager/output_pathmap/" + self.program_start_time + ".png"
        print("Pathmap image saved at " + image_save_path)
        rospy.loginfo("Pathmap image saved at " + image_save_path)
        pil_image.save(image_save_path, "PNG")
        self.map_img = np.ndarray(pil_image)
        #self.crop_image()
        # Convert RGB to BGR
        #cv2.cvtColor(open_cv_image, cv2.cv.CV_BGR2RGB)

        bridge = CvBridge()
        img_msg = bridge.cv2_to_imgmsg(open_cv_image, "rgb8")

        # global map

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
        self.cmd_vel.publish(Twist())
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
                self.cmd_vel.publish(self.move_cmd)
                self.r.sleep()
                break
            self.cmd_vel.publish(self.move_cmd)
            self.r.sleep()

    def wait_for_seconds(self, _input):
        cnt_loop = (int)(_input * 50.0)
        for i in range(cnt_loop):
            self.cmd_vel.publish(Twist())
            self.r.sleep()

    def real_globalmap_run(self):
        try:
            position = [self.point.x, self.point.y, self.heading.data]
            #print("input for real_globmap_run = " + str(position))
            offset = self.real_globalmap.run(self.letter_index, position)

            #realign the frame position, according to calculated offset from global map
            self.offset_change_x_publisher.publish(offset[0])   #add offset_x by offset[0]
            self.offset_change_y_publisher.publish(offset[1])   #add offset_y by offset[1]
            self.offset_change_theta_publisher.publish(offset[2])   #add offset_theta by offset[2]

        except rospy.ServiceException, e:
            print("Service call failed")
            rospy.loginfo("Service call failed")
