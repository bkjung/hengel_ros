#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Float32
import tf
from tf.transformations import euler_from_quaternion
from time import sleep
from math import pi
import sys

odometry_method = 0
ODOMETRY_WHEEL = 1
ODOMETRY_LIDAR = 2

class LidarOdometry():
    def __init__(self):
        try:
            rospy.init_node('hengel_odometry_publisher', anonymous=False, disable_signals=True)
            self.position_publisher = rospy.Publisher('/current_position', Point, queue_size=10) 
            self.heading_publisher = rospy.Publisher('/current_heading', Float32, queue_size=10)
            self.blam_position_estimate = rospy.Subscriber('/blam/blam_slam/localization_integrated_estimate', PoseStamped, self.callback_blam_position)

            self.offset_x=0
            self.offset_y=0
            self.offset_rot=0

            self.pnt = Point()
            self.rotation = 0

            self.isFirst = True

            while(True):
                if self.isFirst:
                    self.offset_x=self.pnt.x
                    self.offset_y=self.pnt.y
                    self.offset_rot=self.rotation-pi/2.0
                    self.isFirst = False

                self.pnt.x=self.pnt.x-self.offset_x
                self.pnt.y=self.pnt.y-self.offset_y
                #self.rotation = normalize_rad( normalize_rad(self.rotation)-self.offset_rot )
                self.rotation = self.rotation

                heading=Float32()                
                heading.data=self.rotation
                self.position_publisher.publish(self.pnt)
                self.heading_publisher.publish(heading)

                sleep(0.01)

        except Exceptoin as e:
            print(e)
            sys.exit()


        def callback_blam_position(self, data):
            pnt=Point()
            pnt.x=data.pose.position.x
            pnt.y=data.pose.position.y

            quat=[data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]
            rotation=euler_from_quaternion(quat)

            self.pnt = pnt
            self.rotation = normalize_rad(rotation[2])


class WheelOdometry():
    def __init__(self):
        try:
            rospy.init_node('hengel_odometry_publisher', anonymous=False, disable_signals=True)
            self.position_publisher = rospy.Publisher('/current_position', Point, queue_size=10) 
            self.heading_publisher = rospy.Publisher('/current_heading', Float32, queue_size=10)

            self.offset_x=0
            self.offset_y=0
            self.offset_rot=0

            self.pnt = Point()
            self.rotation = 0

            self.isFirst = True

            self.tf_listener = tf.TransformListener()
            self.odom_frame = '/odom'

            try:
                self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = '/base_footprint'
                print("base_footprint")
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                try:
                    self.tf_listener.waitForTransform(self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
                    self.base_frame = '/base_link'
                    print("base_link")
                except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                    rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
                    rospy.signal_shutdown("tf Exception")

            while(True):
                (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
                self.pnt=Point(*trans)
                self.rotation = euler_from_quaternion(rot)[2]

                if self.isFirst:
                    self.offset_x=self.pnt.x
                    self.offset_y=self.pnt.y
                    self.offset_rot=self.rotation-pi/2.0
                    self.isFirst = False

                self.pnt.x=self.pnt.x-self.offset_x
                self.pnt.y=self.pnt.y-self.offset_y
                #self.rotation = normalize_rad( normalize_rad(self.rotation)-self.offset_rot )
                self.rotation = self.rotation

                heading=Float32()                
                heading.data=self.rotation
                self.position_publisher.publish(self.pnt)
                self.heading_publisher.publish(heading)

                sleep(0.01)

        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception at wheel_odometry.py)
            sys.exit()
        
        except Exceptoin as e:
            print(e)
            sys.exit()

def initialOptionSelect():
    global odometry_method
    word=raw_input("There are two options of odometry.\n[1] Wheel Odometry.\n[2] Lidar (Velodyne) Odometry.\nType 1 or 2 :")
    print("Input:"+word)
    odometry_method = int(word)


if __name__ == '__main__':
    try:
        initialOptionSelect()
        if odometry_method == 1:
            WheelOdometry()
        elif odometry_method == 2:
            LidarOdometry()
        else:
            raise Exception("WRONG INPUT OPTION FOR ODOMETRY (Neither 1 nor 2)")

        print("End of Main Function")

    except Exception as e:
        print(e)
        rospy.loginfo("shutdown program.")
        sys.exit()