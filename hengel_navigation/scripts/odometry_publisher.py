#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Float32
import tf
from tf.transformations import euler_from_quaternion
from time import sleep
from math import pi
import sys

class LidarOdometry():
    def __init__(self):
        try:
            self.position_publisher = rospy.Publisher('/current_position', Point, queue_size=10)
            self.heading_publisher = rospy.Publisher('/current_heading', Float32, queue_size=10)
            self.blam_position_estimate = rospy.Subscriber('/blam/blam_slam/localization_integrated_estimate', PoseStamped, self.callback_blam_position)

            self.offset_x=0
            self.offset_y=0
            self.offset_rot=0

            self.pnt = Point()
            self.heading = Float32()

            self.isFirst = True

            while(True):
                if self.isFirst:
                    self.offset_x=self.pnt.x
                    self.offset_y=self.pnt.y
                    self.offset_rot=self.heading-pi/2.0
                    self.isFirst = False

                self.pnt.x=self.pnt.x-self.offset_x
                self.pnt.y=self.pnt.y-self.offset_y
                #self.heading.data = normalize_rad( normalize_rad(self.heading.data)-self.offset_rot )


                self.position_publisher.publish(self.pnt)
                self.heading_publisher.publish(self.heading)

                sleep(0.01)

        except Exception as e:
            print(e)
            sys.exit()


        def callback_blam_position(self, _data):
            pnt=Point()
            pnt.x=_data.pose.position.x
            pnt.y=_data.pose.position.y

            quat=[_data.pose.orientation.x, _data.pose.orientation.y, _data.pose.orientation.z, _data.pose.orientation.w]
            rotation=euler_from_quaternion(quat)

            self.pnt = pnt
            self.heading.data = normalize_rad(rotation[2])


class WheelOdometry():
    def __init__(self):
        try:
            self.position_publisher = rospy.Publisher('/current_position', Point, queue_size=10)
            self.heading_publisher = rospy.Publisher('/current_heading', Float32, queue_size=10)

            self.offset_x=0
            self.offset_y=0
            self.offset_rot=0

            self.pnt = Point()
            self.heading = Float32()

            self.isFirst = True

            self.tf_listener = tf.TransformListener()
            self.odom_frame = '/odom'

            try:
                self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = '/base_footprint'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                try:
                    self.tf_listener.waitForTransform(self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
                    self.base_frame = '/base_link'
                except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                    rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
                    rospy.signal_shutdown("tf Exception")

            while(True):
                (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
                self.pnt=Point(*trans)
                self.heading.data = euler_from_quaternion(rot)[2]

                if self.isFirst:
                    self.offset_x=self.pnt.x
                    self.offset_y=self.pnt.y
                    self.offset_rot=self.heading.data-pi/2.0
                    self.isFirst = False

                self.pnt.x=self.pnt.x-self.offset_x
                self.pnt.y=self.pnt.y-self.offset_y
                #self.heading.data = normalize_rad( normalize_rad(self.heading.data)-self.offset_rot )

                self.position_publisher.publish(self.pnt)
                self.heading_publisher.publish(self.heading)

                sleep(0.01)

        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception at wheel_odometry.py")
            sys.exit()

        except Exception as e:
            print(e)
            sys.exit()


def initialOptionSelect():
    if sys.argv[1]=='wheel':
        WheelOdometry()
        print("Wheel odometry method selected")
    elif sys.argv[1]=='lidar':
        LidarOdometry()
        print("Lidar odometry method selected")
    else:
        print("!!!!!WRONG launch_file input of argv choosing odometry method. Wheel odometry selected as default.")
        raise Exception("WRONG INPUT OPTION FOR ODOMETRY (Neither wheel nor lidar)")

if __name__ == '__main__':
    try:
        rospy.init_node('hengel_odometry_publisher', anonymous=False, disable_signals=True)
        initialOptionSelect()

        print("End of Main Function")

    except Exception as e:
        print(e)
        rospy.loginfo("shutdown program.")
        sys.exit()
