#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
import tf
from tf.transformations import euler_from_quaternion
from time import sleep
from math import pi


class WheelOdometry():
    def __init__(self):
        try:
            rospy.init_node('hengel_wheel_odometry', anonymous=False, disable_signals=True)
            self.position_publisher = rospy.Publisher('/current_position', Point, queue_size=10) 
            self.heading_publisher = rospy.Publisher('/current_heading', Float32, queue_size=10)

            
            self.offset_x=0
            self.offset_y=0
            self.offset_rot=0

            self.pnt = Point()
            self.rotation = 0

            self.isFirst = True

            while(True):
                (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
                self.pnt=Point(*trans)
                self.rotation = euler_from_quaternion(rot)[2]

                if self.isFirst:
                    self.offset_x=pnt.x
                    self.offset_y=pnt.y
                    self.offset_rot=rotation-pi/2.0
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


if __name__ == '__main__':
    try:
        WheelOdometry()
        print("End of Main Function")

    except Exception as e:
        print(e)
        rospy.loginfo("shutdown program.")
        sys.exit()
