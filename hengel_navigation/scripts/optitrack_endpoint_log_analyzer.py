import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Time
from rosgraph_msgs.msg import Clock
# from std_msgs.msg import Float32, Time, Int32
# from sensor_msgs.msg import Image, CompressedImage
# from nav_msgs.msg import Path
# from visualization_msgs.msg import Marker
# from hengel_navigation.msg import ValveInput, OperationMode
# import tf
# from math import radians, copysign, sqrt, pow, pi, atan2, sin, floor, cos, asin, ceil
# from tf.transformations import euler_from_quaternion
# import numpy as np
import sys
import time
import os
# import cv2
# import cv_bridge
# import logging
# import matplotlib.pyplot as plt


class bagfile_analyzer():
    def __init__(self):
        rospy.init_node('vision_offset_analyzer', anonymous=False)

        self.endpoint_x = 0.0
        self.endpoint_y = 0.0
        self.endpoint_z = 0.0

        self.offset_x = 0.0
        self.offset_y = 0.0
        self.offset_z = 0.0
        self.offset_accepted_flag = False

        self.time_sec = 0
        self.time_nsec = 0

        package_base_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
        home_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../../home/parallels"))

        self.newFile=open(home_path+"/endpoint_offset_log"+time.strftime("%y%m%d_%H%M%S")+".txt", "w")

        self.dt = 0.1  # [s]
        self.r = rospy.Rate(1.0/self.dt)

        rospy.Subscriber('/endpoint', Point, self.endpoint_callback)
        rospy.Subscriber('/offset_change', Point, self.offset_callback)
        # rospy.Subscriber('/clock', Time, self.time_callback)
        rospy.Subscriber('/clock', Clock, self.time_callback)

        # rospy.spin()

        print("init finished")

    def endpoint_callback(self, _endpoint):
        self.endpoint_x = _endpoint.x
        self.endpoint_y = _endpoint.y
        self.endpoint_z = _endpoint.z

    def offset_callback(self, _offset):
        self.offset_x = _offset.x
        self.offset_y = _offset.y
        self.offset_z = _offset.z
        self.offset_accepted_flag = True

    def time_callback(self, _time):
        self.time_nsec = _time.clock.nsecs
        self.time_sec = _time.clock.secs


    def run(self):
        while True:
            if self.offset_accepted_flag == True:
                self.newFile.write(str(self.time_sec)+str(self.time_nsec)+"\t"+str(self.endpoint_x)+"\t"+str(self.endpoint_y)+"\t"+str(self.endpoint_z)+"\t"+str(self.offset_x)+"\t"+str(self.offset_y)+"\t"+str(self.offset_z)+"\t"+"accepted"+"\n")
                self.offset_accepted_flag = False
            else:
                self.newFile.write(str(self.time_sec)+str(self.time_nsec)+"\t"+str(self.endpoint_x)+"\t"+str(self.endpoint_y)+"\t"+str(self.endpoint_z)+"\t"+str(0.0)+"\t"+str(0.0)+"\t"+str(0.0)+"\n")

            print("logged")

            self.r.sleep()


if __name__=='__main__':
    app = bagfile_analyzer()
    app.run()


