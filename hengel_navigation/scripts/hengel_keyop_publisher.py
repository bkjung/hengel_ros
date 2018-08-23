#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion, PoseStamped
from std_msgs.msg import Float32, Time, Int32
from sensor_msgs.msg import Image, CompressedImage
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from hengel_navigation.msg import ValveInput, OperationMode
import tf
from math import radians, copysign, sqrt, pow, pi, atan2, sin, floor, cos, asin
from tf.transformations import euler_from_quaternion
import numpy as np
import sys
import tty
import termios
import select
import time

class Keyop_Publisher():
    def __init__(self):
        rospy.init_node('hengel_keyop_publisher', anonymous=False)
        self.pub=rospy.Publisher('/keyboard_input', Int32, queue_size=5)

        self.settings = termios.tcgetattr(sys.stdin)
        
        self.run()


    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key=sys.stdin.read(1)
        else:
            key=''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        try:
            print("w: forward, x: backward, a:left, d: right")
            while True:
                # key_input = raw_input("w: forward, x: backward, a:left, d: right")
                key_input= self.getKey()     
                if key_input=='a':
                    # print("left")
                    self.pub.publish(3)
                    print("w: forward, x: backward, a:left, d: right")
                elif key_input=='w':
                    # print("straight")
                    self.pub.publish(0)
                    print("w: forward, x: backward, a:left, d: right")
                    
                elif key_input=='d':
                    # print("right")
                    self.pub.publish(2)
                    print("w: forward, x: backward, a:left, d: right")
                elif key_input=='x':
                    # print("backward")
                    self.pub.publish(1)
                    print("w: forward, x: backward, a:left, d: right")
                elif key_input=='s':
                    # print("stop")
                    self.pub.publish(4)
                    print("w: forward, x: backward, a:left, d: right")
                else:
                    if (key_input=='\x03'):
                        break
                
        except KeyboardInterrupt:
            print("Got KeyboardInterrupt")
            rospy.signal_shutdown("KeyboardInterrupt")
            
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)                



if __name__=='__main__':
    Keyop_Publisher()