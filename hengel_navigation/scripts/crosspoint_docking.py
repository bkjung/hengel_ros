#!/usr/bin/env python
import rospy
import numpy as np
import cv2, sys, time, math
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
import sys
import time


class CrosspiontDocking():
    def __init__(self):
        self.floorcam_subscriber = rospy.Subscriber('/pi_floorcam/image_raw/compressed',
                                                    CompressedImage, self.callback_floorcam)
        
        #initialize
        self.np_arr = np.empty(0)
        self.image_cv = np.zeros((0,0,3), np.uint8)
        self.image_cv_box = np.zeros((0,0,3), np.uint8)
        self.image_cv_gray = np.zeros((0,0,1), np.uint8)

        self.imageReceived = False

        self.r = rospy.Rate(100) #100Hz

        self.save_mode = False

    def callback_floorcam(self, _img):
        try:
            self.imageReceived = True

            self.np_arr = np.fromstring(_img.data, np.uint8)
            self.image_cv = cv2.imdecode(self.np_arr, cv2.IMREAD_COLOR)
            self.image_cv_box = self.image_cv
            self.image_cv_gray = cv2.cvtColor(self.image_cv, cv2.COLOR_BGR2GRAY)

            
            if self.save_mode:
                cv2.imwrite(
                    "~/Dropbox/intern_share/experiment_data/rosbag_analysis/" + self.filename
                    + ".jpg", self.image_cv_gray)
                self.save_mode = False
                self.filename = ""


            msg_box = CompressedImage()
            msg_box.format = "jpeg"

            for i in range(199, 300):
                self.image_cv_box[i][169] = [255, 0, 0]
                self.image_cv_box[i][329] = [255, 0, 0]
            for j in range(169, 330):
                self.image_cv_box[199][j] = [255, 0, 0]
                self.image_cv_box[299][j] = [255, 0, 0]
            
            msg_box.data = np.array(self.image_cv_box).tostring()
            pub_box.publish(msg_box)


        except Exception as e:
            print(e)
            sys.exit(1)

        # a, b = self.image_cv.shape
        # print(a, b)
        # print(self.image_cv[479, 639])

    def analysis(self):
        if self.imageReceived:
            return self.image_cv[199:299, 169:329].mean()
            # if :
            #     return "white"
            # elif ...:
            #     return "black"
            # else ...:
            #     return "unknown"

    def save(self, _filename):
        self.save_mode = True
        self.filename = _filename

    def run(self):
        cnt = 0
        while True:
            if rospy.is_shutdown():
                break

            if self.imageReceived:
                # self.save(str(cnt))
                cnt=cnt+1
                print(self.analysis())
            self.r.sleep()


if __name__ == "__main__":
    try:
        rospy.init_node('hengel_crosspoint_docking')
        app = CrosspiontDocking()
        app.run()

    except KeyboardInterrupt:
        print("Keyboard Interrupted")
        sys.exit(0)

    except Exception as e:
        print(e)
        rospy.loginfo("shutdown program")
        sys.exit(1)
