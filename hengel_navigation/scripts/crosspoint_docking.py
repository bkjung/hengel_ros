#!/usr/bin/env python
import rospy
import numpy as np
import cv2, sys, time, math
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
import sys
import time

# X_MIN = 199
# X_MAX = 299

X_MIN = 150
X_MAX = 250
Y_MIN = 169
Y_MAX = 329

class CrosspointDocking():
    def __init__(self, _starttime):
        self.floorcam_subscriber = rospy.Subscriber('/pi_floorcam/image_raw/compressed',
                                                    CompressedImage, self.callback_floorcam)
        self.pub_box = rospy.Publisher(
        '/pi_floorcam_box/image_raw/compressed', CompressedImage, queue_size=3)

        #initialize
        self.np_arr = np.empty(0)
        self.image_cv = np.zeros((0,0,3), np.uint8)
        self.image_cv_box = np.zeros((0,0,3), np.uint8)
        self.image_cv_gray = np.zeros((0,0,1), np.uint8)
        self.msg_box = CompressedImage()
        self.msg_box.format = "jpeg"
        self.imageReceived = False
        self.r = rospy.Rate(10) #10Hz
        # self.save_mode = False
        self.isStarted = False

    def callback_floorcam(self, _img):
        try:
            self.imageReceived = True

            self.np_arr = np.fromstring(_img.data, np.uint8)
            self.image_cv = cv2.imdecode(self.np_arr, cv2.IMREAD_COLOR)
            self.image_cv_box = self.image_cv
            self.image_cv_gray = cv2.cvtColor(self.image_cv, cv2.COLOR_BGR2GRAY)

            # if self.save_mode:
            #     cv2.imwrite(
            #         "~/Dropbox/intern_share/experiment_data/rosbag_analysis/" + self.filename
            #         + ".jpg", self.image_cv_gray)
            #     self.save_mode = False
            #     self.filename = ""

            #print("Box Average: " + str(self.analysis()))
            #rospy.loginfo("Box Average: " + str(self.analysis()))

        except Exception as e:
            print(e)
            sys.exit(1)

    def analysis(self):
        if self.imageReceived and self.isStarted:
            for i in range(Y_MIN, Y_MAX+1):
                self.image_cv_box[i][X_MIN] = [255, 0, 0]
                self.image_cv_box[i][X_MAX] = [255, 0, 0]
            for j in range(X_MIN, X_MAX+1):
                self.image_cv_box[Y_MIN][j] = [255, 0, 0]
                self.image_cv_box[Y_MAX][j] = [255, 0, 0]

            self.msg_box.data = np.array(cv2.imencode('.jpg', self.image_cv_box)[1]).tostring()
            self.pub_box.publish(self.msg_box)

            return self.image_cv[Y_MIN:Y_MAX+1, X_MIN:X_MAX+1].mean()

    def save(self, _filename):
        # self.save_mode = True
        self.filename = _filename

    def run(self):
        while True:
            if rospy.is_shutdown():
                break

            if self.imageReceived:
                self.analysis()

            self.r.sleep()

    def start(self):
        self.isStarted = True


if __name__ == "__main__":
    try:
        rospy.init_node('hengel_crosspoint_docking')
        app = CrosspointDocking("")
        app.start()
        app.run()

    except KeyboardInterrupt:
        print("Keyboard Interrupted")
        sys.exit(0)

    except Exception as e:
        print(e)
        rospy.loginfo("shutdown program")
        sys.exit(1)
