#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
import cv2
from cv_bridge import CvBridge, CvBridgeError
import message_filters


class Undistort():
    def __init__(self):
        rospy.init_node('undistortion', anonymous=True)

        rospy.Subscriber('/genius1/compressed', CompressedImage, self.callback_undistort1)
        rospy.Subscriber('/genius2/compressed', CompressedImage, self.callback_undistort2)
        rospy.Subscriber('/genius3/compressed', CompressedImage, self.callback_undistort3)
        rospy.Subscriber('/genius4/compressed', CompressedImage, self.callback_undistort4)
        rospy.spin()
        self.rate=rospy.Rate(10)

        # self.ts=message_filters.TimeSynchronizer([self.callback_undistort1, self.callback_undistort2, self.callback_undistort3, self.callback_undistort4], 10)
        # ts.registerCallback(self.callback)

    def callback_undistort1(self,_img):
        try:
            print("SUBSCRIBE-1")
            bridge=CvBridge()
            rawimg=bridge.compressed_imgmsg_to_cv2(_img, "bgr8")
            print(rawimg.shape)
        except CvBridgeError as e:
            print(e)
        mtx=[[392.457559, 0, 307.489055],[0, 393.146087, 314.555479], [0,0,1]]
        #mtx=[[329.446963, 0, 315.383397],[0, 328.784203, 239.504433], [0,0,1]]
        mtx=np.array(mtx)
        dst=[-0.005695, -0.017562, -0.000497, 0.001201]
        dst=np.array(dst)
        if len(mtx)!=0 and len(dst)!=0:
            if rawimg is not None:
                undistImg=cv2.undistort(rawimg, mtx, dst, None, mtx)
                homoImg=self.homography_matrix(undistImg,1)
                cv2.imwrite('homo1.png', homoImg)
            else:
                print("Image1 is None")

    def callback_undistort2(self,_img):
        try:
            print("SUBSCRIBE-2")
            bridge=CvBridge()
            rawimg=bridge.compressed_imgmsg_to_cv2(_img, "bgr8")
        except CvBridgeError as e:
            print(e)
        mtx=[[397.515439, 0, 427.319496], [0, 396.393346, 359.074317],[0,0,1]]
        mtx=np.array(mtx)
        dst=[0.008050, -0.019082, 0.002712, 0.009123]
        dst=np.array(dst)
        if len(mtx)!=0 and len(dst)!=0:
            if rawimg is not None:
                undistImg=cv2.undistort(rawimg, mtx, dst, None, mtx)
                homoImg=self.homography_matrix(undistImg, 2)
                cv2.imwrite('homo2.png', homoImg)
            else:
                print("Image2 is None")

    def callback_undistort3(self,_img):
        try:
            print("SUBSCRIBE-3")
            bridge=CvBridge()
            rawimg=bridge.compressed_imgmsg_to_cv2(_img, "bgr8")
        except CvBridgeError as e:
            print(e)
        mtx=[[389.940243, 0, 366.042362],[0, 389.110547, 376.957547],[0,0,1]]
        mtx=np.array(mtx)
        dst=[0.001656, -0.022658, 0.005813, -0.003150]
        dst=np.array(dst)
        if len(mtx)!=0 and len(dst)!=0:
            if rawimg is not None:
                undistImg=cv2.undistort(rawimg, mtx, dst, None, mtx)
                homoImg=self.homography_matrix(undistImg, 3)
                cv2.imwrite('homo3.png', homoImg)
            else:
                print("Image3 is None")

    def callback_undistort4(self,_img):
        try:
            print("SUBSCRIBE-4")
            bridge=CvBridge()
            rawimg=bridge.compressed_imgmsg_to_cv2(_img, "bgr8")
        except CvBridgeError as e:
            print(e)
        mtx=[[373.550865, 0, 385.121920],[0, 373.744498, 317.403774], [0,0,1]]
        mtx=np.array(mtx)
        dst=[-0.018599, -0.009035, 0.001095, -0.004048]
        dst=np.array(dst)
        if len(mtx)!=0 and len(dst)!=0:
            if rawimg is not None:
                undistImg=cv2.undistort(rawimg, mtx, dst, None, mtx)
                homoImg=self.homography_matrix(undistImg, 4)
                cv2.imwrite('homo4.png', homoImg)
            else:
                print("Image4 is None")

    def homography_matrix(self, _img, index):
        robotPtsArr=[]
        robotPtsArr.append([[20,25],[10,25],[10, 45], [0, 45], [0, 25], [-10, 45], [-10, 65], [-30, 45],[25, 20]])
        robotPtsArr.append([[-60,25], [-70,25], [-60,45], [-45,-30], [-45,-50], [-50,15]])
        robotPtsArr.append([[15,-30], [5,-30],[-5,-30],[-15,-30],[-25,-30],[-35,-30],[-45,-30],
                [35,-40],[25,-40],[15,-40],[5,-40],[-5,-40],[-15, -40],[-25,-40],[-35,-40],[-45,-40],
                [25,-50],[15,-50],[5,-50],[-5,-50],[-45,-50]])
        robotPtsArr.append([
            [20,25],[20,15],
                [25,20],[25,0],[25,-20],[25,-40],
                [45,20],[45,0],[45,-20],[45,-40],[45,-60],
                [50,45]
                ])

        objPts = [[[point_r[0]*(-5.0)+640.0, point_r[1]*(-5.0)+640.0]  for point_r in robotPts]  for robotPts in robotPtsArr]

        imgPtsArr=[]
        imgPtsArr.append([[419.5,478.5], [516.5,480.5], [614.4,482.6], [684.4, 535.8], [340.7,327.3],
        [194.7,323.0], [351.0,236.7], [293.7,236.0], [61.7,232.0], [560.7,333.7]])
        imgPtsArr.append([[616.0,319.7], [598.0,278.0], [748.0, 320.2], [216.5,401.8], [65.8,401.8], [565.0, 370.0]])
        imgPtsArr.append([[199.8, 550.2], [297,549],[395.5,548],[495.7,548.1],[594.8,547.1],[692.9,545.1],[783.6,542.1],
            [62,464],[142.2,463.8],[225.2,463.2],[308.2,462],[392.5,461.2],[476.9,459.6],[562, 460],[647, 459],[729.8, 457.8],
            [171.2,399.2], [244,398],[316.5,398],[388.5, 345.7],[650.9,343.1]])
        imgPtsArr.append([
            # [96.2,522],[197,524],
                [166,471], [357.4,473.4],[549,475.6],[733.7, 474.7],
                # [222,326],[366,327],[510,329],[652,330],[786,331],
                # [65.6,299]
                ])

        homography, status=cv2.findHomography(np.array(imgPtsArr[index-1]), np.array(objPts[index-1],np.float32))

        return cv2.warpPerspective(_img, homography, (1280, 1280))


if __name__=="__main__":
    Undistort()
    cv2.destroyAllWindows()

