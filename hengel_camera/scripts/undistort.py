#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
import cv2
from cv_bridge import CvBridge, CvBridgeError
import message_filters
import time


class Undistort():
    def __init__(self):
        rospy.init_node('undistortion', anonymous=True)

        # rospy.Subscriber('/genius1/compressed', CompressedImage, self.callback_undistort1)
        # rospy.Subscriber('/genius2/compressed', CompressedImage, self.callback_undistort2)
        # rospy.Subscriber('/genius3/compressed', CompressedImage, self.callback_undistort3)
        # rospy.Subscriber('/genius4/compressed', CompressedImage, self.callback_undistort4)
        # rospy.spin()

        self.sum_pub=rospy.Publisher('/summed_image/compressed', CompressedImage, queue_size=3)

        self.rate=rospy.Rate(10)

        self.callback1=message_filters.Subscriber('/genius1/compressed', CompressedImage)
        self.callback2=message_filters.Subscriber('/genius2/compressed', CompressedImage)
        self.callback3=message_filters.Subscriber('/genius3/compressed', CompressedImage)
        self.callback4=message_filters.Subscriber('/genius4/compressed', CompressedImage)

        self.ts=message_filters.ApproximateTimeSynchronizer([self.callback1, self.callback2, self.callback3, self.callback4], 10, 0.1, allow_headerless=True)
        self.ts.registerCallback(self.sync_callback)
        rospy.spin()

    def sync_callback(self, _img1, _img2, _img3, _img4):
        _time=time.time()
        img1,time1=self.callback_undistort1(_img1)
        img2,time2=self.callback_undistort2(_img2)
        img3,time3=self.callback_undistort3(_img3)
        img4,time4=self.callback_undistort4(_img4)
        print("1: "+str(time.time()-_time))
        im_mask_inv1, im_mask1=self.find_mask(img1)
        im_mask_inv3, im_mask3=self.find_mask(img3)
        print("2: "+str(time.time()-_time))

        img1_masked=np.multiply(img1, im_mask_inv1)
        img2_masked=np.multiply(np.multiply(img2, im_mask1), im_mask3)
        print("3: "+str(time.time()-_time))
        img3_masked=np.multiply(img3, im_mask_inv3)
        img4_masked=np.multiply(np.multiply(img4, im_mask1), im_mask3)

        summed_image= img1_masked+img2_masked+img3_masked+img4_masked
        print("4: "+str(time.time()-_time))

        #cv2.imshow('summed.png', summed_image)
        #cv2.waitKey(3)
        bridge=CvBridge()
        summed_msg=bridge.cv2_to_compressed_imgmsg(summed_image)
        print("5: "+str(time.time()-_time))
        self.sum_pub.publish(summed_msg)
        print("img1: "+str(time1)+", img2: "+str(time2-time1)+", img3: "+str(time3-time1)+", img4: "+str(time4-time1)+", summ: "+str(time.time()-time1))

        #cv2.imwrite('homo1.png',img1)
        #cv2.imwrite('homo2.png',img2)
        #cv2.imwrite('homo3.png',img3)
        #cv2.imwrite('homo4.png',img4)

    def find_mask(self, img):
        _time=time.time()
        black_range1=np.array([0,0,0])
        time1=time.time()-_time
        im_mask=(cv2.inRange(img, black_range1, black_range1)).astype('bool')
        time2=time.time()-_time
        im_mask=np.dstack((im_mask, im_mask, im_mask))
        time3=time.time()-_time
        im_mask_inv=(1-im_mask).astype('bool')
        time4=time.time()-_time
        print("1: "+str(time1)+", 2:"+str(time2)+", 3:"+str(time3)+", 4:"+str(time4))
        return im_mask_inv, im_mask

    def callback_undistort1(self,_img):
        try:
            time1=time.time()
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
                print("undistort time: "+str(time.time()-time1))
                homoImg=self.homography_matrix(undistImg,1)
                print("homography time: "+str(time.time()-time1))
                # cv2.imwrite('homo1.png', homoImg)
                return homoImg, time.time()
            else:
                print("Image1 is None")

    def callback_undistort2(self,_img):
        try:
            time1=time.time()
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
                # cv2.imwrite('homo2.png', homoImg)
                return homoImg, time.time()
            else:
                print("Image2 is None")

    def callback_undistort3(self,_img):
        try:
            time1=time.time()
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
                # cv2.imwrite('homo3.png', homoImg)
                return homoImg, time.time()
            else:
                print("Image3 is None")

    def callback_undistort4(self,_img):
        try:
            time1=time.time()
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
                # cv2.imwrite('homo4.png', homoImg)
                return homoImg, time.time()
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

        objPts = [[[point_r[1]*(-5.0)+640.0, point_r[0]*(-5.0)+640.0]  for point_r in robotPts]  for robotPts in robotPtsArr]

        imgPtsArr=[]
        imgPtsArr.append([[614.2, 483],[516.8,480.2],[487.5,331.3],[414,329.3],[419.3,478.5],[340.5,326.7],[352.2,237.8],[194.4,323.5],[684.6,536]])
        imgPtsArr.append([[616.0,319.7], [598.0,278.0], [748.0, 320.2], [216.5,401.8], [65.8,401.8], [565.0, 370.0]])
        imgPtsArr.append([[199.8, 550.2], [297,549],[395.5,548],[495.7,548.1],[594.8,547.1],[692.9,545.1],[783.6,542.1],
            [62,464],[142.2,463.8],[225.2,463.2],[308.2,462],[392.5,461.2],[476.9,459.6],[562, 460],[647, 459],[729.8, 457.8],
            [170.8,399], [243.8, 397.7],[316, 397],[390.2,396.5],[686.2,393.5]])
        imgPtsArr.append([
             [96.2,522],[197,524],
                [166,471], [357.4,473.4],[549,475.6],[733.7, 474.7],
                 [222,326],[366,327],[510,329],[652,330],[786,331],
                 [65.6,299]
                ])

        homography, status=cv2.findHomography(np.array(imgPtsArr[index-1]), np.array(objPts[index-1],np.float32))

        return cv2.warpPerspective(_img, homography, (1280, 1280))


if __name__=="__main__":
    Undistort()
    cv2.destroyAllWindows()

