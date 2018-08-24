#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from std_msgs.msg import Bool
import cv2
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt
import message_filters
import time


class Undistort():
    def __init__(self):
        rospy.init_node('undistortion', anonymous=True)

        self.is_initiated=False
        self.sum_pub=rospy.Publisher('/summed_image/compressed', CompressedImage, queue_size=3)
        self.cam_activator_pub=rospy.Publisher('/initiator', Bool, queue_size=1)
        self.rate=rospy.Rate(10)

        self.homo1= np.array([[-1.67130692e-01,  5.99743615e+00, -5.29293784e+02],
            [-2.40710331e+00,  4.76090267e+00,  1.55119117e+03],
            [-2.21043846e-04,  7.30990701e-03,  1.00000000e+00]])
        self.homo2= np.array([[-2.36547415e+00,  4.44589419e+00,  1.65240597e+03],
            [-1.11902669e-02,  2.88055561e+00,  2.03902843e+03],
            [-5.36747061e-06,  6.70728023e-03,  1.00000000e+00]])
        self.homo3= np.array([[ 2.55132452e-01,  9.82372337e+00,  4.09600642e+03],
            [ 6.45201391e+00,  1.30885948e+01, -1.66201249e+03],
            [ 3.88669729e-04,  2.00259308e-02,  1.00000000e+00]])
        self.homo4= np.array([[ 2.57420243e+00,  5.85803823e+00, -4.05003547e+02],
            [-1.15034759e-01,  7.22474987e+00, -7.29546146e+02],
            [-1.92621119e-04,  8.88963498e-03,  1.00000000e+00]])

        self.callback1=message_filters.Subscriber('/genius1/compressed', CompressedImage)
        self.callback2=message_filters.Subscriber('/genius2/compressed', CompressedImage)
        self.callback3=message_filters.Subscriber('/genius3/compressed', CompressedImage)
        self.callback4=message_filters.Subscriber('/genius4/compressed', CompressedImage)

        self.ts=message_filters.ApproximateTimeSynchronizer([self.callback1, self.callback2, self.callback3, self.callback4], 10, 0.1, allow_headerless=True)
        self.ts.registerCallback(self.sync_callback)

        self.initiate_camera()
        rospy.spin()

    def initiate_camera(self):
        while not self.is_initiated:
            msg=Bool()
            msg.data=True
            self.cam_activator_pub.publish(msg)


    def sync_callback(self, _img1, _img2, _img3, _img4):
        _time=time.time()
        bridge=CvBridge()
        self.is_initiated=True
        img1 = cv2.warpPerspective(bridge.compressed_imgmsg_to_cv2(_img1), self.homo1, (1280,1280))
        img2 = cv2.warpPerspective(bridge.compressed_imgmsg_to_cv2(_img2), self.homo2, (1280,1280))
        img3 = cv2.warpPerspective(bridge.compressed_imgmsg_to_cv2(_img3), self.homo3, (1280,1280))
        img4 = cv2.warpPerspective(bridge.compressed_imgmsg_to_cv2(_img4), self.homo4, (1280,1280))

        im_mask_inv1, im_mask1=self.find_mask(img1)
        im_mask_inv3, im_mask3=self.find_mask(img3)
        _, im_mask2=self.find_mask(img2)
        _, im_mask4=self.find_mask(img4)

        print(im_mask1)
        img_white=np.full((1280, 1280,3), 255)

        im_mask13=cv2.bitwise_and(np.array(im_mask1), np.array(im_mask3))
        im_mask24=cv2.bitwise_and(np.array(im_mask2), np.array(im_mask4))
        im_mask1234=cv2.bitwise_and(im_mask13, im_mask24)

        # # cv2.bitwise_and(img1, img1_masked, mask= im_mask_inv1)
        # cv2.bitwise_and(img2, img2_masked, mask=im_mask1234)
        # # img2_masked=np.multiply(np.multiply(img2, im_mask1), im_mask3)
        # cv2.bitwise_and(img3, img3_masked, mask=im_mask1234)
        # # img3_masked=np.multiply(img3, im_mask_inv3)
        # cv2.bitwise_and(img4, img4_masked, mask=im_mask1234)
        # # cv2.bitwise_and(img4_masked, img4_masked, mask=im_mask3)

        # # img_white_masked=np.multiply(np.multiply(np.multiply(np.multiply(img_white, im_mask1),im_mask3), im_mask2), im_mask4)
        img_white_masked=np.multiply(img_white, im_mask1234)
        img2_masked=np.multiply(img2, im_mask13)
        img4_masked=np.multiply(img4, im_mask13)
        summed_image= img1+img2_masked+img3+img4_masked+img_white_masked

        print("4: "+str(time.time()-_time))

        bridge=CvBridge()
        summed_msg=bridge.cv2_to_compressed_imgmsg(summed_image)
        self.sum_pub.publish(summed_msg)


    def find_mask(self, img):
        _time=time.time()
        black_range1=np.array([0,0,0])
        time1=time.time()-_time
        # im_mask=(cv2.inRange(img, black_range1, black_range1)).astype('bool')
        im_mask=(cv2.inRange(img, black_range1, black_range1))
        time2=time.time()-_time
        im_mask=np.dstack((im_mask, im_mask, im_mask))
        time3=time.time()-_time
        # im_mask_inv=(1-im_mask).astype('bool')
        im_mask_inv=(1-im_mask)
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

def homography_matrix(index):
    robotPtsArr=[]
    imgPtsArr=[]
    #genius1
#     robotPtsArr.append([[-20,15],[-20,25],
#     [-30,25],[-30,35],[-40,35],
# [-40,25],[-40,15],[-40,5],
# [-50,15],[-50,25],
# [-60,35],[-60,45],[-60,15],
# [-50,5],
# [-35,-10]
# ])
#     #genius1
#     imgPtsArr.append([[511.7,524.7],
# [625,528.7],
# [591.2,414.5],
# [687.3,418.7],
# [649.3,336.9],
# [567.5,335.1],
# [486.5,333.3],
# [406.4,330.2],
# [479,272.5],
# [550.2,274.2],
# [599.5,228.9],
# [663.4,230.6],
# [473,226.1],
# [408.1,271.4],
# [275.5,364.7]
# ])

#     #genius2
#     robotPtsArr.append([[-15,-20],[-5,-20],
# [-5,-30],[-15,-30],[-25,-30],[-35,-30],
# [-35,-20],
# [-35,-40],
# [5,-30],[15,-30],
# [15,-20],[35,-20],
# [25,-30],
# [25,-40],
# [-5,-50],[5,-50],[-25,-50]
# ])
#     #genius2
#     imgPtsArr.append([[555,455.8],
#         [456.8,455],
#         [450.2,367.9],
#         [535,369],
#         [618.2,369.9],
#         [701.5,371.5],
#         [749.7,458.5],
#         [665,305.9],
#         [364,366],
#         [278.9,364.5],
#         [256.4,453.1],
#         [52.1,450],
#         [191,363.1],
#         [219.9,296.4],
#         [441.6,250.7],
#         [375.1,248.5],
#         [572,254]
#         ])
#     #genius3
#     robotPtsArr.append([
#         [20,35], [20,25],
#     [30,45],[30,35],[30,25],[30,5],
#     [40,35],[40,25],
#     [45,35],[45,25],[45,20],[45,15],[45,10],[45,0],[45,-10],[45,-20],
#     [55,25],[55,-10]
#     ])
#     #genius3
#     imgPtsArr.append([
#         [71.8,556.5], [183.3,556.9],
#     [24.4,447.3],[120,445.98],[216.7,445.6],[406.7, 444.7],
#     [157.5,365.5],[239.8,364.9],
#     [172,332.9],[249.6,331.8],[287,332.4],[326.5,332],[365,331.5],[442.1,330.6],[519.5,331.1],[597.5,331.2],
#     [264.7,277.1],[505.4,277.2]
#     ])




#     #genius4
#     robotPtsArr.append([[-30,35],[-20,35],[-15,35],[-10,35],[10,35],[20,35],[30,35],
#     [-40,45],[-30,45],[-20,45],[-10,45],[20,45],[30,45],
#     [-40,55],[-20,55],[-10,55],[20,55],[30,55],
#     [-60,65],[-40,65],[10,65]])
#     #genius4
#     imgPtsArr.append([[144.5,513.6],[238.5,514.5],[284.9,514.5],[332.1,514.8],[525.3,514.6],[624.0,516.8], [725.9,517.1],
#     [103.4,431.1],[184.5,430.9],[265.2,430.6],[346.3,431.0],[595.4,431.5],[681.6,431.7],
#     [144.2,370],[284.7,368.9],[356.9,368.4],[576,369],[650.5,367.2],
#     [50.7,322.4],[175.8,321.1],[493.8,320.2]])
#     #pi_left
#     robotPtsArr.append([[10,25],
#         [20,25],
#         [20,35],
#         [20,45],
#         [10,35],
#         [0,25],
#         [-5,30],
#         [10,45],
#         [0,45],
#         [-10,45],
#         [10,30],
#         [15,30],
#         [5,30],
#         [20,30],
#         [15,40]
#         ]
#         )
#     #pi left
#     imgPtsArr.append([[573.5,376.5],
#         [899.5,380.8],
#         [787.8,169.8],
#         [720.5,48.5],
#         [547.5,162.2],
#         [229.5,372.5],
#         [121.2,246.5],
#         [531.2,42.2],
#         [336.2,37.2],
#         [134.8,26.2],
#         [558.2,251.8],
#         [698.5,255.0],
#         [413.9,248.9],
#         [834.7,258.0],
#         [643.5,97.8]
#         ])
#     #pi_right
#     robotPtsArr.append([[-20,25],[-15,25],[-10,25],[-5,25],[0,25],
#     [-5,30],
#     [-20,35],[-15,35],[-10,35],[-5,35],[0,35],[10,35],
#     [-20,45],[0,45]])
#     objPts = [[[point_r[1]*(5.0)+640.0, point_r[0]*(5.0)+640.0]  for point_r in robotPts]  for robotPts in robotPtsArr]

#     #pi_right
#     imgPtsArr.append([[1.5,373],[177.5,368],[349,360.5],[525.3,354.4],[694.5,348.7],
#     [511.5,229.2],
#     [120,152],[246,149.3],[374.3, 140.7],[501.5,139.8],[623.8,136.5],[862.2,131.5],
#     [184.3,23.7],[582,18]])

#     img=[[626.7,2093.3], [653, 2090.3],  [553,2207],
#                     [827,2232.7]
#                     # ,   [795.3,2239.3], [853,2285],
#                     # [853,2254.7], [553.7,2206.3],[775,2346.7],[779.7,2378],      
#                     # [941,2461],    
#                     # [680, 2054.7],[775,2359],
#                     #  [286,2220], [2068,488],[1444,1132]
#                     ]
#     img_resized=[ [a[0]/2560*1280 ,a[1]/2560*1280 ] for a in img]
#     imgPtsArr.append(img_resized)

#     obj=[[786.3,2169.3], [784.7, 2166.7], [747,2260],
#                     [980.3, 2232.3]
#                     # ,[939,2244.7], [1019,2283],
#                     # [1004,2244],  [743, 2259],   [953,2358.3],[968.3,2397.3],
#                     # [1129.3,2452.3],
#                     # [659,2425.2],[965.7,1841.7],
#                     # [1370,962], [1580,1504], [1330,1630] ##OUTLIER
#                     ]
#     obj_resized=[[b[0]/2560*1280, b[1]/2560*1280] for b in obj]
#     objPts.append(obj_resized)







    #genius1
    robotPtsArr.append([
[75,-80],[55,-80],[55,-100],[35,-80],[15,-80],[65,-60],[45,-60],[25,-60],[5,-60],[15,-50],[35,-50],[55,-50],[65,-40],[45,-40],[25,-40],[45,-30],[45,-20],[10,25],[20,25],[30,25],[40,25],[50,25],[60,35],[50,35],[30,35],[30,45],[40,45],[10,55],[30,55],[40,55],[10,65],[20,65],[50,65],[30,75]
])
    #genius1
    imgPtsArr.append([
[670,264],[696,330],[776,332],[728,410],[768,513],[607,295],[627,367],[652,456],[658,572],[618,509],[596,407],[578,392],[533,293],[545,366],[559,457],[503,365],[462,365],[239,534],[252,478],[261,427],[270,382],[277,343],[247,307],[236,343],[215,428],[171,428],[184,382],[87,534],[124,427],[140,382],[34,534],[58,477],[115,342],[35,426]
])

    #genius2
    robotPtsArr.append([
[-5,-100],[-25,-100],[75,-100],[-65,-80],[-45,-80],[-25,-80],[-5,-80],[35,-80],[75,-80],[-65,-70],[-5,-70],[45,-70],[75,-60],[45,-60],[-5,-60],[-55,-60],[-85,-60],[-65,-50],[-5,-50],[25,-50],[45,-50],[65,-50],[75,-40],[45,-40],[25,-40],[-25,-40],[-55,-40],[-55,-30],[65,-30]
] )
    #genius2
    imgPtsArr.append([
[430,285],[495,287],[162,282],[651,344],[579,344],[505,343],[431,342],[283,340],[128,339],[667,379],[432,376],[232,373],[82,414],[217,414],[433,415],[644,418],[771,418],[709,463],[434,461],[294,460],[200,460],[104,460],[19,517],[180,516],[284,516],[535,516],[684,518],[708,580],[38,582]
])


    #genius3
    robotPtsArr.append([[-10,65], [-30, 65], [-60,65], [-80, 65],
[-20, 55], [-30, 55], [-50,55], [-80, 55],
[-10,35], [-30, 35], [-50,35], [-80, 35],
[-40, 25] , [-60, 25],
[-40, 15], [-50, 15], [-70, 15],
[ -35, -30], [-45, -30], [-55, -30], [-85, -30], [-105, -30],
[ -25 ,-40], [-35, -40], [-45, -40], [-55, -40],
[-15, -60], [-25, -60], [-45, -60], [-55, -60], [-65, -60],
[-25, -80], [-45, -80], [-65, -80],
[-65, -100]]
)
    #genius3
    imgPtsArr.append([[775.3, 569], [729.3, 460.3], [676.2, 338.8], [648.8, 275],
[702.7, 513], [683.7, 462], [650,375.3], [613.8,276],
[604, 513.3], [590, 462.3], [567, 375.7], [542.8,276.8],
[534.8, 416.5], [520.2, 339],
[491, 417], [485.8, 376.5], [475.8, 307.5], 
[294.8, 441.8], [300,399.2], [304.2, 360.5], [314.2, 266.5], [318.8, 215.8],
[241.8, 490.5], [249.8, 442.7], [257.6, 399.5], [264, 361],
[130, 548], [146.8, 492.8], [173.8, 401], [185, 362.3], [232, 327.2],
[51.7, 494.3], [88.7, 402.7], [118.3, 329.3], 
[41.7, 330.3]])




    #genius4
    robotPtsArr.append([[70,25], [70,55], [70, 95],
[60,55],
[50,25], [50,35], [50,45], [50,75], [50, 105],
[30,35], [30, 55],
[20, 35], [20, 45], [20,105],
[0,35], [0,45],
[-10, 45], [-10,55], [-10,105],
[-20, 55], [-20, 95],
[-30, 35], [-30, 55],
[-50, 25], [-50, 35], [-50, 95],
[-60,45], [-60, 55], [-60, 65], [-60, 85],
[-70,25], [-70, 45], [-70, 95],
[-80, 35], [-80,55],
[-90, 55], [-90, 95])
    #genius4
    imgPtsArr.append([[777,586.7], [704.2, 451.5], [643, 339.5],
[664,452.2],
[676, 587], [655.8, 535.2], [637.8, 490.4], [611.1, 419.6], [572.9,319.1],
[563.2,535.2], [544,452.8],
[517.8, 535.8], [511, 492], [484.2, 320.4],
[428, 535.8], [428.3, 492.3],
[385.3, 493], [387.5, 453.8], [395.6, 320.5],
[348.9, 454.5], [363.6, 342.4],
[294, 537], [310.2, 455.2],
[184.8, 589.2], [203, 538.2], [ 272, 344.8], 
[177.2, 494.2], [193, 456.8],[208, 423.2], [231.5, 367.2], 
[88.2, 590.2], [134.5, 494.5], [210.2, 344.2], 
[68.2, 539], [115.2, 456.8], 
[74.8, 457.2], [148, 344]]
)

    objPts = [[[(point_r[1]+9.33)*(4)+640.0, point_r[0]*(4)+640.0]  for point_r in robotPts]  for robotPts in robotPtsArr]

    homography, status=cv2.findHomography(np.array(imgPtsArr[index-1]), np.array(objPts[index-1],np.float32), cv2.RANSAC, 10)

    path='/home/mjlee/homo/'+str(index)
    img= cv2.imread(path+'/photo.png')

    print("====%d homography====" %(index))

    print(homography)

    img_homography = cv2.warpPerspective(img, homography, (1280,1280))
    cv2.imwrite(path+'/photo_homography.png', img_homography)

    # img=cv2.imread('/home/mjlee/')


if __name__=="__main__":
    homography_matrix(1)
    homography_matrix(2)
    homography_matrix(3)
    homography_matrix(4)
    # Undistort()
    # cv2.destroyAllWindows()

