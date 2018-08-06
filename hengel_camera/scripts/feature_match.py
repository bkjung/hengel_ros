#!/usr/bin/env python
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt
import numpy as np
import cv2
import time


class FeatureMatch():
    def __init__(self, _folder_path):
        self.status = False
        self.delta_x = 0.0
        self.delta_y = 0.0
        self.delta_theta = 0.0
        self.folder_path  = _folder_path

    def SIFT_KNN_matching(self, img1, img2):
        sift = cv2.xfeatures2d.SIFT_create()
        kp1, des1= sift.detectAndCompute(img1, None)
        kp2, des2= sift.detectAndCompute(img2, None)

        bf=cv2.BFMatcher()
        matches=bf.knnMatch(des2, des1, k=2)

        # Apply ratio test
        good = []
        for m, n in matches:
            if m.distance < 0.75*n.distance:
                good.append([m])

        img3= np.ndarray([])
        img3=cv2.drawMatchesKnn(img2, kp2, img1, kp1, good, img3, flags=2)
        plt.imshow(img3), plt.show()

    def ORB_BF_matching(self, img1, img2):
        orb= cv2.ORB_create(nfeatures=1500)
        # find the keypoints and descriptors with SIFT
        kp1, des1 = orb.detectAndCompute(img1, None)
        kp2, des2 = orb.detectAndCompute(img2, None)

        # create BFMatcher object
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        # Match descriptors
        matches = bf.match(des2, des1)
        # Sort them in the order of their distance
        matches = sorted(matches, key=lambda x:x.distance)

        img3=np.ndarray([])
        # Draw first 10 matches
        img3=cv2.drawMatches(img2, kp2, img1, kp1, matches[:30], img3, flags=2)

        img=np.ndarray([])
        plt.imshow(img3), plt.show()
        # img=cv2.drawKeypoints(img, keypoints, None)
        cv2.imshow("Image", img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def SIFT_FLANN_matching(self, img1, img2):
        #img1: virtual
        #img2: photo
        M=None
        _time=time.time()

        self.status = False
        sift=cv2.xfeatures2d.SIFT_create()

        # cv2.imwrite("/home/bkjung/img1.png", img1)
        # cv2.imwrite("/home/bkjung/img2.png", img2)

        # print("img1: "+str(img1.shape)+", img2: "+str(img2.shape))

        print("sift_falnn 0 Time: "+str(time.time()-_time))

        ############ Slow Part ############
        kp1, des1 = sift.detectAndCompute(img1, None)
        kp2, des2 = sift.detectAndCompute(img2, None)
        ############ Slow Part ############

        print("sift_falnn 1 Time: "+str(time.time()-_time))

        MIN_MATCH_COUNT=10
        FLANN_INDEX_KDTREE=0

        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        search_params = dict(checks=50)   # or pass empty dictionary

        flann = cv2.FlannBasedMatcher(index_params,search_params)

        matches = flann.knnMatch(des1,des2,k=2)

        print("sift_falnn 2 Time: "+str(time.time()-_time))

        #store all the good matches as per Lowe's ratio test
        good=[]
        for m,n in matches:
            if m.distance < 0.7*n.distance:
                good.append(m)

        # print(len(kp1))

        # print("abc",kp1[good[3].queryIdx].pt)

        plt.figure(1, figsize=(10, 20))
        plt.subplot(311)
        plt.imshow(img1, cmap='gray')
        plt.subplot(312)
        plt.imshow(img2, cmap='gray')

        print("sift_falnn 3 Time: "+str(time.time()-_time))

        if len(good)>MIN_MATCH_COUNT:
            print("FEATURE MATCH COUNT > MIN_MATCH_COUNT")
            print([m.queryIdx for m in good])

            # print(np.float32([ kp1[m.queryIdx].pt for m in good ]))
            src_pts=np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
            dst_pts=np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)



            print("DEBUG-0")
            M, mask= cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
            if M is None:
                print("Homography mtx M is None !!!!")
            else:
                self.status = True
                print(M)

                # print("DEBUG-1")
                # img1_homo=cv2.warpPerspective(img1, M, (1280, 1280))
                # print("DEBUG-2")
                # img2_homo=cv2.warpPerspective(img2, M, (1280, 1280))
                # print("DEBUG-3")

                # cv2.imwrite("/home/bkjung/img1_homo.png", img1_homo)
                # cv2.imwrite("/home/bkjung/img2_homo.png", img2_homo)
                # matchesMask=mask.ravel().tolist()

                # h,w = img1.shape
                # pts=np.float32([[0,0],[0,h-1],[w-1,h-1], [w-1,0]]).reshape(-1,1,2)
                # dst=cv2.perspectiveTransform(pts,M)


                # Need to draw only good matches, so create a mask
                matchesMask = [[0,0] for i in xrange(len(matches))]
                # ratio test as per Lowe's paper
                for i,(m,n) in enumerate(matches):
                    if m.distance < 0.7*n.distance:
                        matchesMask[i]=[1,0]
                draw_params = dict(matchColor = (0,255,0),
                                singlePointColor = (255,0,0),
                                matchesMask = matchesMask,
                                flags = 0)
                img3 = cv2.drawMatchesKnn(img2,kp2,img1,kp1,matches,None,**draw_params)

                print("DEBUG-4")

                # cv2.imwrite(self.folder_path+"/SIFT_FLANN_MATCH_"+time.strftime("%y%m%d_%H%M%S")+".png", img3)

                plt.subplot(313)
                plt.imshow(img3, cmap='gray')
        else:
            print("Feature Match FAILED")

        plt.savefig(self.folder_path+"/SIFT_FLANN_"+time.strftime("%y%m%d_%H%M%S")+".png")

        # plt.draw()
        # plt.pause(0.00000000001)

        print("sift_falnn 4 Time: "+str(time.time()-_time))

        return M

if __name__=="__main__":
    img_virtual= cv2.imread("/home/bkjung/demo_1280.png", cv2.IMREAD_GRAYSCALE)
    img_photo= cv2.imread("/home/bkjung/LABS_6.png", cv2.IMREAD_GRAYSCALE)

    app = FeatureMatch('/home/bkjung')

    app.SIFT_FLANN_matching(img_photo, img_virtual)
    # SIFT_KNN_matching(img1, img2)
    # ORB_BF_matching(img1, img2)

    # surf = cv2.xfeatures2d.SURF_create()


