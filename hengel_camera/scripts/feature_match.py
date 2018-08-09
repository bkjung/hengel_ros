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
        self.folder_path  = _folder_path

    def SIFT_BF_matching(self, img1, img2):
        sift = cv2.xfeatures2d.SIFT_create()
        kp1, des1= sift.detectAndCompute(img1, None)
        kp2, des2= sift.detectAndCompute(img2, None)

        bf=cv2.BFMatcher()
        matches=bf.knnMatch(des2, des1, k=2)

        # Apply ratio test
        good = []
        for m, n in matches:
            if m.distance < 0.7*n.distance:
                good.append([m])

        img3= np.ndarray([])
        img3=cv2.drawMatchesKnn(img2, kp2, img1, kp1, good, img3, flags=2)
        plt.imshow(img3), plt.show()

    def ORB_BF_matching(self, img1, img2):
        plt.figure(1, figsize=(10, 20))
        plt.subplot(311)
        plt.imshow(img1, cmap='gray')
        plt.subplot(312)
        plt.imshow(img2, cmap='gray')

        M=None
        MIN_MATCH_COUNT=10
        _time=time.time()

        self.status = False

        orb= cv2.ORB_create(nfeatures=1500)
        # find the keypoints and descriptors with SIFT
        kp1, des1 = orb.detectAndCompute(img1, None)
        kp2, des2 = orb.detectAndCompute(img2, None)

        # create BFMatcher object
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        if des1 is not None and des2 is not None:
            # Match descriptors
            matches = bf.match(des2, des1)
            # Sort them in the order of their distance
            matches = sorted(matches, key=lambda x:x.distance)
            src_pts=np.float32([ kp2[m.queryIdx].pt for m in matches]).reshape(-1,1,2)
            dst_pts=np.float32([ kp1[m.trainIdx].pt for m in matches]).reshape(-1,1,2)
            M, mask= cv2.findHomography(dst_pts, src_pts, cv2.RANSAC, 5.0)
            if M is None:
                print("Homography mtx M is None !!!!")
            else:
                self.status = True
                matchesMask = [[0,0] for i in xrange(len(matches))]
                draw_params = dict(matchColor = (0,255,0),
                                singlePointColor = (255,0,0),
                                flags = 0)
                img3 = cv2.drawMatches(img2,kp2,img1,kp1,matches[:10],None,**draw_params)
                plt.subplot(313) 
                plt.imshow(img3, cmap='gray')

                print("sift_flann match finished")
            # else:
            #     print("Feature Match FAILED (Not enough features)")
        else:
            print("Feature Match FAILED (Empty Descriptor)")

        file_time = time.strftime("%y%m%d_%H%M%S")
        plt.savefig(self.folder_path+"/SIFT_FLANN_"+file_time+".png")
        plt.close("all")
        cv2.imwrite(self.folder_path+"/SUMMED_"+file_time+".png", img1)
        cv2.imwrite(self.folder_path+"/VIRTUAL_"+file_time+".png", img2)
        cv2.destroyAllWindows()
        print("FeatureMatch Saved to "+file_time)

        return M


    def SIFT_FLANN_matching(self, img1, img2):
        plt.figure(1, figsize=(10, 20))
        plt.subplot(311)
        # cv2.imshow("white", img1)
        # cv2.waitKey(3)
        plt.imshow(img1, cmap='gray')
        plt.subplot(312)
        plt.imshow(img2, cmap='gray')

        M=None
        _time=time.time()

        self.status = False

        sift=cv2.xfeatures2d.SIFT_create()
        ############ Slow Part ############
        kp1, des1 = sift.detectAndCompute(img1, None)
        kp2, des2 = sift.detectAndCompute(img2, None)
        ############ Slow Part ############

        # orb=cv2.ORB_create(nfeatures=1500)
        # kp1,des1=orb.detectAndCompute(img1, None)
        # kp2, des2=orb.detectAndCompute(img2, None)

        print("feature detection Time: "+str(time.time()-_time))

        # MIN_MATCH_COUNT=10
        MIN_MATCH_COUNT=5
        FLANN_INDEX_KDTREE=0

        # index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        # search_params = dict(checks=50)   # or pass empty dictionary

        FLANN_INDEX_LSH=5
        index_params=dict(algorithm= FLANN_INDEX_LSH,
                        table_number = 6,
                        key_size=12,
                        multi_probe_level =1 )
        search_params = dict(checks=50)   # or pass empty dictionary

        flann = cv2.FlannBasedMatcher(index_params,search_params)

        if des1 is not None and des2 is not None:
            print("debug0")
            matches = flann.knnMatch(des1,des2,k=2)

            # print("sift_flann 2 Time: "+str(time.time()-_time))

            #store all the good matches as per Lowe's ratio test
            good=[]
            matchesMask = [[0,0] for i in xrange(len(matches))]
            for i,  (m,n) in enumerate(matches):
                if m.distance < 0.7*n.distance:
                    good.append(m)
                    matchesMask[i]=[1,0]

            if len(good)>MIN_MATCH_COUNT:
                # print("FEATURE MATCH COUNT > MIN_MATCH_COUNT")
                # print([m.queryIdx for m in good])

                # print(np.float32([ kp1[m.queryIdx].pt for m in good ]))
                src_pts=np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
                dst_pts=np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

                M, mask= cv2.findHomography(dst_pts, src_pts, cv2.RANSAC, 5.0)
                if M is None:
                    print("FAILED (Homography mtx M is None)")
                else:
                    self.status = True
                    draw_params = dict(matchColor = (0,255,0),
                                    singlePointColor = (255,0,0),
                                    matchesMask = matchesMask,
                                    flags = 0)
                    print("debug2")
                    try:
                        img3 = cv2.drawMatchesKnn(img2,kp2,img1,kp1,matches,None,**draw_params)
                        # img3 = cv2.drawMatchesKnn(img2,kp2,img1,kp1,matches,None,flags=2)
                        print("debug3")

                        # cv2.imwrite(self.folder_path+"/SIFT_FLANN_MATCH_"+time.strftime("%y%m%d_%H%M%S")+".png", img3)

                        plt.subplot(313)
                        plt.imshow(img3, cmap='gray')

                        print("sift_flann match finished")
                        
                        # plt.draw()
                        # plt.pause(0.00000000001)
                    except Exception as e:
                        print(e)
                        sys.exit("debug2-1")
            else:
                print("FAILED (Not enough features, %d <= %d)" %(len(good), MIN_MATCH_COUNT))
        else:
            print("FAILED (Empty Descriptor)")

        # plt.draw()
        # plt.pause(0.00000000001)
        # print("sift_flann 4 Time: "+str(time.time()-_time))
        file_time = time.strftime("%y%m%d_%H%M%S")
        plt.savefig(self.folder_path+"/SIFT_FLANN_"+file_time+".png")
        print("FeatureMatch Saved to "+file_time)

        plt.close("all")

        return M

if __name__=="__main__":
    img_virtual= cv2.imread("/home/bkjung/VIRTUAL_PROCESSED.png", cv2.IMREAD_GRAYSCALE)
    img_photo= cv2.imread("/home/bkjung/SUMMED_PROCESSED_CONNECTED.png", cv2.IMREAD_GRAYSCALE)


    app = FeatureMatch('/home/bkjung')


    app.SIFT_FLANN_matching(img_photo, img_virtual)
    # SIFT_KNN_matching(img1, img2)
    # ORB_BF_matching(img1, img2)
    # surf = cv2.xfeatures2d.SURF_create()


