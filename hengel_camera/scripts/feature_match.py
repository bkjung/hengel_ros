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

    def SIFT_KNN_matching(self, img1, img2):
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
        # cv2.imshow("white", img1)
        # cv2.waitKey(3)
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
                self.status = Trues
                print("Homography mtx M is None !!!!")
            else:
                print("sift_flann match finished")
            # else:
            #     print("Feature Match FAILED (Not enough features)")
        else:
            print("Feature Match FAILED (Empty Descriptor)")

        matchesMask = [[0,0] for i in xrange(len(matches))]
        draw_params = dict(matchColor = (0,255,0),
                        singlePointColor = (255,0,0),
                        flags = 0)
        img3 = cv2.drawMatches(img2,kp2,img1,kp1,matches[:10],None,**draw_params)
        plt.subplot(313) 
        plt.imshow(img3, cmap='gray')
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
        # print("kp1 shape")
        # print(kp1)
        # print(len(kp1))
        # print("des1 shape")
        # print(len(des1))
        # print(des1[0])
        # print(des1[0][0])
        # for i in range(len(des1)):
        #     print(des1[i])
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
            

            #match keypoints here!!!
            #1st
            matches = flann.knnMatch(des1,des2,k=2)

            # print("sift_flann 2 Time: "+str(time.time()-_time))

            #filter matched keypoints
            #2nd
            #store all the good matches as per Lowe's ratio test
            good=[]
            matchesMask = [[0,0] for i in xrange(len(matches))]
            for i,  (m,n) in enumerate(matches):
                if m.distance < 0.7*n.distance:
                    good.append(m)
                    matchesMask[i]=[1,0]

            # print(len(kp1))

            # print("abc",kp1[good[3].queryIdx].pt)


            # print("sift_flann 3 Time: "+str(time.time()-_time))

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
                    print("sift_flann match finished")
            else:
                print("FAILED (Not enough features, %d <= %d)" %(len(good), MIN_MATCH_COUNT))
        

            draw_params = dict(matchColor = (0,255,0),
                            singlePointColor = (255,0,0),
                            matchesMask = matchesMask,
                            flags = 0)
            print("debug2")
            img3 = cv2.drawMatchesKnn(img2,kp2,img1,kp1,matches,None,**draw_params)
            # img3 = cv2.drawMatchesKnn(img2,kp2,img1,kp1,matches,None,flags=2)
            print("debug3")

            # cv2.imwrite(self.folder_path+"/SIFT_FLANN_MATCH_"+time.strftime("%y%m%d_%H%M%S")+".png", img3)

            plt.subplot(313)
            plt.imshow(img3, cmap='gray')

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

    def SURF_BF_matching(self, img1, img2):
        plt.figure(1, figsize=(10, 20))
        plt.subplot(311)
        plt.imshow(img1, cmap='gray')
        plt.subplot(312)
        plt.imshow(img2, cmap='gray')

        M=None
        MIN_MATCH_COUNT=10
        _time=time.time()

        self.status = False

        #cv2 error
        # surf_detector= cv2.SURF(400, 5, 5)

        surf = cv2.xfeatures2d.SURF_create()
        # find the keypoints and descriptors with SIFT
        kp1, des1 = surf.detectAndCompute(img1, None)
        kp2, des2 = surf.detectAndCompute(img2, None)

        # create BFMatcher object
        bf = cv2.BFMatcher(cv2.NORM_L2)

        if des1 is not None and des2 is not None:
            # Match descriptors
            # matches = bf.match(des2, des1)
            matches = bf.knnMatch(des1, trainDescriptors = des2, k = 2)
            # Sort them in the order of their distance
            # matches = sorted(matches, key=lambda x:x.distance)
            
            mkp1, mkp2 = [], []
            ratio = 0.75
            for m in matches:
                if len(m) == 2 and m[0].distance < m[1].distance * ratio:
                    m = m[0]
                    mkp1.append( kp1[m.queryIdx] )
                    mkp2.append( kp2[m.trainIdx] )
            # kp_pairs = zip(mkp1, mkp2)

            # src_pts=np.float32([ kp2[m.queryIdx].pt for m in matches]).reshape(-1,1,2)
            # dst_pts=np.float32([ kp1[m.trainIdx].pt for m in matches]).reshape(-1,1,2)
            src_pts=np.float32([kp.pt for kp in mkp1])
            dst_pts=np.float32([kp.pt for kp in mkp2])
            M, mask= cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
            if M is None:
                print("Homography mtx M is None !!!!")
            else:
                self.status = True
                matchesMask = [[0,0] for i in xrange(len(matches))]
                draw_params = dict(matchColor = (0,255,0),
                                singlePointColor = (255,0,0),
                                flags = 0)
                img3 = cv2.drawMatchesKnn(img1,mkp1,img2,mkp2,matches[:10],None,**draw_params)
                plt.subplot(313) 
                plt.imshow(img3, cmap='gray')

                print("surf_bf match finished")
            # else:
            #     print("Feature Match FAILED (Not enough features)")
        else:
            print("Feature Match FAILED (Empty Descriptor)")

        file_time = time.strftime("%y%m%d_%H%M%S")
        plt.savefig(self.folder_path+"/SURF_BF_"+file_time+".png")
        plt.close("all")
        cv2.imwrite(self.folder_path+"/SUMMED_"+file_time+".png", img1)
        cv2.imwrite(self.folder_path+"/VIRTUAL_"+file_time+".png", img2)
        cv2.destroyAllWindows()
        print("FeatureMatch Saved to "+file_time)

        return M


if __name__=="__main__":
    img_virtual= cv2.imread("/home/bkjung/Pictures/virtual_B_thin.png", cv2.IMREAD_GRAYSCALE)
    img_photo= cv2.imread("/home/bkjung/Pictures/SUMMED_B.png", cv2.IMREAD_GRAYSCALE)


    app = FeatureMatch('/home/bkjung/Pictures')

    # app = FeatureMatch('/home/bkjung/Pictures')

    # app.SIFT_FLANN_matching(img_photo, img_virtual)
    app.SURF_BF_matching(img_photo, img_virtual)
    # SIFT_KNN_matching(img1, img2)
    # app.ORB_BF_matching(img_virtual, img_photo)
    # surf = cv2.xfeatures2d.SURF_create()


