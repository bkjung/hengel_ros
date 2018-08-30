#!/usr/bin/env python
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt
from math import sqrt, pow, atan2, cos, sin
import numpy as np
import cv2
from numpy.linalg import inv
import sys
import time


class FeatureMatch():
    def __init__(self, _folder_path, _option_without_save):
        self.status = False
        self.folder_path  = _folder_path
        self.option_without_save = _option_without_save

    def SIFT_BF_matching(self, img1, img2,img1_copy, img2_copy):
        if not self.option_without_save:
            plt.figure(1, figsize=(10,20))
            plt.subplot(221)
            plt.imshow(img1, cmap='gray')
            plt.subplot(222)
            plt.imshow(img2, cmap='gray')

        M=None
        _time=time.time()

        self.status=False

        sift = cv2.xfeatures2d.SIFT_create()

        kp1, des1= sift.detectAndCompute(img1, None)
        kp2, des2= sift.detectAndCompute(img2, None)

        print("feature detection time: "+str(time.time()-_time))

        MIN_MATCH_COUNT=5

        bf=cv2.BFMatcher(cv2.NORM_L2)
        file_time = time.strftime("%y%m%d_%H%M%S")

        if des1 is not None and des2 is not None:
            matches=bf.knnMatch(des2, des1, k=2)

            good=[]
            ratio= 0.7
            matchesMask=[[0,0] for i in range(len(matches))]
            for i, (m,n) in enumerate(matches):
                if m.distance < ratio* n.distance:
                    good.append(m)
                    matchesMask[i]=[1,0]
            if len(good)>MIN_MATCH_COUNT:
                src_pts=np.float32([kp2[m.queryIdx].pt for m in good]).reshape(-1,1,2)
                dst_pts=np.float32([kp1[m.trainIdx].pt for m in good]).reshape(-1,1,2)

                H = cv2.estimateRigidTransform(dst_pts, src_pts, False)

                if H is None:
                    print("FAILED (Homography mtx M is None")
                else:
                    M=np.eye(3)
                    M[:2]=H
                    scale= sqrt(pow(H[0][0],2)+pow(H[0][1], 2))
                    if scale <=0.9 or scale>=1.1:
                        print("FAILED (scale error)")
                    elif abs(atan2(M[0][1],M[0][0])) >= 0.3:
                        print("FAILED (angel error")
                    else:
                        self.status=True
                        print("sift_bf match finished")
                        print(M)
                        img4=cv2.warpPerspective(img1, M, (1280,1280))

                        if not self.option_without_save:
                            plt.subplot(224)
                            plt.imshow(img4, cmap='gray')

            else:
                print("FAILED (Not enough features, %d <= %d)" %(len(good), MIN_MATCH_COUNT))

            draw_params = dict(matchColor = (0,255,0),
                                singlePointColor = (255,0,0),
                                matchesMask = matchesMask,
                                flags = 0)
            img3 = cv2.drawMatchesKnn(img2_copy,kp2,img1_copy,kp1,matches,None,**draw_params)

            if not self.option_without_save:
                cv2.imwrite(self.folder_path+"/MATCH_"+file_time+".png", img3)
                plt.subplot(223)
                plt.imshow(img3, cmap='gray')

        else:
            print("FAILED (Empty Descriptor)")

        if not self.option_without_save:
            plt.savefig(self.folder_path+"/SIFT_BF_"+file_time+".png")

        if not self.option_without_save:
            # cv2.imwrite(self.folder_path+"/SUMMED_IMAGE_"+file_time+".png", img1)
            # cv2.imwrite(self.folder_path+"/ VIRTUAL_IMAGE_"+file_time+".png", img2)
            pass

        cv2.destroyAllWindows()
        print("FeatureMatch Saved to "+file_time)

        if not self.option_without_save:
            plt.close("all")

        return M

    def ORB_BF_matching(self, img1, img2):
        plt.figure(1, figsize=(10, 20))
        plt.subplot(221)
        plt.imshow(img1, cmap='gray')
        plt.subplot(222)
        plt.imshow(img2, cmap='gray')

        M=None
        MIN_MATCH_COUNT=5
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
            # M, mask= cv2.findHomography(dst_pts, src_pts, cv2.RANSAC, 5.0)
            H=cv2.estimateRigidTransform(dst_pts, src_pts, False)
            M=np.eye(3)
            M[:2]=H

            if H is None:
                print("Homography mtx M is None !!!!")
            else:
                self.status = True
                print(M)
                print("sift_flann match finished")
                img4= cv2.warpPerspective(img1, M, (1280,1280))
                plt.subplot(224)
                plt.imshow(img4, cmap='gray')

        else:
            print("Feature Match FAILED (Empty Descriptor)")

        matchesMask = [[0,0] for i in range(len(matches))]
        draw_params = dict(matchColor = (0,255,0),
                        singlePointColor = (255,0,0),
                        flags = 0)
        img3 = cv2.drawMatches(img2,kp2,img1,kp1,matches[:10],None,**draw_params)
        plt.subplot(223)
        plt.imshow(img3, cmap='gray')
        file_time = time.strftime("%y%m%d_%H%M%S")
        plt.savefig(self.folder_path+"/ORB_BF_"+file_time+".png")
        plt.close("all")

        if not self.option_without_save:
            cv2.imwrite(self.folder_path+"/SUMMED_"+file_time+".png", img1)
            cv2.imwrite(self.folder_path+"/VIRTUAL_"+file_time+".png", img2)
        cv2.destroyAllWindows()
        print("FeatureMatch Saved to "+file_time)

        return M

        # if des1 is not None and des2 is not None:
        #     matches=bf.knnMatch(des2, des1, k=2)

        #     good=[]
        #     ratio= 0.7

        #     matchesMask=[[0,0] for i in range(len(matches))]
        #     for i, (m,n) in enumerate(matches):
        #         if m.distance < 0.7* n.distance:
        #             good.append(m)
        #             matchesMask[i]=[1,0]
        #     if len(good)>MIN_MATCH_COUNT:
        #         src_pts=np.float32([kp2[m.queryIdx].pt for m in good]).reshape(-1,1,2)
        #         dst_pts=np.float32([kp1[m.trainIdx].pt for m in good]).reshape(-1,1,2)

        #         # M, mask= cv2.findHomography(dst_pts, src_pts, cv2.RANSAC, 5.0)
        #         H = cv2.estimateRigidTransform(dst_pts, src_pts, False)
        #         M=np.eye(3)
        #         M[:2]=H

        #         if H is None:
        #             print("FAILED (Homography mtx M is None")
        #         else:
        #             self.status=True
        #             print("sift_flann match finished")
        #             # print(M)
        #             img4=cv2.warpPerspective(img1, M, (1280,1280))

        #             plt.subplot(224)
        #             plt.imshow(img4, cmap='gray')

        #     else:
        #         print("FAILED (Not enough features, %d <= %d)" %(len(good), MIN_MATCH_COUNT))

        #     draw_params = dict(matchColor = (0,255,0),
        #                         singlePointColor = (255,0,0),
        #                         matchesMask = matchesMask,
        #                         flags = 0)
        #     img3 = cv2.drawMatchesKnn(img2,kp2,img1,kp1,matches,None,**draw_params)

        #     plt.subplot(223)
        #     plt.imshow(img3, cmap='gray')

        # else:
        #     print("FAILED (Empty Descriptor)")

        # file_time = time.strftime("%y%m%d_%H%M%S")
        # plt.savefig(self.folder_path+"/SIFT_BF_"+file_time+".png")
        # cv2.imwrite(self.folder_path+"/SUMMED_IMAGE_"+file_time+".png", img1)
        # cv2.imwrite(self.folder_path+"/ VIRTUAL_IMAGE_"+file_time+".png", img2)
        # cv2.destroyAllWindows()
        # print("FeatureMatch Saved to "+file_time)

        # plt.close("all")

        # return M


    def SIFT_FLANN_matching(self, img1, img2):
        plt.figure(1, figsize=(10, 20))
        plt.subplot(221)
        # cv2.imshow("white", img1)
        # cv2.waitKey(3)
        plt.imshow(img1, cmap='gray')
        plt.subplot(222)
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
            matchesMask = [[0,0] for i in range(len(matches))]
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
                src_pts=np.float32([ kp2[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
                dst_pts=np.float32([ kp1[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

                M, mask= cv2.findHomography(dst_pts, src_pts, cv2.RANSAC, 5.0)
                if M is None:
                    print("FAILED (Homography mtx M is None)")
                else:
                    self.status = True
                    print("sift_flann match finished")
                    print(M)
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

            plt.subplot(223)
            plt.imshow(img3, cmap='gray')

            img4=cv2.warpPerspective(img1, M, (1280,1280))
            plt.subplot(224)
            plt.imshow(img4, cmap='gray')

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

    def SURF_BF_matching(self, img1, img2, img1_marked, img2_marked):
        #img1: summed image
        #img2: virtual map
        plt.figure(1, figsize=(10, 20))
        plt.subplot(221)
        plt.imshow(img1, cmap='gray')
        plt.subplot(222)
        plt.imshow(img2, cmap='gray')

        M=None
        MIN_MATCH_COUNT=5
        _time=time.time()

        self.status = False

        #cv2 error
        surf = cv2.xfeatures2d.SURF_create()

        # find the keypoints and descriptors with SIFT
        kp1, des1 = surf.detectAndCompute(img1, None)
        kp2, des2 = surf.detectAndCompute(img2, None)

        # create BFMatcher object
        bf = cv2.BFMatcher(cv2.NORM_L2)
        file_time = time.strftime("%y%m%d_%H%M%S")

        if des1 is not None and des2 is not None:
            matches=bf.knnMatch(des2, des1, k=2)

            good=[]

            matchesMask=[[0,0] for i in range(len(matches))]
            for i, (m,n) in enumerate(matches):
                if m.distance < 0.5* n.distance:
                    good.append(m)
                    matchesMask[i]=[1,0]
            if len(good)>MIN_MATCH_COUNT:
                src_pts=np.float32([kp2[m.queryIdx].pt for m in good]).reshape(-1,1,2)
                dst_pts=np.float32([kp1[m.trainIdx].pt for m in good]).reshape(-1,1,2)

                M, mask= cv2.findHomography(dst_pts, src_pts, cv2.RANSAC, 5.0)

                # H = cv2.estimateRigidTransform(dst_pts, src_pts, False)
                # print(H)
                # M = np.eye(3)
                # M[:2]=H
                if M is None:
                # if H is None:
                    print("FAILED (Homography mtx M is None")
                else:
                    self.status=True
                    print("surf_bf match finished")
                    print(M)
                    img4 = cv2.warpPerspective(img1_marked, M,(1280, 1280))
                    if not self.option_without_save:
                        img5= cv2.warpPerspective(img2_marked, inv(M), (1280,1280))
                        cv2.imwrite(self.folder_path+"/WARPED_IMAGE_"+file_time+".png", img4)
                        cv2.imwrite(self.folder_path+"/WARPED_IMAGE_2_"+file_time+".png", img5)

                    plt.subplot(224)
                    plt.imshow(img4, cmap='gray')

            else:
                print("FAILED (Not enough features, %d <= %d)" %(len(good), MIN_MATCH_COUNT))

            draw_params = dict(matchColor = (0,255,0),
                                singlePointColor = (255,0,0),
                                matchesMask = matchesMask,
                                flags = 0)
            img3 = cv2.drawMatchesKnn(img2_marked,kp2,img1_marked,kp1,matches,None,**draw_params)
            if not self.option_without_save:
                cv2.imwrite(self.folder_path+"/MATCH_"+file_time+".png", img3)

            plt.subplot(223)
            plt.imshow(img3, cmap='gray')

        else:
            print("FAILED (Empty Descriptor)")

        plt.savefig(self.folder_path+"/SURF_BF_"+file_time+".png")
        # if not self.option_without_save:
            # cv2.imwrite(self.folder_path+"/SUMMED_IMAGE_"+file_time+".png", img1)
            # cv2.imwrite(self.folder_path+"/ VIRTUAL_IMAGE_"+file_time+".png", img2)
        cv2.destroyAllWindows()
        print("FeatureMatch Saved to "+file_time)

        plt.close("all")

        return M

    def relocalization(self, homography):
        self.pixMetRatio=250
        self.mid_real_photo_x=640
        self.mid_real_photo_y=640
        # self.mid_real_photo_x=0
        # self.mid_real_photo_y=0
        self.current_mid_predict_canvas_th= 0
        # homography_inv=inv(homography)
        # homography=homography_inv
        print(homography)
        mid_real_virtual_x, mid_real_virtual_y,a_= np.dot(homography, [self.mid_real_photo_x, self.mid_real_photo_y, 1])
        print("real virtual: %f, %f, %f " %(mid_real_virtual_x, mid_real_virtual_y, a_))
        print("real virtual divided: %d, %d" %(mid_real_virtual_x/a_, mid_real_virtual_y/a_))

        del_x_virtual=mid_real_virtual_x/a_-self.mid_real_photo_x

        del_y_virtual=mid_real_virtual_y/a_-self.mid_real_photo_y
        del_th_virtual=-atan2(homography[0][1],homography[0][0])

        rotation=np.array([[cos(self.current_mid_predict_canvas_th), -sin(self.current_mid_predict_canvas_th)],
                            [sin(self.current_mid_predict_canvas_th), cos(self.current_mid_predict_canvas_th)]])

        del_x_canvas, del_y_canvas = np.matmul(rotation, [-del_x_virtual, -del_y_virtual])
        # *(-1) in del_x_virtual for calibration of x waypoint coordinate
        # *(-1) in del_y_virtual for calibration of image coordiate to canvas coordinate

        # print("virtual photo mid: %d, %d / real photo midpnt: %d, %d" %(mid_real_virtual_x, mid_real_virtual_y, self.mid_real_photo_x, self.mid_real_photo_y))

        # offset=Point()
        # offset.x=del_x_canvas/self.pixMetRatio
        # offset.y=del_y_canvas/self.pixMetRatio
        # offset.z=del_th_virtual

        print(del_x_canvas/self.pixMetRatio)
        print(del_y_canvas/self.pixMetRatio)
        print(del_th_virtual)
        # print(homography)

        # self.success_try += 1

        # dist= sqrt(pow(offset.x,2)+pow(offset.y,2))

        #if dist>=0.02:
        # if dist>=0.1:
        #     offset=Point()
        # else:
        #     #print("OFFSET less than limit (= 0.02)")
        #     print("OFFSET less than limit (= 0.1)")
        #     self.sum_compensation_distance += sqrt(offset.x*offset.x+offset.y*offset.y)
        # return offset

        # self.pub_offset.publish(offset)


if __name__=="__main__":
    option=True
    if len(sys.argv)==2:
        if sys.argc[1]=='debug':
            option=False
    img_virtual_marked= cv2.imread("/home/mjlee/tools/OFFSET_TEST/virtual_map_invert_marked.png", cv2.IMREAD_GRAYSCALE)
    img_photo_marked= cv2.imread("/home/mjlee/tools/OFFSET_TEST/real_photo_invert_marked.png", cv2.IMREAD_GRAYSCALE)
    img_virtual= cv2.imread("/home/mjlee/tools/OFFSET_TEST/virtual_map_invert.png", cv2.IMREAD_GRAYSCALE)
    img_photo= cv2.imread("/home/mjlee/tools/OFFSET_TEST/real_photo_invert.png", cv2.IMREAD_GRAYSCALE)


    # app = FeatureMatch('/home/mjlee/tools/OFFSET_TEST/TEST', option)

    # app = FeatureMatch('/home/bkjung/Pictures')

    # app.SIFT_FLANN_matching(img_photo, img_virtual)
    # app.SIFT_BF_matching(img_photo, img_virtual, img_photo, img_virtual)
    # SIFT_KNN_matching(img1, img2)
    # app.ORB_BF_matching(img_virtual, img_photo)
    app.SURF_BF_matching(img_photo, img_virtual, img_photo_marked, img_virtual_marked)
    # surf = cv2.xfeatures2d.SURF_create()