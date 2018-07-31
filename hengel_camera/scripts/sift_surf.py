#!/usr/bin/env python
import numpy as np
import cv2
from matplotlib import pyplot as plt

def SIFTmatching(img1, img2):
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

def ORBmatching(img1, img2):
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

def FLANNmatching(img1, img2):
    sift=cv2.xfeatures2d.SIFT_create()
    kp1, des1 = sift.detectAndCompute(img1, None)
    kp2, des2 = sift.detectAndCompute(img2, None)

    FLANN_INDEX_KDTREE=0
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    search_params = dict(checks=50)   # or pass empty dictionary
    flann = cv2.FlannBasedMatcher(index_params,search_params)
    matches = flann.knnMatch(des2,des1,k=2)
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
    plt.imshow(img3,),plt.show()


if __name__=="__main__":
    img2= cv2.imread("/home/intern05/demo.png", cv2.IMREAD_GRAYSCALE)
    img1= cv2.imread("/home/intern05/test/frame0000.jpg", cv2.IMREAD_GRAYSCALE)

    FLANNmatching(img1, img2)
    SIFTmatching(img1, img2)
    ORBmatching(img1, img2)

    surf = cv2.xfeatures2d.SURF_create()

    