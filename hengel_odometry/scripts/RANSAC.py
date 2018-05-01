#!/usr/bin/env python

from scipy import *
from scipy.linalg import *
from scipy.special import *
from random import choice
from PIL import Image
import sys

from homography import *

import pdb

def ransac(points_list, iters = 50 , error = 10, good_model_num = 5):
    '''
        This function uses RANSAC algorithm to estimate the
        shift and rotation between the two given images
    '''

    model_error = 255
    model_H = None

    for i in range(iters):
        consensus_set = []
        points_list_temp = copy(points_list).tolist()
        # Randomly select 3 points
        for j in range(3):
            temp = choice(points_list_temp)
            consensus_set.append(temp)
            points_list_temp.remove(temp)

        # Calculate the homography matrix from the 3 points
        fp0 = []
        fp1 = []
        fp2 = []

        tp0 = []
        tp1 = []
        tp2 = []

        for line in consensus_set:
            fp0.append(line[0][0])
            fp1.append(line[0][1])
            fp2.append(1)

            tp0.append(line[1][0])
            tp1.append(line[1][1])
            tp2.append(1)

        fp = array([fp0, fp1, fp2])
        tp = array([tp0, tp1, tp2])

        H = Haffine_from_points(fp, tp)

        # Transform the second image
        # imtemp = transform_im(im2, [-xshift, -yshift], -theta)
        # Check if the other points fit this model

        for p in points_list_temp:
            x1, y1 = p[0]
            x2, y2 = p[1]

            A = array([x1, y1, 1]).reshape(3,1)
            B = array([x2, y2, 1]).reshape(3,1)

            out = B - dot(H, A)
            dist_err = hypot(out[0][0], out[1][0])
            if dist_err < error:
                consensus_set.append(p)


        # Check how well is our speculated model
        if len(consensus_set) >= good_model_num:
            dists = []
            for p in consensus_set:
                x0, y0 = p[0]
                x1, y1 = p[1]

                A = array([x0, y0, 1]).reshape(3,1)
                B = array([x1, y1, 1]).reshape(3,1)

                out = B - dot(H, A)
                dist_err = hypot(out[0][0], out[1][0])
                dists.append(dist_err)
            if (max(dists) < error) and (max(dists) < model_error):
                model_error = max(dists)
                model_H = H
    return model_H