#!/usr/bin/env python

import numpy as np
import cv2
from matplotlib import pyplot as plt
MIN_MATCH_COUNT = 5

# Pass (training, query)
def find_lcd(img1, img2):
	global MIN_MATCH_COUNT

	# SIFT
	sift = cv2.xfeatures2d.SIFT_create()
	kp1, des1 = sift.detectAndCompute(img1,None)
	kp2, des2 = sift.detectAndCompute(img2,None)

	# Find Matches
	bf = cv2.BFMatcher()
	matches = bf.knnMatch(des1,des2, k=2)

	# store all the good matches as per Lowe's ratio test.
	good = []
	for m,n in matches:
	    if m.distance < 0.7*n.distance:
	        good.append(m)

	# Check if minimum points exist
	if len(good)>=MIN_MATCH_COUNT:
		# Get corresponding matches in both images
	    src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
	    dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
	    # Homography with RANSAC
	    M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,1.0)
	    matchesMask = mask.ravel().tolist()
	    # Find H matrix
	    h,w = img1.shape
	    pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
	    dst = cv2.perspectiveTransform(pts,M)
	    # Draw bounding box
	    tl = np.int32(dst)[0][0][0],np.int32(dst)[0][0][1]
	    bl = np.int32(dst)[1][0][0],np.int32(dst)[1][0][1]
	    br = np.int32(dst)[2][0][0],np.int32(dst)[2][0][1]
	    tr = np.int32(dst)[3][0][0],np.int32(dst)[3][0][1]
	    return [tl,bl,br,tr]
	else:
	    #print "Not enough matches are found - %d/%d" % (len(good),MIN_MATCH_COUNT)
	    matchesMask = None
	    return
	    # return [past_tl,past_bl,past_br,past_tr]