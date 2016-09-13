#!/usr/bin/python2

import numpy as np
import cv2
from common import splitfn

import sys, getopt
from glob import glob

args, img_mask = getopt.getopt(sys.argv[1:], '')

try: img_mask = img_mask[0]
except: img_mask = './*.jpg'

img_names = glob(img_mask)
square_size = 1.0

pattern_size = (9, 6)
pattern_points = np.zeros( (np.prod(pattern_size), 3), np.float32 )
pattern_points[:,:2] = np.indices(pattern_size).T.reshape(-1, 2)
pattern_points *= square_size

obj_points = []
img_points = []
h, w = 0, 0
print("")
print("Altered version of original calibrate.py srcipt from opencv2.")
print("Prints calibration in format which suits for ORB_SLAM2 *.yaml config file")
print("Usage: $python2 ./calibrate.py \"/path/*.jpg\"")

print("")
for fn in img_names:
    sys.stdout.write('processing ' + fn+ ' ...')
    
    img = cv2.imread(fn, 0)
    h, w = img.shape[:2]
    found, corners = cv2.findChessboardCorners(img, pattern_size)
    if found:
        term = ( cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1 )
        cv2.cornerSubPix(img, corners, (5, 5), (-1, -1), term)
    if not found:
        print('chessboard not found')
        continue
    img_points.append(corners.reshape(-1, 2))
    obj_points.append(pattern_points)

    print('ok')

print("")
rms, camera_matrix, dist_coefs, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, (w, h))
print("RMS: "+str(rms))
print("")
print("Camera.fx: "+str(camera_matrix[0,0]))
print("Camera.fy: "+str(camera_matrix[1,1]))
print("Camera.cx: "+str(camera_matrix[0,2]))
print("Camera.cy: "+str(camera_matrix[1,2]))
print("")
print("Camera.k1: "+str(dist_coefs.ravel()[0]))
print("Camera.k2: "+str(dist_coefs.ravel()[1]))
print("Camera.p1: "+str(dist_coefs.ravel()[2]))
print("Camera.p2: "+str(dist_coefs.ravel()[3]))
print("Camera.k3: "+str(dist_coefs.ravel()[4]))

#print "camera matrix:\n", camera_matrix
#print "distortion coefficients: ", dist_coefs.ravel()
cv2.destroyAllWindows()
