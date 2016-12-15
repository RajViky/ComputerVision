#!/usr/bin/python2

import numpy as np
import cv2
import os
import sys
from glob import glob
import time
import yaml

remove = False
capture = False

#VIDEO FILE
if(len(sys.argv) == 3 and sys.argv[1] == "--video"):
  remove = True
  img_mask = '/tmp/frame*.jpg'
  video_file = sys.argv[2]
  vidcap = cv2.VideoCapture(video_file)
  success,image = vidcap.read()
  count = 0
  success = True
  skip = 30;
  while success:
    success,image = vidcap.read()
    if(count % skip == 0):
	cv2.imwrite("/tmp/frame%d.jpg" % count, image)     # save frame as JPEG file
    count += 1
#VIDEO CAP
elif(len(sys.argv) == 3 and sys.argv[1] == "--videoCap"):
    remove = False
    capture = True
    cap = cv2.VideoCapture(int(sys.argv[2]))
    img_names = range(0,60)
#PHOTOS
elif(len(sys.argv) == 2):
  print "TEST"
  try: img_mask = sys.argv[1]
  except: img_mask = './*.jpg'
  print(img_mask)

if(not capture):
    img_names = glob(img_mask)

square_size = 24.0
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
print("Usage: $ ./calibrate.py \"/path/*.jpg\"")
print("       $ ./calibrate.py --video \"/path/video.ext\"")
print("       $ ./calibrate.py --videoCap deviceID")

print("")
for fn in img_names:
    sys.stdout.write('processing ' + str(fn)+ ' ...')
    if(capture):
        r,img = cap.read()
        cv2.imshow('img',img)
    else:
        img = cv2.imread(fn, 0)
    try:h, w = img.shape[:2]
    except:
        print('error processing images - skip')
        continue
    
    print(str(h) + "x" + str(w))    
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCorners(gray, pattern_size,None)
    if found:
        term = ( cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1 )
        #cv2.cornerSubPix(img, corners, (5, 5), (-1, -1), term)
        cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),term)
        if(capture):
            # Draw and display the corners
            cv2.drawChessboardCorners(img, (9,6), corners,found)
            cv2.imshow('img',img)
            cv2.waitKey(1000)
            
    if not found:
        print('chessboard not found - skip')
        if(capture):
            time.sleep(1)
        continue
    img_points.append(corners.reshape(-1, 2))
    obj_points.append(pattern_points)

    print('ok')
    if(remove):
        os.remove(fn)

print("")
rms, camera_matrix, dist_coefs, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, (w, h),None,None)
print("RMS: "+str(rms))
print("")

#Camera matrix
#fx,fy = focal length
#cy,cy = optical center
#k1,k2,k3 = radial distortion
#p1,p2 = tangential distortion

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

fname = "/tmp/data.yaml"
with open(fname, "w") as f:
    yaml.dump({'camera_name': "usbTest"}, f)
    yaml.dump({'image_width': w}, f)
    yaml.dump({'image_height': h}, f)

#print "camera matrix:\n", camera_matrix
#print "distortion coefficients: ", dist_coefs.ravel()
cv2.destroyAllWindows()
