import numpy as np
import cv2
import glob

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*9,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

images = glob.glob('/home/ayush/Pictures/Calibration/*.JPG')

cv2.namedWindow("img", cv2.WINDOW_NORMAL)
cv2.resizeWindow("img", 500, 500)


cv2.namedWindow("original", cv2.WINDOW_NORMAL)
cv2.resizeWindow("original", 500, 500)

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    og=img
    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (9,6),None)
    cv2.imshow("original", og)
    cv2.waitKey(1)
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, (9,6), corners2,ret)
        cv2.imshow('img',img)
        cv2.imshow("original", og)
        cv2.waitKey(1)
    else:
        print("Corners not found")

    if cv2.waitKey(1) & 0xFF==ord('q'):
        break


'''img1=cv2.imread('/home/ayush/Pictures/Calibration/2020_0219_165341_001.JPG')

gray=cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
# undistort
dst = cv2.undistort(img, mtx, dist, None, newcameramtx)

# crop the image
x,y,w,h = roi
dst = dst[y:y+h, x:x+w]
#cv2.imwrite('calibresult.png',dst)
cv2.imshow('RESULT', dst)
'''
cv2.destroyAllWindows()
