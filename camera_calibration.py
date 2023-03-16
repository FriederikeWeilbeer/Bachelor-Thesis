import numpy as np
import cv2
import glob

WAIT_TIME = 100

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

rows = 6
cols = 7

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
# IMPORTANT : Object points must be changed to get real physical distance.
objp = np.zeros((rows * cols, 3), np.float32)
objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

images = glob.glob('calib_imgs/*.jpg')
for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # find chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, (7, 6), None)

    # If found, add object points, img points (after refining them)
    if ret:
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

        # draw and display corners
        img = cv2.drawChessboardCorners(img, (cols, rows), corners2, ret)
        cv2.imshow('img', img)
        cv2.waitKey(WAIT_TIME)

cv2.destroyAllWindows()
ret, mtx, dist, rvecs, tvec = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# ---------- Saving the calibration -----------------
cv_file = cv2.FileStorage("calib_imgs/test.yaml", cv2.FILE_STORAGE_WRITE)
cv_file.write("camera_matrix", mtx)
cv_file.write("dist_coeff", dist)

# note you *release* you don't close() a FileStorage object
cv_file.release()
