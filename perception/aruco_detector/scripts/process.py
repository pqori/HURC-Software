#!/usr/bin/env python3
import cv2, glob, numpy as np, pathlib

cb_w, cb_h = 9, 6
square_mm   = 18.8  # edge length in millimetres
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            30, 0.001)

objp = np.zeros((cb_w*cb_h, 3), np.float32)
objp[:, :2] = np.mgrid[0:cb_w, 0:cb_h].T.reshape(-1,2) * square_mm

objpoints, imgpoints = [], []
imgs = sorted(glob.glob('images/*.jpg'))

for fname in imgs:
    img  = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, (cb_w, cb_h), None)
    if ret:
        objpoints.append(objp)
        corners_sub = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners_sub)
        cv2.drawChessboardCorners(img, (cb_w, cb_h), corners_sub, ret)
        cv2.imshow('img', img); cv2.waitKey(100)

cv2.destroyAllWindows()
ret, mtx, dist, *_ = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None)

print("Camera matrix:\n", mtx)
print("Distortion:\n", dist)

pathlib.Path("camera_cal.npz").write_bytes(
    np.savez_compressed(None, mtx=mtx, dist=dist).getbuffer())
print("Saved camera_cal.npz")
