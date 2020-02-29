import cv2
import numpy as np


def calibrate(cap, saveFile):
    """
    This function calibrate the camera.

    :param cap:         video source
    :param saveFile:    file name to save intrinsic and distortion matrix
    :return:            intrinsic and distortion matrix
    """

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    Nx_cor = 9
    Ny_cor = 6

    objp = np.zeros((Nx_cor * Ny_cor, 3), np.float32)
    objp[:, :2] = np.mgrid[0:Nx_cor, 0:Ny_cor].T.reshape(-1, 2)
    objpoints = []  # 3d points in real world space
    imgpoints = []  # 2d points in image plane.

    count = 0  # the number of frames detected
    while (True):

        ret, frame = cap.read()

        if cv2.waitKey(1) & 0xFF == ord(' '):

            # Our operations on the frame come here
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            ret, corners = cv2.findChessboardCorners(gray, (Nx_cor, Ny_cor), None)  # Find the corners
            # If found, add object points, image points
            if ret == True:
                corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)
                objpoints.append(objp)
                imgpoints.append(corners)
                cv2.drawChessboardCorners(frame, (Nx_cor, Ny_cor), corners, ret)
                count += 1

                if count > 20:
                    break

        # Display the resulting frame
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    print(mtx, dist)

    mean_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        mean_error += error

    print("total error: ", mean_error / len(objpoints))

    np.savez(saveFile, mtx=mtx, dist=dist[0:4])
    return mtx, dist

def undistortion(img, mtx, dist):
    """
    This function un-distort the image

    :param img:     the image to un-distort
    :param mtx:     intrinsics matrix
    :param dist:    distortion matrix
    :return:        undistorted image
    """
    h, w = img.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))

    dst = cv2.undistort(img, mtx, dist, None, newcameramtx)

    # crop the image
    x, y, w, h = roi
    if roi != (0, 0, 0, 0):
        dst = dst[y:y + h, x:x + w]

    return dst
