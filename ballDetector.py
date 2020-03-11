#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose2D
from KalmanFilters import _KalmanFilter_

import cv2
import numpy as np
import imutils
import argparse
import math
import calibration


class ballDetector:
    """class to detect ball and get distance"""

    def __init__(self, D, f, color):
        """
        This is the initializing function for class ballDetector.

        :param D:       real diameter of the ball (in mm)
        :param f:       focal length of the camera (in mm)
        :param color:   ball hsv color (a list consist of upper and lower boundaries of hsv color)
        """

        self.D = D  # real diameter of the ball (in mm)
        self.f = f  # focal length of the camera (in mm)
        self.color = color  # ball color

    def setRealDiameter(self, D):
        """
        This function sets the real diameter of the ball (in mm).

        :param D: real diameter of the ball (in mm) to be set
        """

        self.D = D

    def setF(self, f):
        """
        This function sets the focal length of the camera (in mm).

        :param f: focal length of the camera (in mm) to be set
        """

        self.f = f

    def setColor(self, color):
        """
        This function sets the ball color to be detected.

        :param color: ball color
        """

        self.color = color

    def detectBallDiameter(self, img):
        """
        This function gets the pixel diameter of the ball

        :param img: the undistorted image that the ball is on
        :return:    the diameter of the detected ball (if < 0, then ball isn't detected)
        """

        radius = -1

        color = self.color
        colorLower = color[0]
        colorUpper = color[1]

        # blur the img, and convert it to the HSV color space
        blurred = cv2.GaussianBlur(img, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # construct a mask for the color, then perform a series of dilations and erosions to remove any small
        # blobs left in the mask
        mask = cv2.inRange(hsv, colorLower, colorUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        centers = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
        centers = imutils.grab_contours(centers)

        # only proceed if at least one contour was found
        if centers:
            # find the largest contour in the mask, then use it to compute the minimum enclosing circle
            # and centroid
            c = max(centers, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            # only proceed if the radius meets a minimum size
            if radius > 10:
                # draw the circle and centroid on the img
                cv2.circle(img, (int(x), int(y)), int(radius),
                           (0, 255, 255), 2)
                cv2.circle(img, center, 5, (0, 0, 255), -1)

        d = 2 * radius
        print("Diameter: ", d)
        return d

    def getDistance(self, img):
        """
        This function gets the distance between the ball with color in img and the camera.

        :param img:     the undistorted image that the ball is on
        :return:        the distance between the ball and the camera
        """

        d = self.detectBallDiameter(img)
        distance = self.D * self.f / d
        print("Distance:", distance)
        return distance


def getLocation(cor1, dist1, cor2, dist2, height):
    """
    This function calculates the coordinates of robot

    :param cor1:    coordinates of camera1
    :param dist1:   distance between robot and camera1
    :param cor2:    coordinates of camera2
    :param dist2:   distance between robot and camera2
    :return:        coordinates of robot
    """

    x1 = cor1[0]
    y1 = cor1[1]
    r1 = dist1
    x2 = cor2[0]
    y2 = cor2[1]
    r2 = dist2
    d = math.sqrt((abs(x2 - x1)) ** 2 + (abs(y2 - y1)) ** 2)
    if d > (r1 + r2) or d < (abs(r1 - r2)):
        print("Error. Two circles have no intersection. Can't find robot location.")
        return -1, -1, -1
    else:
        A = (r1 ** 2 - r2 ** 2 + d ** 2) / (2 * d)
        h = math.sqrt(r1 ** 2 - A ** 2)
        a = x1 + A * (x2 - x1) / d
        b = y1 + A * (y2 - y1) / d
        x3 = round(a - h * (y2 - y1) / d, 2)
        y3 = round(b + h * (x2 - x1) / d, 2)
        x4 = round(a + h * (y2 - y1) / d, 2)
        y4 = round(b - h * (x2 - x1) / d, 2)
        if y3 > 0:
            print("Ball location: (", x3, " ,", y3, " ,", height, ")")
            return x3, y3, height
        else:
            print("Ball location: (", x3, ",", y3, ",", height, ")")
            return x4, y4, height


if __name__ == '__main__':
    #ROS node initialization
    rospy.init_node('ball_detector', anonymous = True)
    pub = rospy.Publisher('ball_pose', Pose2D, queue_size = 1000)

    # read arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-cu", "--colorUpper", default="180,255,255",
                    help="upper bound of ball color in hsv, format 180,255,255")
    ap.add_argument("-cl", "--colorLower", default="0,0,0",
                    help="lower bound of ball color in hsv, format 0,0,0")
    ap.add_argument("-f", "--focal", type=int, default=10,
                    help="focal length of the camera")
    ap.add_argument("-d", "--diameter", type=int, default=10,
                    help="the real diameter of the ball")
    args = vars(ap.parse_args())

    cu = args["colorUpper"].split(',')
    cu = tuple(map(int, cu))
    cl = args["colorLower"].split(',')
    cl = tuple(map(int, cl))
    c = [cl, cu]
    focal = args["focal"]
    diameter = args["d"]

    cor1 = (1, 0)   # The 2d coordinates of camera1
    cor2 = (2, 0)   # The 2d coordinates of camera2
    height = 1      # The height of the ball in meter

    detector = ballDetector(diameter, focal, c)

    cap1 = cv2.VideoCapture(0)
    cap2 = cv2.VideoCapture(1)

    # calibration
    mtx1 = []
    dist1 = []
    mtx2 = []
    dist2 = []
    
    filterValid = False

    try:
        npzfile1 = np.load('calibration1.npz')
        mtx1 = npzfile1['mtx']
        dist1 = npzfile1['dist']
    except IOError:
        mtx1, dist1 = calibration.calibrate(cap1, 'calibration1.npz')

    try:
        npzfile2 = np.load('calibration2.npz')
        mtx2 = npzfile2['mtx']
        dist2 = npzfile2['dist']
    except IOError:
        mtx2, dist2 = calibration.calibrate(cap2, 'calibration2.npz')

    # start detecting
    while True:
        ret1, frame1 = cap1.read()
        ret2, frame2 = cap2.read()

        print("Camera 1:")
        frame1 = calibration.undistortion(frame1, mtx1, dist1[0:4])
        distance1 = detector.getDistance(frame1)
        if distance1 < 0:
            print("Ball not detected.")
            continue

        print("Camera 2:")
        frame2 = calibration.undistortion(frame2, mtx2, dist2[0:4])
        distance2 = detector.getDistance(frame2)
        if distance2 < 0:
            print("Ball not detected.")
            continue

        location = getLocation(cor1, distance1/1000, cor2, distance2/1000, height)
        
        ball_pose = Pose2D()
        
        # Stop filtering with invalid measure
        if location[0] == -1 or location[1] == -1:
            filterValid = False
        
        # Start filtering
        if filterValid == False:
            kalman = _KalmanFilter_(location[0], location[1])
            filterValid = True
        else:
            location = kalman._filterUpdate(location[0], location[1])
            
        # ROS node send pose out
        ball_pose.x, ball_pose.y, ball_pose.theta = location[0], location[1], 0.0
        pub.publish(ball_pose)
        

