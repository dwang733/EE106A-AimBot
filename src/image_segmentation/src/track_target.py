#!/usr/bin/env python

from __future__ import division
import rospy
import sys
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Threshold on light blue
LOWER_THRESH = (78,30,50)
UPPER_THRESH = (108,255,100)
CAMERA_HEIGHT = 800
CAMERA_WIDTH = 1280

def detect_target_circle(msg):
    bridge = CvBridge()
    try:
        img = bridge.imgmsg_to_cv2(msg, "passthrough")
        # img = img[:, 200:1080, :]
        img_gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        img_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        # Threshold on yellow
        # LOWER_THRESH = (15,60,80)
        # UPPER_THRESH = (42,255,255)
        mask = cv.inRange(img_hsv, LOWER_THRESH, UPPER_THRESH)
        img_gray = cv.bitwise_and(img_gray, img_gray, mask=mask)
        img_gray = cv.medianBlur(img_gray, 5)
        cv.imshow('thresholding', img_gray)
        cv.waitKey(1)

        all_circles = cv.HoughCircles(img_gray, cv.HOUGH_GRADIENT, 2, 0.1,
                            param1=50, param2=100, minRadius=10, maxRadius=75)
        # all_circles = cv.HoughCircles(img_gray, cv.HOUGH_GRADIENT, 4, 0.1,
        #                     param1=50, param2=100, minRadius=5, maxRadius=50)
        if all_circles is not None:
            circle = all_circles[0, 0, :]
            circle = np.uint16(np.around(circle))
            # draw the outer circle
            cv.circle(img, (circle[0], circle[1]), circle[2], (0, 255, 0), 2)
            # draw the center of the circle
            cv.circle(img, (circle[0], circle[1]), 2, (0, 0, 255), 3)

            # for i in all_circles[0,:]:
            #     # draw the outer circle
            #     cv.circle(img,(i[0],i[1]),i[2],(0,255,0),2)
            #     # draw the center of the circle
            #     cv.circle(img_gray,(i[0],i[1]),2,(0,0,255),3)
            cv.imshow('detected circles', img)
            cv.waitKey(1)
            return circle
    except CvBridgeError, e:
        rospy.logerr("CvBridge Error: {0}".format(e))


def calc_target_position(circle):
    # Diameter (in m) of the circle detected by the algorithm
    CIRCLE_DIAMETER = (0.12 + 0.145) / 2
    # Diameter (in m) of the target
    TARGET_DIAMETER = 0.255
    # Distance (in m) from camera which causes target to be the height of the camera image
    TARGET_FULL_DIST = 0.125

    circle_center = circle[0:2]
    circle_radius = circle[2]
    # rospy.loginfo('radius: %f', circle_radius)

    circle_frac = circle_radius * 2 / CAMERA_HEIGHT
    # rospy.loginfo(circle_frac)
    image_height = CIRCLE_DIAMETER / circle_frac  # height in meters
    # rospy.loginfo(image_height)
    target_x_dist = image_height * TARGET_FULL_DIST / TARGET_DIAMETER
    rospy.loginfo("x dist: %f", target_x_dist)


def callback(msg):
    circle = detect_target_circle(msg)
    if circle is not None:
        calc_target_position(circle)


def main():
    rospy.init_node('camera_sub', anonymous=True)
    rospy.Subscriber('/cameras/left_hand_camera/image', Image, callback)
    # rospy.Subscriber('/io/internal_camera/head_camera/image_raw', Image, callback)
    rospy.spin()

if __name__ == '__main__':
    main()
