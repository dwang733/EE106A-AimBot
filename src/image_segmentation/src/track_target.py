#!/usr/bin/env python

import rospy
import sys
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def callback(msg):
    bridge = CvBridge()
    try:
        img = bridge.imgmsg_to_cv2(msg, "passthrough")
        # img = img[:, 200:1080, :]
        img_gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        img_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        # Threshold on light blue
        lower_thresh = (78,30,54)
        upper_thresh = (108,255,255)
        # Threshold on yellow
        # lower_thresh = (15,60,80)
        # upper_thresh = (42,255,255)
        mask = cv.inRange(img_hsv, lower_thresh, upper_thresh)
        img_gray = cv.bitwise_and(img_gray, img_gray, mask=mask)
        img_gray = cv.medianBlur(img_gray, 13)
        cv.imshow('thresholding', img_gray)
        cv.waitKey(1)

        all_circles = cv.HoughCircles(img_gray, cv.HOUGH_GRADIENT, 2, 0.1,
                            param1=50, param2=100, minRadius=15, maxRadius=100)
        # all_circles = cv.HoughCircles(img_gray, cv.HOUGH_GRADIENT, 4, 0.1,
        #                     param1=50, param2=100, minRadius=5, maxRadius=50)
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
    except CvBridgeError, e:
        rospy.logerr("CvBridge Error: {0}".format(e))

def main():
    rospy.init_node('camera_sub', anonymous=True)
    rospy.Subscriber('/cameras/left_hand_camera/image', Image, callback)
    # rospy.Subscriber('/io/internal_camera/head_camera/image_raw', Image, callback)
    rospy.spin()

if __name__ == '__main__':
    main()
