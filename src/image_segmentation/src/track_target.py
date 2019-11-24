#!/usr/bin/env python

import rospy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def callback(msg):
    bridge = CvBridge()
    try:
        img = bridge.imgmsg_to_cv2(msg, "passthrough")
        img_gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        all_circles = cv.HoughCircles(img_gray, cv.HOUGH_GRADIENT, 1, 20,
                            param1=50, param2=30, minRadius=5, maxRadius=30)
        circle = all_circles[0, 0, :]
        circle = np.uint16(np.around(circle))
        # draw the outer circle
        cv.circle(img, (circle[0], circle[1]), circle[2], (0, 255, 0), 2)
        # draw the center of the circle
        cv.circle(img, (circle[0], circle[1]), 2, (0, 0, 255), 3)
        cv.imshow('detected circles', img)
        cv.waitKey(1)
    except CvBridgeError, e:
        rospy.logerr("CvBridge Error: {0}".format(e))

def main():
    rospy.init_node('camera_sub', anonymous=True)
    rospy.Subscriber('/cameras/head_camera/image', Image, callback)
    rospy.spin()

if __name__ == '__main__':
    main()
