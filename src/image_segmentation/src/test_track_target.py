#!/usr/bin/env python

from __future__ import division
import rospy
import sys
import cv2 as cv
import numpy as np
import tf2_ros
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Threshold on light blue
# LOWER_THRESH = (78,50,50)
# UPPER_THRESH = (108,255,120)
# Threshold on yellow
LOWER_THRESH = (15,70,70)
UPPER_THRESH = (50,255,255)
CAMERA_HEIGHT = 800
CAMERA_WIDTH = 1280

# Diameter (in m) of the circle detected by the algorithm
CIRCLE_DIAMETER = 0.048
# Diameter (in m) of the target
# TARGET_DIAMETER = 0.255
TARGET_DIAMETER = 0.250
# Distance (in m) from camera which causes target to be the height of the camera image
# TARGET_FULL_DIST = 0.125
TARGET_FULL_DIST = 0.118

# Copied from imutils
def grab_contours(cnts):
    # if the length the contours tuple returned by cv2.findContours
    # is '2' then we are using either OpenCV v2.4, v4-beta, or
    # v4-official
    if len(cnts) == 2:
        cnts = cnts[0]

    # if the length of the contours tuple is '3' then we are using
    # either OpenCV v3, v4-pre, or v4-alpha
    elif len(cnts) == 3:
        cnts = cnts[1]

    # otherwise OpenCV has changed their cv2.findContours return
    # signature yet again and I have no idea WTH is going on
    else:
        raise Exception(("Contours tuple must have length 2 or 3, "
            "otherwise OpenCV changed their cv2.findContours return "
            "signature yet again. Refer to OpenCV's documentation "
            "in that case"))

    # return the actual contours array
    return cnts

def detect_target_circle(img):
    img_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    mask = cv.inRange(img_hsv, LOWER_THRESH, UPPER_THRESH)
    mask = cv.medianBlur(mask, 9)
    cv.imshow('thresholding', mask)
    cv.waitKey(1)

    cnts = cv.findContours(mask.copy(), cv.RETR_EXTERNAL,
        cv.CHAIN_APPROX_SIMPLE)
    cnts = grab_contours(cnts)
    # cv.drawContours(img, cnts, -1, (0,255,0), 3)
    # cv.imshow('contours', img)
    # cv.waitKey(1)
    center = None

    # only proceed if at least one contour was found
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts, key=cv.contourArea)
        ((x, y), radius) = cv.minEnclosingCircle(c)
        M = cv.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        # only proceed if the radius meets a minimum size
        if radius > 3:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv.circle(img, (int(x), int(y)), int(radius),
                (0, 255, 255), 2)
            cv.circle(img, center, 5, (0, 0, 255), -1)
            cv.imshow('circle', img)
            cv.waitKey(1)
            print((x, y, radius))
            return (x, y, radius)


def calc_target_position(circle, CAMERA_HEIGHT, CAMERA_WIDTH):
    circle_x, circle_y, circle_radius = circle
    rospy.loginfo('radius: %f', circle_radius)

    circle_frac = circle_radius * 2 / CAMERA_HEIGHT
    image_height = CIRCLE_DIAMETER / circle_frac  # height in meters
    print("image_height: {}".format(image_height))
    relative_depth = image_height * TARGET_FULL_DIST / TARGET_DIAMETER
    print("relative_depth: {}".format(relative_depth))

    circle_y_frac = circle_y / CAMERA_HEIGHT
    print(circle_y_frac)
    abs_y_pos = image_height * circle_y_frac
    print(abs_y_pos)
    relative_y_pos = abs_y_pos - image_height / 2
    print("relative_y_pos: {}".format(relative_y_pos))

    image_width = image_height * CAMERA_WIDTH / CAMERA_HEIGHT
    print("image_width: {}".format(image_width))
    circle_x_frac = circle_x / CAMERA_WIDTH
    print(circle_x_frac)
    abs_x_pos = image_width * circle_x_frac
    print(abs_x_pos)
    relative_x_pos = abs_x_pos - image_width / 2
    print("relative_x_pos: {}".format(relative_x_pos))
    print("-------------")

    # br = tf2_ros.TransformBroadcaster()
    # t = TransformStamped()
    # t.header.stamp = rospy.Time.now()
    # t.header.frame_id = "left_hand_camera_axis"
    # t.child_frame_id = "target"
    # t.transform.translation.x = relative_y_pos
    # t.transform.translation.y = -relative_x_pos
    # t.transform.translation.z = relative_depth
    # t.transform.rotation.w = 1
    # br.sendTransform(t)


def callback(msg):
    try:
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(msg, "passthrough")
        # img = img[:, 200:-200, :]
        CAMERA_HEIGHT, CAMERA_WIDTH, _ = img.shape

        circle = detect_target_circle(img)
        if circle is not None:
            calc_target_position(circle, CAMERA_HEIGHT, CAMERA_WIDTH)
    except CvBridgeError, e:
        rospy.logerr("CvBridge Error: {0}".format(e))


def main():
    rospy.init_node('camera_sub', anonymous=True)

    rospy.Subscriber('/cameras/left_hand_camera/image', Image, callback)
    rospy.spin()


if __name__ == '__main__':
    main()