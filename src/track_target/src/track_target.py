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
from watershed import TargetFinder

# Threshold on YELLOW
#LOWER_THRESH = (0,58,53) #(120,30,70)
#UPPER_THRESH = (87,102,102) #(170,120,200)

#Threshold on YELLOW LARGE
LOWER_THRESH = (13,57,73) #(120,30,70)
UPPER_THRESH = (87,117,255) #(170,120,200)

# Threshold on BLUE. Commented out is YELLOW
#LOWER_THRESH = (36, 50, 9)#(150,45,5)#(45,85,72)
#UPPER_THRESH = (118, 105, 29)#(206,79,70)#(177,242,251)

# Threshold on green
# LOWER_THRESH = (60,28,0)
# UPPER_THRESH = (81,255,255)
target_finder = TargetFinder(LOWER_THRESH, UPPER_THRESH)

# Diameter (in m) of the circle detected by the algorithm
# YELLOW Diameter
CIRCLE_DIAMETER = 0.210
# BLUE Diameter
#CIRCLE_DIAMETER = 0.152
# Diameter (in m) of the target
TARGET_DIAMETER = 0.250
# Distance (in m) from camera which causes target to be the height of the camera image
TARGET_FULL_DIST = 0.118
# Bridge used to convert camera image to OpenCV format
bridge = CvBridge()


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


# Detects the yellow circle on the target and returns the circle's center coords and radius
def detect_target_circle(img):
    # Convert to HSV color format and threshold on the yellow color
    img_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    #img_hsv[:,:,0] = np.mod(img_hsv[:,:,0] + 100, 255)
    mask = cv.inRange(img_hsv, LOWER_THRESH, UPPER_THRESH)
    mask = cv.medianBlur(mask, 9)
    cv.imshow('thresholding', mask)
    cv.waitKey(1)

    # Find all the contours in the image
    cnts = cv.findContours(mask.copy(), cv.RETR_EXTERNAL,
        cv.CHAIN_APPROX_SIMPLE)
    cnts = grab_contours(cnts)
    # cv.drawContours(img, cnts, -1, (0,255,0), 3)
    # cv.imshow('contours', img)
    # cv.waitKey(1)

    # only proceed if at least one contour was found
    if len(cnts) == 0:
        return

    # # find the largest contour in the mask, then use
    # # it to compute the minimum enclosing circle and
    # # centroid
    c = max(cnts, key=cv.contourArea)
    ((x, y), radius) = cv.minEnclosingCircle(c)
    # M = cv.moments(c)
    # center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

    # only proceed if the radius meets a minimum size
    if radius > 3:
        # draw the circle and centroid on the frame,
        # then update the list of tracked points
        cv.circle(img, (int(x), int(y)), int(radius),
            (0, 255, 255), 2)
        cv.circle(img, (int(x), int(y)), 5, (0, 0, 255), -1)
        cv.imshow('contour method', img)
        cv.waitKey(1)
        print((x, y, radius))
        return (x, y, radius)

    # cnts.sort(key=cv.contourArea, reverse=True)
    # target_circle = None
    # target_circle_radius = 0
    # target_eccentricity = 0
    # for c in cnts[:2]:
    #     try:
    #         ellipse = cv.fitEllipse(c)
    #         (center, axes, orientation) = ellipse
    #         major_axis = max(axes)
    #         minor_axis = min(axes)
    #         eccentricity = np.sqrt(1 - (minor_axis / major_axis) ** 2)
    #         # cv.ellipse(img, ellipse, (0, 0, 255), 2)
    #         # print(eccentricity)

    #         if eccentricity > 0.7:
    #             print(eccentricity)
    #             # cv.ellipse(img, ellipse, (0, 0, 255), 2)
    #             axis_avg = (major_axis + minor_axis) / 2
    #             if target_circle is None or eccentricity > target_eccentricity:
    #                 target_circle_radius = axis_avg
    #                 target_circle = (center[0], center[1], target_circle_radius)
    #                 target_eccentricity = eccentricity

    #         # # only proceed if the radius meets a minimum size
    #         # if radius > 3:
    #         #     # draw the circle and centroid on the frame,
    #         #     # then update the list of tracked points
    #         #     cv.circle(img, (int(x), int(y)), int(radius),
    #         #         (0, 255, 255), 2)
    #         #     cv.circle(img, (int(x), int(y)), 5, (0, 0, 255), -1)
    #         #     cv.imshow('circle', img)
    #         #     cv.waitKey(1)
    #         #     print((x, y, radius))
    #         #     return (x, y, radius)
    #     except Exception:
    #         pass

    # print("-------------------------")
    # if target_circle is not None:
    #     cv.circle(img, (int(target_circle[0]), int(target_circle[1])), int(target_circle[2]), (0, 255, 255), 2)
    #     cv.circle(img, (int(target_circle[0]), int(target_circle[1])), 5, (0, 0, 255), -1)
    #     print(target_circle)
    # cv.imshow('ellipses', img)


# Calculate the target's position relative to the camera and to base
def calc_target_position(circle, CAMERA_HEIGHT, CAMERA_WIDTH):
    circle_x, circle_y, circle_radius = circle
    rospy.loginfo('radius: %f', circle_radius)

    # Calculate how far target is from the camera
    circle_frac = circle_radius * 2 / CAMERA_HEIGHT
    image_height = CIRCLE_DIAMETER / circle_frac  # height in meters
    print("image_height: {}".format(image_height))
    relative_depth = image_height * TARGET_FULL_DIST / TARGET_DIAMETER
    print("relative_depth: {}".format(relative_depth))

    # Calculate how far above/below the target is relative to camera
    # circle_y_frac = circle_y / CAMERA_HEIGHT
    # print(circle_y_frac)
    # abs_y_pos = image_height * circle_y_frac
    # print(abs_y_pos)
    # relative_y_pos = abs_y_pos - image_height / 2
    # print("relative_y_pos: {}".format(relative_y_pos))

    # # Calculate how far left/right the target is relative to camera
    # image_width = image_height * CAMERA_WIDTH / CAMERA_HEIGHT
    # print("image_width: {}".format(image_width))
    # circle_x_frac = circle_x / CAMERA_WIDTH
    # print(circle_x_frac)
    # abs_x_pos = image_width * circle_x_frac
    # print(abs_x_pos)
    # relative_x_pos = abs_x_pos - image_width / 2
    # print("relative_x_pos: {}".format(relative_x_pos))
    # print("-------------")
    #
    # # Broadcast this transform as "target" relative to the left hand camera's axis
    # br = tf2_ros.TransformBroadcaster()
    # t = TransformStamped()
    # t.transform.translation.x = relative_y_pos
    # t.transform.translation.y = -relative_x_pos
    # t.transform.translation.z = relative_depth
    # t.transform.rotation.w = 1
    # t.header.stamp = rospy.Time.now() + rospy.Duration(337)  # Compensating for robot publishing wrong timestamps
    # t.header.frame_id = "reference/left_hand_camera_axis"
    # t.child_frame_id = "target"
    # br.sendTransform(t)

    relative_x_pos = relative_depth * (circle_x - 667.5471339020004) / 412.07883144382936
    relative_y_pos = relative_depth * (circle_y - 426.61500902127045) / 413.0922741963357
    print('relative_x_pos: {}'.format(relative_x_pos))
    print('relative_y_pos: {}'.format(relative_y_pos))
    print('--------------------------------')

    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()
    t.transform.translation.x = relative_y_pos
    t.transform.translation.y = -relative_x_pos
    t.transform.translation.z = relative_depth
    t.transform.rotation.w = 1
    t.header.stamp = rospy.Time.now() + rospy.Duration(337)  # Compensating for robot publishing wrong timestamps
    t.header.frame_id = "reference/left_hand_camera_axis"
    t.child_frame_id = "target_new"
    br.sendTransform(t)


def callback(msg):
    try:
        img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # img = img[:, 200:-200, :]
        CAMERA_HEIGHT, CAMERA_WIDTH, _ = img.shape

        circle = detect_target_circle(img)
        # circle = target_finder.detect_target_circle(img)
        if circle is not None:
            calc_target_position(circle, CAMERA_HEIGHT, CAMERA_WIDTH)
    except CvBridgeError, e:
        rospy.logerr("CvBridge Error: {0}".format(e))


def main():
    rospy.init_node('track_target')

    rospy.Subscriber('/cameras/left_hand_camera/image', Image, callback)

    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)

    rospy.spin()


if __name__ == '__main__':
    main()
