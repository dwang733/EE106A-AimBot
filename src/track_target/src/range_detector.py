#!/usr/bin/env python
# -*- coding: utf-8 -*-

# USAGE: You need to specify a filter and "only one" image source
#
# (python) range-detector --filter RGB --image /path/to/image.png
# or
# (python) range-detector --filter HSV --webcam

import rospy
import cv2
import argparse
from operator import xor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


def callback(arg):
    pass


def camera_callback(msg, args):
    range_filter = args[0]
    bridge = CvBridge()
    try:
        img = bridge.imgmsg_to_cv2(msg, "passthrough")
        # img_yuv = cv2.cvtColor(img, cv2.COLOR_BGR2YUV)
        # img_yuv[:,:,0] = cv2.equalizeHist(img_yuv[:,:,0])
        # img = cv2.cvtColor(img_yuv, cv2.COLOR_YUV2BGR)
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        threshold(range_filter, img_hsv)
    except CvBridgeError, e:
        rospy.logerr("CvBridge Error: {0}".format(e))


def threshold(range_filter, frame_to_thresh):
    v1_min, v2_min, v3_min, v1_max, v2_max, v3_max = get_trackbar_values(range_filter)

    thresh = cv2.inRange(frame_to_thresh, (v1_min, v2_min, v3_min), (v1_max, v2_max, v3_max))
    thresh = cv2.medianBlur(thresh, 9)

    cv2.imshow("Original", cv2.cvtColor(frame_to_thresh, cv2.COLOR_HSV2BGR))
    cv2.imshow("thresholded image", thresh)
    # cv2.imshow("Thresh", thresh)
    # cv2.waitKey(1)

    if cv2.waitKey(1) & 0xFF is ord('q'):
        return


def setup_trackbars(range_filter):
    cv2.namedWindow("Trackbars", 0)

    for i in ["MIN", "MAX"]:
        v = 0 if i == "MIN" else 255

        for j in range_filter:
            cv2.createTrackbar("%s_%s" % (j, i), "Trackbars", v, 255, callback)


def get_arguments():
    ap = argparse.ArgumentParser()
    ap.add_argument('-f', '--filter', required=True,
                    help='Range filter. RGB or HSV')
    ap.add_argument('-i', '--image', required=False,
                    help='Path to the image')
    ap.add_argument('-w', '--webcam', required=False,
                    help='Use webcam', action='store_true')
    ap.add_argument('-p', '--preview', required=False,
                    help='Show a preview of the image after applying the mask',
                    action='store_true')
    args = vars(ap.parse_args())

    if not xor(bool(args['image']), bool(args['webcam'])):
        ap.error("Please specify only one image source")

    if not args['filter'].upper() in ['RGB', 'HSV']:
        ap.error("Please speciy a correct filter.")

    return args


def get_trackbar_values(range_filter):
    values = []

    for i in ["MIN", "MAX"]:
        for j in range_filter:
            v = cv2.getTrackbarPos("%s_%s" % (j, i), "Trackbars")
            values.append(v)

    return values


def main():
    args = get_arguments()

    range_filter = args['filter'].upper()

    if args['image']:
        image = cv2.imread(args['image'])

        if range_filter == 'RGB':
            frame_to_thresh = image.copy()
        else:
            frame_to_thresh = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # else:
    #     camera = cv2.VideoCapture(0)

    setup_trackbars(range_filter)

    if args['webcam']:
        rospy.init_node('range_detector')
        while not rospy.is_shutdown():
            img = rospy.wait_for_message('/cameras/left_hand_camera/image', Image)
            camera_callback(img, (range_filter,))
    else:
        while True:
            threshold(range_filter, frame_to_thresh)


if __name__ == '__main__':
    main()