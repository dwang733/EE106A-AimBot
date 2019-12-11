import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt



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
    # cv.imshow('camera', img)
    # cv.waitKey(1)
    # Convert to HSV color format and threshold on the yellow color
    img_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    mask = cv.inRange(img_hsv, LOWER_THRESH, UPPER_THRESH)
    mask = cv.medianBlur(mask, 9)
    return mask

    # # Find all the contours in the image
    # cnts = cv.findContours(mask.copy(), cv.RETR_EXTERNAL,
    #     cv.CHAIN_APPROX_SIMPLE)
    # cnts = grab_contours(cnts)
    # # cv.drawContours(img, cnts, -1, (0,255,0), 3)
    # # cv.imshow('contours', img)
    # # cv.waitKey(1)
    #
    # # only proceed if at least one contour was found
    # if len(cnts) > 0:
    #     # find the largest contour in the mask, then use
    #     # it to compute the minimum enclosing circle and
    #     # centroid
    #     c = max(cnts, key=cv.contourArea)
    #     ((x, y), radius) = cv.minEnclosingCircle(c)
    #     # M = cv.moments(c)
    #     # center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
    #
    #     # only proceed if the radius meets a minimum size
    #     if radius > 3:
    #         # draw the circle and centroid on the frame,
    #         # then update the list of tracked points
    #         cv.circle(img, (int(x), int(y)), int(radius),
    #             (0, 255, 255), 2)
    #         cv.circle(img, (int(x), int(y)), 5, (0, 0, 255), -1)
    #         cv.imshow('circle', img)
    #         cv.waitKey(1)
    #         print((x, y, radius))
    #         return (x, y, radius)


# Threshold on yellow
# LOWER_THRESH = (15,50,50)
LOWER_THRESH = (15,55,50)
# UPPER_THRESH = (50,255,255)
UPPER_THRESH = (70,255,255)
# CAMERA_HEIGHT = 800
# CAMERA_WIDTH = 1280

# Diameter (in m) of the circle detected by the algorithm
CIRCLE_DIAMETER = 0.048
# Diameter (in m) of the target
TARGET_DIAMETER = 0.250
# Distance (in m) from camera which causes target to be the height of the camera image
TARGET_FULL_DIST = 0.118
# Bridge used to convert camera image to OpenCV format
img = cv.imread('../img/archytas_left_camera_5.png')
ogimg = img.copy()
CAMERA_HEIGHT, CAMERA_WIDTH, _ = img.shape
mask = detect_target_circle(img)


# img = cv.imread('../img/archytas_left_camera.png')
# cv.imshow('image', img)
# cv.waitKey(0)
gray = mask#cv.cvtColor(img,cv.COLOR_BGR2GRAY)
ret, thresh = cv.threshold(gray,0,255,cv.THRESH_OTSU)
# cv.imshow('image', thresh)
# cv.waitKey(0)
# noise removal
kernel = np.ones((3,3),np.uint8)
opening = cv.morphologyEx(thresh,cv.MORPH_OPEN,kernel, iterations = 2)
# sure background area
sure_bg = cv.dilate(opening,kernel,iterations=3)
# Finding sure foreground area
dist_transform = cv.distanceTransform(opening,cv.DIST_L2,5)
ret, sure_fg = cv.threshold(dist_transform,0.7*dist_transform.max(),255,0)
# Finding unknown region
sure_fg = np.uint8(sure_fg)
unknown = cv.subtract(sure_bg,sure_fg)
# Marker labelling
ret, markers = cv.connectedComponents(sure_fg)
# Add one to all labels so that sure background is not 0, but 1
markers = markers+1
# Now, mark the region of unknown with zero
markers[unknown==255] = 0
markers = cv.watershed(img,markers)
img[markers == -1] = [255,255,255]
img[markers != -1] = [0,0,0]
# print()
# cv.imshow('image', img)
# cv.waitKey(0)
# print(np.sum(img, axis=2))
# vis2 = cv.CreateMat(h, w, cv.CV_32FC3)
img = np.sum(img, axis=2).astype('uint8')
img = cv.UMat(img)

# img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
# # detect circles in the image
# circles = cv.HoughCircles(img, cv.HOUGH_GRADIENT, 0.2, 100)
# print(circles)
# # ensure at least some circles were found
# if circles is not None:
# 	# convert the (x, y) coordinates and radius of the circles to integers
# 	circles = np.round(circles[0, :]).astype("int")
#
# 	# loop over the (x, y) coordinates and radius of the circles
# 	for (x, y, r) in circles:
# 		# draw the circle in the output image, then draw a rectangle
# 		# corresponding to the center of the circle
# 		cv.circle(output, (x, y), r, (0, 255, 0), 4)
# 		cv.rectangle(output, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
#
# 	# show the output image
# 	cv.imshow("output", np.hstack([image, output]))
# 	cv.waitKey(0)


# Find all the contours in the image

cnts = cv.findContours(img, cv.RETR_EXTERNAL,
    cv.CHAIN_APPROX_SIMPLE)
cnts = grab_contours(cnts)
print(cnts)
cv.drawContours(img, cnts, -1, (0,255,0), 3)
cv.imshow('contours', img)
cv.waitKey(0)

# only proceed if at least one contour was found
if len(cnts) > 0:
    # find the largest contour in the mask, then use
    # it to compute the minimum enclosing circle and
    # centroid
    c = max(cnts, key=cv.contourArea)
    ((x, y), radius) = cv.minEnclosingCircle(c)
    # M = cv.moments(c)
    # center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

    # only proceed if the radius meets a minimum size
    if radius > 3:
        # draw the circle and centroid on the frame,
        # then update the list of tracked points
        cv.circle(ogimg, (int(x), int(y)), int(radius),
            (0, 255, 255), 2)
        cv.circle(ogimg, (int(x), int(y)), 5, (0, 0, 255), -1)
        cv.imshow('circle', ogimg)
        cv.waitKey(0)
        print((x, y, radius))
