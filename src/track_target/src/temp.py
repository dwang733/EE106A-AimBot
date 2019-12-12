#!/usr/bin/env python
import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt



LOWER_THRESH = (0, 54, 209)#(15,65,50)
UPPER_THRESH = (190,83,255)
img = cv.imread('/home/eecs106a/ros_workspaces/EE106A-AimBot/src/track_target/img/archytas_left_camera_6.png')

# img_yuv = cv.cvtColor(img, cv.COLOR_BGR2YUV)

# # equalize the histogram of the Y channel
# img_yuv[:,:,0] = cv.equalizeHist(img_yuv[:,:,0])

# # convert the YUV image back to RGB format
# img_output = cv.cvtColor(img_yuv, cv.COLOR_YUV2BGR)

# cv.imshow('Color input image', img)
# cv.imshow('Histogram equalized', img_output)

gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
equ = cv.equalizeHist(gray)
# equ = cv.inRange(equ, 120, 150)

cv.imshow('original', gray)
cv.imshow('equalized', equ)
cv.waitKey(0)
equ = cv.medianBlur(equ, 13)
cv.imshow('blurred', equ)
canny = cv.Canny(equ, 50, 200)
cv.imshow('canny', canny)
cv.waitKey(0)
img_output = canny
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
    # img_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    # mask = cv.inRange(img_hsv, LOWER_THRESH, UPPER_THRESH)
    # mask = cv.medianBlur(mask, 9)
    # cv.imshow('thresholding', mask)
    # cv.waitKey(0)

    # Find all the contours in the image
    # cnts = cv.findContours(mask.copy(), cv.RETR_EXTERNAL,
    #     cv.CHAIN_APPROX_SIMPLE)
    cnts = cv.findContours(img.copy(), cv.RETR_EXTERNAL,
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
    print(cnts)
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
        cv.waitKey(0)
        print((x, y, radius))
        return (x, y, radius)
circle = detect_target_circle(img_output)

# img = cv.imread('/home/eecs106a/ros_workspaces/EE106A-AimBot/src/track_target/img/archytas_left_camera_5.png')
# gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
# equ = cv.equalizeHist(gray)
# print(gray/equ)
# cv.imshow('original', img)
# cv.imshow('equalized', equ)
# cv.waitKey(0)

# img = cv.imread('/home/eecs106a/ros_workspaces/EE106A-AimBot/src/track_target/img/archytas_left_camera_5.png',0)
# img2 = img.copy()
# template = cv.imread('/home/eecs106a/ros_workspaces/EE106A-AimBot/src/track_target/img/target_image.PNG',0)
# w, h = template.shape[::-1]
# # All the 6 methods for comparison in a list
# methods = ['cv.TM_CCOEFF', 'cv.TM_CCOEFF_NORMED', 'cv.TM_CCORR',
#             'cv.TM_CCORR_NORMED', 'cv.TM_SQDIFF', 'cv.TM_SQDIFF_NORMED']
# for meth in methods:
#     img = img2.copy()
#     method = eval(meth)
#     # Apply template Matching
#     res = cv.matchTemplate(img,template,method)
#     min_val, max_val, min_loc, max_loc = cv.minMaxLoc(res)
#     # If the method is TM_SQDIFF or TM_SQDIFF_NORMED, take minimum
#     if method in [cv.TM_SQDIFF, cv.TM_SQDIFF_NORMED]:
#         top_left = min_loc
#     else:
#         top_left = max_loc
#     bottom_right = (top_left[0] + w, top_left[1] + h)
#     cv.rectangle(img,top_left, bottom_right, 255, 2)
#     plt.subplot(121),plt.imshow(res,cmap = 'gray')
#     plt.title('Matching Result'), plt.xticks([]), plt.yticks([])
#     plt.subplot(122),plt.imshow(img,cmap = 'gray')
#     plt.title('Detected Point'), plt.xticks([]), plt.yticks([])
#     plt.suptitle(meth)
#     plt.show()



# class TargetFinder:
#     def __init__(self, low_thresh, high_thresh, show_circle=True):
#         self.low_thresh = low_thresh
#         self.high_thresh = high_thresh
#         self.show_circle = show_circle

#     # Detect the yellow circle on the target
#     def detect_target_circle(self, img):
#         try:
#             mask = self._find_mask(img)
#             img_watershed = self._watershed(img, mask)
#             img_ff = self._flood_fill(img_watershed)
#             x, y, radius = self._contour(img_ff, img)
#             return x, y, radius
#         except Exception as e:
#             print(e)
#             print('Could not find target')
#             return

#     def _find_mask(self, img):
#         # Convert to HSV color format and threshold on the yellow color
#         img_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
#         mask = cv.inRange(img_hsv, self.low_thresh, self.high_thresh)
#         mask = cv.medianBlur(mask, 9)
#         return mask

#     def _watershed(self, img, mask):
#         _, thresh = cv.threshold(mask, 0, 255, cv.THRESH_OTSU)
#         # noise removal
#         kernel = np.ones((3, 3), np.uint8)
#         opening = cv.morphologyEx(thresh, cv.MORPH_OPEN,kernel, iterations=1)  #changed from 2 to 1
#         # sure background area
#         sure_bg = cv.dilate(opening, kernel, iterations=1)    #changed from 3 to 1
#         # Finding sure foreground area
#         dist_transform = cv.distanceTransform(opening, cv.DIST_L2, 5)
#         _, sure_fg = cv.threshold(dist_transform, 0.7 * dist_transform.max(), 255, 0)
#         # Finding unknown region
#         sure_fg = np.uint8(sure_fg)
#         unknown = cv.subtract(sure_bg, sure_fg)
#         # Marker labelling
#         _, markers = cv.connectedComponents(sure_fg)
#         # Add one to all labels so that sure background is not 0, but 1
#         markers = markers + 1
#         # Now, mark the region of unknown with zero
#         markers[unknown == 255] = 0
#         markers = cv.watershed(img, markers)
#         img = img.copy()
#         img[markers == -1] = [255, 255, 255]
#         img[markers != -1] = [0, 0, 0]
#         return img

#     def _flood_fill(self, img):
#         img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
#         h, w = img.shape[:2]
#         mask = np.zeros((h + 2, w + 2), np.uint8)
#         img_ff = img.copy()
#         cv.floodFill(img_ff, mask, (2, 2), 255)
#         img_ff = cv.bitwise_not(img_ff)
#         return img_ff

#     def _grab_contours(self, cnts):
#         # if the length the contours tuple returned by cv2.findContours
#         # is '2' then we are using either OpenCV v2.4, v4-beta, or
#         # v4-official
#         if len(cnts) == 2:
#             cnts = cnts[0]

#         # if the length of the contours tuple is '3' then we are using
#         # either OpenCV v3, v4-pre, or v4-alpha
#         elif len(cnts) == 3:
#             cnts = cnts[1]

#         # otherwise OpenCV has changed their cv2.findContours return
#         # signature yet again and I have no idea WTH is going on
#         else:
#             raise Exception(("Contours tuple must have length 2 or 3, "
#                 "otherwise OpenCV changed their cv2.findContours return "
#                 "signature yet again. Refer to OpenCV's documentation "
#                 "in that case"))

#         # return the actual contours array
#         return cnts

#     def _contour(self, img, orig_img):
#         cnts = cv.findContours(img, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
#         cnts = self._grab_contours(cnts)
#         # cv.drawContours(img, cnts, -1, (0, 255, 0), 3)
#         # cv.imshow('contours', img)
#         # cv.waitKey(0)

#         # only proceed if at least one contour was found
#         if len(cnts) > 0:
#             # find the largest contour in the mask, then use
#             # it to compute the minimum enclosing circle and
#             # centroid
#             c = max(cnts, key=cv.contourArea)
#             ((x, y), radius) = cv.minEnclosingCircle(c)
#             # only proceed if the radius meets a minimum size
#             if radius > 3:
#                 # draw the circle and centroid on the frame,
#                 # then update the list of tracked points
#                 if self.show_circle:
#                     cv.circle(orig_img, (int(x), int(y)), int(radius),
#                         (0, 255, 255), 2)
#                     cv.circle(orig_img, (int(x), int(y)), 5, (0, 0, 255), -1)
#                     cv.imshow('circle', orig_img)
#                     cv.waitKey(0)
#                     print((x, y, radius))
#                 return x, y, radius


# if __name__ == '__main__':
#     LOWER_THRESH = (15,55,50)
#     UPPER_THRESH = (70,255,255)

#     # target_finder = findTarget(LOWER_THRESH, UPPER_THRESH, '../img/far_target.png')
#     # print(target_finder.detect_target_circle())
#     target_finder = TargetFinder(LOWER_THRESH, UPPER_THRESH)
#     img = cv.imread('../img/archytas_left_camera_5.png')
#     # mask = target_finder._find_mask(img)
#     # _, thresh = cv.threshold(mask, 0, 255, cv.THRESH_OTSU)
#     # cv.imshow('mask', cv.resize(mask, None, fx=0.5, fy=0.5))
#     # cv.imshow('thresh', cv.resize(thresh, None, fx=0.5, fy=0.5))
#     # cv.waitKey(0)
#     circle = target_finder.detect_target_circle(img)
# <<<<<<< HEAD
# import os
# # # import queue


# # class TargetFinder:
# #     def __init__(self, low_thresh, high_thresh, show_circle=True):
# #         self.low_thresh = low_thresh
# #         self.high_thresh = high_thresh
# #         self.show_circle = show_circle

# #     # Detect the yellow circle on the target
# #     def detect_target_circle(self, img):
# #         try:
# #             mask = self._find_mask(img)
# #             img_watershed = self._watershed(img, mask)
# #             img_ff = self._flood_fill(img_watershed)
# #             x, y, radius = self._contour(img_ff, img)
# #             return x, y, radius
# #         except Exception as e:
# #             print(e)
# #             print('Could not find target')
# #             return

# #     def _find_mask(self, img):
# #         # Convert to HSV color format and threshold on the yellow color
# #         img_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
# #         mask = cv.inRange(img_hsv, self.low_thresh, self.high_thresh)
# #         mask = cv.medianBlur(mask, 9)
# #         return mask

# #     def _watershed(self, img, mask):
# #         _, thresh = cv.threshold(mask, 0, 255, cv.THRESH_OTSU)
# #         # noise removal
# #         kernel = np.ones((3, 3), np.uint8)
# #         opening = cv.morphologyEx(thresh, cv.MORPH_OPEN,kernel, iterations=1)  #changed from 2 to 1
# #         # sure background area
# #         sure_bg = cv.dilate(opening, kernel, iterations=1)    #changed from 3 to 1
# #         # Finding sure foreground area
# #         dist_transform = cv.distanceTransform(opening, cv.DIST_L2, 5)
# #         _, sure_fg = cv.threshold(dist_transform, 0.7 * dist_transform.max(), 255, 0)
# #         # Finding unknown region
# #         sure_fg = np.uint8(sure_fg)
# #         unknown = cv.subtract(sure_bg, sure_fg)
# #         # Marker labelling
# #         _, markers = cv.connectedComponents(sure_fg)
# #         # Add one to all labels so that sure background is not 0, but 1
# #         markers = markers + 1
# #         # Now, mark the region of unknown with zero
# #         markers[unknown == 255] = 0
# #         markers = cv.watershed(img, markers)
# #         img = img.copy()
# #         img[markers == -1] = [255, 255, 255]
# #         img[markers != -1] = [0, 0, 0]
# #         return img

# #     def _flood_fill(self, img):
# #         img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
# #         h, w = img.shape[:2]
# #         mask = np.zeros((h + 2, w + 2), np.uint8)
# #         img_ff = img.copy()
# #         cv.floodFill(img_ff, mask, (2, 2), 255)
# #         img_ff = cv.bitwise_not(img_ff)
# #         return img_ff

# #     def _grab_contours(self, cnts):
# #         # if the length the contours tuple returned by cv2.findContours
# #         # is '2' then we are using either OpenCV v2.4, v4-beta, or
# #         # v4-official
# #         if len(cnts) == 2:
# #             cnts = cnts[0]

# #         # if the length of the contours tuple is '3' then we are using
# #         # either OpenCV v3, v4-pre, or v4-alpha
# #         elif len(cnts) == 3:
# #             cnts = cnts[1]

# #         # otherwise OpenCV has changed their cv2.findContours return
# #         # signature yet again and I have no idea WTH is going on
# #         else:
# #             raise Exception(("Contours tuple must have length 2 or 3, "
# #                 "otherwise OpenCV changed their cv2.findContours return "
# #                 "signature yet again. Refer to OpenCV's documentation "
# #                 "in that case"))

# #         # return the actual contours array
# #         return cnts

# #     def _contour(self, img, orig_img):
# #         cnts = cv.findContours(img, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
# #         cnts = self._grab_contours(cnts)
# #         # cv.drawContours(img, cnts, -1, (0, 255, 0), 3)
# #         # cv.imshow('contours', img)
# #         # cv.waitKey(0)

# #         # only proceed if at least one contour was found
# #         if len(cnts) > 0:
# #             # find the largest contour in the mask, then use
# #             # it to compute the minimum enclosing circle and
# #             # centroid
# #             c = max(cnts, key=cv.contourArea)
# #             ((x, y), radius) = cv.minEnclosingCircle(c)
# #             # only proceed if the radius meets a minimum size
# #             if radius > 3:
# #                 # draw the circle and centroid on the frame,
# #                 # then update the list of tracked points
# #                 if self.show_circle:
# #                     cv.circle(orig_img, (int(x), int(y)), int(radius),
# #                         (0, 255, 255), 2)
# #                     cv.circle(orig_img, (int(x), int(y)), 5, (0, 0, 255), -1)
# #                     cv.imshow('circle', orig_img)
# #                     cv.waitKey(0)
# #                     print((x, y, radius))
# #                 return x, y, radius


# # if __name__ == '__main__':
# #     LOWER_THRESH = (15,55,50)
# #     UPPER_THRESH = (70,255,255)

# #     # target_finder = findTarget(LOWER_THRESH, UPPER_THRESH, '../img/far_target.png')
# #     # print(target_finder.detect_target_circle())
# #     target_finder = TargetFinder(LOWER_THRESH, UPPER_THRESH)
# #     # img = cv.imread('close_target.png')
# #     # img = cv.imread(os.path.abspath(__file__ + "/../img/close_target.png"))
# #     img = cv.imread('/home/eecs106a/ros_workspaces/EE106A-AimBot/src/track_target/img/archytas_left_camera_5.png')
# #     # print(os.path.abspath(__file__ + "/../img/close_target.png"))
# #     # mask = target_finder._find_mask(img)
# #     # _, thresh = cv.threshold(mask, 0, 255, cv.THRESH_OTSU)
# #     # cv.imshow('mask', cv.resize(mask, None, fx=0.5, fy=0.5))
# #     # cv.imshow('thresh', cv.resize(thresh, None, fx=0.5, fy=0.5))
# #     # cv.waitKey(0)
# #     circle = target_finder.detect_target_circle(img)





# import cv2

# image = cv2.imread('/home/eecs106a/ros_workspaces/EE106A-AimBot/src/track_target/img/archytas_left_camera_4.png')
# print(image.shape)
# img_hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
# # mask = cv.inRange(img_hsv, (15,65,50), (65,255,255))
# # mask = cv.medianBlur(mask, 9)
# for i in range(0,255,10):
#     for j in range(i+10,255,10):
#         mask = cv.inRange(img_hsv, (15,i,50), (65,j,255))
#         m = mask[i][j]
#         # cv.imshow('thresholding', m)
#         # cv.waitKey(0)

#         # gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) 
#         blur = cv2.medianBlur(mask, 17)
#         # print(blur.shape)
#         thresh = blur#cv2.adaptiveThreshold(blur,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV,27,6)
#         # cv.imshow('t', thresh)
#         # cv.waitKey(0)
#         kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
#         close = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=2)
#         dilate = cv2.dilate(close, kernel, iterations=2)
#         # cv.imshow('dilate', dilate)
#         # cv.waitKey(0)
#         cnts = cv2.findContours(dilate, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#         # cnts = cnts[0] if len(cnts) == 2 else cnts[1]
#         # cnts = sorted(cnts, key=cv2.contourArea, reverse=True)[:10]


#         def grab_contours(cnts):
#             # if the length the contours tuple returned by cv2.findContours
#             # is '2' then we are using either OpenCV v2.4, v4-beta, or
#             # v4-official
#             if len(cnts) == 2:
#                 cnts = cnts[0]

#             # if the length of the contours tuple is '3' then we are using
#             # either OpenCV v3, v4-pre, or v4-alpha
#             elif len(cnts) == 3:
#                 cnts = cnts[1]

#             # otherwise OpenCV has changed their cv2.findContours return
#             # signature yet again and I have no idea WTH is going on
#             else:
#                 raise Exception(("Contours tuple must have length 2 or 3, "
#                     "otherwise OpenCV changed their cv2.findContours return "
#                     "signature yet again. Refer to OpenCV's documentation "
#                     "in that case"))

#             # return the actual contours array
#             return cnts
#         cnts = grab_contours(cnts)
#         # cv.drawContours(img, cnts, -1, (0,255,0), 3)
#         # cv.imshow('contours', img)
#         # cv.waitKey(1)

#         # only proceed if at least one contour was found
#         if len(cnts) == 0:
#             continue

#         # # find the largest contour in the mask, then use
#         # # it to compute the minimum enclosing circle and
#         # # centroid
#         c = max(cnts, key=cv2.contourArea)
#         ((x, y), radius) = cv2.minEnclosingCircle(c)
#         # M = cv.moments(c)
#         # center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

#         # only proceed if the radius meets a minimum size
#         if radius > 3:
#             # draw the circle and centroid on the frame,
#             # then update the list of tracked points
#             cv2.circle(image, (int(x), int(y)), int(radius),
#                 (0, 255, 255), 2)
#             cv2.circle(image, (int(x), int(y)), 5, (0, 0, 255), -1)
#             # cv2.imshow('circle', image)
#             # cv2.waitKey(0)
#             # print((x, y, radius))

# # minimum_area = 500
# # for c in cnts:
# #     area = cv2.contourArea(c)
# #     if area > minimum_area:
# #         # Find centroid
# #         M = cv2.moments(c)
# #         cX = int(M["m10"] / M["m00"])
# #         cY = int(M["m01"] / M["m00"])
# #         cv2.circle(image, (cX, cY), 20, (36, 255, 12), 2) 
# #         x,y,w,h = cv2.boundingRect(c)
# #         cv2.putText(image, 'Radius: {}'.format(w/2), (10,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (36,255,12), 2)
# #         break

# # cv2.imshow('thresh', thresh)
# # cv2.imshow('close', close)
# # cv2.imshow('image', image)
# # cv2.waitKey(0)