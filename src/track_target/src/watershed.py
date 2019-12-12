import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt
import queue


class findTarget():
    def __init__(self, low_thresh, high_thresh, img_path):
        self.low_thresh = low_thresh
        self.high_thresh = high_thresh
        self.img = cv.imread(img_path)
    def findMask(self):
        # Convert to HSV color format and threshold on the yellow color
        img_hsv = cv.cvtColor(self.img, cv.COLOR_BGR2HSV)
        mask = cv.inRange(img_hsv, self.low_thresh, self.high_thresh)
        mask = cv.medianBlur(mask, 9)
        return mask
    def watershed(self, mask):
        ret, thresh = cv.threshold(mask,0,255,cv.THRESH_OTSU)
        # noise removal
        kernel = np.ones((3,3),np.uint8)
        opening = cv.morphologyEx(thresh,cv.MORPH_OPEN,kernel, iterations = 1)  #changed from 2 to 1
        # sure background area
        sure_bg = cv.dilate(opening,kernel,iterations=1)    #changed from 3 to 1
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
        print(markers.dtype)
        markers = cv.watershed(self.img,markers)
        img = self.img.copy()
        img[markers == -1] = [255,255,255]
        img[markers != -1] = [0,0,0]
        return img
    def floodFill(self, img):
        img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        h, w = img.shape[:2]
        mask = np.zeros((h+2, w+2), np.uint8)
        img_ff = img.copy()
        cv.floodFill(img_ff, mask, (2, 2), 255)
        img_ff = cv.bitwise_not(img_ff)
        img = img_ff
        return img
    def grab_contours(self, cnts):
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
    def contour(self, img):
        cnts = cv.findContours(img, cv.RETR_EXTERNAL,
            cv.CHAIN_APPROX_SIMPLE)
        cnts = self.grab_contours(cnts)
        print(cnts)
        cv.drawContours(img, cnts, -1, (0,255,0), 3)
        # cv.imshow('contours', img)
        # cv.waitKey(0)

        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv.contourArea)
            ((x, y), radius) = cv.minEnclosingCircle(c)
            # only proceed if the radius meets a minimum size
            if radius > 3:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv.circle(self.img, (int(x), int(y)), int(radius),
                    (0, 255, 255), 2)
                cv.circle(self.img, (int(x), int(y)), 5, (0, 0, 255), -1)
                cv.imshow('circle', self.img)
                cv.waitKey(0)
                print((x, y, radius))
                return x, y, radius
    def getCircle(self):
        try:
            mask = self.findMask()
            img = self.watershed(mask)
            img = self.floodFill(img)
            x, y, radius = self.contour(img)
            return x, y, radius
        except:
            print('Could not find target')
            return None, None, None

LOWER_THRESH = (15,55,50)
UPPER_THRESH = (70,255,255)

target_finder = findTarget(LOWER_THRESH, UPPER_THRESH, '../img/far_target.png')
print(target_finder.getCircle())
