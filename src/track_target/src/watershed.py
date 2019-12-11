import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt
import queue


class TargetFinder:
    def __init__(self, low_thresh, high_thresh, show_circle=True):
        self.low_thresh = low_thresh
        self.high_thresh = high_thresh
        self.show_circle = show_circle

    # Detect the yellow circle on the target
    def detect_target_circle(self, img):
        try:
            mask = self._find_mask(img)
            img_watershed = self._watershed(img, mask)
            img_ff = self._flood_fill(img_watershed)
            x, y, radius = self._contour(img_ff, img)
            return x, y, radius
        except Exception as e:
            print(e)
            print('Could not find target')
            return

    def _find_mask(self, img):
        # Convert to HSV color format and threshold on the yellow color
        img_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        mask = cv.inRange(img_hsv, self.low_thresh, self.high_thresh)
        mask = cv.medianBlur(mask, 9)
        return mask

    def _watershed(self, img, mask):
        _, thresh = cv.threshold(mask, 0, 255, cv.THRESH_OTSU)
        # noise removal
        kernel = np.ones((3, 3), np.uint8)
        opening = cv.morphologyEx(thresh, cv.MORPH_OPEN,kernel, iterations=1)  #changed from 2 to 1
        # sure background area
        sure_bg = cv.dilate(opening, kernel, iterations=1)    #changed from 3 to 1
        # Finding sure foreground area
        dist_transform = cv.distanceTransform(opening, cv.DIST_L2, 5)
        _, sure_fg = cv.threshold(dist_transform, 0.7 * dist_transform.max(), 255, 0)
        # Finding unknown region
        sure_fg = np.uint8(sure_fg)
        unknown = cv.subtract(sure_bg, sure_fg)
        # Marker labelling
        _, markers = cv.connectedComponents(sure_fg)
        # Add one to all labels so that sure background is not 0, but 1
        markers = markers + 1
        # Now, mark the region of unknown with zero
        markers[unknown == 255] = 0
        markers = cv.watershed(img, markers)
        img = img.copy()
        img[markers == -1] = [255, 255, 255]
        img[markers != -1] = [0, 0, 0]
        return img

    def _flood_fill(self, img):
        img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        h, w = img.shape[:2]
        mask = np.zeros((h + 2, w + 2), np.uint8)
        img_ff = img.copy()
        cv.floodFill(img_ff, mask, (2, 2), 255)
        img_ff = cv.bitwise_not(img_ff)
        return img_ff

    def _grab_contours(self, cnts):
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

    def _contour(self, img, orig_img):
        cnts = cv.findContours(img, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        cnts = self._grab_contours(cnts)
        # cv.drawContours(img, cnts, -1, (0, 255, 0), 3)
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
                if self.show_circle:
                    cv.circle(orig_img, (int(x), int(y)), int(radius),
                        (0, 255, 255), 2)
                    cv.circle(orig_img, (int(x), int(y)), 5, (0, 0, 255), -1)
                    cv.imshow('circle', orig_img)
                    cv.waitKey(1)
                    print((x, y, radius))
                return x, y, radius


if __name__ == '__main__':
    LOWER_THRESH = (15,55,50)
    UPPER_THRESH = (70,255,255)

    # target_finder = findTarget(LOWER_THRESH, UPPER_THRESH, '../img/far_target.png')
    # print(target_finder.detect_target_circle())
    target_finder = TargetFinder(LOWER_THRESH, UPPER_THRESH)
    img = cv.imread('../img/archytas_left_camera_5.png')
    # mask = target_finder._find_mask(img)
    # _, thresh = cv.threshold(mask, 0, 255, cv.THRESH_OTSU)
    # cv.imshow('mask', cv.resize(mask, None, fx=0.5, fy=0.5))
    # cv.imshow('thresh', cv.resize(thresh, None, fx=0.5, fy=0.5))
    # cv.waitKey(0)
    circle = target_finder.detect_target_circle(img)
