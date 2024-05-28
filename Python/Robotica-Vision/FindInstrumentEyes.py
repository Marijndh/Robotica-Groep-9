import numpy as np
import cv2 as cv
import math

def get_contour_centroid(c):
    # compute the center of the contour
    M = cv.moments(c)
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    # draw the contour and center of the shape on the image
    cv.drawContours(frame, [c], -1, (0, 255, 0), 2)
    cv.circle(frame, (cX, cY), 7, (255, 255, 255), -1)

def test():
    frame = cv.imread('Images/roze-tang.jpeg')
    frame = cv.resize(frame, (640, 640))
    imgray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    _, thresh = cv.threshold(imgray,80,100,cv.THRESH_BINARY)
    contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    hierarchy = hierarchy[0]
    instruments = []
    indexes = []
    for cnr in range(len(contours)):
        cnt = contours[cnr]
        area = cv.contourArea(cnt)
        #perimeter = cv.arcLength(cnt, True)
        #factor = 4 * math.pi * area / perimeter ** 2
        if 5000.0 < area < 400000:
            print(str(cnr)+": "+str(area))
            indexes.append(cnr)
            instruments.append(cnt)
    for i in range(len(instruments)):
        child = hierarchy[i][2]  # Get the first child
        while child != -1:
            instruments.append(contours[child])
            child = hierarchy[child][0]  # Get the next sibling

    cv.drawContours(frame, instruments, -1, (0, 0, 0), 3)
    cv.imshow('Contours', frame)
    cv.waitKey(0)
    cv.destroyAllWindows()


test()
