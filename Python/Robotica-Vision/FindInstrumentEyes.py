import numpy as np
import cv2 as cv
import math


class Instrument:
    def __init__(self, index, body):
        self.index = index
        self.body = body
        self.children = []

    def add_child(self, child):
        self.children.append(child)


def get_instrument_centroid(instrument):
    # compute the center of the contour
    M = cv.moments()
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    # draw the contour and center of the shape on the image
    cv.drawContours(frame, [c], -1, (0, 255, 0), 2)
    cv.circle(frame, (cX, cY), 7, (255, 255, 255), -1)


# noinspection PyShadowingNames
def draw_instruments(frame, instruments):
    for i in instruments:
        objects = i.children
        objects.append(i.body)
        cv.drawContours(frame, objects, -1, (0, 0, 0), 3)


frame = cv.imread('Images/blauwe-tang.jpeg')
frame = cv.resize(frame, (640, 640))
imgray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
_, thresh = cv.threshold(imgray, 80, 100, cv.THRESH_BINARY)
contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
hierarchy = hierarchy[0]
instruments = []
for cnr in range(len(contours)):
    cnt = contours[cnr]
    area = cv.contourArea(cnt)
    # perimeter = cv.arcLength(cnt, True)
    # factor = 4 * math.pi * area / perimeter ** 2
    if 5000.0 < area < 400000:
        instruments.append(Instrument(cnr, cnt))
#TODO dubbele for loop optimaliseren
for i in instruments:
    child_list = []
    for j in range(len(hierarchy)):
        if hierarchy[j][3] == i.index and cv.contourArea(contours[j]) > 1000:
            i.add_child(contours[j])
draw_instruments(frame, instruments)
cv.imshow('Contours', frame)
cv.waitKey(0)
cv.destroyAllWindows()
