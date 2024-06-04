import numpy as np
import cv2 as cv
from ColorFiltering import primary_colors, colors
from create_grid_on_frame import create_grid_on_image


frame = cv.imread('Images/twee_scharen.png')
frame = cv.resize(frame, (640, 640))
imgray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
imgray = cv.medianBlur(imgray, 5)
hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)


class Instrument:
    def __init__(self, index, body):
        self.index = index
        self.body = body
        self.color = None
        self.children = []
        self.centroid = [0, 0]
        self.hsv = []

    def set_hsv(self, hsv):
        self.hsv = hsv

    def add_child(self, child):
        self.children.append(child)

    def set_color(self, color):
        self.color = color


def get_instrument_centroid(instrument):
    # compute the center of the contour
    M = cv.moments(instrument.body)
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    # draw the contour and center of the shape on the image
    cv.circle(frame, (cX, cY), 7, (255, 255, 255), -1)


def draw_instruments(frame, instruments):
    for i in instruments:
        objects = [i.body] + i.children
        cv.drawContours(frame, objects, -1, (0, 255, 0), 3)


def get_instrument_colors(instruments):
    for instrument in instruments:
        mask = np.zeros(hsv.shape[:2], np.uint8)
        contours = [instrument.body] + instrument.children
        cv.drawContours(mask, contours, -1, 255, -1)
        mean = cv.mean(hsv, mask=mask)
        instrument.set_hsv(mean)
        for color_name in primary_colors:
            if color_name in colors:
                color = colors[color_name]
                if color.is_color(mean[0], mean[1], mean[2]):
                    instrument.set_color(color_name)
                    break


_, thresh = cv.threshold(imgray, 127, 255, cv.THRESH_BINARY)
contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE,
                                      cv.CHAIN_APPROX_SIMPLE)
hierarchy = hierarchy[0]
instruments = []
for cnr in range(len(contours)):
    cnt = contours[cnr]
    area = cv.contourArea(cnt)
    # perimeter = cv.arcLength(cnt, True)
    # factor = 4 * math.pi * area / perimeter ** 2
    if 20000.0 < area < 360000:
        instruments.append(Instrument(cnr, cnt))
for i in instruments:
    child_list = []
    for j in range(len(hierarchy)):
        if hierarchy[j][3] == i.index and cv.contourArea(contours[j]) > 1000:
            i.add_child(contours[j])
get_instrument_colors(instruments)
draw_instruments(frame, instruments)
for i in instruments:
    if i.color is not None:
        print(str(i.index) + ": " + i.color)
    get_instrument_centroid(i)
cv.imshow('Contours', frame)
cv.imshow('Grid', create_grid_on_image(frame, 10, 640))
cv.waitKey(0)
cv.destroyAllWindows()
