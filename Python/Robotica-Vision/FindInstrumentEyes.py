import numpy as np
import cv2 as cv
from ColorFiltering import Color, colors, primary_colors

frame = cv.imread('Images/blauwe-tang.jpeg')
frame = cv.resize(frame, (640, 640))
imgray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)


class Instrument:
    def __init__(self, index, body):
        self.index = index
        self.body = body
        self.color = None
        self.children = []

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
        objects = i.children
        objects.append(i.body)
        cv.drawContours(frame, objects, -1, (0, 255, 0), 3)

#https://cullensun.medium.com/agglomerative-clustering-for-opencv-contours-cd74719b678e
def get_instrument_colors(frame, instruments):
    combined_mask = None
    colored_contours = []
    for color_name in primary_colors:
        if color_name in colors:
            color = colors[color_name]
            mask = color.get_mask(hsv)

            if combined_mask is None:
                combined_mask = mask
            else:
                combined_mask = combined_mask | mask

            contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL,
                                          cv.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                if cv.contourArea(contour) > 1000:
                    colored_contours.append([color_name, contour])
                    cv.drawContours(frame, [contour], -1, (255, 0, 0), 2)
    for i in instruments:
        for j in colored_contours:
            print(cv.matchShapes(i.body, j[1], cv.CONTOURS_MATCH_I3, 0.0))


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
get_instrument_colors(frame, instruments)
draw_instruments(frame, instruments)
cv.imshow('Contours', frame)
cv.waitKey(0)
cv.destroyAllWindows()
