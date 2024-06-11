import numpy as np
import cv2 as cv
from color_filtering import primary_colors, colors
from create_grid_on_frame import create_grid_on_image


def set_centroid(body):
    # compute the center of the contour
    M = cv.moments(body)
    centroidX = int(M["m10"] / M["m00"])
    centroidY = int(M["m01"] / M["m00"])
    # draw the contour and center of the shape on the image
    centroid = (centroidX, centroidY)
    return centroid


def find_children(instruments, contours, hierarchy):
    for i in instruments:
        for j in range(len(hierarchy)):
            if hierarchy[j][3] == i.index and cv.contourArea(contours[j]) > 1000:
                child = contours[j]
                i.add_child(child)


class Instrument:
    def __init__(self, index, body):
        self.index = index
        self.body = body
        self.color = None
        self.children = []
        self.centroid = set_centroid(body)
        self.hsv = []

    def set_hsv(self, hsv):
        self.hsv = hsv

    def add_child(self, child):
        self.children.append(child)

    def set_color(self, color):
        self.color = color


def draw_instruments(frame, instruments):
    for i in instruments:
        objects = [i.body] + i.children
        cv.drawContours(frame, objects, -1, (0, 255, 0), 3)
        cv.circle(frame, i.centroid, 5, (0, 0, 0), -1)
        if i.color is not None:
            print(str(i.index) + ": " + i.color)


def get_instrument_colors(instruments, hsv):
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


def get_instruments(contours):
    instruments = []
    for cnr in range(len(contours)):
        cnt = contours[cnr]
        area = cv.contourArea(cnt)
        # perimeter = cv.arcLength(cnt, True)
        # factor = 4 * math.pi * area / perimeter ** 2
        if 5000.0 < area < 1080 * 720 * 0.8:  # instrumenten mogen niet groter dan 80% van de afbeelding zijn
            instruments.append(Instrument(cnr, cnt))
    return instruments


def main():
    frame = cv.imread('../Images/twee_scharen.png')
    frame = cv.resize(frame, (1080, 720))
    imgray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    imgray = cv.medianBlur(imgray, 5)
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    _, thresh = cv.threshold(imgray, 127, 255, cv.THRESH_BINARY)
    contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE,
                                          cv.CHAIN_APPROX_SIMPLE)
    hierarchy = hierarchy[0]
    instruments = get_instruments(contours)
    find_children(instruments, contours, hierarchy)
    get_instrument_colors(instruments, hsv)
    draw_instruments(frame, instruments)

    cv.imshow('Contours', frame)
    cv.waitKey(0)
    cv.destroyAllWindows()


if __name__ == '__main__':
    main()
