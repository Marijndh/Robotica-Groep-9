import cv2 as cv
import numpy as np

from brick import Brick
from color import Color
from geometry_utils import GeometryUtils
from instrument import Instrument
from target import Target


class Frame:
    def __init__(self, img, width=0, height=0):
        self.img = img
        self.width = width
        self.height = height
        self.img = cv.resize(self.img, (width, height))
        self.gray_image = cv.cvtColor(self.img, cv.COLOR_BGR2GRAY)
        self.hsv_image = cv.cvtColor(self.img, cv.COLOR_BGR2HSV)
        self.contours, self.hierarchy = None, None
        self.instruments = []
        self.targets = []
        self.bricks = []

    def draw_contours(self):
        for contour in self.contours:
            cv.drawContours(self.img, [contour], -1, (0, 255, 0), 2)

    def find_instruments(self):
        contours = self.contours
        for index in range(len(contours)):
            contour = contours[index]
            area = cv.contourArea(contour)
            # area is calibrated based on the camera position and size of the instruments
            if 3000.0 < area < 4000:
                instrument = Instrument(contour, index, area)
                self.instruments.append(instrument)
                instrument.get_color(self.hsv_image)

    def find_children(self):
        for instrument in self.instruments:
            for j in range(len(self.hierarchy)):
                if self.hierarchy[j][3] == instrument.index and cv.contourArea(
                        self.contours[j]) > 1000:
                    child = self.contours[j]
                    instrument.add_child(child)

    def draw_instruments(self):
        for instrument in self.instruments:
            objects = [instrument.body] + instrument.children
            cv.drawContours(self.img, objects, -1, (0, 255, 0), 3)
            cv.circle(self.img, instrument.centroid, 5, (0, 0, 0), -1)
            instrument.draw_points(self.img)

    def print_instruments(self):
        for instrument in self.instruments:
            print(instrument.__str__())

    def find_targets(self):
        for contour in self.contours:
            area = cv.contourArea(contour)
            if area > 10000 < self.width * self.height * 0.8:
                is_ellipse_flag, center = GeometryUtils.is_ellipse(contour)
                if is_ellipse_flag:
                    self.check_bull_color(contour, center)

    def draw_targets(self):
        for target in self.targets:
            center = target.hitpoint
            ellipse = target.body
            cv.circle(self.img, center, 5, (0, 0, 0), -1)
            cv.ellipse(self.img, ellipse, (0, 255, 0), 2)

    def print_targets(self):
        for target in self.targets:
            print(target.__str__())

    def check_bull_color(self, contour, center):
        ellipse = cv.fitEllipse(contour)
        mask = np.zeros(self.gray_image.shape, dtype=np.uint8)
        cv.circle(mask, center, 5, (255, 255, 255), -1)
        mean = cv.mean(self.hsv_image, mask=mask)
        expected_bulls_eye_color = Color('bulls_eye', np.array([20, 0, 0]), np.array([25, 255, 255]))
        if expected_bulls_eye_color.is_color(mean[0], mean[1], mean[2]):
            self.targets.append(Target(center, ellipse))

    def find_bricks(self):
        contours = self.contours
        for index in range(len(contours)):
            contour = contours[index]
            area = cv.contourArea(contour)
            if 300 < area < 500:
                approx = cv.approxPolyDP(contour, 0.04 * cv.arcLength(contour, True), True)
                if len(approx) == 4:
                    x, y, w, h = cv.boundingRect(contour)
                    ratio = float(w) / h
                    # Controleer of de verhouding dicht bij 1:1 ligt voor vierkanten en rechthoeken
                    if 1.5 > ratio > 1.2:
                        print(ratio)
                        brick = Brick(contour, index)
                        self.bricks.append(brick)
                        brick.get_color(self.hsv_image, self.gray_image)

    def draw_bricks(self):
        for brick in self.bricks:
            cv.putText(self.img, brick.color, brick.centroid, cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            cv.drawContours(self.img, [brick.body], -1, (0, 255, 0), 1)

    def print_bricks(self):
        for brick in self.bricks:
            print(brick.__str__())

    def find_robot(self):
        self.find_bricks()
        for brick in self.bricks:
            cv.drawContours(self.img, [brick.body], -1, (0, 255, 0), 4)
        print(len(self.bricks))
        if len(self.bricks) == 1:
            return self.bricks[0].centroid
        else:
            amount_blue = []
            for brick in self.bricks:
                if brick.color == 'blue':
                    amount_blue.append(brick)
            if len(amount_blue) == 1:
                return amount_blue[0].centroid
        return self.width/2, self.height/2


    def show(self):
        cv.imshow('Result', self.img)
