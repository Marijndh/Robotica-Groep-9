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

    # Draw the found contours on the image
    def draw_contours(self):
        for contour in self.contours:
            area = cv.contourArea(contour)
            # area is calibrated based on the camera position and size of the instruments
            if 1500.0 < area < 3500:
                cv.drawContours(self.img, [contour], -1, (0, 255, 0), 2)

    # Find the instruments within the image
    def find_instruments(self, color):
        found_instruments = []
        right_color = []
        contours = self.contours
        for index in range(len(contours)):
            contour = contours[index]
            area = cv.contourArea(contour)
            # area is calibrated based on the camera position and size of the instruments
            if 1500.0 < area < 3500:
                instrument = Instrument(contour, index, area)
                if instrument.centroid[1] < 440:
                    found_instruments.append(instrument)
                    instrument_color = instrument.get_color(self.hsv_image)
                    if instrument_color == color:
                        right_color.append(instrument)
        if len(right_color) > 0:
            self.instruments = right_color
        else:
            self.instruments = found_instruments

    # Find the children of each instrument within the frame
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

    # Print the details of every found instrument
    def print_instruments(self):
        for instrument in self.instruments:
            print(instrument.__str__())

    def is_close_to_a_target(self, center):
        for target in self.targets:
            centroid = np.array(target.centroid)
            if np.linalg.norm(np.array(center) - centroid) <= 50:
                return True
        return False

    def find_instrument_targets(self):
        result = []
        contours = self.contours
        for index in range(len(contours)):
            contour = contours[index]
            area = cv.contourArea(contour)
            # area is calibrated based on the camera position and size of the instruments
            if 10000.0 < area < 15000:
                instrument = Instrument(contour, index, area)
                instrument.get_color(self.hsv_image)
                if self.check_target_color(instrument.hsv):
                    result.append(instrument)
                    print("Target gevonden: " + str(instrument.centroid), instrument.type)
        return result

    # Find every target within the frame
    def find_targets(self):
        for contour in self.contours:
            area = cv.contourArea(contour)
            if 15000 < area < 20000:
                is_ellipse_flag, center = GeometryUtils.is_ellipse(contour)
                mask = np.zeros(self.gray_image.shape, dtype=np.uint8)
                cv.circle(mask, center, 5, (255, 255, 255), -1)
                mean = cv.mean(self.hsv_image, mask=mask)
                if is_ellipse_flag and self.check_target_color(mean):
                    self.targets.append(Target(center))

    # Draw the targets on the image
    def draw_targets(self):
        for target in self.targets:
            center = target.centroid
            cv.circle(self.img, center, 5, (0, 0, 0), -1)

    # Print details of targets
    def print_targets(self):
        for target in self.targets:
            print(target.__str__())

    # Check if the found contour is the right one by checking the color of the center
    def check_target_color(self, color):
        expected_bulls_eye_color = Color('bulls_eye', np.array([20, 0, 0]), np.array([25, 255, 255]))
        if expected_bulls_eye_color.is_color(color[0], color[1], color[2]):
            return True
        else:
            return False

    # Find every brick (rectangle or square) within the frame
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
                        brick = Brick(contour, index)
                        self.bricks.append(brick)
                        brick.get_color(self.hsv_image, self.gray_image)

    # Draw each brick on the image
    def draw_bricks(self):
        for brick in self.bricks:
            cv.putText(self.img, brick.color, brick.centroid, cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            cv.drawContours(self.img, [brick.body], -1, (0, 255, 0), 1)

    # Print details of bricks
    def print_bricks(self):
        for brick in self.bricks:
            print(brick.__str__())

    # Show the resulting image
    def show(self):
        cv.imshow('Result', self.img)
