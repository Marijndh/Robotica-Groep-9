import cv2 as cv
import numpy as np
from color_manager import ColorManager
from geometry_utils import GeometryUtils


class Brick:
    def __init__(self, body, index):
        self.color = ""
        self.hsv = []
        self.index = index
        self.body = body
        self.centroid = GeometryUtils.calculate_centroid(body)
        self.direction = 'Not found'

    def __str__(self):
        return "Brick (" + str(
            self.index) + "): centroid:" + self.centroid.__str__() + ", color: " + self.color + ", direction: " + self.direction + ", hsv: " + str(
            self.hsv) + "\n"

    def get_color(self, hsv_image, gray_image):
        color_manager = ColorManager()
        mask = np.zeros(gray_image.shape, dtype=np.uint8)
        cv.circle(mask, self.centroid, 4, (255, 255, 255), -1)
        mean = cv.mean(hsv_image, mask=mask)
        mask2 = np.zeros(gray_image.shape, dtype=np.uint8)
        cv.drawContours(mask2, [self.body], -1, (255, 255, 255), -1)
        mean2 = cv.mean(hsv_image, mask=mask)
        h = mean[0]
        s = mean[1]
        v = mean[2]
        self.hsv = mean2
        primary_colors = color_manager.primary_colors
        colors = color_manager.colors
        for color_name in primary_colors:
            if color_name in colors:
                color = colors[color_name]
                if color.is_color(h, s, v):
                    self.color = color_name
                    break
        if self.color == "":
            self.color = 'silver'
