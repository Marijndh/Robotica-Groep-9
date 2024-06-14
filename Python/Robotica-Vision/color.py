import cv2 as cv


class Color:
    def __init__(self, name, lower_bound, upper_bound):
        self.name = name
        self.lower_bound = lower_bound
        self.upper_bound = upper_bound

    def get_mask(self, hsv_frame):
        return cv.inRange(hsv_frame, self.lower_bound, self.upper_bound)

    def is_color(self, h, s, v):
        upper = self.upper_bound
        lower = self.lower_bound
        return lower[0] < h < upper[0] and lower[1] < s < upper[1] and lower[2] < v < upper[2]
