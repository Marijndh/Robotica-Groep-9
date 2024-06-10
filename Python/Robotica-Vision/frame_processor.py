import cv2 as cv
import numpy as np
from instrument import Instrument

class FrameProcessor:
    def __init__(self, image_path):
        self.image_path = image_path
        self.img = cv.imread(image_path)
        self.img = cv.resize(self.img, (1080, 720))
        self.gray_img = cv.cvtColor(self.img, cv.COLOR_BGR2GRAY)
        self.gray_img = cv.medianBlur(self.gray_img, 5)
        self.contours = []
        self.hierarchy = []  # Initialize as an empty list
        self.instruments = []

    def process_frame(self):
        _, thresh = cv.threshold(self.gray_img, 127, 255, cv.THRESH_BINARY)
        self.contours, self.hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        self.hierarchy = self.hierarchy[0]

    def get_instruments(self):
        self.instruments = [Instrument(body=contour, index=i) for i, contour in enumerate(self.contours)]
        for instrument in self.instruments:
            instrument.calculate_centroid()
            instrument.calculate_rotation()
        return self.instruments

    def find_children(self):
        if self.hierarchy is not None:
            for instrument in self.instruments:
                for j in range(len(self.hierarchy)):
                    if self.hierarchy[j][3] == instrument.index and cv.contourArea(self.contours[j]) > 1000:
                        child = self.contours[j]
                        instrument.add_child(child)

    def draw_centroids(self, points):
        for p in points:
            cv.circle(self.img, p, 5, (0, 0, 0), -1)
        cv.imshow(self.image_path, self.img)
