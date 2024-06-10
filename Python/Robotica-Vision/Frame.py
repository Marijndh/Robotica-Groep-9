import cv2 as cv

from instrument import Instrument
from target_processor import TargetProcessor
from geometry_utils import GeometryUtils


class Frame:
    def __init__(self, img, contours=None, width=0, height=0, hierarchy=None):
        self.img = img
        self.contours = contours if contours is not None else []
        self.width = width
        self.height = height
        self.hierarchy = hierarchy if hierarchy is not None else []

    def draw_instruments(self):
        # Placeholder implementation
        for contour in self.contours:
            cv.drawContours(self.img, [contour], -1, (0, 255, 0), 2)

    def get_instruments(self):
        # Placeholder implementation
        instruments = []
        for i, contour in enumerate(self.contours):
            instrument = Instrument(body=contour, index=i)
            instrument.calculate_centroid()
            instrument.calculate_rotation()
            instruments.append(instrument)
        return instruments

    def find_children(self, instruments):
        for instrument in instruments:
            for j in range(len(self.hierarchy)):
                if self.hierarchy[j][3] == instrument.index and cv.contourArea(self.contours[j]) > 1000:
                    child = self.contours[j]
                    instrument.add_child(child)

    def get_targets(self):
        # Placeholder implementation
        targets = []
        for contour in self.contours:
            aspect_ratio, center = GeometryUtils.is_ellipse(contour)
            if aspect_ratio:
                targets.append((center, contour))
        return targets

    def process_targets(self):
        target_processor = TargetProcessor(self.img, self.contours)
        for contour in self.contours:
            area = cv.contourArea(contour)
            if area > 500:
                hull = cv.convexHull(contour)
                m = cv.moments(contour)
                if m["m00"] != 0:
                    centroid_x = int(m["m10"] / m["m00"])
                    centroid_y = int(m["m01"] / m["m00"])
                else:
                    centroid_x, centroid_y = 0, 0
                centroid = [centroid_x, centroid_y]
                extreme_points = [
                    tuple(hull[hull[:, :, 0].argmin()][0]),
                    tuple(hull[hull[:, :, 0].argmax()][0]),
                    tuple(hull[hull[:, :, 1].argmin()][0]),
                    tuple(hull[hull[:, :, 1].argmax()][0]),
                ]
                filtered = target_processor.get_point_of_target(extreme_points, centroid)
                point = filtered["point"]
                handle1 = filtered["handle1"]
                handle2 = filtered["handle2"]
                degrees = target_processor.get_gripper_degrees(point, centroid)
                target_processor.draw_gripper(centroid, degrees)
                cv.circle(self.img, handle1, 5, (255, 0, 0), -1)
                cv.circle(self.img, handle2, 5, (255, 0, 0), -1)
                cv.circle(self.img, point, 5, (0, 255, 0), -1)
                cv.circle(self.img, tuple(centroid), 5, (0, 0, 0), -1)
