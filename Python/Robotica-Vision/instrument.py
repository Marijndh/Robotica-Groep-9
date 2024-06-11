import cv2 as cv
import numpy as np
from geometry_utils import GeometryUtils
from color_manager import ColorManager


def get_points(contour, centroid):
    hull = cv.convexHull(contour)

    extreme_left = tuple(hull[hull[:, :, 0].argmin()][0])
    extreme_right = tuple(hull[hull[:, :, 0].argmax()][0])
    extreme_top = tuple(hull[hull[:, :, 1].argmin()][0])
    extreme_bottom = tuple(hull[hull[:, :, 1].argmax()][0])

    points = np.array([extreme_left, extreme_right, extreme_top, extreme_bottom])

    distances_to_centroid = np.linalg.norm(points - centroid, axis=1)

    point_index = np.argmax(distances_to_centroid)
    point = points[point_index]

    remaining_points = np.delete(points, point_index, axis=0)

    direction = point - centroid

    perpendicular_direction = np.array([-direction[1], direction[0]])

    handle1 = None
    handle2 = None

    for pt in remaining_points:
        vector = pt - centroid
        dot_product_main = np.dot(vector, direction)
        dot_product_perp = np.dot(vector, perpendicular_direction)

        if dot_product_main > 0:
            if dot_product_perp > 0:
                if handle1 is None or np.linalg.norm(pt - centroid) < np.linalg.norm(handle1 - centroid):
                    handle1 = pt
            else:
                if handle2 is None or np.linalg.norm(pt - centroid) < np.linalg.norm(handle2 - centroid):
                    handle2 = pt
        else:
            if dot_product_perp > 0:
                if handle1 is None or np.linalg.norm(pt - centroid) < np.linalg.norm(handle1 - centroid):
                    handle1 = pt
            else:
                if handle2 is None or np.linalg.norm(pt - centroid) < np.linalg.norm(handle2 - centroid):
                    handle2 = pt

    categorized_points = {
        'handle1': handle1,
        'handle2': handle2,
        'point': point,
    }

    return categorized_points


def calculate_centroid(body):
    """Calculate the centroid of a contour body."""
    M = cv.moments(body)
    if M["m00"] != 0:
        return int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
    else:
        return 0, 0


def get_rotation(point, centroid):
    # Determine vector
    vector = point - centroid

    # Calculate orientation
    theta = np.arctan2(vector[1], vector[0])
    degrees = np.degrees(theta) + 90

    # Make sure angle is between 0 and 360
    if degrees < 0:
        degrees += 360

    return degrees


class Instrument:
    def __init__(self, body, index, area):
        self.color = ""
        self.body = body
        self.index = index
        self.area = area
        self.children = []
        self.centroid = calculate_centroid(body)
        self.points = get_points(body, self.centroid)
        self.rotation = get_rotation(self.points["point"], self.centroid)

    def __str__(self):
        return "Instrument (" + str(self.index) + "): centroid:" + self.centroid.__str__() + ", color: " + self.color + ", rotation: "\
            + str(self.rotation) + ", area: " + str(self.area) + "\n"

    def add_child(self, child):
        """Add a child contour to the instrument."""
        self.children.append(child)

    def draw_points(self, img):
        point = self.points["point"]
        handle1 = self.points["handle1"]
        handle2 = self.points["handle2"]
        cv.circle(img, handle1, 5, (0, 0, 0), -1)
        cv.circle(img, handle2, 5, (0, 0, 0), -1)
        cv.circle(img, point, 5, (0, 0, 0), -1)

    def get_color(self, hsv_image):
        color_manager = ColorManager()
        mask = np.zeros(hsv_image.shape[:2], np.uint8)
        contours = [self.body] + self.children
        cv.drawContours(mask, contours, -1, 255, -1)
        mean = cv.mean(hsv_image, mask=mask)
        primary_colors = color_manager.primary_colors
        colors = color_manager.colors
        h = mean[0]
        s = mean[1]
        v = mean[2]
        for color_name in primary_colors:
            if color_name in colors:
                color = colors[color_name]
                if color.is_color(h, s, v):
                    self.color = color_name
                    break
