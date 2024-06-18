import cv2 as cv
import numpy as np
from geometry_utils import GeometryUtils
from color_manager import ColorManager


# Calculate the points of both types of instrument
def get_points(contour, centroid, type):
    hull = cv.convexHull(contour)

    # Get extreme outer points of the contour
    extreme_left = tuple(hull[hull[:, :, 0].argmin()][0])
    extreme_right = tuple(hull[hull[:, :, 0].argmax()][0])
    extreme_top = tuple(hull[hull[:, :, 1].argmin()][0])
    extreme_bottom = tuple(hull[hull[:, :, 1].argmax()][0])

    points = np.array([extreme_left, extreme_right, extreme_top, extreme_bottom])

    # Calculate the distance from point to centroid for each point
    distances_to_centroid = np.linalg.norm(points - centroid, axis=1)

    point_index = np.argmax(distances_to_centroid)
    # Determine the point of the instrument, which is the point that is the furtest from centroid
    point = points[point_index]

    remaining_points = np.delete(points, point_index, axis=0)

    # Initialize handle values
    handle1 = None
    handle2 = None

    if type == 'straight':
        # Calculate opposite direction of the point of instrument
        direction = point - centroid
        perpendicular_direction = np.array([-direction[1], direction[0]])

        # Determine where the handles of the instrument are based on the dot product
        # One is top right from centroid while the other one is bottom left of the centroid
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
    elif type == 'crooked':
        # Define the line from point to centroid
        direction = point - centroid
        normal_direction = np.array([-direction[1], direction[0]])

        same_side_points = []

        for pt in remaining_points:
            vector = pt - centroid
            # Check if point is on the same side of the line
            dot_product = np.dot(vector, normal_direction)

            if dot_product > 0:
                same_side_points.append(pt)
            else:
                same_side_points.append(pt)

        # Ensure we have at least two points on the same side
        if len(same_side_points) >= 2:
            min_distance = float('inf')
            for i in range(len(same_side_points) - 1):
                for j in range(i + 1, len(same_side_points)):
                    dist = np.linalg.norm(same_side_points[i] - same_side_points[j])
                    if dist < min_distance:
                        min_distance = dist
                        handle1 = same_side_points[i]
                        handle2 = same_side_points[j]
        else:
            # This is a fallback in case our logic has an issue (shouldn't occur with proper input)
            if len(same_side_points) == 1:
                handle1 = same_side_points[0]

    categorized_points = {
        'handle1': handle1,
        'handle2': handle2,
        'point': point,
    }

    return categorized_points


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


# Determine the type of the instrument based on the fact if the centroid is inside or outside its body
def get_instrument_type(body, centroid):
    distance = cv.pointPolygonTest(body, centroid, measureDist=False)
    if distance > 0:
        return 'straight'
    elif distance < 0:
        return 'crooked'
    else:
        return 'unknown'


class Instrument:
    def __init__(self, body, index, area):
        self.color = ""
        self.hsv = []
        self.body = body
        self.index = index
        self.area = area
        self.children = []
        self.centroid = GeometryUtils.calculate_centroid(body)
        self.type = get_instrument_type(body, self.centroid)
        self.points = get_points(body, self.centroid, self.type)
        self.rotation = get_rotation(self.points["point"], self.centroid)
        self.direction = 'Not found'

    def __str__(self):
        return "Instrument (" + str(
            self.index) + "): centroid:" + self.centroid.__str__() + ", color: " + self.color + ", rotation: " \
            + str(self.rotation) + ", area: " + str(
                self.area) + ", type: " + self.type + ", direction: " + self.direction + ", hsv: " + str(
                self.hsv) + "\n"

    # Add child to child list
    def add_child(self, child):
        self.children.append(child)

    # Draw the usefull points of the instrument
    def draw_points(self, img):
        point = self.points["point"]
        handle1 = self.points["handle1"]
        handle2 = self.points["handle2"]
        cv.circle(img, handle1, 5, (0, 0, 0), -1)
        cv.circle(img, handle2, 5, (0, 0, 0), -1)
        cv.circle(img, point, 5, (150, 65, 132), -1)

    # Determine the color of the instrument
    def get_color(self, hsv_image):
        color_manager = ColorManager()

        # Determine the average color of the contour by drawing it on to a mask and getting the average color
        mask = np.zeros(hsv_image.shape[:2], np.uint8)
        contours = [self.body] + self.children
        cv.drawContours(mask, contours, -1, 255, -1)
        mean = cv.mean(hsv_image, mask=mask)

        self.hsv = mean
        primary_colors = color_manager.primary_colors
        colors = color_manager.colors
        h = mean[0]
        s = mean[1]
        v = mean[2]
        # See in which hsv-range the contour fits
        # If it doesn't fit anything it is silver
        for color_name in primary_colors:
            if color_name in colors:
                color = colors[color_name]
                if color.is_color(h, s, v):
                    self.color = color_name
                    break
        if self.color == "":
            self.color = 'silver'

    def calculate_pick_up_point(self):
        if self.type == "straight":
            return self.centroid
        elif self.type == "crooked":
            # Vind het bounding rectangle van de contour
            x, y, w, h = cv2.boundingRect(self.body)

            # Hoekpunten van de bounding rectangle
            rect_points = [(x, y), (x + w, y), (x, y + h), (x + w, y + h)]

            # Bereken de afstand van elk hoekpunt tot het centroid en vind het dichtstbijzijnde hoekpunt
            closest_point = min(rect_points, key=lambda point: np.linalg.norm(np.array(point) - np.array(centroid)))
            return closest_point


