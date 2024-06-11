import cv2 as cv
import numpy as np
from geometry_utils import GeometryUtils


def get_points(contour, centroid):
    hull = cv.convexHull(contour)

    extreme_left = tuple(hull[hull[:, :, 0].argmin()][0])
    extreme_right = tuple(hull[hull[:, :, 0].argmax()][0])
    extreme_top = tuple(hull[hull[:, :, 1].argmin()][0])
    extreme_bottom = tuple(hull[hull[:, :, 1].argmax()][0])

    points = np.array([extreme_left, extreme_right, extreme_top, extreme_bottom])

    # Bepaal de afstanden tussen elk punt en het gemiddelde (centroïde) punt
    distances_to_centroid = np.linalg.norm(points - centroid, axis=1)

    # Zoek de index van het punt dat het verst van het gemiddelde punt ligt, dit is de 'punt'
    point_index = np.argmax(distances_to_centroid)
    point = points[point_index]

    # Nu hebben we het punt, laten we het verwijderen uit de lijst van punten om de resterende punten te analyseren
    remaining_points = np.delete(points, point_index, axis=0)

    # Bepaal de richting van de lijn van het centroid naar de punt van de tang
    direction = point - centroid

    # Draai de richting 90 graden om de loodrechte lijn te krijgen
    perpendicular_direction = np.array([-direction[1], direction[0]])

    # Categoriseer punten op basis van hun positie ten opzichte van de twee lijnen
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

    # Teruggeven van de gecategoriseerde punten
    categorized_points = {
        'handvat1': handle1,
        'handvat2': handle2,
        'punt': point,
    }

    return categorized_points


def calculate_centroid(body):
    """Calculate the centroid of a contour body."""
    M = cv.moments(body)
    if M["m00"] != 0:
        return int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
    else:
        return 0, 0


def get_rotation(punt, centroid):
    # Vector bepalen van centroid naar punt van de tang
    v = punt - centroid

    # Oriëntatie berekenen
    theta = np.arctan2(v[1], v[0])
    hoek_graden = np.degrees(theta) + 90

    # Zorg ervoor dat de hoek binnen 0-360 graden valt
    if hoek_graden < 0:
        hoek_graden += 360

    return hoek_graden


class Instrument:
    def __init__(self, body, index, area):
        self.color = ""
        self.body = body
        self.index = index
        self.area = area
        self.children = []
        self.centroid = calculate_centroid(body)
        self.points = get_points(body, self.centroid)
        self.rotation = get_rotation(self.points["punt"], self.centroid)
        self.hsv_values = []

    def __str__(self):
        return str(self.index) + ": centroid:" + self.centroid.__str__() + ", color: " + self.color + ", rotation: "\
            + str(self.rotation) + ", area: " + str(self.area) + "\n"

    def add_child(self, child):
        """Add a child contour to the instrument."""
        self.children.append(child)

    def set_hsv(self, hsv):
        self.hsv_values = hsv

    def set_color(self, color):
        """Set the color of the instrument."""
        self.color = color
