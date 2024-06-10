import numpy as np
import cv2 as cv

class TargetProcessor:
    def __init__(self, img, contours):
        self.img = img
        self.contours = contours

    def get_point_of_target(self, coordinates, centroid):
        points = np.array(coordinates)
        distances_to_centroid = np.linalg.norm(points - centroid, axis=1)
        point_index = np.argmax(distances_to_centroid)
        point = points[point_index]
        remaining_points = np.delete(points, point_index, axis=0)
        direction = point - centroid
        perpendicular_direction = np.array([-direction[1], direction[0]])

        handle1, handle2 = None, None
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

    def get_gripper_degrees(self, point, centroid):
        v = point - centroid
        theta = np.arctan2(v[1], v[0])
        degrees = np.degrees(theta) + 90
        if degrees < 0:
            degrees += 360
        return degrees

    def draw_gripper(self, centroid, degrees):
        length = 100
        endpoint_x = int(centroid[0] + length * np.cos(np.radians(degrees)))
        endpoint_y = int(centroid[1] + length * np.sin(np.radians(degrees)))
        endpoint = (endpoint_x, endpoint_y)
        cv.line(self.img, tuple(centroid), endpoint, (0, 255, 0), 2)
        return endpoint
