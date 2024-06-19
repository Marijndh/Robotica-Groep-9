import numpy as np
import cv2 as cv
import math


class GeometryUtils:
    # Determine if a contour is the shape of an ellipse
    # Returns a boolean and the center coordinate
    @staticmethod
    def is_ellipse(contour, min_aspect_ratio=0.5):
        if len(contour) < 5:
            return False, (0, 0)
        ellipse = cv.fitEllipse(contour)
        (x, y), (MA, ma), angle = ellipse
        aspect_ratio = min(MA, ma) / max(MA, ma)
        return aspect_ratio >= min_aspect_ratio, (int(x), int(y))

    # Calculate the distance between two points
    @staticmethod
    def calculate_distance(centroid1, centroid2):
        return math.sqrt((centroid1[0] - centroid2[0]) ** 2 + (centroid1[1] - centroid2[1]) ** 2)

    # Find the closest object to the given object out of a list of objects
    @staticmethod
    def find_closest_object(point, objects):
        min_distance = float('inf')
        closest_obj = None
        for other_obj in objects:
            distance = GeometryUtils.calculate_distance(point, other_obj.centroid)
            if distance < min_distance:
                min_distance = distance
                closest_obj = other_obj
        return closest_obj

    # Returns the centroid of a given body
    @staticmethod
    def calculate_centroid(body):
        M = cv.moments(body)
        if M["m00"] != 0:
            return int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
        else:
            return 0, 0

    # Determine the direction based on two x values
    @staticmethod
    def determine_direction(previous_x, current_x):
        if current_x > previous_x:
            return 'East'
        elif current_x < previous_x:
            return 'West'
        else:
            return 'Stationary'

    @staticmethod
    def get_direction_and_speed(target, previous_location, time):
        direction = GeometryUtils.determine_direction(previous_location[0], target.centroid[0])
        length = GeometryUtils.calculate_distance(target.centroid, previous_location)
        speed = length / time
        return direction, speed

    @staticmethod
    def calculate_trajectory(previous_locations, speed, direction):
        # Extract the last known position and the time difference
        last_position, time_diff = previous_locations[-1]
        last_x, last_y = last_position

        # Predict amount of time to move to location
        move_time = 2

        # Total time including the grasp time
        total_time = time_diff + move_time

        # Calculate the distance traveled
        distance_traveled = speed * total_time

        # Calculate the new position based on the direction
        if direction.lower() == 'east':
            new_x = last_x + distance_traveled
            new_y = last_y
        elif direction.lower() == 'west':
            new_x = last_x - distance_traveled
            new_y = last_y

        # Return the new position
        return new_x, new_y

    @staticmethod
    def calculate_affine_transform(src_points, dst_points):
        A = []
        B = []

        for src, dst in zip(src_points, dst_points):
            A.append([src[0], src[1], 1, 0, 0, 0])
            A.append([0, 0, 0, src[0], src[1], 1])
            B.append(dst[0])
            B.append(dst[1])

        A = np.array(A)
        B = np.array(B)

        affine_params = np.linalg.lstsq(A, B, rcond=None)[0]
        return affine_params

    @staticmethod
    def apply_affine_transform(pixel, affine_params):
        x, y = pixel
        a, b, c, d, e, f = affine_params

        coord_x = a * x + b * y + c
        coord_y = d * x + e * y + f

        return round(coord_x), round(coord_y)

    @staticmethod
    def map_coordinate(pixel):
        pixel_points = [(650, 0), (200, 440), (1090, 490)]
        coord_points = [(600, 0), (0, 600), (0, -600)]
        affine_params = GeometryUtils.calculate_affine_transform(pixel_points, coord_points)
        return GeometryUtils.apply_affine_transform(pixel, affine_params)
