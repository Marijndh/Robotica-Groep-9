import numpy as np
import cv2 as cv
import math


class GeometryUtils:

    #Predict the 4 next positions of an instrument based on 4 given points
    @staticmethod
    def predict_future_positions(points, num_future_points=4, degree=2):
        points = np.array(points)
        x = points[:, 0]
        y = points[:, 1]

        # Fit a polynomial to the given points
        coefficients = np.polyfit(x, y, degree)
        polynomial = np.poly1d(coefficients)

        # Predict future x-values based on the average step between x-values in the input
        average_step = np.mean(np.diff(x))
        future_x = np.arange(x[-1] + average_step, x[-1] + average_step *
                             (num_future_points + 1), average_step)
        future_y = polynomial(future_x)

        future_x = np.round(future_x).astype(int)
        future_y = np.round(future_y).astype(int)

        return list(zip(future_x, future_y))

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
    def find_closest_object(obj, objects):
        min_distance = float('inf')
        closest_obj = None
        for other_obj in objects:
            distance = GeometryUtils.calculate_distance(obj.centroid, other_obj.centroid)
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
