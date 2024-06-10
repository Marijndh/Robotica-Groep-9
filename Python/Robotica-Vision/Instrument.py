import cv2 as cv

from geometry_utils import GeometryUtils


class Instrument:
    def __init__(self, body, index, children=None, centroid=None, points=None, rotation=0):
        self.color = ""
        self.body = body
        self.index = index
        self.children = children if children is not None else []
        self.centroid = centroid
        self.points = points if points is not None else {}
        self.rotation = rotation

    def add_child(self, child):
        """Add a child contour to the instrument."""
        self.children.append(child)

    def set_color(self, color):
        """Set the color of the instrument."""
        self.color = color

    def calculate_centroid(self):
        """Calculate the centroid of the instrument's body."""
        M = cv.moments(self.body)
        if M["m00"] != 0:
            self.centroid = (int(M["m10"] / M["m00"]),
                             int(M["m01"] / M["m00"]))
        else:
            self.centroid = (0, 0)

    def get_points(self):
        """Get the points of the instrument's body."""
        # Placeholder implementation
        self.points = self.body.reshape(-1, 2).tolist()
        return self.points

    # TODO: Waar wordt dit voor gebruikt?
    def calculate_rotation(self):
        """Calculate the rotation of the instrument's body."""
        if len(self.body) >= 5:
            ellipse = cv.fitEllipse(self.body)
            self.rotation = ellipse[2]
        else:
            self.rotation = 0

    @staticmethod
    def predict_future_positions(points, num_future_points=4, degree=2):
        """Predict future positions based on given points."""
        return GeometryUtils.predict_future_positions(points,
                                                      num_future_points,
                                                      degree)

    @staticmethod
    def is_ellipse(contour, min_aspect_ratio=0.5):
        """Check if a contour is ellipse-shaped."""
        return GeometryUtils.is_ellipse(contour, min_aspect_ratio)
