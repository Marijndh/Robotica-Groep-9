import numpy as np
import cv2 as cv


class GeometryUtils:
    @staticmethod
    def predict_future_positions(points, num_future_points=4, degree=2):
        """Predict future positions based on given points."""
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

    @staticmethod
    def is_ellipse(contour, min_aspect_ratio=0.5):
        """Check if a contour is ellipse-shaped."""
        if len(contour) < 5:
            return False, (0, 0)
        ellipse = cv.fitEllipse(contour)
        (x, y), (MA, ma), angle = ellipse
        aspect_ratio = min(MA, ma) / max(MA, ma)
        return aspect_ratio >= min_aspect_ratio, (int(x), int(y))
