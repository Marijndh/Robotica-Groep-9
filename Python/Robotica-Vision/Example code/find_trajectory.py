import os

import numpy as np
import cv2 as cv
from find_instruments import find_children, get_instruments


def predict_future_positions(points, num_future_points=4, degree=2):
    points = np.array(points)
    x = points[:, 0]
    y = points[:, 1]

    # Fit a polynomial to the given points
    coefficients = np.polyfit(x, y, degree)
    polynomial = np.poly1d(coefficients)

    # Predict future x-values based on the average step between x-values in the input
    average_step = np.mean(np.diff(x))
    future_x = np.arange(x[-1] + average_step, x[-1] + average_step * (num_future_points + 1), average_step)
    future_y = polynomial(future_x)

    future_x = np.round(future_x).astype(int)
    future_y = np.round(future_y).astype(int)

    return list(zip(future_x, future_y))


def main():
    dirs = [r'../Images/links-naar-rechts', r'../Images/rechts-naar-links']
    for d in dirs:
        frame = None
        points = []
        centroids = []
        for file in os.listdir(d):
            frame = cv.imread(d + '/' + file)
            imgray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            imgray = cv.medianBlur(imgray, 5)

            _, thresh = cv.threshold(imgray, 127, 255, cv.THRESH_BINARY)
            contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE,
                                                  cv.CHAIN_APPROX_SIMPLE)
            hierarchy = hierarchy[0]
            instruments = get_instruments(contours)
            find_children(instruments, contours, hierarchy)
            for i in instruments:
                centroids.append(i.centroid)
                points.append(i.centroid)
        points.extend(predict_future_positions(centroids))
        for p in points:
            print(p)
            cv.circle(frame, p, 5, (0, 0, 0), -1)
        frame = cv.resize(frame, (1080, 720))
        cv.imshow(str(d), frame)
    cv.waitKey(0)
    cv.destroyAllWindows()


if __name__ == '__main__':
    main()
