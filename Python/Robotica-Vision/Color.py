import numpy as np
import cv2 as cv
import math


def find_color(color):
    frame = cv.imread('image.jpg')
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    if color == "r":
        lower1 = np.array([0, 50, 50])
        upper1 = np.array([10, 255, 255])

        lower2 = np.array([170, 50, 50])
        upper2 = np.array([179, 255, 255])

        lower_mask = cv.inRange(hsv, lower1, upper1)
        upper_mask = cv.inRange(hsv, lower2, upper2)

        mask = upper_mask | lower_mask

        res = cv.bitwise_and(frame, frame, mask=mask)
        cv.imshow('res', res)
    elif color == "g":
        lower_green = np.array([50, 50, 50])
        upper_green = np.array([70, 255, 255])

        mask = cv.inRange(hsv, lower_green, upper_green)
        res = cv.bitwise_and(frame, frame, mask=mask)
        cv.imshow('res', res)
    elif color == "b":
        lower_blue = np.array([110, 50, 50])
        upper_blue = np.array([130, 255, 255])

        mask = cv.inRange(hsv, lower_blue, upper_blue)
        res = cv.bitwise_and(frame, frame, mask=mask)
        cv.imshow('res', res)

    k = cv.waitKey(0)
    print(k)
    cv.destroyAllWindows()


find_color("r")
