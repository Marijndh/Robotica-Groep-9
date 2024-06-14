import numpy as np
import cv2 as cv
import math

frame = cv.imread('image.jpg')


def draw_convexhull_and_center():
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    # use red mask
    lower1 = np.array([0, 50, 50])
    upper1 = np.array([10, 255, 255])

    lower2 = np.array([170, 50, 50])
    upper2 = np.array([179, 255, 255])

    lower_mask = cv.inRange(hsv, lower1, upper1)
    upper_mask = cv.inRange(hsv, lower2, upper2)

    mask = upper_mask | lower_mask

    _, thresh = cv.threshold(mask, 0, 255, cv.THRESH_BINARY)
    # find red object
    contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE,
                                          cv.CHAIN_APPROX_SIMPLE)
    M = cv.moments(contours[0])
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    # calculate center point
    centroid = (cx, cy)
    hull = cv.convexHull(contours[0])
    # draw center point
    cv.circle(frame, centroid, 5, (0, 255, 255), -1)
    # draw line around red object
    cv.drawContours(frame, [hull], 0, (0, 255, 255), 2)


def filter_objects_based_on_area_and_factor():
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    grayBlur = cv.GaussianBlur(gray, (25, 25), 0)
    ret, th = cv.threshold(grayBlur, 180, 255, cv.THRESH_BINARY_INV)
    contours, hierarchy = cv.findContours(th, cv.RETR_TREE,
                                          cv.CHAIN_APPROX_SIMPLE)

    hierarchy = hierarchy[0]

    for cnr in range(len(contours)):
        cnt = contours[cnr]
        area = cv.contourArea(cnt)
        perimeter = cv.arcLength(cnt, True)
        factor = 4 * math.pi * area / perimeter ** 2
        holes = 0
        child = hierarchy[cnr][2]
        while child > 0:
            holes += cv.contourArea(contours[child])
            child = hierarchy[child][0]
        if holes > 300:
            color = (255, 0, 0)
            cv.drawContours(frame, [cnt], -1, color, 3)
        elif 0.4 < factor < 0.45:
            color = (0, 255, 0)
            cv.drawContours(frame, [cnt], -1, color, 3)
        elif area > 2000 and factor > 0.8:
            color = (0, 255, 255)
            cv.drawContours(frame, [cnt], -1, color, 3)
        elif 1000 < area < 2000 and 0.7 < factor < 0.8:
            color = (255, 0, 255)
            cv.drawContours(frame, [cnt], -1, color, 3)


def contour_to_point(contour):
    x, y, w, h = cv.boundingRect(contour)

    center_x = x + w // 2
    center_y = y + h // 2

    return center_x, center_y


def count_points_in_squares(squares, points):
    results = []
    for square in squares:
        count = 0
        x, y, w, h = square
        for point in points:
            px, py = point
            if x <= px < x + w and y <= py < y + h:
                count += 1
        results.append(count)
    return sorted(results)


def find_eyes_inside_square():
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    lower = np.array([0, 0, 100])
    upper = np.array([179, 50, 255])

    mask = cv.inRange(hsv, lower, upper)
    eyes, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    dices, _ = cv.findContours(gray, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    found_dices = []

    for contour in dices:
        x, y, w, h = cv.boundingRect(contour)
        contour_area_threshold = 100
        contour_area = cv.contourArea(contour)
        if contour_area > contour_area_threshold:
            cv.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            found_dices.append([x, y, w, h])

    found_eyes = []
    for contour in eyes:
        perimeter = cv.arcLength(contour, True)
        approx_circle = cv.approxPolyDP(contour, 0.04 * perimeter, True)
        if len(approx_circle) >= 5 and cv.contourArea(contour) > 100:
            cv.drawContours(frame, [contour], 0, (0, 0, 255), 2)
            found_eyes.append(contour_to_point(contour))

    print("Aantal ogen in elk vierkant:",
          count_points_in_squares(found_dices, found_eyes))
