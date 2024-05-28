import numpy as np
import cv2 as cv
import math
import Color


def test():
    frame = cv.imread('Images/roze-tang.jpeg')
    frame = cv.resize(frame, (640, 640))
    imgray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    _, thresh = cv.threshold(imgray, 150, 255, cv.THRESH_BINARY)
    contours, hierarchy = cv.findContours(thresh,
                                          cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    result = []
    for cnr in range(len(contours)):
        cnt = contours[cnr]
        area = cv.contourArea(cnt)
        # perimeter = cv.arcLength(cnt, True)
        # factor = 4 * math.pi * area / perimeter ** 2
        if 5000.0 < area < 400000:
            print(str(cnr)+": "+str(area))
            result.append(cnt)
    print(len(result))
    for res in result:

    cv.drawContours(frame, result, -1, (0, 0, 0), 3)
    cv.imshow('Contours', frame)
    cv.waitKey(0)
    cv.destroyAllWindows()


test()
