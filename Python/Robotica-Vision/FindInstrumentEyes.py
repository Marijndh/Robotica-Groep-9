import numpy as np
import cv2 as cv

#https://docs.opencv.org/4.5.1/de/d62/tutorial_bounding_rotated_ellipses.html
def test():
    frame = cv.imread('kleuren_tangen.jpg')
    frame = cv.resize(frame, (640, 640))
    imgray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    _, thresh = cv.threshold(imgray,145,255,cv.THRESH_BINARY)
    contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    print(len(contours))
    cv.drawContours(frame, contours, -1, (0, 255, 0), 2)
    cv.imshow('Contours', frame)
    cv.waitKey(0)
    cv.destroyAllWindows()


test()
