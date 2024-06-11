from instrument import Instrument
from frame import Frame
from color import Color
from target import Target
import cv2 as cv


#mode = 'instruments'
mode = 'targets'
def main():
    frame = Frame('Images/bulls-eye.jpg', 1080, 720)
    if mode == 'instruments':
        _, thresh = cv.threshold(frame.gray_image, 127, 255, cv.THRESH_BINARY)
        frame.contours, hierarchy = cv.findContours(frame.thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        frame.hierarchy = hierarchy[0]
        frame.find_instruments()
        frame.find_children()
        frame.draw_instruments()
        frame.print_instruments()
    elif mode == 'targets':
        _, thresh = cv.threshold(frame.gray_image, 127, 255, cv.THRESH_BINARY_INV)
        frame.contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        frame.hierarchy = hierarchy[0]
        frame.find_targets()
        frame.draw_targets()
        frame.print_targets()
    frame.show()


if __name__ == '__main__':
    main()
