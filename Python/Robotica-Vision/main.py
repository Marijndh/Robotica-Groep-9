import math

from instrument import Instrument
from frame import Frame
from color import Color
from target import Target
import cv2 as cv

#mode = 'instruments'
# mode = 'targets'
mode = 'bricks'

def main():
    vid = cv.VideoCapture(0, cv.CAP_DSHOW)
    previous_objects = []

    while (True):
        ret, img = vid.read()
        
        if ret:
            frame = Frame(img, 640, 480)
            if mode == 'instruments':
                gray_blurred = cv.GaussianBlur(frame.gray_image, (5, 5), 0)
                _, thresh = cv.threshold(gray_blurred, 127, 255, cv.THRESH_BINARY)
                frame.contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
                if len(frame.contours) > 0 and hierarchy is not None:
                    frame.hierarchy = hierarchy[0]
                    frame.find_instruments()
                    frame.find_children()
                    if previous_objects:
                        frame.compare_instruments(previous_objects, 'instrument')
                    previous_objects = frame.instruments
                    frame.draw_instruments()
                    frame.print_instruments()
            elif mode == 'targets':
                gray_blurred = cv.GaussianBlur(frame.gray_image, (9, 9), 2)
                _, thresh = cv.threshold(gray_blurred, 127, 255, cv.THRESH_BINARY_INV)
                frame.contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
                if len(frame.contours) > 0 and hierarchy is not None:
                    frame.hierarchy = hierarchy[0]
                    frame.find_targets()
                    if previous_objects:
                        frame.compare_instruments(previous_objects, 'target')
                    previous_objects = frame.targets
                    frame.draw_targets()
                    frame.print_targets()
            elif mode == 'bricks':
                #gray_blurred = cv.GaussianBlur(frame.gray_image, (5, 5), 0)
                _, thresh = cv.threshold(frame.gray_image, 127, 255, cv.THRESH_BINARY)
                frame.contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
                if len(frame.contours) > 0 and hierarchy is not None:
                    frame.hierarchy = hierarchy[0]
                    frame.find_bricks()
                    if previous_objects:
                        frame.compare_instruments(previous_objects, 'brick')
                    previous_objects = frame.bricks
                    frame.draw_bricks()
                    frame.print_bricks()

            frame.show()

        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    # After the loop release the cap object
    vid.release()
    # Destroy all the windows
    cv.destroyAllWindows()


if __name__ == '__main__':
    main()

