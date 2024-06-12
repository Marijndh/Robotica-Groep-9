from instrument import Instrument
from frame import Frame
from color import Color
from target import Target
import cv2 as cv

vid = cv.VideoCapture(1, cv.CAP_DSHOW)
previous_instruments = []

while (True):
    ret, img = vid.read()
    if ret:
        frame = Frame(img, 640, 480)
        gray_blurred = cv.medianBlur(frame.gray_image, 5)
        _, thresh = cv.threshold(gray_blurred, 127, 255, cv.THRESH_BINARY)
        frame.contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        if len(frame.contours) > 0 and hierarchy is not None:
            frame.hierarchy = hierarchy[0]
            frame.find_instruments()
            frame.find_children()
            if previous_instruments:
                frame.compare_instruments(previous_instruments)
            previous_instruments = frame.instruments
            frame.draw_instruments()
        frame.show()

    if cv.waitKey(1) & 0xFF == ord('q'):
        break

# After the loop release the cap object
vid.release()
# Destroy all the windows
cv.destroyAllWindows()