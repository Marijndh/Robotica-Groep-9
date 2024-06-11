from instrument import Instrument
from frame import Frame
from color import Color
from target import Target
from server import Server
import cv2 as cv


mode = 'instruments'
#mode = 'targets'

def determine_direction(initial_position, final_position):
    x1, y1 = initial_position
    x2, y2 = final_position

    if x2 > x1:
        return "r"
    elif x2 < x1:
        return "l"
def calculate_speed(initial_position, final_position, time):
    x1, y1 = initial_position
    x2, y2 = final_position
    # Calculate the distance in pixels
    distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    # Calculate the speed in pixels per second
    speed = distance / time
    return speed


def main():
    frame = Frame('Images/silver2.jpg', 1080, 720)
    if mode == 'instruments':
        gray_blurred = cv.medianBlur(frame.gray_image, 5)
        _, thresh = cv.threshold(gray_blurred, 127, 255, cv.THRESH_BINARY)
        frame.contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        frame.hierarchy = hierarchy[0]
        frame.find_instruments()
        frame.find_children()
        frame.draw_instruments()
        frame.print_instruments()
    elif mode == 'targets':
        gray_blurred = cv.GaussianBlur(frame.gray_image, (9, 9), 2)
        _, thresh = cv.threshold(gray_blurred, 127, 255, cv.THRESH_BINARY_INV)
        frame.contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        frame.hierarchy = hierarchy[0]
        frame.find_targets()
        frame.draw_targets()
        frame.print_targets()
    frame.show()


# if __name__ == '__main__':
#     main()

if __name__ == '__main__':
    server = Server('localhost', 5000)
    server.run()
