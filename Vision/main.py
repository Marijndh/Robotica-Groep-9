import cv2 as cv
import numpy as np
from frame import Frame
from robot import Robot
from geometry_utils import GeometryUtils
import time


def handle_instrument_mode(frame, robot, previous_locations, time_difference, speed, direction):
    gray_blurred = cv.GaussianBlur(frame.gray_image, (5, 5), 0)
    _, thresh = cv.threshold(gray_blurred, 127, 255, cv.THRESH_BINARY)
    frame.contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    if robot.target is None:
        target = find_target(frame, robot.location, hierarchy)
        if target is None:
            return False
        robot.target_point = target.calculate_pick_up_point()
        previous_locations = [[target.centroid, time_difference]]
    else:
        target = trace_target(frame, robot.target, hierarchy)
        if target is None:
            return False
        previous_locations.append([target.centroid, time_difference])
    if len(previous_locations) == 1:
        direction, speed = GeometryUtils.get_direction_and_speed(target, previous_locations[-1][0],
                                                                 time_difference)
    if len(previous_locations) == 4 and direction != 'Stationary':
        prediction = GeometryUtils.calculate_trajectory(previous_locations, speed, direction)
        previous_objects.pop(0)
        robot.target_point = prediction


def handle_target_mode(frame):
    gray_blurred = cv.GaussianBlur(frame.gray_image, (9, 9), 2)
    _, thresh = cv.threshold(gray_blurred, 127, 255, cv.THRESH_BINARY_INV)
    frame.contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    if len(frame.contours) > 0 and hierarchy is not None:
        frame.hierarchy = hierarchy[0]
        frame.find_targets()
        frame.draw_targets()
        frame.print_targets()
        if len(frame.targets) > 0:
            target, length = GeometryUtils.find_closest_object(robot.location, frame.targets)
            if previous_objects:
                direction, speed = GeometryUtils.get_direction_and_speed(target, previous_objects, time)
                # time_to_get_to_location = length/speed
            previous_objects = frame.instruments


def trace_target(frame, target, hierarchy):
    if len(frame.contours) > 0 and hierarchy is not None:
        frame.hierarchy = hierarchy[0]
        frame.find_instruments()
        frame.find_children()
        if len(frame.instruments) > 0:
            return GeometryUtils.find_closest_object(target.centroid, frame.instruments)


def find_target(frame, location, hierarchy):
    if len(frame.contours) > 0 and hierarchy is not None:
        frame.hierarchy = hierarchy[0]
        frame.find_instruments()
        frame.find_children()
        if len(frame.instruments) > 0:
            closest = GeometryUtils.find_closest_object(location, frame.instruments)
            return closest



# TODO make methods for each mode, clean-up main method
# TODO replace videocapture with images from Raspberry Pi -> image_requester.py
# TODO map pixel to coordinate usefull for servo's
# TODO get mode from Raspberry using http request
# TODO add mode to search for certain color
# TODO implement finding trajectory for target -> Example code/find_trajectory.py
def main():
    vid = cv.VideoCapture(0, cv.CAP_DSHOW)
    direction = 'Stationary'
    speed = 0
    robot = Robot()
    previous_locations = []
    previous_time = 0
    while True:
        current_time = time.time()
        time_difference = current_time - previous_time
        previous_time = current_time
        mode = robot.get_mode()
        ret, img = vid.read()
        if ret:
            frame = Frame(img, 1280, 720)
            if mode == 'instruments':
                handle_instrument_mode(frame, robot, previous_locations, time_difference, speed, direction)
            elif mode == 'targets':
                handle_target_mode(frame)
            frame.show()
        robot.move_to_location()
        if cv.waitKey(1) & 0xFF == ord('q'):
            break
    # After the loop release the cap object
    vid.release()
    # Destroy all the windows
    cv.destroyAllWindows()


if __name__ == '__main__':
    main()
