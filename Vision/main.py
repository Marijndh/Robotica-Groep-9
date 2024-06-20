import cv2 as cv
import numpy as np
from frame import Frame
from robot import Robot
from geometry_utils import GeometryUtils
from image_requester import ImageRequester
import time


def determine_trajectory(robot, previous_locations, speed, direction):
    if len(previous_locations) == 10 and direction != 'Stationary':
        prediction = GeometryUtils.calculate_trajectory(previous_locations, speed, direction)
        previous_locations.pop(0)
        robot.target_point = prediction


def handle_instrument_mode(frame, robot, previous_locations, time_difference, color):
    gray_blur = cv.GaussianBlur(frame.gray_image, (5, 5), 0)
    thresh = cv.adaptiveThreshold(gray_blur, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY, 11, 2)

    # Define kernel for morphological operations
    kernel = np.ones((3, 3), np.uint8)

    # Apply morphological operations
    # Dilation followed by erosion (closing)
    morph = cv.morphologyEx(thresh, cv.MORPH_CLOSE, kernel, iterations=2)

    # Apply Canny edge detection
    edges = cv.Canny(morph, 50, 150)
    frame.contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    if len(robot.instrument_targets) == 0:
        robot.instrument_targets = frame.find_instrument_targets()
    if robot.target is None:
        target = find_target(frame, robot.location, hierarchy, color)
        if target is None:
            return
        robot.target = target
        robot.target_point = target.calculate_pick_up_point()
        previous_locations = [[target.centroid, time_difference]]
    else:
        target = trace_target(frame, robot.target, hierarchy, color)
        if target is None:
            return
        robot.target = target
        robot.target_point = target.calculate_pick_up_point()
        previous_locations.append([robot.target.centroid, time_difference])
    if len(previous_locations) > 1:
        direction, speed = GeometryUtils.get_direction_and_speed(robot.target, previous_locations[-2][0],
                                                                 time_difference)
        determine_trajectory(robot, previous_locations, speed, direction)
        robot.move_to_instrument()


def handle_target_mode(frame, robot, previous_locations, time_difference):
    gray_blur = cv.GaussianBlur(frame.gray_image, (9, 9), 2)
    thresh = cv.adaptiveThreshold(gray_blur, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY, 11, 2)
    frame.contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    if len(frame.contours) > 0 and hierarchy is not None:
        frame.hierarchy = hierarchy[0]
        frame.find_targets()
        if len(frame.targets) > 0:
            if robot.target is None:
                robot.reset()
                target = GeometryUtils.find_closest_object(robot.location, frame.targets)
                if target is None:
                    return
                robot.target = target
                robot.target_point = target.centroid
                previous_locations = [[target.centroid, time_difference]]
            else:
                target = GeometryUtils.find_closest_object(robot.target.centroid, frame.targets)
                if target is None:
                    return
                robot.target = target
                previous_locations.append([robot.target.centroid, time_difference])
            if len(previous_locations) > 1:
                direction, speed = GeometryUtils.get_direction_and_speed(robot.target, previous_locations[-2][0],
                                                                         time_difference)
                determine_trajectory(robot, previous_locations, speed, direction)
                robot.move_to_target()


def trace_target(frame, target, hierarchy, color):
    if len(frame.contours) > 0 and hierarchy is not None:
        frame.hierarchy = hierarchy[0]
        frame.find_instruments(color)
        frame.find_children()
        if len(frame.instruments) > 0:
            return GeometryUtils.find_closest_object(target.centroid, frame.instruments)


def find_target(frame, location, hierarchy, color):
    if len(frame.contours) > 0 and hierarchy is not None:
        frame.hierarchy = hierarchy[0]
        frame.find_instruments(color)
        frame.find_children()
        if len(frame.instruments) > 0:
            closest = GeometryUtils.find_closest_object(location, frame.instruments)
            return closest


def main():
    requester = ImageRequester()
    robot = Robot()
    robot.fetch_values()
    robot.reset()
    print("Modus: " + str(robot.mode))
    print("Kleur: " + str(robot.color))
    previous_mode = robot.mode
    previous_color = robot.color
    previous_locations = []
    previous_time = 0
    while True:
        robot.fetch_values()
        current_time = time.time()
        time_difference = current_time - previous_time
        previous_time = current_time
        color = robot.color
        mode = robot.mode
        if mode == "handmatig":
            break
        if mode != previous_mode or color != previous_color:
            print("Nieuwe modus of kleur: " + mode + ", " + color)
            previous_locations = []
            previous_time = 0
            previous_mode = mode
            previous_color = color
        images = requester.fetch_image(1)
        for img in images:
            frame = Frame(img, 1280, 720)
            if mode == 'instruments':
                handle_instrument_mode(frame, robot, previous_locations, time_difference, color)
                frame.draw_instruments()
            elif mode == 'targets':
                handle_target_mode(frame, robot, previous_locations, time_difference)
            frame.show()
        if cv.waitKey(1) & 0xFF == ord('q'):
            break


if __name__ == '__main__':
    main()
