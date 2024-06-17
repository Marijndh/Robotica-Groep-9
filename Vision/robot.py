import cv2 as cv
import numpy as np
from geometry_utils import GeometryUtils
from color_manager import ColorManager


class Robot:
    def __init__(self):
        self.mode = 'instruments'
        self.location = (0, 0)
        self.target = None
        self.target_point = (0,0)
        self.moving = False

    def get_moving(self):
        # use http request to get status
        return False
    def get_mode(self):
        # use http post request to get mode
        return self.mode

    def move_to_location(self):
        if self.target_point != (0, 0):
            self.location = self.target_point
            result = GeometryUtils.map_coordinate(self.target_point)
            print(result)
        # use http post request to move robot
        self.target_point = (0, 0)

