import cv2 as cv
import numpy as np
from geometry_utils import GeometryUtils
from color_manager import ColorManager


class Robot:
    def __init__(self):
        self.mode = 'instruments'
        self.location = (0, 0)

    def get_mode(self):
        # use http post request to get mode
        return self.mode
