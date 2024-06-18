import cv2 as cv
import numpy as np
from geometry_utils import GeometryUtils
from color_manager import ColorManager
import requests
import json
import socket

class Robot:
    def __init__(self):
        self.mode = 'targets'
        self.location = (0, 0)
        self.target = None
        self.target_point = (0, 0)
        self.moving = False
        self.instrument_targets = []
        self.hostname = 'ubuntu'
        self.IP = socket.gethostbyname(self.hostname)


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

            self.target_point = (0, 0)
            # De URL van de server
            url = "http://"+ self.IP +":5000/servo/kinematics"

            data = str(result[0]) + " " + str(result[1])
            # De payload (gegevens) die we willen versturen
            data = {"command": data}

            # De headers die we willen versturen
            headers = {"Content-Type": "application/json"}

            # Verstuur het POST-verzoek
            response = requests.post(url, headers=headers, data=json.dumps(data))

            # Controleer de respons
            if response.status_code == 200:
                print("Bewogen naar: " + str(result))

