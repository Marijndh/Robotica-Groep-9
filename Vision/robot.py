import cv2 as cv
import numpy as np
from geometry_utils import GeometryUtils
from color_manager import ColorManager
import requests
import json
import socket


class Robot:
    def __init__(self):
        self.color = None
        self.mode = 'targets'
        self.location = (0, 0)
        self.target = None
        self.target_point = (0, 0)
        self.instrument_targets = []
        self.hostname = 'ubuntu'
        self.IP = "192.168.218.40"

    def fetch_values(self):
        # De URL van de server
        url = "http://" + self.IP + ":5000/bluetooth"

        # Verstuur het POST-verzoek
        response = requests.get(url)

        # Controleer de respons
        if response.status_code == 200:
            # Parse de JSON-respons
            data = response.json()

            # Haal de waarde van 'color' uit de respons
            color = data.get('color', 'N/A')
            self.color = color

            # Haal de waarde van 'mode' uit de respons
            mode = data.get('mode', 'N/A')
            self.mode = mode

    def send_command(self, x, y, z, r, gripper):
        # De URL van de server
        url = "http://" + self.IP + ":5000/servo/kinematics"

        data = str(x) + " " + str(y)
        # De payload (gegevens) die we willen versturen
        data = {"command": data}

        # De headers die we willen versturen
        headers = {"Content-Type": "application/json"}

        # Verstuur het POST-verzoek
        response = requests.post(url, headers=headers, data=json.dumps(data))

        return response.status_code

    def move_to_instrument(self):
        if self.target_point != (0, 0):
            self.location = self.target_point
            result = GeometryUtils.map_coordinate(self.target_point)
            self.target_point = (0, 0)

            #lower z-axes to 13
            
            # move to instrument

            # open gripper and grab object

            # increase z-axes to 20

            # move to instrument location, involve instrument.type

            #lower z-axes to 13

            #release gripper

            # move to new target or 600, 0

    def move_to_target(self):
        if self.target_point != (0, 0):
            self.location = self.target_point
            result = GeometryUtils.map_coordinate(self.target_point)
            self.target_point = (0, 0)

            #move to target

            #lower z-axes to tap target

            #increase z-axes to around 17
