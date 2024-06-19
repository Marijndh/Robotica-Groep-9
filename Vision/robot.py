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
        self.mode = ''
        self.location = (0, 0)
        self.target = None
        self.target_point = (0, 0)
        self.instrument_targets = []
        self.hostname = 'ubuntu'
        self.IP = "192.168.218.40"

    def fetch_values(self):
        try:
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
        except:
            print("Kan data van controller niet ophalen")

    def send_command(self, x, y, z, r, gripper):
        try:
            # De URL van de server
            url = "http://" + self.IP + ":5000/controller"

            # De payload (gegevens) die we willen versturen
            data = {
                "x": x,
                "y": y,
                "z": z,
                "r": r,
                "gripper": gripper
            }

            # De headers die we willen versturen
            headers = {"Content-Type": "application/json"}

            # Verstuur het POST-verzoek
            response = requests.post(url, headers=headers, data=json.dumps(data))

            return response.status_code
        except:
            print("Versturen mislukt")
            return 300

    def move_to_instrument(self):
        if self.target_point != (0, 0) and self.target_point != self.location:
            self.location = self.target_point
            x, y = GeometryUtils.map_coordinate(self.target_point)
            r = 1024 * self.target.rotation / 360
            # lower z-axes to 13, move to instrument, open gripper
            self.send_command(x, y, 13, r, 240)
            # grab object
            self.send_command(x, y, 13, r, 600)
            # increase z-axes to 20
            self.send_command(x, y, 20, r, 400)
            # move to instrument location, involve instrument.type
            target = None
            for location in self.instrument_targets:
                if location.type == "straight":
                    target = location.centroid
                elif location.type == "crooked":
                    target = location.calculate_pick_up_point()
            if target is not None:
                target_rotation = 1024 * self.target.rotation / 360
                self.send_command(target[0], target[1], 20, target_rotation, 400)
                # lower z-axes to 13
                self.send_command(target[0], target[1], 13, target_rotation, 400)
                # release gripper
                self.send_command(target[0], target[1], 20, target_rotation, 240)
            self.target_point = (0, 0)
            self.target = None

    def reset(self):
        if self.location != (650, 0):
            self.send_command(600, 0, 20, 512, 240)
            self.location = (650, 0)

    def move_to_target(self):
        if self.target_point != (0, 0) and self.target_point != self.location:
            self.location = self.target_point
            x, y = GeometryUtils.map_coordinate(self.target_point)
            print(self.target_point)
            print(x, y)
            # move to target
            self.send_command(x, y, 20, 512, 400)
            # lower z-axes to tap target
            self.send_command(x, y, 17, 512, 400)
            # increase z-axes to around 17
            self.send_command(x, y, 20, 512, 400)
            self.target_point = (0, 0)
            self.target = None

