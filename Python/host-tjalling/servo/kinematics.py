import sys
import numpy as np
from servo.servo_controller import ServoController



class Kinematics:
    def __init__(self, link1, link2, range_l1, range_l2):
        self.Link1 = link1
        self.Link2 = link2
        # Define the angle limits in radians
        self.theta1_min = np.deg2rad(-range_l1)
        self.theta1_max = np.deg2rad(range_l1)
        self.theta2_min = np.deg2rad(-range_l2)
        self.theta2_max = np.deg2rad(range_l2)
        self.servo_controller = ServoController()
        #hele arm naar basis positities 
        self.servo_controller.execute_command(self.servobase_id, 30, 512,50)
        self.servo_controller.execute_command(self.servomid_id, 30, 512,50)
        #self.servo_controller.execute_command(41, 32, self.pos_z,50) #TODO change to be command 32 for move with the wheelmode
        self.servo_controller.execute_command(41, 30, self.ax12_range,50) 
        self.servo_axisZ_id = 41
        self.servobase_id = 5
        self.servomid_id = 3
        self.gripper_servo_id = 1
        self.range_l1 = 90
        self.range_l2 = 100
        
    def map_angle_to_servo_position(self, angle,range_link):
        offset = (300-(range_link*2))/2 
        return int((1023 / 300) * (offset + angle+range_link))

    def get_encoder(self):
        something = something

    def get_servo_load(self):
        self.servo_controller.execute_getstatus() #load opvragen 

    def translate_z(self):
        something = something

    def kinematics(self):
        something = something
        #servos aansturen naar hoek x
        self.servo_controller.execute_command(servobase_id, 30,)
        



        #grijp ophalen functie uit bluetooth_controller
        #grijpen open/dicht
        #z as bepalen wheelmode encoder afstemmen, while true totdat Z as zoveel beweegt



    def inverse_kinematics(self, x, y):
        # Calculate Distance
        distance = (x ** 2 + y ** 2 - self.Link1 ** 2 - self.Link2 ** 2) / (2 * self.Link1 * self.Link2)

        # Clamp distance to ensure it's within the valid range for arccos
        distance = np.clip(distance, -1, 1)

        # Calculate Elbow Angle
        theta2 = np.arccos(distance)

        # Adjust the angle based on the y coordinate
        if y < 0:
            theta2 = -theta2

        # Calculate Shoulder Angle
        theta1 = np.arctan2(y, x) - np.arctan2(self.Link2 * np.sin(theta2), self.Link1 + self.Link2 * np.cos(theta2))

        # Clamp the angles to their respective limits
        theta1 = np.clip(theta1, self.theta1_min, self.theta1_max)
        theta2 = np.clip(theta2, self.theta2_min, self.theta2_max)

        # Convert angles to degrees
        theta1_deg = np.rad2deg(theta1)
        theta2_deg = np.rad2deg(theta2)
        
        print("theta1",theta1_deg)
        print("theta2",theta2_deg)


        return theta1_deg, theta2_deg

    def stretch_arm(self):
        # Set joint angles to stretch the arm
        theta1_stretch = np.clip(0, self.theta1_min, self.theta1_max)
        theta2_stretch = np.clip(0, self.theta2_min, self.theta2_max)
        
        return np.rad2deg(theta1_stretch), np.rad2deg(theta2_stretch)
