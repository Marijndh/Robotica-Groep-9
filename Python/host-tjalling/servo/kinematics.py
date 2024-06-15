import numpy as np


class Kinematics:
    def __init__(self, link1, link2, range_l1, range_l2):
        self.Link1 = link1
        self.Link2 = link2
        # Define the angle limits for the joints
        self.theta1_min = np.deg2rad(-range_l1)
        self.theta1_max = np.deg2rad(range_l1)
        self.theta2_min = np.deg2rad(-range_l2)
        self.theta2_max = np.deg2rad(range_l2)

    def inverse_kinematics(self, x, y):
        # Calculate Distance
        distance = (x ** 2 + y ** 2 - self.Link1 ** 2 - self.Link2 ** 2) / (2 * self.Link1 * self.Link2)

        # Clamp distance to ensure it's within the valid range for arccos
        distance = np.clip(distance, -1, 1)

        # Calculate Elbow Angle
        theta2 = np.arccos(distance)

        # Adjust the angle based on the y coordinate
        if y > 0:
            theta2 = theta2
        else:
            theta2 = -theta2
        # Calculate Shoulder Angle
        theta1 = np.arctan2(y, x) - np.arctan2(self.Link2 * np.sin(theta2), self.Link1 + self.Link2 * np.cos(theta2))
        # Clamp the angles to their respective limits
        theta1 = np.clip(theta1, self.theta1_min, self.theta1_max)
        theta2 = np.clip(theta2, self.theta2_min, self.theta2_max)
        # Convert angles to degrees
        theta1_deg = np.rad2deg(theta1)
        theta2_deg = np.rad2deg(theta2)

        return (2 * self.theta1_max) - theta1_deg, theta2_deg

    def stretch_arm(self):
        # Set joint angles to stretch the arm
        theta1_stretch = np.clip(0, self.theta1_min, self.theta1_max)
        theta2_stretch = np.clip(0, self.theta2_min, self.theta2_max)

        return np.rad2deg(theta1_stretch), np.rad2deg(theta2_stretch)
