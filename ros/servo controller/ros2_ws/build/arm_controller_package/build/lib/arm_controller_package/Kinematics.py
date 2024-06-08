import numpy as np
import math
#import tracetools_trace.tools as ttt

def calculate_distance_and_angle(x, y):
    #ttt.tracepoint('my_tracepoint', 'calculate_distance_and_angle_start')
    # Calculate distance from origin to point (x, y)
    distance = math.sqrt(x**2 + y**2)
    
    # Calculate angle of the line from origin to point (x, y) with respect to x-axis
    angle = math.atan2(y, x)
    #ttt.tracepoint('my_tracepoint', 'calculate_distance_and_angle_end')
    return distance, math.degrees(angle)

def calculate_triangle_angles(distance, L1, L2):
    #ttt.tracepoint('my_tracepoint', 'calculate_triangle_angles_start')
    if not distance >= L1 + L2:
        # Calculate angle theta1 (angle opposite to side L1)
        cos_theta1 = (L2**2 + distance**2 - L1**2) / (2 * L2 * distance)
        theta1 = math.acos(cos_theta1)
    
        # Calculate angle theta2 (angle opposite to side L2)
        cos_theta2 = (L1**2 + distance**2 - L2**2) / (2 * L1 * distance)
        theta2 = math.acos(cos_theta2)

        # Calculate angle theta3 (angle opposite to the hypotenuse)
        theta3 = math.pi - theta1 - theta2
    else:
        # If no triangle is formed, assign angles to 0
        theta1 = 0
        theta2 = 0
        theta3 = 0
    #ttt.tracepoint('my_tracepoint', 'calculate_triangle_angles_end')
    return math.degrees(theta1), math.degrees(theta2), math.degrees(theta3)

def calculate_joint_angles(distance, L1, L2, angle):
    #ttt.tracepoint('my_tracepoint', 'calculate_joint_angles_start')
    theta1, theta2, theta3 = calculate_triangle_angles(distance, L1, L2)
    if theta3 > 0:
        # Convert theta2 and theta3 to radians
        jointbase = math.radians(theta2 + angle)
        jointmiddle = math.radians(0-180+theta3)
    
        # Check if angle for jointbase is more than 180 degrees
        if jointbase > math.pi:
            jointbase -= 2 * math.pi
        #ttt.tracepoint('my_tracepoint', 'calculate_joint_angles_end')
        return jointbase, jointmiddle, theta1, theta2, theta3
    else:
        # No angle for middle joint, if the distance is too large or small
        jointbase = math.radians(theta2 + angle)
        jointmiddle = math.radians(theta3)
        #ttt.tracepoint('my_tracepoint', 'calculate_joint_angles_end')        
        return jointbase, jointmiddle, theta1, theta2, theta3

class Kinematics:
    def __init__(self, link1, link2, range_l1, range_l2):
        self.Link1 = link1
        self.Link2 = link2
        # Define the angle limits in radians
        self.theta1_min = np.deg2rad(-range_l1)
        self.theta1_max = np.deg2rad(range_l1)
        self.theta2_min = np.deg2rad(-range_l2)
        self.theta2_max = np.deg2rad(range_l2)

    def inverse_kinematics(self, x, y):
        #ttt.tracepoint('my_tracepoint', 'inverse_kinematics_start')
        distance, angle = calculate_distance_and_angle(x, y)
        print("x:",x,"y:",y) 
        # Ensure the distance is within the valid range for the arm
        if distance > self.Link1 + self.Link2 or distance < abs(self.Link1 - self.Link2):
            print("outside of reach") 
            jointbase = np.deg2rad(angle)
            jointmiddle = np.deg2rad(0) 
            
            
        elif angle > 0: 
            jointbase, jointmiddle, theta1, theta2, theta3 = calculate_joint_angles(distance, self.Link1, self.Link2, angle)
            jointmiddle = -jointmiddle
        else:
            jointbase, jointmiddle, theta1, theta2, theta3 = calculate_joint_angles(distance, self.Link1, self.Link2, angle)
            
        jointbase += np.deg2rad(180)
        jointmiddle += np.deg2rad(180)
        if np.rad2deg(jointbase)>30:
            jointbase -= np.deg2rad(30)
        else:
            jointbase = 0
        
        if np.rad2deg(jointmiddle)>30:
            jointmiddle -= np.deg2rad(30)
        else:
            jointmiddle = 0
            

        #jointmiddle -= np.deg2rad(30)
        # Convert radians to degrees and clamp to the given limits
        #print("joint base angle before clip: ", np.rad2deg(jointbase),"joint middle anglebefore clip: ",np.rad2deg(jointmiddle))
        #jointbase = np.clip(jointbase, self.theta1_min, self.theta1_max)
        #jointmiddle = np.clip(jointmiddle, self.theta2_min, self.theta2_max)
        #print("joint base angle: ", np.rad2deg(jointbase),"joint middle angle: ",np.rad2deg(jointmiddle)) 
        #ttt.tracepoint('my_tracepoint', 'inverse_kinematics_end')        
        return np.rad2deg(jointbase), np.rad2deg(jointmiddle)

