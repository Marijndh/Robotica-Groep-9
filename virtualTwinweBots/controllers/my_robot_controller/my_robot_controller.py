from controller import Robot, Motor
import math


def calculate_distance_and_angle(x, y):
    # Calculate distance from origin to point (x, y)
    distance = math.sqrt(x**2 + y**2)
    
    # Calculate angle of the line from origin to point (x, y) with respect to x-axis
    angle = math.atan2(y, x)
    
    return distance, math.degrees(angle)

def calculate_triangle_angles(distance, L1, L2):
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

    return math.degrees(theta1), math.degrees(theta2), math.degrees(theta3)

    
def calculate_joint_angles(distance, L1, L2, angle):
    theta1, theta2, theta3 = calculate_triangle_angles(distance, L1, L2)
    if theta3 > 0:
        # Convert theta2 and theta3 to radians
        jointbase = math.radians(theta2 + angle)
        jointmiddle = math.radians(0-180+theta3)
    
        # Check if angle for jointbase is more than 180 degrees
        if jointbase > math.pi:
            jointbase -= 2 * math.pi
    
        return jointbase, jointmiddle, theta1, theta2, theta3
    else:
        # no angle for middle joint is the distance to large or small?
        jointbase = math.radians(theta2 + angle)
        jointmiddle = math.radians(theta3)
        return jointbase, jointmiddle, theta1, theta2, theta3
# Create a robot instance
robot = Robot()

# Get the hinge joints
hinge_joint = robot.getDevice("hinge1")
hinge_joint3 = robot.getDevice("hinge3")

# Set arm lengths
L1 = 500  # Length of the first arm
L2 = 300  # Length of the second arm

# Example target Cartesian coordinates
x = 200
y = 200
    
distance, angle = calculate_distance_and_angle(x, y)
print("Distance from center in straight line:", distance)
print("Angle from x+:", angle)
# Convert Cartesian coordinates to servo angles
# theta1, theta2 = inverse_kinematics(x, y, L1, L2)
jointbase, jointmiddle, theta1, theta2, theta3 = calculate_joint_angles(distance, L1, L2, angle)
print("targetX:",x,"targetY:",y,"Theta1:", theta1, "Theta2:", theta2, "Theta3:", theta3)
print("jointbase(rad):",jointbase,"jointmiddle(rad):",jointmiddle)

# Main control loop
while robot.step(64) != -1:
    # Update joint angles based on current position or target position if needed
    
    # Set servo positions (convert back to radians)
    hinge_joint.setPosition(jointbase)
    hinge_joint3.setPosition(jointmiddle)
    # You might need to set additional joints depending on your robot configuration
