from controller import Robot, Motor
import math

def cartesian_to_servo_angles(x, y, arm1_length, arm2_length):
    # Calculate angle between first arm and horizontal (x-axis)
    theta1 = math.atan2(y, x)
    
    # Calculate distance from base to end-effector projection on xy plane
    distance_xy = math.sqrt(x**2 + y**2)
    
    # Calculate angle between first arm and line connecting base to end-effector in xy plane
    alpha = math.acos((arm1_length**2 + distance_xy**2 - arm2_length**2) / (2 * arm1_length * distance_xy))
    
    # Calculate angle between second arm and line connecting the two arms
    beta = math.acos((arm1_length**2 + arm2_length**2 - distance_xy**2) / (2 * arm1_length * arm2_length))
    
    # Adjust angles if necessary (depends on the orientation of your robot)
    # For example, if your zero angles are not aligned with the x-axis, you'll need to add/subtract offsets.
    
    return theta1, alpha, beta

# Create a robot instance
robot = Robot()

# Get the hinge joint
hinge_joint = robot.getDevice("hinge1")
hinge_joint3 = robot.getDevice("hinge3")

# Set arm lengths
arm1_length = 500  # Length of the first arm
arm2_length = 300   # Length of the second arm

# Example target Cartesian coordinates
target_x = 0
target_y = 800
    
# Convert Cartesian coordinates to servo angles
theta1, alpha, beta = cartesian_to_servo_angles(target_x, target_y, arm1_length, arm2_length)
print("Theta1:", theta1, "Alpha:", alpha, "Beta:", beta)

# Main control loop
while robot.step(64) != -1:
    
    # Set servo positions
    hinge_joint3.setPosition(theta1)
    hinge_joint.setPosition(beta)  # Assuming this is for the second joint
    # You might need to set additional joints depending on your robot configuration
    
    pass
