from controller import Robot, Motor
# Create a robot instance
robot = Robot()

# Get the hinge joint
hinge_joint = robot.getDevice("hinge1")
hinge_joint3 = robot.getDevice("hinge3")
slider_joint = robot.getDevice("hinge4")

target_position = 1

hinge_joint.setPosition(target_position)
hinge_joint3.setPosition(target_position)
slider_joint.setPosition(0.18)

# Main control loop
while robot.step(64) != -1:
    # Your control logic here
    pass