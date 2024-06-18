from servo.servo_controller import ServoController
from servo.kinematics import Kinematics  # Correct import for Kinematics as its in a different sibling folder
import threading
import sys
from bluedot.btcomm import BluetoothClient
from time import sleep
sys.path.append("..")


class BluetoothController:
    """
        Initializes the BluetoothController with given links and ranges,
        sets up initial positions and servo controller.

        Args:
            link1 (float): Length of the first link of the robot arm.
            link2 (float): Length of the second link of the robot arm.
            range1 (float): Range of motion for the first servo.
            range2 (float): Range of motion for the second servo.
    """

    def __init__(self, link1, link2, range1, range2):
        self.link1, self.link2 = link1, link2
        self.range1, self.range2 = range1, range2
        self.max_reach = link1 + link2
        self.km = Kinematics(link1, link2, range1, range2)
        self.pos_x, self.pos_y = 600, 0
        self.pos_z = 500
        self.pos_r = 512
        self.pos_gripper = 300
        self.ax12_range = 600
        self.offset_range1 = ((self.ax12_range - (self.range1*2)) / 2)
        self.offset_range2 = ((self.ax12_range - (self.range2*2)) / 2)
        self.client = None
        self.servobase_id = 1
        self.servomid_id = 3
        self.servorotation_id = 2
        self.gripper_servo_id = 5
        self.gripper_open = True
        self.max_pos_multiplier = 1.5
        self.servo_controller = ServoController()
        self.servo_controller.execute_command(self.servobase_id, 30, 512, 50)
        self.servo_controller.execute_command(self.servomid_id, 30, 512, 50)
        # TODO change to be command 32 for move with the wheelmode
        # Wat bedoel je hiermee? @tjalling
        self.servo_controller.execute_command(2, 30, self.pos_r, 50)

    """
    Executes a command to control the servo motors.

    Args:
        angle0 (int): The desired angle for servo1.
        angle1 (int): The desired angle for servo2.
    """

    def command(self, angle1, angle2):
        print(f'Angle1: {(2*self.range1)-angle1}, Angle2: {angle2}')

        thread1 = threading.Thread(
                target=self.execute_command_threaded,
                args=(self.servobase_id, 30, angle1, self.range1, 100))
        thread2 = threading.Thread(
                target=self.execute_command_threaded,
                args=(self.servomid_id, 30, angle2, self.range2, 100))

        thread1.start()
        thread2.start()

        thread1.join()
        thread2.join()

    def execute_command_threaded(self, id, address, angle, range_link, speed):
        self.servo_controller.execute_command(
            id,
            address,
            self.map_angle_to_servo_position(angle, range_link),
            speed)

    """
    Converts the given angle in degrees to the corresponding unit value for the AX-12 servo motor.

    Args:
        degrees (int): The angle in degrees to be converted.

    Returns:
        int: The converted unit value for the AX-12 servo motor.
    """

    def convert_unit(self, degrees):
        return round((1023 / self.ax12_range) * degrees)

    """
        Handles the input commands for the Bluetooth controller.
        The input commands are used to control the movement of the robot.

        The commands include:
        - "forward": Moves the robot forward by increasing the x position.
        - "backward": Moves the robot backward by decreasing the x position.
        - "left": Moves the robot to the left by increasing the y position.
        - "right": Moves the robot to the right by decreasing the y position.
        - "up": Moves the robot up by increasing the z position.
        - "down": Moves the robot down by decreasing the z position.
        - "Grijpen": Opens or closes the gripper based on the current position.
        - "init": Moves the robot to the start positions.

        Args:
            input (str): The input command string. Multiple commands can be separated by semicolons.

        Returns:
            None
        """

    def move_arm(self, x, y, z, r, gripper):
        target_x = self.pos_x
        target_y = self.pos_y
        target_z = self.pos_z
        # target_r is never used waarom ??
        target_r = self.pos_r
        target_gripper = self.pos_gripper
        gripper_open = self.gripper_open

        if self.pos_x != target_x or self.pos_y != target_y or self.pos_z != target_z or self.pos_r != target_r or self.pos_gripper != target_gripper:
            self.threaded_move_x_y(target_x, target_y)
            self.move_z_axis(target_z)
            self.move_r_axis(target_r)
            self.open_close_gripper(target_gripper, gripper_open)

        if x != target_x or y != target_y :
            if x < self.max_reach and y < self.max_reach:
                self.threaded_move_x_y(x, y)

        if z != target_z:
            if z < 1000:
                self.move_z_axis(z)
        # Process each message separately they are split by a semicolon(;)
        messages = input.split(';')
        # Remove leading and trailing whitespace from the message
        message = messages[0]
        message = message.strip()
        print("Message: ", message)

        # Process the message based on its content
        match message:
            case "forward":
                # If the target x position is within the valid range
                if target_x < (self.ax12_range*self.max_pos_multiplier):
                    target_x += 40
                    self.move_x_y(target_x, target_y)

            case "backward":
                # If the target x position is within the valid range
                if target_x > -(self.ax12_range*self.max_pos_multiplier):
                    target_x -= 40
                    self.move_x_y(target_x, target_y)

            case "left":
                # If the target y position is within the valid range
                if target_y < (self.ax12_range*self.max_pos_multiplier):
                    target_y += 40
                    self.move_x_y(target_x, target_y)

            case "right":
                # If the target y position is within the valid range
                if target_y > -(self.ax12_range*self.max_pos_multiplier):
                    target_y -= 40
                    self.move_x_y(target_x, target_y)

            case "up":
                # If the target z position is within the valid range
                if target_z < 1000:
                    target_z += 20

                self.move_z_axis(target_z)

            case "down":
                # If the target z position is within the valid range
                if target_z > 80:
                    target_z -= 20
                self.move_z_axis(target_z)

            case "Grijpen":
                self.open_close_gripper(target_gripper, gripper_open)

            case "cw":
                self.pos_r += 20
                self.move_r_axis(self.pos_r)

            case "ccw":
                self.pos_r -= 20
                self.move_r_axis(self.pos_r)

            # move to start position xy and z
            case "init":
                self.move_x_y(target_x, target_y)
                self.move_z_axis(target_z)

            case _:
                print("Invalid input: ", input)
        # set the position to the new target position(possible not needed?)
        self.pos_x = target_x
        self.pos_y = target_y

    """
    Starts a new thread to move to the target x and y coordinates.

    Args:
        target_x (float): Target x-coordinate.
        target_y (float): Target y-coordinate.
    """

    def threaded_move_x_y(self, target_x, target_y):
        thread = threading.Thread(
                target=self.move_x_y,
                args=(target_x, target_y))
        thread.start()

    """
    Moves to the target x and y coordinates if they are within reach.

    Args:
        target_x (float): Target x-coordinate.
        target_y (float): Target y-coordinate.
    """

    def move_x_y(self, target_x, target_y):
        if self.is_within_reach(target_x, target_y):
            self.pos_x, self.pos_y = target_x, target_y
            angle1, angle2 = self.km.inverse_kinematics(target_x, target_y)
            # self.servo_controller.execute_command(self.servobase_id, 30,target_x, 30)
            # self.servo_controller.execute_command(self.servomid_id, 30,target_y, 30)

            self.command(angle1, angle2)
            print("Target position reached", target_x, target_y)
        else:
            self.servo_controller.execute_command(self.servobase_id, 30, target_x, 30)
            self.servo_controller.execute_command(self.servomid_id, 30, target_y, 30)
            # we cant reach the coordinate but will try to get as close as possible
            angle1, angle2 = self.km.inverse_kinematics(target_x, target_y)
            self.command(angle1, angle2)
            print("Target position is out of reach", target_x, target_y)

    """
    Opens or closes the gripper based on the current state.

    Args:
        target_gripper (float): Target position for the gripper.
        gripper_open (bool): Whether the gripper should be open or closed.
    """

    def open_close_gripper(self, target_gripper, gripper_open):

        # If the gripper is currently closed open it
        if gripper_open == False:
            # While the target position is within the valid range
            while 0 <= target_gripper <= 820:
                try:
                    # Get the current load on the servo
                    load = self.servo_controller.execute_getstatus(5, 40, 2)
                    print("load is:",load)
                    # If the load is within a certain range,the gripper is open(means it is experiencing external load)
                    if load > 1600 and load < 2048:
                        gripper_open = True 
                        self.gripper_open = gripper_open
                        break
                    # If it is not experiencing load but is open past the threshold it is open
                    if target_gripper >= 750:
                        gripper_open = True
                        self.gripper_open = gripper_open
                        break
                    # If the gripper is not open and not experiencing load, open it
                    else:
                        target_gripper += 40
                        self.servo_controller.execute_command(
                                5,
                                30,
                                target_gripper,
                                300)
                except Exception as e:
                    print("Error opening: ", e)

        # If the gripper is currently open close it
        elif gripper_open == True:
            # While the target position is within the valid range
            while 0 <= target_gripper <= 820:
                try:
                    # Get the current load on the servo
                    load = self.servo_controller.execute_getstatus(5, 40, 2)
                    print("load is:", load)
                    # If the load is within a certain range,the gripper is closed(means it is experiencing external load)
                    if load > 400 and load < 1023:
                        gripper_open = False
                        self.gripper_open = gripper_open
                        break
                    if target_gripper <= 50:
                        # If its not experiencing load then it is not closed and we need to close it
                        gripper_open = False
                        self.gripper_open = gripper_open
                        break
                    else:
                        target_gripper -= 40
                        self.servo_controller.execute_command(
                                5,
                                30,
                                target_gripper,
                                300)
                except Exception as e:
                    print("Error closing: ", e)

    def move_z_axis(self, target_z):
        # Check if the target position is above or below the current position
        if target_z > self.pos_z:
            # Move up
            self.servo_controller.move_for_duration(41, 32, target_z, 1500)
        elif target_z < self.pos_z:
            # Move down
            self.servo_controller.move_for_duration(41, 32, target_z, 400)
        else:
            # Target position is the same as the current position, do nothing maybe unnecessary
            pass
        # Update the current position
        self.pos_z = target_z

    """
    Moves the rotation-axis to the target position.

    Args:
        target_r (float): Target position for the rotation-axis of the gripper.
    """

    def move_r_axis(self, target_r):
        self.servo_controller.execute_command(self.servorotation_id, 30, target_r, 600)

    """
    check if the target position is within reach
    Args:
        target_x (float): Target x-coordinate.
        target_y (float): Target y-coordinate.
    """

    def is_within_reach(self, target_x, target_y):
        distance = (target_x**2 + target_y**2)**0.5
        return 10 < distance <= self.max_reach
    """
    Handles received data.

    Args:
        data (str): The received data string.
    """

    def data_received(self, data):
        if data is not None and isinstance(data, str):
            print("Received data: ", data)
            self.handle_input(data)

    """
    Maps the given angle to a servo position.

    Args:
        angle (float): The angle to map.
        range_link (float): The range of motion for the servo.

    Returns:
        int: The mapped servo position.
    """

    def map_angle_to_servo_position(self, angle, range_link):
        offset = (300 - (range_link * 2)) / 2
        position = int((1023 / 300) * (offset + angle + range_link))
        print(position)
        return position
