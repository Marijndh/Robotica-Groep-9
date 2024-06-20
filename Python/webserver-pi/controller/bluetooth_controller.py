from servo.servo_controller import ServoController
from servo.kinematics import Kinematics  # Correct import for Kinematics as its in a different sibling folder
import threading
import sys
from bluedot.btcomm import BluetoothClient
from time import sleep
from controller.controller import Controller
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
        self.mode = ""
        self.color = ""
        #self.max_reach = link1 + link2
        #self.km = Kinematics(link1, link2, range1, range2)
        #self.pos_x, self.pos_y = 600, 0
        #self.pos_z = 500
        #self.pos_r = 512
        #self.pos_gripper = 300
        #self.ax12_range = 600
        #self.offset_range1 = ((self.ax12_range - (self.range1*2)) / 2)
        #self.offset_range2 = ((self.ax12_range - (self.range2*2)) / 2)
        self.client = None
        #self.servobase_id = 1
        #self.servomid_id = 3
        #self.servorotation_id = 2
        #self.gripper_servo_id = 5
        #self.gripper_open = True
        #self.max_pos_multiplier = 1.5
        self.controller = Controller(self.link1, self.link2, self.range1, self.range2)
        #self.servo_controller.execute_command(self.servobase_id, 30, 512, 50)
        #self.servo_controller.execute_command(self.servomid_id, 30, 512, 50)
        # TODO change to be command 32 for move with the wheelmode
        # Wat bedoel je hiermee? @tjalling
        #self.servo_controller.execute_command(2, 30, self.pos_r, 50)

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

    def handle_input(self, input):
        target_x = self.controller.pos_x
        target_y = self.controller.pos_y
        target_z = self.controller.pos_z
        # target_r is never used waarom ??
        target_r = self.controller.pos_r
        target_gripper = self.controller.pos_gripper
        #gripper_open = self.gripper_open

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
                if target_x < (self.controller.ax12_range*self.controller.max_pos_multiplier):
                    target_x += 15
                    self.controller.move_arm(target_x, target_y, target_z, target_r, target_gripper)

            case "backward":
                # If the target x position is within the valid range
                if target_x > -(self.controller.ax12_range*self.controller.max_pos_multiplier):
                    target_x -= 15
                    self.controller.move_arm(target_x, target_y, target_z, target_r, target_gripper)

            case "left":
                # If the target y position is within the valid range
                if target_y < (self.controller.ax12_range*self.controller.max_pos_multiplier):
                    target_y += 15
                    self.controller.move_arm(target_x, target_y, target_z, target_r, target_gripper)

            case "right":
                # If the target y position is within the valid range
                if target_y > -(self.controller.ax12_range*self.controller.max_pos_multiplier):
                    target_y -= 15
                    self.controller.move_arm(target_x, target_y, target_z, target_r, target_gripper)

            case "up":
                # If the target z position is within the valid range
                if target_z < 27:
                    target_z += 0.5
                    self.controller.move_arm(target_x, target_y, target_z, target_r, target_gripper)

            case "down":
                # If the target z position is within the valid range
                if target_z > 12:
                    target_z -= 1
                    self.controller.move_arm(target_x, target_y, target_z, target_r, target_gripper)


            case "Grijpen":
                if self.mode == "handmatig":
                    print("Gripper: ", target_gripper)
                    if target_gripper == 400:
                        target_gripper = 200
                        print("Gripper: ", target_gripper)
                        self.controller.move_arm(target_x, target_y, target_z, target_r, target_gripper)
                    elif target_gripper < 400:
                        target_gripper = 512
                        self.controller.move_arm(target_x, target_y, target_z, target_r, target_gripper)
                    else:
                        target_gripper = 350 
                        self.controller.move_arm(target_x, target_y, target_z, target_r, target_gripper)
                    
                else:
                    target_gripper = 240
                    self.controller.move_arm(target_x, target_y, target_z, target_r, target_gripper)
                    target_z = 12.5
                    target_gripper = 240
                    self.controller.move_arm(target_x, target_y, target_z, target_r, target_gripper)
                    target_gripper = 500
                    target_z = 12.8
                    self.controller.move_arm(target_x, target_y, target_z, target_r, target_gripper)
                    sleep(0.5)
                    
                    target_z = 20
                    self.controller.move_arm(target_x, target_y, target_z, target_r, target_gripper)
            
            case "cw":
                target_r += 10
                self.controller.move_arm(target_x, target_y, target_z, target_r, target_gripper)

            case "ccw":
                target_r -= 10
                self.controller.move_arm(target_x, target_y, target_z, target_r, target_gripper)

            # move to start position xy and z
            case "init":
                target_gripper = 500
                target_r = 512
                target_z = 18
                target_x = 590
                target_y = 5
                self.controller.move_arm(target_x, target_y, target_z, target_r, target_gripper)
            
            case "pink":
                self.color = "pink"
            case "red":
                self.color = "red"
            case "green":
                self.color = "green"
            case "blue":
                self.color = "blue"
            case "silver":
                self.color = "silver"
            case "autonoom":
                self.mode = "instruments"
            case "handmatig":
                self.mode = "handmatig"
            case "mollenmeppen":
                self.mode = "targets"
            case _:
                print("Invalid input: ", input)
        # set the position to the new target position(possible not needed?)
        #self.pos_x = target_x
        #self.pos_y = target_y

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


    def data_received(self, data):
        if data is not None and isinstance(data, str):
            print("Received data: ", data)
            self.handle_input(data)

    """
    Starts the BluetoothController, attempts to connect to the Bluetooth client (multithreaded so it does not block).
    Returns:
        None(TODO change to say its returned or a thread or something so no rogue threads are left behind)
    """

    def start(self):
        def attempt_connection():
            while True:
                if self.client is None:
                    print("Connecting...")
                    mac_address = "D4:8A:FC:A4:AF:06"
                    self.client = BluetoothClient(
                            mac_address,
                            self.data_received)
                    if self.client.connected:
                        print("Connected")
                        self.handle_input("init")
                        break
                    else:
                        sleep(0.1)

        connection_thread = threading.Thread(target=attempt_connection)
        connection_thread.start()
        connection_thread.join()
