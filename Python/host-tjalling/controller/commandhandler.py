from .PiClient import PiClient
from servo.servo_controller import ServoController  # Import ServoController

class ArmController:
    def __init__(self):
        self.servo_controller = ServoController()
        # Initialize other necessary attributes

    def send_command(self, servoID, angle):
        # Map angle to servo command and send it
        command = f"#{servoID}P{angle}\n"
        self.servo_controller.send_command(command)

    def get_data(self, servoID, address, length):
        # Implement this method if servo_controller.py has a similar functionality
        pass

    def command_callback(self, msg):
        # Implement this method if servo_controller.py has a similar functionality
        pass

def main():
    arm_controller = ArmController()

    # Start pi client and look for bluetooth connection
    client = PiClient(199, 173, 90, 100, arm_controller)
    client.start()

if __name__ == '__main__':
    main()
