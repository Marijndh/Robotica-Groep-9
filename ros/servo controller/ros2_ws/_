import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import re
import serial
import RPi.GPIO as GPIO
from time import sleep

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')
        self.subscription = self.create_subscription(
            String,
            'servo_command',
            self.command_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # Initialize serial port and GPIO pins
        self.direction_pin = 18
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.direction_pin, GPIO.OUT)
        GPIO.setwarnings(False)
        self.serial_port = serial.Serial('/dev/ttyS0', baudrate=1000000, timeout=1)

        # Define servo operation parameters
        self.ADDR_MX_GOAL_POSITION = 30
        self.ADDR_MX_MOVING_SPEED = 32
        self.ADDR_MX_PRESENT_POSITION = 36

    def command_callback(self, msg):
        # Define regex pattern to match servo command format: "servo_id command duration position [value2]"
        pattern = r'(\d+) (\d+) (\d+) (\d+)(?: (\d+))?'
        match = re.match(pattern, msg.data)
        if match:
            # Extract parameters from regex groups
            servo_id = int(match.group(1))
            command = int(match.group(2))
            duration = int(match.group(3))
            value = int(match.group(4))
            value2 = int(match.group(5)) if match.group(5) else None

            print("servo id:", servo_id)
            print("command:", command)
            print("duration(ms):", duration)
            print("value:", value)
            print("value2:", value2)

            # Perform servo operation based on parameters
            if value2 is not None:
                self.execute_command(servo_id, command, value, value2)
            elif command != 32:
                self.execute_command(servo_id, command, value)
            else:
                self.move_for_duration(servo_id, command, duration, value)
        else:
            self.get_logger().warn("Invalid command format")

    def execute_command(self, servo_id, command, value, value2=None):
        if value2 is not None:
            packet = self.build_packet(servo_id, command, value, value2)
        else:
            packet = self.build_packet(servo_id, command, value)
        print(packet)
        self.send_packet(packet)

    def move_for_duration(self, servo_id, command, duration, value, value2=None):
        if value2 is not None:
            self.execute_command(servo_id, command, value, value2)
        else:
            self.execute_command(servo_id, command, value)
        
        sleep(duration / 1000)
        self.stop(servo_id)

    def read_position(self, servo_id):
        packet = self.build_packet(servo_id, self.ADDR_MX_PRESENT_POSITION, 0)
        response = self.send_packet(packet)
        if response:
            position = self.parse_position_response(response)
            return position
        else:
            return None

    def build_packet(self, dxl_id, address, value, value2=None):
        if value2 is not None:
            length = 7  # Length of parameters + instruction + checksum
            checksum = ~(dxl_id + length + 3 + address + (value & 0xFF) + ((value >> 8) & 0xFF) + (value2 & 0xFF) + ((value2 >> 8) & 0xFF)) & 0xFF
            packet = bytearray([
                0xFF, 0xFF,
                dxl_id,
                length,
                3,
                address,
                value & 0xFF,
                (value >> 8) & 0xFF,
                value2 & 0xFF,
                (value2 >> 8) & 0xFF,
                checksum & 0xFF
            ])
        else:
            length = 5  # Length of parameters + instruction + checksum
            checksum = ~(dxl_id + length + 3 + address + (value & 0xFF) + ((value >> 8) & 0xFF)) & 0xFF
            packet = bytearray([
                0xFF, 0xFF,
                dxl_id,
                length,
                3,
                address,
                value & 0xFF,
                (value >> 8) & 0xFF,
                checksum & 0xFF
            ])
        return packet

    def send_packet(self, packet):
        GPIO.output(self.direction_pin, GPIO.HIGH)
        self.serial_port.write(packet)
        self.serial_port.flush()
        GPIO.output(self.direction_pin, GPIO.LOW)
        response = self.serial_port.read(6)
        self.get_logger().info(f'Response: {response.hex()}')
        return response

    def stop(self, servo_id):
        packet = self.build_packet(servo_id, self.ADDR_MX_MOVING_SPEED, 0)
        self.send_packet(packet)

    def parse_position_response(self, response):
        if len(response) < 6:
            self.get_logger().error('Incomplete response received')
            return None
        position = (response[5] << 8) | response[6]
        return position

def main(args=None):
    rclpy.init(args=args)
    servo_controller = ServoController()
    rclpy.spin(servo_controller)
    servo_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
