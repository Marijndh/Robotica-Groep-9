import rclpy
from rclpy.node import Node
import serial
import RPi.GPIO as GPIO
import time
from std_msgs.msg import Float64

class DynamixelController(Node):
    def __init__(self):
        super().__init__('dynamixel_controller')

        # Initialize GPIO pin for direction control
        self.direction_pin = 18
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.direction_pin, GPIO.OUT)

        # Initialize serial port
        self.serial_port = serial.Serial('/dev/ttyS0', baudrate=1000000, timeout=1)
        
        # Dynamixel protocol constants
        self.ADDR_MX_GOAL_POSITION = 30
        self.ADDR_MX_PRESENT_POSITION = 36
        self.PROTOCOL_VERSION = 1.0
        self.DXL_ID = 3  # Change to your servo ID
        self.BAUDRATE = 1000000
        self.DEVICENAME = '/dev/ttyS0'
        
        # Create a publisher to publish the position
        self.pub = self.create_publisher(Float64, 'servo_position', 10)
        
        # Set initial position
        self.set_position(512)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def set_position(self, position):
        # Create a packet to set the goal position
        packet = self.build_packet(self.DXL_ID, self.ADDR_MX_GOAL_POSITION, position)
        
        # Send the packet
        self.send_packet(packet)

    def build_packet(self, dxl_id, address, value):
        length = 5  # Length of the packet
        checksum = ~(dxl_id + length + 3 + address + (value & 0xFF) + (value >> 8) & 0xFF) & 0xFF
        packet = bytearray([
            0xFF, 0xFF,  # Header
            dxl_id,
            length,
            3,  # Write instruction
            address,
            value & 0xFF,
            (value >> 8) & 0xFF,
            checksum
        ])
        return packet

    def send_packet(self, packet):
        # Set direction to transmit
        GPIO.output(self.direction_pin, GPIO.HIGH)
        
        # Write to the serial port
        self.serial_port.write(packet)
        self.serial_port.flush()
        
        # Set direction to receive
        GPIO.output(self.direction_pin, GPIO.LOW)
        
        # Read response (if necessary)
        response = self.serial_port.read(6)
        self.get_logger().info(f'Response: {response.hex()}')

    def get_position(self):
        # Create a packet to read the present position
        packet = self.build_read_packet(self.DXL_ID, self.ADDR_MX_PRESENT_POSITION, 2)
        
        # Send the packet
        self.send_packet(packet)

        # Read the position
        response = self.serial_port.read(8)
        if len(response) == 8:
            position = response[5] + (response[6] << 8)
            return position
        else:
            self.get_logger().error("Failed to get position")
            return None

    def build_read_packet(self, dxl_id, address, length):
        checksum = ~(dxl_id + 4 + 2 + address + length) & 0xFF
        packet = bytearray([
            0xFF, 0xFF,  # Header
            dxl_id,
            4,
            2,  # Read instruction
            address,
            length,
            checksum
        ])
        return packet

    def timer_callback(self):
        # Example: set servo position to 200 (move to another position)
        self.set_position(430)
        time.sleep(1)
        self.set_position(712)  # Set position to 512 (center position)
        time.sleep(1)

        # Publish the updated position of the servo
        position = self.get_position()
        msg = Float64()
        msg.data = position if position is not None else -1
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    dynamixel_controller = DynamixelController()

    try:
        rclpy.spin(dynamixel_controller)
    except KeyboardInterrupt:
        pass

    dynamixel_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
