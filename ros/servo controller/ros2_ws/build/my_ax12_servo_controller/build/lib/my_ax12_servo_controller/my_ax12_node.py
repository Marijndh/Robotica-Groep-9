import rclpy
from rclpy.node import Node
import serial
import RPi.GPIO as GPIO
from std_msgs.msg import Float64

class DynamixelController(Node):
    def __init__(self):
        super().__init__('dynamixel_controller')
        self.direction_pin = 18
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.direction_pin, GPIO.OUT)
        GPIO.setwarnings(False)

        self.serial_port = serial.Serial('/dev/ttyS0', baudrate=1000000, timeout=1)
        
        self.ADDR_MX_OPERATING_MODE = 11#incorrect should be the adress of clockwise moving
        self.ADDR_MX_GOAL_POSITION = 32
        self.ADDR_MX_MOVING_SPEED = 32   # Address for setting moving speed
        self.ADDR_MX_PRESENT_POSITION = 36
        self.PROTOCOL_VERSION = 1.0
        self.DXL_ID = 5
        
        self.create_subscription(Float64, 'servo_position', self.listener_callback, 10)
        self.get_logger().info('Dynamixel Controller Node has been started')
        #self.set_normal_mode()

    #def set_normal_mode(self):
        # Set the servo to normal mode
        #packet = self.build_packet(self.DXL_ID, self.ADDR_MX_OPERATING_MODE, 0)
        #self.send_packet(packet)

    def listener_callback(self, msg):
        position = int(msg.data)
        if 0 <= position <= 1023:
            self.get_logger().info(f'Received position command: {position}')
            self.set_position(position)
        else:
            self.get_logger().error(f'Invalid position value: {position}')
    def degrees_to_position(self, degrees):
        if 0 <= degrees <= 300:
            return int(degrees * 1023 / 300)
        else:
            self.get_logger().error('Degrees out of range (0-300)')
            return None
    
    def set_position_degrees(self, degrees):
        position = self.degrees_to_position(degrees)
        if position is not None:
            self.set_position(position)
        
    def set_position(self, position):
        packet = self.build_packet(self.DXL_ID, self.ADDR_MX_GOAL_POSITION, position)
        self.get_logger().info(f'Sending packet: {packet.hex()}')
        self.send_packet(packet)

    def build_packet(self, dxl_id, address, value):
        length = 5
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
        self.parse_response(response)

    def parse_response(self, response):
        if len(response) < 6:
            self.get_logger().error('Incomplete response received')
            return
        
        header = response[:2]
        dxl_id = response[2]
        length = response[3]
        error = response[4]
        checksum = response[5]

        if header != b'\xff\xff':
            self.get_logger().error('Invalid header')
            return

        if error != 0:
            self.get_logger().error(f'Error in response: {error:08b}')
            self.log_error_details(error)

    def log_error_details(self, error):
        errors = {
            0x01: "Input Voltage Error",
            0x02: "Angle Limit Error",
            0x04: "Overheating Error",
            0x08: "Range Error",
            0x10: "Checksum Error",
            0x20: "Overload Error",
            0x40: "Instruction Error",
        }
        for bit, message in errors.items():
            if error & bit:
                self.get_logger().error(message)

def main(args=None):
    rclpy.init(args=args)
    dynamixel_controller = DynamixelController()
    rclpy.spin(dynamixel_controller)
    dynamixel_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
