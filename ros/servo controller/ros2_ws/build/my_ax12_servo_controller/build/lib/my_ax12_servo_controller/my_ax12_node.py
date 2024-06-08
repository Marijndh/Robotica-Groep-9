import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import re
import serial
import RPi.GPIO as GPIO
from time import sleep
from rclpy.qos import qos_profile_sensor_data
import time
import binascii

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')
        self.subscription = self.create_subscription(
            String,
            'servo_command',
            self.command_callback,
            qos_profile_sensor_data 
        )
        self.subscription  # prevent unused variable warning

        # Initialize serial port and GPIO pins
        self.direction_pin = 18
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.direction_pin, GPIO.OUT)
        GPIO.setwarnings(False)
        self.serial_port = serial.Serial('/dev/ttyS0', baudrate=1000000, timeout=0.5)
        self.publisher_ = self.create_publisher(String, 'servo_response', qos_profile_sensor_data)
        
        # Define servo operation parameters
        #self.ADDR_MX_GOAL_POSITION = 30
        self.ADDR_MX_MOVING_SPEED = 32
        #self.ADDR_MX_PRESENT_POSITION = 36

    def command_callback(self, msg):
        # Define regex pattern to match servo command format: "servo_id command duration position [value2]"
        pattern = r'(\d+) (\d+) (\d+)(?: (\d+))?(?: (\d+))?'
        match = re.match(pattern, msg.data)
        if match:
            # Extract parameters from regex groups
            servo_id = int(match.group(1))
            command = int(match.group(2))
            duration = int(match.group(3))
            value = int(match.group(4)) if match.group(4) else None
            value2 = int(match.group(5)) if match.group(5) else None

            #print("servo id:", servo_id)
            #print("command:", command)
            #print("duration(ms):", duration)
            #print("value:", value)
            #print("value2:", value2)

            # Perform servo operation based on parameters
            if command != 32:
                
                if value is None:
                    self.execute_getstatus(servo_id, command, duration)
                elif value is not None and value2 is None:
                    self.execute_command(servo_id,command,value)
                elif value2 is not None:
                    self.execute_command(servo_id, command, value, value2)
            
            elif command == 32:
                self.move_for_duration(servo_id, command, duration, value)        
    

            #if value is None:
            #    self.execute_getstatus(servo_id, command, duration)
            #elif value2 is not None:
            #    self.execute_command(servo_id, command, value, value2)
            #elif command != 32:
            #    self.execute_command(servo_id, command, value)
            #elif command == 32:
            #    self.move_for_duration(servo_id, command, duration, value)
        else:
            self.get_logger().warn("Invalid command format ")
            #print("message was", msg.data )
    
    def execute_getstatus(self, servo_id, command,duration):
        packet = self.build_packet(servo_id,command,duration)
        self.send_packet(packet)

    def execute_command(self, servo_id, command, value, value2=None):
        if value2 is not None:
            packet = self.build_packet(servo_id, command,None, value, value2)
        else:
            packet = self.build_packet(servo_id, command,None, value)
        #print(packet)
        self.send_packet(packet)

    def move_for_duration(self, servo_id, command, duration, value):
        print("execute for duration")
        packet = self.build_packet(servo_id, command,None, value)
        print("packet=",packet)
        self.send_packet(packet)
        
        sleep(duration / 1000)
        self.stop(servo_id)

    #def read_position(self, servo_id):
    #    packet = self.build_packet(servo_id, self.ADDR_MX_PRESENT_POSITION, 0)
    #    response = self.send_packet(packet)
    #    if response:
    #        position = self.parse_position_response(response)
    #        return position
    #    else:
    #        return None

    def build_packet(self, dxl_id, address, readsize , value=None, value2=None):
        if value2 is not None:
            print("readsize: ", readsize)
            print("value1: ", value)
            print("value2: ",value2)
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
        elif value is not None:
            print("sending 1 parameter")
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
        else:
            # Handle the case when no value is given
            print("reading shit from address: ", address)
            length = 4  # Length of parameters + instruction + checksum
            checksum = ~(dxl_id + length + 2 + address + readsize) & 0xFF
            packet = bytearray([
                0xFF, 0xFF,
                dxl_id,
                length,
                2,
                address,
                readsize,
                checksum & 0xFF
            ])
            #print("packet to read is: ", packet)

        return packet

    def decode_response(self, response):
        header = response[:2]
        if header != b'\xff\xff':
            return "Invalid header"
            

        servo_id = response[2]
        length = response[3]
        error = response[4]
        parameters = response[5:-1]
        checksum = response[-1]

        calculated_checksum = (~sum(response[2:-1]) & 0xFF)
        if calculated_checksum != checksum:
            return "Checksum error"

        # Convert parameters to int
        parameters_as_int = int.from_bytes(parameters, byteorder='little')
        decoded_data = {
            "servo_id": servo_id,
            "length": length,
            "error": error,
            "parameters": parameters_as_int,  # Store as integer
            "checksum": checksum
        }

        return decoded_data

    def parse_parameters(self, parameters):
        # Example parsing, adjust according to the specific parameter being read
        if len(parameters) == 2:
            value = int.from_bytes(parameters, byteorder='little')
            return value
        return int(parameters.hex(), 16)

    # Modify your send_packet method to decode the response
    def send_packet(self, packet):
        GPIO.output(self.direction_pin, GPIO.HIGH)
        self.serial_port.write(packet)
        while self.serial_port.out_waiting > 0:
            time.sleep(0.000000001)
        time.sleep(0.000001)
        #self.serial_port.flush()
        GPIO.output(self.direction_pin, GPIO.LOW)
        #print("packet sent is: ", packet)
        response = bytearray()
        no_input = False
        start = time.clock_gettime_ns(0)
        while True:
            byte = self.serial_port.read(1)
            if not byte:
                #print("nothing to read")
                no_input = True
            else:
                response.extend(byte)
            if len(response) >= 4 and no_input:
                #print("end of message")
                break
            if time.clock_gettime_ns(0) - start > 5000000:
                #print("time is up")
                break
        #response = self.serial_port.read(6)  # Adjust the read length as needed
        decoded_response = self.decode_response(response)

        if isinstance(decoded_response, str):
            self.get_logger().info(decoded_response)
        else:
            parameter_value = decoded_response['parameters']
            parameter_hex = hex(parameter_value)  # Convert integer to hex string
            self.get_logger().info(f"Decoded Response: {decoded_response}, Parameter Value: {parameter_value} (hex: {parameter_hex})")

            msg = String()
            msg.data = f"Servo ID: {decoded_response['servo_id']}, Parameter: {parameter_value} (hex: {parameter_hex})"
            self.publisher_.publish(msg)

        return response

    def stop(self, servo_id):
        packet = self.build_packet(servo_id, self.ADDR_MX_MOVING_SPEED, None,0)
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
