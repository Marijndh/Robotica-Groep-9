import threading
import RPi.GPIO as GPIO
import serial
import time
import re
import binascii
import sys
from flask import request, jsonify
from time import sleep
sys.path.append("..")
from utils.common import waiting



class ServoController:
    ADDR_MX_MOVING_SPEED = 32  # Address for the moving speed register of the servo

    def __init__(self):
        print("Initializing ServoController...")
        self.lock = threading.Lock()
        self.direction_pin = 18  # GPIO pin used for direction control
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.direction_pin, GPIO.OUT)
        GPIO.setwarnings(False)
        print("GPIO setup complete.")
        self.serial_port = serial.Serial('/dev/ttyS0', baudrate=1000000, timeout=0.001)  # Setup serial communication
        print("Serial port setup complete.")
        self.lock = threading.Lock()

    # Method to execute a get status command on a servo
    def execute_getstatus(self, servo_id, command, duration):
        packet = self.build_packet(servo_id, command, duration)  # Build the packet
        response = self.send_packet(packet)
        #decoded = self.decode_response(response)
        #parameters = self.parse_parameters(decoded)
        return response  # Send the packet and return the response

    # Method to execute a command on a servo
    def execute_command(self, servo_id, command, value, value2=None):
        if value2 is not None:
            packet = self.build_packet(servo_id, command, None, value, value2)
        else:
            packet = self.build_packet(servo_id, command, None, value)
        return self.send_packet(packet)

    # Method to move the servo for a specified duration
    def move_for_duration(self, servo_id, command, duration, value):
        packet = self.build_packet(servo_id, command, None, value)
        self.send_packet(packet)
        sleep(duration / 1000)  # Sleep for the specified duration (in milliseconds)
        self.stop(servo_id)  # Stop the servo after the duration
        waiting(2)  # Wait for the servo to stop moving
        self.stop(servo_id)
        
    # Method to build a packet for communication with the servo(should be switchcase or something as its not dry now)
    def build_packet(self, dxl_id, address, readsize, value=None, value2=None):
        if value2 is not None:
            length = 7
            checksum = ~(dxl_id + length + 3 + address + (value & 0xFF) + ((value >> 8) & 0xFF) + (value2 & 0xFF) + (
                        (value2 >> 8) & 0xFF)) & 0xFF
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
        else:
            length = 4
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
        return packet

    def decode_response(self, response):
        header = response[:2]
        if header != b'\xff\xff':
            back = ("invalid header" + binascii.hexlify(response).decode('utf-8'))
            return back

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

    # Method to send a packet to the servo and receive a response
    def send_packet(self, packet):
        # lock so no other thread can write/read at the same time
        self.lock.acquire()
        GPIO.output(self.direction_pin, GPIO.HIGH)  # Set direction to send data
        self.serial_port.write(packet)  # Write packet to the serial port
        while self.serial_port.out_waiting > 0:  # Wait until the packet is completely sent
            waiting(0.001)
        waiting(1)  # Small delay so it waits a little after its empty so we are sure it is done must be below the delay time of servo(500us default)
        GPIO.output(self.direction_pin, GPIO.LOW)  # Set direction to receive data
        response = bytearray()
        no_input = False
        start = time.clock_gettime_ns(0)
        while True:
            byte = self.serial_port.read(1)  # Read one byte at a time
            if not byte:
                no_input = True
            else:
                response.extend(byte)
            if len(response) >= 6 and no_input:
                break
            if time.clock_gettime_ns(0) - start > 4000000:  # Timeout after 5 seconds(need to be changed to 5ms or so)
                break
        self.lock.release()
        # free the lock
        decoded_response = self.decode_response(response)  # Decode the response
        if isinstance(decoded_response, str):
            return {"status": "error", "message": decoded_response}
        else:
            parameter_value = decoded_response['parameters']
            parameter_hex = hex(parameter_value)
            return {"status": "success", "response": decoded_response, "parameter_value": parameter_value,
                    "parameter_hex": parameter_hex}

    # Method to stop the servo
    def stop(self, servo_id):
        packet = self.build_packet(servo_id, self.ADDR_MX_MOVING_SPEED, None, 0)
        self.send_packet(packet)

    # Flask route to handle servo commands
    def servo_command(self):
        data = request.json
        print(f"Request data: {data}")  # Debug print

        if data is None:
            return jsonify({"status": "error", "message": "Invalid JSON"}), 400

        command_string = data.get('command')
        print(f"Command string: {command_string}")  # Debug print

        if not command_string:
            return jsonify({"status": "error", "message": "No command provided"}), 400

        pattern = r'(\d+) (\d+) (\d+)(?: (\d+))?(?: (\d+))?'
        match = re.match(pattern, command_string)
        if match:
            servo_id = int(match.group(1))
            command = int(match.group(2))
            duration = int(match.group(3))
            value = int(match.group(4)) if match.group(4) else None
            value2 = int(match.group(5)) if match.group(5) else None

            if command != self.ADDR_MX_MOVING_SPEED:
                if value is None:
                    response = self.execute_getstatus(servo_id, command, duration)
                elif value is not None and value2 is None:
                    response = self.execute_command(servo_id, command, value)
                elif value2 is not None:
                    response = self.execute_command(servo_id, command, value, value2)
            elif command == self.ADDR_MX_MOVING_SPEED:
                self.move_for_duration(servo_id, command, duration, value)
                response = "Servo moved"

            return jsonify({"status": "success", "response": response}), 200
        else:
            return jsonify({"status": "error", "message": "Invalid command format"}), 400
