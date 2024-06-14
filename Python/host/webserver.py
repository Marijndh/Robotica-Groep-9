import cv2
from flask import Flask, Response, request, jsonify
from flask_cors import CORS
import re
import serial
import RPi.GPIO as GPIO
from time import sleep
import time


class WebCam:
    def __init__(self):
        self.camera = cv2.VideoCapture(-1)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    def generate_frames(self):
        while True:
            success, frame = self.camera.read()
            if not success:
                break
            else:
                ret, buffer = cv2.imencode('.jpg', frame)
                frame = buffer.tobytes()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

    def capture_image(self):
        success, frame = self.camera.read()
        if not success:
            return None
        else:
            ret, buffer = cv2.imencode('.jpg', frame)
            return buffer.tobytes()


class ServoController:
    ADDR_MX_MOVING_SPEED = 32

    def __init__(self):
        print("Initializing ServoController...")
        self.direction_pin = 18
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.direction_pin, GPIO.OUT)
        GPIO.setwarnings(False)
        print("GPIO setup complete.")
        self.serial_port = serial.Serial('/dev/ttyS0', baudrate=1000000, timeout=0.5)
        print("Serial port setup complete.")

    def execute_getstatus(self, servo_id, command, duration):
        packet = self.build_packet(servo_id, command, duration)
        return self.send_packet(packet)

    def execute_command(self, servo_id, command, value, value2=None):
        if value2 is not None:
            packet = self.build_packet(servo_id, command, None, value, value2)
        else:
            packet = self.build_packet(servo_id, command, None, value)
        return self.send_packet(packet)

    def move_for_duration(self, servo_id, command, duration, value):
        packet = self.build_packet(servo_id, command, None, value)
        self.send_packet(packet)
        sleep(duration / 1000)
        self.stop(servo_id)

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
            return "Invalid header"
        servo_id = response[2]
        length = response[3]
        error = response[4]
        parameters = response[5:-1]
        checksum = response[-1]
        calculated_checksum = (~sum(response[2:-1]) & 0xFF)
        if calculated_checksum != checksum:
            return "Checksum error"
        parameters_as_int = int.from_bytes(parameters, byteorder='little')
        decoded_data = {
            "servo_id": servo_id,
            "length": length,
            "error": error,
            "parameters": parameters_as_int,
            "checksum": checksum
        }
        return decoded_data

    def send_packet(self, packet):
        GPIO.output(self.direction_pin, GPIO.HIGH)
        self.serial_port.write(packet)
        while self.serial_port.out_waiting > 0:
            time.sleep(0.000000001)
        time.sleep(0.000001)
        GPIO.output(self.direction_pin, GPIO.LOW)
        response = bytearray()
        no_input = False
        start = time.time()
        while True:
            byte = self.serial_port.read(1)
            if not byte:
                no_input = True
            else:
                response.extend(byte)
            if len(response) >= 4 and no_input:
                break
            if time.time() - start > 5:
                break
        decoded_response = self.decode_response(response)
        if isinstance(decoded_response, str):
            return {"status": "error", "message": decoded_response}
        else:
            parameter_value = decoded_response['parameters']
            parameter_hex = hex(parameter_value)
            return {"status": "success", "response": decoded_response, "parameter_value": parameter_value,
                    "parameter_hex": parameter_hex}

    def stop(self, servo_id):
        packet = self.build_packet(servo_id, self.ADDR_MX_MOVING_SPEED, None, 0)
        self.send_packet(packet)


class WebServer:
    def __init__(self):
        print("Initializing WebServer...")
        self.app = Flask(__name__)
        CORS(self.app)
        self.webcam = WebCam()
        self.servo_controller = ServoController()
        print("Components initialized.")

        self.app.add_url_rule('/video_feed', 'video_feed', self.video_feed)
        self.app.add_url_rule('/', 'index', self.index)
        self.app.add_url_rule('/image', 'image', self.image)
        self.app.add_url_rule('/servo_command', 'servo_command', self.servo_command, methods=['POST'])

    def video_feed(self):
        return Response(self.webcam.generate_frames(),
                        mimetype='multipart/x-mixed-replace; boundary=frame')

    def index(self):
        return "Webcam streaming. Go to /video_feed to see the feed."

    def image(self):
        image = self.webcam.capture_image()
        if image is None:
            return jsonify({"status": "error", "message": "Failed to capture image"}), 500
        else:
            return Response(image, mimetype='image/jpeg')

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

            if command != 32:
                if value is None:
                    response = self.servo_controller.execute_getstatus(servo_id, command, duration)
                elif value is not None and value2 is None:
                    response = self.servo_controller.execute_command(servo_id, command, value)
                elif value2 is not None:
                    response = self.servo_controller.execute_command(servo_id, command, value, value2)
            elif command == 32:
                self.servo_controller.move_for_duration(servo_id, command, duration, value)

            return jsonify({"status": "success", "response": response}), 200
        else:
            return jsonify({"status": "error", "message": "Invalid command format"}), 400


if __name__ == '__main__':
    print("Starting WebServer...")
    server = WebServer()
    server.app.run(host='0.0.0.0', port=5000)
