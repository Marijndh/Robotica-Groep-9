import threading
from flask import Flask, Response, request, jsonify
from flask_cors import CORS
from webcam.webcam import Webcam
from servo.servo_controller import ServoController
from servo.kinematics import Kinematics
from controller.bluetooth_controller import BluetoothController
from controls.controls import Leds
from controls.controls import Encoder


def map_angle_to_servo_position(angle, range_link):
    offset = (300 - (range_link * 2)) / 2
    print(int((1023 / 300) * (offset + angle + range_link)))
    return int((1023 / 300) * (offset + angle + range_link))


class FlaskServer(object):
    def __init__(self, app, **configs):
        self.app = app
        self.configs(**configs)
        self.link1 = 300
        self.link2 = 300
        self.range_l1 = 100
        self.range_l2 = 120
        self.bluetooth_controller = BluetoothController(self.link1, self.link2, self.range_l1, self.range_l2)
        self.webcam = None
        self.encoder = Encoder()
        self.leds = Leds()
        self.servo_controller = ServoController()
        self.kinematics = Kinematics(link1=self.link1, link2=self.link2, range_l1=self.range_l1, range_l2=self.range_l2)
        self.servo_controller.execute_command(1, 30, 512,
                                              50)  # initialise the servo's to its straight ahead starting position
        self.servo_controller.execute_command(2, 30, 512, 50)
        self.bluetooth_controller.start()

    # Set configuration
    def configs(self, **configs):
        for config, value in configs:
            self.app.config[config.upper()] = value

    # Add an endpoint
    def add_endpoint(self, endpoint=None, endpoint_name=None, handler=None, methods=['GET'], *args, **kwargs):
        self.app.add_url_rule(endpoint, endpoint_name, handler, methods=methods, *args, **kwargs)

    # Run the app
    def run(self, **kwargs):
        self.app.run(**kwargs)

    # Index page for server
    @staticmethod
    def index():
        return "Webcam streaming. Go to /webcam to see the feed."

    # Live feed page using webcam
    def live_feed(self):
        if self.webcam is None:
            self.webcam = Webcam()
        num_frames = int(request.args.get('num_frames', 0))
        return Response(self.webcam.generate_frames(num_frames), mimetype='multipart/x-mixed-replace; boundary=frame')

    # Page to retrieve an image capture from webcam
    def capture(self):
        if self.webcam is None:
            self.webcam = Webcam()
        return Response(self.webcam.image(), mimetype='image/jpeg')

    # Endpoint to execute a servo command
    def execute_servo_command(self):
        data = request.json
        servo_id = data.get('servo_id')
        command = data.get('command')
        value = data.get('value')
        value2 = data.get('value2')
        if not servo_id or not command or value is None:
            return jsonify({"status": "error", "message": "Invalid parameters"}), 400
        response = self.servo_controller.execute_command(servo_id, command, value, value2)
        return jsonify(response)

    # Endpoint to move the servo for a duration
    def move_servo(self):
        data = request.json
        servo_id = data.get('servo_id')
        command = data.get('command')
        duration = data.get('duration')
        value = data.get('value')
        if not servo_id or not command or not duration or value is None:
            return jsonify({"status": "error", "message": "Invalid parameters"}), 400
        self.servo_controller.move_for_duration(servo_id, command, duration, value)
        return jsonify({"status": "success", "message": "Servo moved"})

    # Endpoint to execute a complex servo command
    def servo_command(self):
        return self.servo_controller.servo_command()

    def servo_kinematics(self):
        data = request.json
        command_string = data.get('command')

        if not command_string:
            return jsonify({"status": "error", "message": "Invalid parameters"}), 400

        try:
            # Expecting the command format to be "x y"
            x, y = map(float, command_string.split())
        except ValueError:
            return jsonify({"status": "error", "message": "Invalid command format"}), 400

        joint_base, joint_middle = self.kinematics.inverse_kinematics(x, y)

        # Map angles to servo positions
        servo_base_position = map_angle_to_servo_position(joint_base, self.range_l1)
        servo_middle_position = map_angle_to_servo_position(joint_middle, self.range_l2)



    def controls(self):
        # Implement controls logic here stuff like led and encoder maybe controller to?
        data = request.json
        brightness_r = data.get('brightness_r')
        brightness_g = data.get('brightness_g')
        brightness_b = data.get('brightness_b')
        if brightness_r is None or brightness_g is None or brightness_b is None:
            return jsonify({"status": "error", "message": "Invalid parameters"}), 400
        self.leds.control_rgb_led(brightness_r, brightness_g, brightness_b)
        return jsonify({"status": "success", "message": "Brightness changed"})

    def read_encoder(self):
        data = request.json
        if 'action' not in data:
            return jsonify({"status": "error", "message": "Invalid parameters"}), 400

        if data['action'] == 'read_angle':
            angle = self.encoder.read_angle()
            return jsonify({"status": "success", "angle": angle})

        elif data['action'] == 'read_direction':
            direction = self.encoder.read_direction()
            return jsonify({"status": "success", "direction": direction})

        else:
            return jsonify({"status": "error", "message": "Invalid action"}), 400


def main():
    # Initialise server
    flask = Flask(__name__)
    CORS(flask)
    app = FlaskServer(flask)

    # Add endpoints for the server
    app.add_endpoint('/', 'index', app.index, methods=['GET'])
    app.add_endpoint('/webcam', 'webcam', app.live_feed, methods=['GET'])
    app.add_endpoint('/image', 'image', app.capture, methods=['GET'])

    app.add_endpoint('/encoder', 'encoder', app.read_encoder, methods=['POST'])
    app.add_endpoint('/servo/command', 'servo_command', app.servo_command, methods=['POST'])
    app.add_endpoint('/servo/kinematics', 'kinematics', app.servo_kinematics, methods=['POST'])
    app.add_endpoint('/controls/controls', 'controls', app.controls, methods=['POST'])

    # Run the app
    app.run(host='0.0.0.0', port=5000)


if __name__ == "__main__":
    main()