import threading
from flask import Flask, Response, request, jsonify
from flask_cors import CORS
from webcam.webcam import Webcam
from servo.servo_controller import ServoController
from servo.kinematics import Kinematics
from controller.bluetooth_controller import BluetoothController

class FlaskServer(object):
    def __init__(self, app, **configs):
        self.app = app
        self.configs(**configs)
        self.link1 = 199
        self.link2 = 173
        self.range_l1 = 90
        self.range_l2 = 100
        self.bluetooth_controller = BluetoothController(self.link1,self.link2,self.range_l1,self.range_l2)
        self.webcam = None
        self.servo_controller = ServoController()
        self.kinematics = Kinematics(link1=self.link1, link2=self.link2, range_l1=self.range_l1, range_l2=self.range_l2)  
        self.servo_controller.execute_command(1, 30, 512,50)# initialise the servo's to its straight ahead starting position
        self.servo_controller.execute_command(2, 30, 512,50)
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
        return Response(self.webcam.generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

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

    def map_angle_to_servo_position(self, angle,range_link):
        offset = (300-(range_link*2))/2 
        return int((1023 / 300) * (offset + angle+range_link))


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
        servo_base_position = self.map_angle_to_servo_position(joint_base,self.range_l1)
        servo_middle_position = self.map_angle_to_servo_position(joint_middle,self.range_l2)

        # Function to move the servos
        def move_servo(servo_id, command, value,speed):
            self.servo_controller.execute_command(servo_id, command, value,speed)

        # Create threads for moving servos concurrently
        base_thread = threading.Thread(target=move_servo, args=(1, 30, servo_base_position,30))
        middle_thread = threading.Thread(target=move_servo, args=(2, 30, servo_middle_position,30))

        # Start the threads
        base_thread.start()
        middle_thread.start()

        # Wait for both threads to finish
        base_thread.join()
        middle_thread.join()

        return jsonify({"status": "success", "joint_base": joint_base, "joint_middle": joint_middle,
                        "servo_base_position": servo_base_position, "servo_middle_position": servo_middle_position})


    def controls(self):
        # Implement controls logic here stuff like led and encoder maybe controller to?
        pass

def main():
    # Initialise server
    flask = Flask(__name__)
    CORS(flask)
    app = FlaskServer(flask)

    # Add endpoints for the server
    app.add_endpoint('/', 'index', app.index, methods=['GET'])
    app.add_endpoint('/webcam', 'webcam', app.live_feed, methods=['GET'])
    app.add_endpoint('/image', 'image', app.capture, methods=['GET'])
    
    app.add_endpoint('/servo/command', 'servo_command', app.servo_command, methods=['POST'])
    app.add_endpoint('/servo/kinematics', 'kinematics', app.servo_kinematics, methods=['POST'])
    app.add_endpoint('/controls', 'controls', app.controls, methods=['POST'])
    
# Run the app
    app.run(host='0.0.0.0', port=5000)


if __name__ == "__main__":
    main()
