import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from concurrent.futures import Future

from .PiClient import PiClient
from rclpy.qos import qos_profile_sensor_data

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('arm_controller_publisher')
        self.publisher_ = self.create_publisher(String, 'servo_command', qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            String,
            'servo_response',  # corrected typo
            self.command_callback,
            qos_profile_sensor_data
        )
        self.future = None  # Initialize the future to None

    def send_command(self, servoID, angle):
        message = String()
        # ServoID, Command, Time (ms), Angle, Rotation Speed
        message.data = "{} 30 0 {} 80".format(servoID, angle)
        #print(message.data)
        self.publisher_.publish(message)

    def get_data(self, servoID, address, length):
        message = String()
        message.data = "{} {} {}".format(servoID, address, length)
        print(message.data)

        # Create a new Future object
        self.future = Future()

        # Publish the message
        self.publisher_.publish(message)

        try:
            # Wait for the result from the callback with a timeout
            result = self.future.result(timeout=5.0)  # Wait for up to 5 seconds
            return result
        except concurrent.futures.TimeoutError:
            print("Timeout waiting for response")
            return None

    def command_callback(self, msg):
        # Define what to do when a message is received
        # This will be invoked when a message is received on 'servo_response' topic
        print('Received response: {}'.format(msg.data))
        
        # Extract the parameter value from the message data
        # Assuming the format is "Servo ID: <id>, Parameter: <value> (hex: <hex_value>)"
        parts = msg.data.split(',')
        parameter_part = parts[1].strip()  # "Parameter: <value> (hex: <hex_value>)"
        parameter_value_str = parameter_part.split()[1]  # Extract the value part
        parameter_value = int(parameter_value_str)

        # Set the result of the future
        if self.future is not None and not self.future.done():
            self.future.set_result(parameter_value)
            self.future = None  # Reset the future to None after setting the result

def main(args=None):
    rclpy.init(args=args)

    arm_controller_publisher = MinimalPublisher()

    # Start pi client and look for bluetooth connection
    client = PiClient(199, 173, 90, 100, arm_controller_publisher)
    client.start()

    rclpy.spin(arm_controller_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    arm_controller_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
