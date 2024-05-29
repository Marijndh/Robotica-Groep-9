import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('webcam_publisher')
        self.publisher_ = self.create_publisher(Image, 'image_raw', 10)
        self.timer_ = self.create_timer(0.10, self.publish_image)
        self.bridge = CvBridge()

    def publish_image(self):
        # Capture frame from webcam
        cap = cv2.VideoCapture(0)
        ret, frame = cap.read()
        cap.release()

        # Convert OpenCV image to ROS Image message
        ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')

        # Publish the image
        self.publisher_.publish(ros_image)
        #self.get_logger().info('Image published')

def main(args=None):
    rclpy.init(args=args)
    webcam_publisher = WebcamPublisher()
    rclpy.spin(webcam_publisher)
    webcam_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
