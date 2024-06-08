import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
import cv2
from PIL import Image as pillow_Image
import numpy as np
import io
import threading
import time

class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('webcam_publisher')
        self.publisher_ = self.create_publisher(Image, 'image_raw', qos_profile_sensor_data)
        self.timer_ = self.create_timer((1/30), self.publish_image)
        self.capture = cv2.VideoCapture(-1, cv2.CAP_V4L2)
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)
        self.lock = threading.Lock()

    def compress_image(self, img):
        compressed_image_io = io.BytesIO()
        img.save(compressed_image_io, format='JPEG', quality=20)
        compressed_image_io.seek(0)
        return pillow_Image.open(compressed_image_io)

    def publish_image(self):
        while self.capture.isOpened():
            ret, frame = self.capture.read()
        
            if not ret:
                print("Couldn't connect to camera")
                continue

            with self.lock:
                compressed_data = self.compress_image(pillow_Image.fromarray(frame))

            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.height = compressed_data.height
            msg.width = compressed_data.width
            msg.encoding = 'bgr8'
            msg.is_bigendian = False
            msg.step = 3 * compressed_data.width
            msg.data = np.array(compressed_data).tobytes()

            # Publish the image
            self.publisher_.publish(msg)
            #self.get_logger().info('Image published')

def main(args=None):
    rclpy.init(args=args)
    webcam_publisher = WebcamPublisher()

    try:
        rclpy.spin(webcam_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        webcam_publisher.capture.release()
        webcam_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
