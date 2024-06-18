import cv2
import time


class Webcam:
    def __init__(self):
        self.camera = None
        self.connect_camera()

    def connect_camera(self):
        # Open a connection to the webcam
        self.camera = cv2.VideoCapture(-1)

        # Set the resolution to 640x480
        self.set_resolution(640, 480)

    def set_resolution(self, width, height):
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    def generate_frames(self, num_frames):
        count = 0
        while True:
            # Capture frame-by-frame
            ret, frame = self.camera.read()
            if not ret:
                self.camera.release()
                self.connect_camera()
                time.sleep(5)
                continue
            else:
                # Encode the frame in JPEG format
                ret, buffer = cv2.imencode('.jpg', frame)
                frame = buffer.tobytes()

                # Concatenate frame one by one and show result
                yield b'--frame\r\n'b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n'
                count += 1

                if 0 < num_frames < count:
                    break

    def image(self):
        # Capture a single frame
        ret, frame = self.camera.read()
        if not ret:
            self.camera.release()
            self.connect_camera()
            time.sleep(1)
            self.image()
        else:
            # Encode the frame in JPEG format
            ret, buffer = cv2.imencode('.jpg', frame)

            return buffer.tobytes()
