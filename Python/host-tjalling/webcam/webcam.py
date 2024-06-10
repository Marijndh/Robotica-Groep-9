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
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    def generate_frames(self):
        while True:
            # Capture frame-by-frame
            ret, frame = self.camera.read()
            if not ret:
                self.camera.release()
                self.connect_camera()
                time.sleep(1)
                continue
            else:
                # Encode the frame in JPEG format
                ret, buffer = cv2.imencode('.jpg', frame)
                frame = buffer.tobytes()

                # Concatenate frame one by one and show result
                yield b'--frame\r\n'b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n'

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

            # Concatenate frame one by one and show result
            return buffer.tobytes()
