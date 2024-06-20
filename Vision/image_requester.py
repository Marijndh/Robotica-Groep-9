import requests
import cv2
import time
import socket
import numpy as np


class ImageRequester:
    def __init__(self):
        self.hostname = 'ubuntu'
        self.IP = "192.168.218.40"

    # Fetch an image from the Raspberry Pi by putting a certain post request to the webserver
    # In order to work you have to be on the samen network as the Raspberry
    def fetch_image(self, amount):
        images = []
        try:
            response = requests.get('http://' + self.IP + ':5000/webcam?num_frames=' + str(amount), stream=True)
            if response.status_code != 200:
                print(f"Failed to connect to http://{self.IP}:5000/webcam?num_frames={amount}")
                return images

            bytes_data = b''
            frame_boundary = b'--frame'

            for chunk in response.iter_content(chunk_size=1024):
                bytes_data += chunk

                while True:
                    # Find the frame boundary
                    frame_start = bytes_data.find(frame_boundary)
                    if frame_start == -1:
                        break

                    frame_end = bytes_data.find(frame_boundary, frame_start + len(frame_boundary))
                    if frame_end == -1:
                        break

                    # Extract frame data
                    frame_data = bytes_data[frame_start:frame_end]
                    bytes_data = bytes_data[frame_end:]

                    # Extract JPEG data
                    jpeg_start = frame_data.find(b'\r\n\r\n') + 4
                    jpeg_end = frame_data.find(b'\r\n--', jpeg_start)
                    jpeg_data = frame_data[jpeg_start:jpeg_end]

                    # Convert the JPEG data to an image
                    np_arr = np.frombuffer(jpeg_data, np.uint8)
                    image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                    images.append(image)

                    # Stop if we have enough images
                    if 0 < amount <= len(images):
                        return images

            return images


        except Exception as e:
            print("Kan geen images ophalen: "+str(e))
            return None

