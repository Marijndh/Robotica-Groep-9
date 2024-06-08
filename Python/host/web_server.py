from flask import Flask, Response
from flask_cors import CORS
from webcam.webcam import Webcam


class FlaskServer(object):
    def __init__(self, app, **configs):
        self.app = app
        self.configs(**configs)
        self.webcam = Webcam()

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
        return Response(self.webcam.generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

    # Page to retrieve an image capture from webcam
    def capture(self):
        return Response(self.webcam.image(), mimetype='image/jpeg')


def main():
    # Initialise server
    flask = Flask(__name__)
    CORS(flask)
    app = FlaskServer(flask)

    # Add endpoints for the server
    app.add_endpoint('/', 'index', app.index, methods=['GET'])
    app.add_endpoint('/webcam', 'webcam', app.live_feed, methods=['GET'])
    app.add_endpoint('/image', 'image', app.capture, methods=['GET'])

    # Run the app
    app.run(host='0.0.0.0', port=5000)


if __name__ == "__main__":
    main()
