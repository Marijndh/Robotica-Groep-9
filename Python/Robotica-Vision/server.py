from flask import Flask, request


class Server:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.app = Flask(__name__)

        self.setup_routes()

    def setup_routes(self):
        @self.app.route('/post_data', methods=['POST'])
        def post_data():
            data = request.json
            print("Received data:", data)
            return "Data received", 200

    def run(self):
        self.app.run(host=self.host, port=self.port)
