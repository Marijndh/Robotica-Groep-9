from flask import Flask, request, jsonify


class Server:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.app = Flask(__name__)
        self.received_data = None  # To store the received data

    def setup_routes(self):
        @self.app.route('/post_data', methods=['POST'])
        def post_data():
            try:
                data = request.json  # Get data from the POST request
                print(f"Posting data: {data}")
            except Exception as e:
                print("Error:", e)
            return "Data received", 200

        @self.app.route('/get_data', methods=['GET'])
        def get_data():
            if self.received_data:
                return jsonify(self.received_data)
            else:
                print("Nothing is happening")
                return "No data received", 200

    def run(self):
        self.app.run(host=self.host, port=self.port)


# Now you can create an instance of Server and run it
if __name__ == '__main__':
    server = Server('localhost', 5000)
    server.setup_routes()  # Moved this line after creating the server instance
    server.run()

