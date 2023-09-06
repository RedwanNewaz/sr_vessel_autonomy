import socket

class SimpleSocketServer:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.server_socket = None
        self.is_running = False

    def start(self):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(1)
        print("Server started. Listening on {}:{}".format(self.host, self.port))

        self.is_running = True
        while self.is_running:
            client_socket, client_address = self.server_socket.accept()
            print("Accepted connection from {}:{}".format(client_address[0], client_address[1]))

            # Handle the client connection
            self.handle_client(client_socket, client_address)

    def stop(self):
        self.is_running = False
        if self.server_socket:
            self.server_socket.close()

    def handle_client(self, client_socket, client_address):
        while True:
            data = client_socket.recv(1024)
            if not data:
                break

            received_message = data.decode("utf-8")
            print("Received from client: {}".format(received_message))

            # Echo the received message back to the client
            GPS_MSG = "$GGA,123456.78,3712.1234,N,12156.7890,W,1,08,0.9,545.4,M,46.9,M,,*3A"
            client_socket.sendall(GPS_MSG.encode("utf-8"))

        client_socket.close()
        print("Connection closed with {}:{}".format(client_address[0], client_address[1]))

if __name__ == "__main__":
    server = SimpleSocketServer("127.0.0.1", 8080)
    try:
        server.start()
    except KeyboardInterrupt:
        server.stop()
