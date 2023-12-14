import socket



##########################################################################################
# Base class for controller

class controller():
    def __init__(self, host, port):
        # TCP connection details
        self.host = host
        self.port = port
        self.socket = None
        # Actuation parameter
        self.speed = 0
        self.steer_rate = 0

    # Establishing TCP connection
    def connect(self):
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.host, self.port))
            print("TCP connection established.")

        except Exception as e:
            print("Error establishing TCP connection:", str(e))

    # Sending Data to actuator
    def send_data(self, data):
        try:
            if self.socket:
                self.socket.sendall(data.encode())
                #print("Data sent:", data)
        except Exception as e:
            print("Error sending data:", str(e))

    # Feedback from actuator
    def receive_data(self):
        try:
            if self.socket:
                data = self.socket.recv(1024).decode()
                #print("Received data:", data)
                return data
        except Exception as e:
            print("Error receiving data:", str(e))

    # Closing TCP connection
    def close(self):
        if self.socket:
            self.socket.close()
            print("TCP connection closed.")

# obj = controller(host= '169.254.178.227' , port= 5001)
# obj.connect()
# while(True):
#     obj.send_data("A,N,0,0,0,0,0,0,0,0,0\r\n")
#     recieved_data =  obj.receive_data()
#     print(recieved_data)
