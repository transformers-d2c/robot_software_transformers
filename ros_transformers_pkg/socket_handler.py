import socket
import sys
from central_node import CentralNode as cn

class my_socket:
    def __init__(self, host, port, sock=None):
        self.HOST = host
        self.PORT = port
        if sock is None:
            self.sock = socket.socket(
                socket.AF_INET, socket.SOCK_STREAM)
        else:
            self.sock = sock
    
    def connect(self):
        self.sock.connect((self.HOST, self.PORT))
    
    def send(self, msg):
        self.sock.sendall(msg.encode())

    def recv(self):
        data = self.sock.recv(1024)
        return data.decode()

    def close(self):
        self.sock.close()

def main():
    host = sys.argv[1]
    port = int(sys.argv[2])
    sock = my_socket(host, port)
    sock.connect()
    #TODO: complete function parameters
    data = cn.send_to_node()
    sock.send(data)
    print(sock.recv())
    sock.close()

if __name__ == '__main__':
    main()