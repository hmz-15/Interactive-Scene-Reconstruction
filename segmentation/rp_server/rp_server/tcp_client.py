import socket
import struct
import time


class TcpClient(object):
    
    def __init__(self, ip, port):
        self.ip_ = ip
        self.port_ = port
        self.sock_ = None


    def send(self, data_bin):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect( (self.ip_, self.port_) )

        sock.sendall(data_bin)
        data_bin = self.recv_response_(sock)

        sock.close()

        return data_bin

    
    def recv_response_(self, sock):
        pack_size = sock.recv(4)
        pack_size = struct.unpack(">I", pack_size)[0]
        # fetch data package
        data_bin = self.recv_all_(sock, pack_size)

        return data_bin

    
    def recv_all_(self, sock, msg_length):
        data = b""
        size_left = msg_length

        while len(data) < msg_length:
            recv_data = sock.recv(size_left)
            size_left -= len(recv_data)
            data += recv_data

        return data


if __name__ == "__main__":
    client = TcpClient(ip="0.0.0.0", port=8800)
    client.send(b"hello world :)")