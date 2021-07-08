import socket
import threading
import struct
import time


class TcpServer(object):

    def __init__(self, host, port):        
        self.host_ = host
        self.port_ = port
        
        self.sock_ = None

        self.quit_event_ = threading.Event()


    def launch(self):
        print("Server launched at {}:{}".format(self.host_, self.port_))
        
        self.sock_ = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock_.bind( (self.host_, self.port_) )
        self.sock_.listen(1)

        self.quit_event_.clear()
        
        self.start_server_()

    
    def stop(self):
        self.quit_event_.set()

        if self.sock_ is not None:
            socket.socket(socket.AF_INET, socket.SOCK_STREAM).connect( (self.host_, self.port_))
            self.sock_.close()
            self.sock_ = None

    
    def start_server_(self):
        # listen to connection request
        while not self.quit_event_.is_set():
            # blocked for next connection
            conn, addr = self.sock_.accept()
            thread = threading.Thread(target=self.handle_connection_, args=(conn, addr))
            thread.start()


    # This function need to be override by its child class
    def handle_connection_(self, conn, addr):
        pass


    def recv_all_(self, sock, msg_length):
        data = b""
        size_left = msg_length

        while len(data) < msg_length and not self.quit_event_.is_set():
            recv_data = sock.recv(size_left)
            size_left -= len(recv_data)
            data += recv_data

        return data