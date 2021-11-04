import socket
import threading
import struct
import time
import queue


class TcpServer(object):

    def __init__(self, host, port):        
        self.host_ = host
        self.port_ = port
        
        self.q_ = queue.Queue()
        self.sock_ = None

        self.quit_event_ = threading.Event()


    def launch(self, server_name="Server"):
        print("{} launched at {}:{}".format(server_name, self.host_, self.port_))
        
        self.sock_ = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock_.bind( (self.host_, self.port_) )
        self.sock_.listen(1)

        with self.q_.mutex:
            self.q_.queue.clear()

        self.quit_event_.clear()
        
        pthread = threading.Thread(target=self.start_server_)
        pthread.start()

    
    def stop(self):
        self.quit_event_.set()
        self.q_.put(None)

        if self.sock_ is not None:
            socket.socket(socket.AF_INET, socket.SOCK_STREAM).connect( (self.host_, self.port_))
            self.sock_.close()
            self.sock_ = None

    
    def fetch_package(self):
        while not self.quit_event_.is_set():
            content = self.q_.get()
            if content is None:
                break
            
            yield content

    
    def start_server_(self):
        # listen to connection request
        while not self.quit_event_.is_set():
            # blocked for next connection
            conn, addr = self.sock_.accept()
            thread = threading.Thread(target=self.handle_connection_, args=(conn, addr))
            thread.start()


    def handle_connection_(self, conn, addr):
        conn_id = "{}:{}".format(addr[0], addr[1])
        # print('New connection from {}'.format(conn_id))

        while not self.quit_event_.is_set():
            pack_size = conn.recv(4)
            # end of Connection
            if not pack_size:
                break

            pack_size = struct.unpack('>I', pack_size)[0]
            # fetch data package
            data = self.recv_all_(conn, pack_size)

            self.q_.put(data)

        conn.close()
        # print("Connection {}: closed".format(conn_id))


    def recv_all_(self, sock, msg_length):
        data = b""
        size_left = msg_length

        while len(data) < msg_length and not self.quit_event_.is_set():
            recv_data = sock.recv(size_left)
            size_left -= len(recv_data)
            data += recv_data

        return data