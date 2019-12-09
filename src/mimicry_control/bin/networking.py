__author__ = 'drakita'

import socket

class Networking:
    def __init__(self):
        self.ip = '10.134.71.8'
        self.vibr_ip = '10.134.71.216'
        #self.urip = '10.130.229.132'
        self.UDP_PORT = 11000
        self.VIBR_PORT = 11100
        self.UR_PORT = 30002

    def create_udp_socket(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        return sock

    def bind_socket(self, ip, port):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((ip,port))
        sock.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,512)
        return sock

    def connect_socket(self, ip, port):
        sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        sock.connect((ip, port))
        return sock

    def get_socket_data(self, sock):
        data, addr = sock.recvfrom(256)
        return data

    def send_socket_data(self, sock, data, ip, port):
        sock.sendto(data, (ip, port))

