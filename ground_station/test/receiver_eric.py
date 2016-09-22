
import struct
import time
import socket

UDP_IP = "155.31.42.140"   #155.31.42.140
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

sock.bind((UDP_IP))