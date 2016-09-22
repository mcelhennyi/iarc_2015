
import struct
import time
import socket

UDP_IP = "155.31.42.140"   #155.31.42.140
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

sock.bind((UDP_IP, UDP_PORT))

while True:
    data, addr = sock.recvfrom(1024) #Buffer size 1024

    print("received message:", data)
    