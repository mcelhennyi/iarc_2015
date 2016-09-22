
import struct
import time
import socket

UDP_IP = "127.0.0.1"   #155.31.42.140
UDP_PORT = 5005

recSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

recSock.bind((UDP_IP, UDP_PORT))

while True:
    data, addr = recSock.recvfrom(1024)
    print "Message: ", data