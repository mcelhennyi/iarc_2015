
import struct
import time
import socket

UDP_IP = "155.31.42.140"   #155.31.42.140
UDP_PORT = 5005
MESSAGE = "Hello, World!"

print("UDP target IP:", UDP_IP)
print("UDP target port:", UDP_PORT)
print("message:", MESSAGE)


class TransmitterEric():



    def main(self):

        # Send UDP packets


        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))


