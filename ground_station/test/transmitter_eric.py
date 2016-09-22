
import struct
import time
import socket

UDP_IP = "10.10.16.197"   #155.31.42.140
UDP_PORT = 5005
MESSAGE = "Hello, World!"

print("UDP target IP:", UDP_IP)
print("UDP target port:", UDP_PORT)
print("message:", MESSAGE)


class TransmitterEric():



    def main(self):

        # Send UDP packets


        transmittersock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        transmittersock.sendto(MESSAGE, (UDP_IP, UDP_PORT))


