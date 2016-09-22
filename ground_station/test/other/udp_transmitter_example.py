import socket
import time
import struct

while True:
    UDP_IP = "10.10.16.255" #RAER IP address

    UDP_PORT = 4567

    MESSAGE = struct.pack('d', 3.3)


    print "UDP target IP:", UDP_IP

    print "UDP target port:", UDP_PORT

    print "message:", MESSAGE


    sock = socket.socket(socket.AF_INET,     # Internet

                        socket.SOCK_DGRAM)   # UDP

    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 20)
    sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))

    time.sleep(1/100)