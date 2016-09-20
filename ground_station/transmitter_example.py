import socket
import time

while True:
    UDP_IP = "234.5.6.7"

    UDP_PORT = 4567

    MESSAGE = "Hello, World!"


    print "UDP target IP:", UDP_IP

    print "UDP target port:", UDP_PORT

    print "message:", MESSAGE


    sock = socket.socket(socket.AF_INET,     # Internet

                        socket.SOCK_DGRAM)   # UDP

    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 20)
    sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))

    time.sleep(3)