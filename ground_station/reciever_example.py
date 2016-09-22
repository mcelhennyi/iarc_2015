import socket
import struct


UDP_IP = ""

UDP_PORT = 4567


sock = socket.socket(socket.AF_INET,  socket.SOCK_DGRAM)  # UDP

sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1 )

sock.bind((UDP_IP, UDP_PORT))
#mreq = struct.pack('4sL', socket.inet_aton('10.10.16.255'), socket.INADDR_ANY)

#sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)


while True:

    data = sock.recv(1024)  # buffer size is 1024 bytes

    print "received message:", data

sock.close()