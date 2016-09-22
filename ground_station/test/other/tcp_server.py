#!/usr/bin/env python

import socket


TCP_IP = '127.0.0.1'          # Establishes ip address
TCP_PORT = 5005               # Establishes port
BUFFER_SIZE = 1024            # Establishes byte size?
MESSAGE = "Hello, World!"     # Declares a hello world string and stores as "MESSAGE"

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)       # sets up the socket method
s.connect((TCP_IP, TCP_PORT))                               # connects to the given ip and port
s.send(MESSAGE)                                             # sends the hello world message
data = s.recv(BUFFER_SIZE)                                  # interprets the data received
s.close()                                                   # closes the port?

print "received data:", data                                # prints the data