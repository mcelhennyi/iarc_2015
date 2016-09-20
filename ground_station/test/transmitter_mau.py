import socket
import struct
import time


UDP_IP = ""

UDP_PORT = 5005

MESSAGE = "Hello, World!"


print "UDP target IP:", UDP_IP

print "UDP target port:", UDP_PORT

print "message:", MESSAGE


sock = socket.socket(socket.AF_INET,  # Internet

socket.SOCK_DGRAM)  # UDP

sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))
















'''
class TransmitterMau():

    def __init__(self):
        self.x = 1
        self.main()

    def main(self):
        x = 5
        print(x)
        print(self.x)
        time.sleep(1)
        data = struct.pack('dd', 5.7, 6.5)          #parses data
        print(data)

        number1, number2 = struct.unpack('dd', data)        #look into how to use tuples
        print(number1, number2)




if __name__ == '__main__':

     pid = TransmitterMau()
*/
#broadcasted UDP
'''