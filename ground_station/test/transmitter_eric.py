import socket
import struct
import time


class TransmitterEric():

    def __init__(self):
        self.x=1
        self.main()

    def main(self):
        x=5
        print(x)
        print(self.x)
        time.sleep(5) #5 second delay
        data = struct.pack('dd',5.7, 6.5)
        print(data)

        number1, number2 = struct.unpack('dd', data) #look into using tuple for unpacking variables
        print(number1, number2)



#wireshark tool
#broadcast udp to send


if __name__ == '__main__':

    pid = TransmitterEric()
