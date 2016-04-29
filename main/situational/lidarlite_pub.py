#!/usr/bin/env python
import rospy
from struct import *
from mavros_msgs.msg import Mavlink
from std_msgs.msg import Float64


class LidarlitePub():

    def __init__(self):
        self.pub = rospy.Publisher('down_lidar', Float64, queue_size=10)
        rospy.init_node('lidarlite_pub', anonymous=True)
        self.listener()
        #rate = rospy.Rate(10) # 10hz

    def mavCallback(self, data):
        if int(data.msgid) == 173:
            p = pack("i", data.payload64[0])
            distance = unpack("f", p)
            print("Distance: \n")
            rospy.loginfo("Distance: \n")
            print(distance[0])
            rospy.loginfo(distance[0])
            self.pub.publish(distance[0])

    def listener(self):
        rospy.Subscriber("/mavlink/from", Mavlink, self.mavCallback)
        rospy.spin()

if __name__ == '__main__':
    try:
        lid = LidarlitePub()
        rospy.loginfo("Test\n")

    except rospy.ROSInterruptException:
        pass
