#!/usr/bin/env python
import rospy
from geometry_msgs.msg import *
from std_msgs.msg import *
from mavros_msgs.srv import *
from mavros_msgs.msg import *

#Author: JJ Marcus
#Last Edited Date: 29/10/2016


class Listener():

    def __init__(self):
        rospy.loginfo("init")
        #sets the rate of the loop
        self.rate = rospy.Rate(1)

        while not rospy.is_shutdown():
            self.main()


    def main(self):
        rospy.loginfo("main")

        rospy.Subscriber("/mavros/altitude/",Altitude, self.print_info_callback)
        self.rate.sleep()


    def print_info_callback(self, data):
        rospy.loginfo("callback")
        IncomingData = [0,0,0,0,0]
        IncomingData[0] = data.monotonic
        IncomingData[1] = data.amsl
        IncomingData[2] = data.relative
        IncomingData[3] = data.terrain
        IncomingData[4] = data.bottom_clearance
        if IncomingData[2] > 0.35 and IncomingData[2] < 20:
            rospy.loginfo("relative")
            rospy.loginfo( IncomingData[2] )
        else:
            rospy.loginfo("barometer")
            rospy.loginfo( IncomingData[1] - IncomingData[3] )




if __name__ == '__main__':
    #initate the node
    rospy.init_node('SimpleSubscriber')
    try:
        rospy.loginfo("try")
        Listener()
    except rospy.ROSInterruptException:
        pass