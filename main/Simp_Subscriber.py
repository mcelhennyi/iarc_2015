#!/usr/bin/env python
import rospy
from geometry_msgs.msg import *
from std_msgs.msg import *
from mavros_msgs.srv import *

#Author: JJ Marcus
#Last Edited Date: 29/10/2016


class Listener():

    def __init__(self):
        #sets the rate of the loop
        #self.rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.main()


    def main(self):
        rospy.Subscriber("/mavros/setpoint_velocity/cmd_vel",String ,self.print_info_callback)
        #rate.sleep()


    def print_info_callback(self, data):
         rospy.loginfo(data)

try:
    rospy.loginfo("try")
    Listener()
except rospy.ROSInterruptException:
    pass