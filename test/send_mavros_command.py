#!/usr/bin/env python


import rospy
import mavros_msgs


command = rospy.ServiceProxy('send_command', mavros_msgs.CommandBool.srv)
print command(mavros_msgs.command(True))

