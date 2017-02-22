#!/usr/bin/env python
import rospy
from geometry_msgs.msg import *
from std_msgs.msg import *
from mavros_msgs.srv import *

#Author: JJ Marcus
#Last Edited Date: 29/10/2016


class Listener():

    def __init__(self):
        rospy.loginfo("init")
        #sets the rate of the loop
        self.rate = rospy.Rate(1)
        self.roomba_list = []

        while not rospy.is_shutdown():
            self.main()


    def main(self):

        rospy.Subscriber("/roomba/location_meters", PoseArray, self.roomba_location_callback)
        print("hello")
        self.rate.sleep()


    def roomba_location_callback(self, roomba_location_pose_arrary):
        print("hi")

        previous_roomba_list = self.roomba_list

        self.roomba_list =[]

        #the point message is dist in meteres
        #the orientation message is dist in pixles
        for pose in roomba_location_pose_arrary.poses:
            distance = (pose.position.x**2 + pose.position.y**2)**0.5
            roomba = [pose.position.x, pose.position.y, distance]
            self.roomba_list.append(roomba)
        print(self.roomba_list)


if __name__ == '__main__':
    #initate the node
    rospy.init_node('SimpleSubscriber')
    try:
        rospy.loginfo("try")
        Listener()
    except rospy.ROSInterruptException:
        pass
