#!/usr/bin/env python
import rospy
from std_msgs.msg import String


class SleepTest():
    def __init__(self):
        # create and subscribe to the message /roomba/location
        self.subscriber = rospy.Subscriber("/Sleep_Test/to", String, self.callback)

        # set the rate of ros loop
        self.rate = rospy.Rate(10)  # 10hz

        # runs the loop function
        self.test_loop() 

    def test_loop(self):
        # while the node is still running loop
        while not rospy.is_shutdown():
            # sleep the ros rate
            rospy.loginfo("I Slept")
            self.rate.sleep()

            #############################
            ## When using SLeep the while loop will repeat using the ros rate, however call backs are at the
            # mercy of the publisher.
            #############################


    # runs every time the subcriber above runs
    def callback(self, stringIn):
        rospy.loginfo("I heard: " + stringIn.data)


if __name__ == '__main__':
    # Initiate the node
    rospy.init_node('Sleep_test', anonymous=False)
    try:
        st = SleepTest()
    except rospy.ROSInterruptException:
        pass

# TO find the length on the ground in meters
    # (height in meters times the distance in pixels)/720
