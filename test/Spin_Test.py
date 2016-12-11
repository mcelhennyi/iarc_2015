#!/usr/bin/env python
import rospy
from std_msgs.msg import String


class SpinTest():
    def __init__(self):
        # create and subscribe to the message /roomba/location
        self.subscriber = rospy.Subscriber("/Spin_Test/to", String, self.callback)

        # set the rate of ros loop
        self.rate = rospy.Rate(10)  # 10hz

        # runs the loop function
        self.test_loop()

    def test_loop(self):
        # while the node is still running loop
        # while not rospy.is_shutdown(): ## WHILE LOOP NOT NEEDED FOR SPIN()
        # sleep the ros rate
        rospy.loginfo("I will spin")  # Will only be ran once
        rospy.spin()    # Is a blocking call. Once this call is made, every new subscription will start their
                        # call back method.

        print "I spun..."  # Runs after rosnode kill is run on this node

    # runs every time the subcriber above runs
    def callback(self, stringIn):
        rospy.loginfo("I heard: " + stringIn.data)


if __name__ == '__main__':
    # Initiate the node
    rospy.init_node('Spin_test', anonymous=False)
    try:
        st = SpinTest()
    except rospy.ROSInterruptException:
        pass

# TO find the length on the ground in meters
    # (height in meters times the distance in pixels)/720
