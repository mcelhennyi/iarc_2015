#!/usr/bin/env python
import rospy
from std_msgs.msg import String


class SleepSpinTest():
    def __init__(self):
        # create and subscribe to the message /roomba/location
        self.pub1 = rospy.Publisher("/Sleep_Test/to", String, queue_size=10)
        self.pub2 = rospy.Publisher("/Spin_Test/to", String, queue_size=10)

        self.pubStr1 = String()
        self.pubStr2 = String()

        # set the rate of ros loop
        self.rate = rospy.Rate(10)  # 10hz

        # runs the loop function
        self.test_loop() 

    def test_loop(self):
        # while the node is still running loop
        while not rospy.is_shutdown():
            self.pubStr1.data = "Sleep Test Message!"
            self.pubStr2.data = "Spin Test Message!"

            # sleep the ros rate
            self.pub1.publish(self.pubStr1)
            self.pub2.publish(self.pubStr2)
            self.rate.sleep()


if __name__ == '__main__':
    # Initiate the node
    rospy.init_node('SleepSpin_test', anonymous=False)
    try:
        st = SleepSpinTest()
    except rospy.ROSInterruptException:
        pass

# TO find the length on the ground in meters
    # (height in meters times the distance in pixels)/720
