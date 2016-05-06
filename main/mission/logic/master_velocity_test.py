#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float64
#from mavros.srv import *
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool
import time

class MasterVelocityTest:

    def __init__(self):
        # Create a publisher for acceleration data
        self.pub = rospy.Publisher("/master/control/error", Vector3Stamped, queue_size=10)

        # Create variable for use
        self.error_vector = Vector3Stamped()

        self.rate = rospy.Rate(10)  # 10hz

        self.main()  # Runs the main def

######################################################################################################################
    def main(self):
        while not rospy.is_shutdown():
            start = time.time()
            while start + 5 > time.time():
                self.create_error(2,2,2)
                rospy.loginfo("(2, 2, 2)")
                self.rate.sleep()  # sleep the ros rate

            start = time.time()
            while start + 5 > time.time():
                self.create_error(-2,-2,-2)
                rospy.loginfo("(-2, -2, -2)")
                self.rate.sleep()  # sleep the ros rate

    # Calculates the distance that the quad needs to move
    def create_error(self, x, y, z):
        self.error_vector.vector.x = x
        self.error_vector.vector.y = y
        self.error_vector.vector.z = z
        self.pub.publish(self.error_vector)  # Vector3Stamped type variable

if __name__ == '__main__':
    # Initiate the node
    rospy.init_node('master', anonymous=False)
    try:
        master = MasterVelocityTest()
    except rospy.ROSInterruptException:
        pass
