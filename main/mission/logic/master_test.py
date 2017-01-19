#!/usr/bin/env python
import rospy
#from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float64


class MasterTest():

    def __init__(self):
        # Create a publisher for acceleration data
        self.alt_pub = rospy.Publisher('/mavros/global_position/rel_alt', Float64, queue_size=10)
        self.loc_pub = rospy.Publisher('/roomba/location_meters', Vector3Stamped, queue_size=10)

        # set the rate of ros loop
        self.rate = rospy.Rate(10)  # 10hz


        self.xyz_roomba = Vector3Stamped()


        # runs the loop function
        self.loop()  # Runs roomba subscriber

    def loop(self):
        # while the node is still running loop
        while not rospy.is_shutdown():

            input_cmd = input('Enter a command (1 to change altitude, 2 to Change location): ')
            # Ask what data needs to be changed
            # Change the altitude
            if input_cmd is 1:
                altitude = input('Enter the altitude: ')
                self.alt_pub.publish(altitude)

            # Change the location of the roomba
            if input_cmd is 2:
                x = input('Enter x roomba distance in meters: ')
                y = input('Enter y roomba distance in meters: ')
                self.xyz_roomba.vector.x = x
                self.xyz_roomba.vector.y = y
                self.xyz_roomba.vector.z = 0
                self.loc_pub.publish(self.xyz_roomba)

            # sleep the ros rate
            self.rate.sleep()

if __name__ == '__main__':
    # Initiate the node
    rospy.init_node('master_test', anonymous=True)
    try:
        test = MasterTest()
    except rospy.ROSInterruptException:
        pass
