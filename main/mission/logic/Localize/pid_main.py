#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Vector3Stamped
import pid_controller


class PIDMain():

    def __init__(self):
        # Create a publisher for acceleration data
        self.pub1 = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        self.pub2 = rospy.Publisher('/mavros/setpoint_attitude/cmd_vel', TwistStamped, queue_size=10)
        # set the rate of ros loop
        self.rate = rospy.Rate(10)  # 10hz
        # Create variable for use
        self.vel_twist = TwistStamped()
        self.xyz_roomba = Vector3Stamped()
        self.temp_vel_vector = [0.0, 0.0, 0.0]
        # Create object of PID Controller with tunning arguments
        self.pid_methods = pid_controller.PIDController(KP=1, KI=0, KD=0)  # PID contants P, I, D
        # runs the loop function
        self.loop_roomba_location()  # Runs roomba subscriber

    def loop_roomba_location(self):
        # while the node is still running loop
        while not rospy.is_shutdown():
            # create and subscribe to the message /roomba/location
            self.subscriber = rospy.Subscriber("/master/control/error", Vector3Stamped, self.callback)
            # publish the accel vector to mavros
            self.pub1.publish(self.vel_twist)  # Vector3Stamped type variable
            self.pub2.publish(self.vel_twist)  # Vector3Stamped type variable

            # sleep the ros rate
            self.rate.sleep()
            #rospy.spin()

    # runs every time the subcriber above runs
    def callback(self, xyz_roomba):
        # Method calls that send the XYZ location of the roomba and return xyz acceleration
        self.temp_vel_vector = self.pid_methods.get_output(xyz_roomba.vector.x, xyz_roomba.vector.y, xyz_roomba.vector.z)

        # set linear to value
        self.vel_twist.twist.linear.x = self.temp_vel_vector[0]
        self.vel_twist.twist.linear.y = self.temp_vel_vector[1]
        self.vel_twist.twist.linear.z = self.temp_vel_vector[2]

        # Set angular to zero
        self.vel_twist.twist.angular.x = 0
        self.vel_twist.twist.angular.y = 0
        self.vel_twist.twist.angular.z = 0

        # logs the xyz accel data
        rospy.loginfo(self.vel_twist)


if __name__ == '__main__':
    # Initiate the node
    rospy.init_node('PID_main', anonymous=False)
    try:
        pid = PIDMain()
    except rospy.ROSInterruptException:
        pass

# TO find the length on the ground in meters
    # (height in meters times the distance in pixels)/720
