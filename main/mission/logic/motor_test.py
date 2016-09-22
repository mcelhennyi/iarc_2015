#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TwistStamped


class MotorTest():

    def __init__(self):
        # Create a publisher for acceleration data
        self.pub1 = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel',
        TwistStamped, queue_size=10)
        self.pub2 = rospy.Publisher('/mavros/setpoint_attitude/cmd_vel',
        TwistStamped, queue_size=10)

        # set the rate of ros loop
        self.rate = rospy.Rate(10)  # 10hz

        # Create variable for use
        self.vel_twist = TwistStamped()

        # runs the loop function
        self.loop_roomba_location()  # Runs roomba subscriber

    def loop_roomba_location(self):
        count = 0

        # while the node is still running loop
        while not rospy.is_shutdown():

            if count < 50:
                self.fill_twist(0, 0, 2, 0, 0, 0)
            else:
                self.fill_twist(0, 0, 0, 0, 0, 0)

            if count is 100:
                count = 0

            count += 1

            # Get the time now for the velocity commands
            time = rospy.Time.now()
            self.vel_twist.header.stamp.secs = int(time.to_sec())
            self.vel_twist.header.stamp.nsecs = int((time.to_sec() - int(time.to_sec())) * 1000000000)

            # publish the accel vector to mavros
            self.pub1.publish(self.vel_twist)  # Vector3Stamped type variable
            self.pub2.publish(self.vel_twist)  # Vector3Stamped type variable

            # sleep the ros rate
            self.rate.sleep()
            #rospy.spin()

    # runs every time the subcriber above runs
    def fill_twist(self, x_linear, y_linear, z_linear, x_ang, y_ang, z_ang):

        # set linear to value
        self.vel_twist.twist.linear.x = x_linear
        self.vel_twist.twist.linear.y = y_linear
        self.vel_twist.twist.linear.z = z_linear

        # Set angular to zero
        self.vel_twist.twist.angular.x = x_ang
        self.vel_twist.twist.angular.y = y_ang
        self.vel_twist.twist.angular.z = z_ang


        # logs the xyz accel data
        rospy.loginfo(self.vel_twist)


if __name__ == '__main__':
    # Initiate the node
    rospy.init_node('Motor_Test', anonymous=False)
    try:
        mt = MotorTest()
    except rospy.ROSInterruptException:
        pass

# TO find the length on the ground in meters
    # (height in meters times the distance in pixels)/720