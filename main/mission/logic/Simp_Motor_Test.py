import rospy
frmo geometry_msgs.msg import TwistStamped
#Author: JJ Marcus
#Last Edit Date: 10/26/2016


class SimpMotorTest():
    def __init__(self):
        #Creates the publisher for ROS. (name of message, data type(from geometry_msgs.msg), que size)
        self.velocity_publisher = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', TwistStamped, que_size=10)

        #sets the rate of the loop
        self.rate = rospy.Rate(10)

        # Create variable to publish
        self.Velocity_Vector = TwistStamped() # ros data type
        self.motor_test_loop()
 
    def motor_test_loop(self)
        seq_counter = 0
        while not rospy.is_shutdown():
        
        self.publish_velocity(0,0,0,0,0,0,seq_counter)

        self.rate.sleep()



    # pubhlshes the velocity
    def publish_velocity(self, x_linear, y_linear, z_linear, x_ang, y_ang, z_ang, seq_counter):

        # set linear to value
        self.vel_twist.twist.linear.x = x_linear
        self.vel_twist.twist.linear.y = y_linear
        self.vel_twist.twist.linear.z = z_linear

        # Set angular to zero
        self.vel_twist.twist.angular.x = x_ang
        self.vel_twist.twist.angular.y = y_ang
        self.vel_twist.twist.angular.z = z_ang
        # Get the time now for the velocity commands
        time = rospy.Time.now()
        self.vel_twist.header.stamp.secs = int(time.to_sec())
        self.vel_twist.header.stamp.nsecs = int((time.to_sec() - int(time.to_sec())) * 1000000000)
        self.vel_twist.header.seq = seq_counter

        # logs the xyz accel data
        rospy.loginfo(self.vel_twist)

        # use the publisher
        self.velocity_publisher.publish(self.Velocity_Vector)


if __name__ == '__main__':
    # initate the node
    rospy.init_node('SimpleMotorTest')
    try:
        test = SimpMotorTest():
    except rospy.ROSInterruptException:
        pass

