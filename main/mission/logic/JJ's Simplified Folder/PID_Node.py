#!/usr/bin/env python
import rospy
from PID import PID


# Author: JJ Marcus
# Last Edit Date: 28/11/2016

class Master():
    def __init__(self):

        # Create PID's for each linear dimension of the quadcopter
        self.X_PID = PID( 0.3, 0, 0, 1/4)
        self.Y_PID = PID( 0.3, 0, 0, 1/4)
        self.Z_PID = PID( 0.3, 0, 0, 1/3)

        rospy.Subscriber("/master/controller/pid_control_vector", Float64MultiArrary, self.pid_subscriber_callback)

        self.velocity_publisher = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)

        self.offset_vector = [ 0 , 0 , 0 ]
        self.velocity_vector = TwistStamped

        self.main()


    def main(self):

        while not rospy.is_shutdown():



            ##########################################################################################
            ################# velocity publisher #####################################################
            ##########################################################################################
            #  set linear to value
            self.velocity_vector.twist.linear.x = x_linear
            self.velocity_vector.twist.linear.y = y_linear
            self.velocity_vector.twist.linear.z = z_linear

            # Set angular to zero
            self.velocity_vector.twist.angular.x = x_ang
            self.velocity_vector.twist.angular.y = y_ang
            self.velocity_vector.twist.angular.z = z_ang

            # Get the time now for the velocity commands
            time = rospy.Time.now()
            self.velocity_vector.header.stamp.secs = int(time.to_sec())
            self.velocity_vector.header.stamp.nsecs = int((time.to_sec() - int(time.to_sec())) * 1000000000)
            self.velocity_vector.header.seq = count

            # use the publisher
            self.velocity_publisher.publish(self.velocity_vector)

            rate.sleep()

##########################################################################################
################# Subscriber Call Backs ##################################################
######################################################################

    def pid_subscriber_callback(self, subscriber):
        self.pid_subscriber_data = subscriber.data




##########################################################################################
################# Subscriber Call Backs ##################################################
##########################################################################################

if __name__ == '__main__':
    # initate the node
    rospy.init_node('Master')
    try:
        test = Master()
    except rospy.ROSInterruptException:
        pass
