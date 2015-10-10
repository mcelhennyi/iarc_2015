#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Vector3Stamped
import pid_controller


class PIDMain():

    def __init__(self):
        # Create a publisher for acceleration data
        self.pub = rospy.Publisher('/mavros/setpoint/accel', Vector3Stamped, queue_size=10)
        # set the rate of ros loop
        self.rate = rospy.Rate(10)  # 10hz
        # Create variable for use
        self.accel_vector = Vector3Stamped()
        self.xyz_roomba = Vector3Stamped()
        # Create object of PID Controller with tunning arguments
        self.pid_methods = pid_controller.PIDController(KP=10, KI=0, KD=0)  # PID contants P, I, D
        # runs the loop function
        self.loop_roomba_location()  # Runs roomba subscriber

    def loop_roomba_location(self):
        # while the node is still running loop
        while not rospy.is_shutdown():
            # create and subscribe to the message /roomba/location
            self.subscriber = rospy.Subscriber("/roomba/location", Vector3Stamped, self.callback)
            # publish the accel vector to mavros
            self.pub.publish(self.accel_vector)  # Vector3Stamped type variable

            # sleep the ros rate
            self.rate.sleep()
        # rospy.spin()

    # runs every time the subcriber above runs
    def callback(self, xyz_roomba):
        # Method calls that send the XYZ location of the roomba and return xyz acceleration
        self.accel_vector.vector.x = \
            self.pid_methods.get_output(xyz_roomba.vector.x, xyz_roomba.vector.y, xyz_roomba.vector.z)[0]
        self.accel_vector.vector.y = \
            self.pid_methods.get_output(xyz_roomba.vector.x, xyz_roomba.vector.y, xyz_roomba.vector.z)[1]
        self.accel_vector.vector.z = \
            self.pid_methods.get_output(xyz_roomba.vector.x, xyz_roomba.vector.y, xyz_roomba.vector.z)[2]
        # logs the xyz accel data
        rospy.loginfo(self.accel_vector)


if __name__ == '__main__':
    # Initiate the node
    rospy.init_node('PID_main', anonymous=True)
    try:
        pid = PIDMain()
    except rospy.ROSInterruptException:
        pass
