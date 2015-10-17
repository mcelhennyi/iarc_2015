#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Vector3Stamped
import random
import numpy as np
import matplotlib.pyplot as pp

class Emulator():

    def __init__(self):
        self.distance_x = random.uniform(1, 5)
        self.distance_y = random.uniform(1, 5)
        self.distance_z = 0
        self.d_time = .1
        self.count = 0
        self.arrayx = np.arange(0, 600)

        # Create a publisher for xyz location data
        self.pub = rospy.Publisher('/roomba/location', Vector3Stamped, queue_size=10)
        # set the rate of ros loop
        self.rate = rospy.Rate(10)  # 10hz
        # Create variable for use
        self.accel_vector = Vector3Stamped()
        self.xyz_roomba = Vector3Stamped()

        self.xyz_roomba.vector.x = self.distance_x
        self.xyz_roomba.vector.y = self.distance_y
        self.xyz_roomba.vector.z = self.distance_z
        self.pub.publish(self.xyz_roomba)  # Vector3Stamped type variable
        # runs the loop function
        self.loop_roomba_accel()  # Runs roomba subscriber

    def loop_roomba_accel(self):
        # while the node is still running loop
        # while not rospy.is_shutdown():
        # create and subscribe to the message /mavros/setpoint/accel
        self.subscriber = rospy.Subscriber("/mavros/setpoint/accel", Vector3Stamped, self.callback)
        # publish the xyz distance from roomba to mavros
        # sleep the ros rate
        # self.rate.sleep()
        rospy.spin()

    # runs every time the subcriber above runs
    def callback(self, accel_vector):

        #
        traveled_x = .5*accel_vector.vector.x*self.d_time*self.d_time
        self.distance_x -= traveled_x
        self.xyz_roomba.vector.x = self.distance_x
        #
        traveled_y = .5*accel_vector.vector.y*self.d_time*self.d_time
        self.distance_y -= traveled_y
        self.xyz_roomba.vector.y = self.distance_y
        #
        traveled_z = .5*accel_vector.vector.z*self.d_time*self.d_time
        self.distance_z -= traveled_z
        self.xyz_roomba.vector.z = self.distance_z
        self.pub.publish(self.xyz_roomba)  # Vector3Stamped type variable

        # Graph
        if self.count < 600:
            self.arrayx[self.count] = self.distance_x

        if self.count == 600:
            # plot
            pp.plot(self.arrayx)
            pp.title('Distance')
            pp.ylim([-20, 20])
            pp.show()
        self.count += 1
        rospy.loginfo(self.xyz_roomba)


if __name__ == '__main__':
     # Initiate the node
    rospy.init_node('Emulator', anonymous=True)
    try:
        em = Emulator()
    except rospy.ROSInterruptException:
        pass
