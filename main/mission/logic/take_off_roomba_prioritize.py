import rospy
from geometry_msgs.msg import Vector3Stamped


class TakeOffRoombaPrioritize:

    def __init__(self):
        # Create a publisher for z location data for take off and landing
        self.pub = rospy.Publisher('/roomba/location', Vector3Stamped, queue_size=10)
        # set the rate of ros loop
        self.rate = rospy.Rate(10)  # 10hz
        # Create variable for use
        self.altitude = Vector3Stamped()  # Need to change just the z vector for take off
        self.new_xy_to_land = Vector3Stamped()
        self.over_roomba = 0  # need to subscribe?
        self.need_to_land = 0
        self.take_off()  # Runs the take off def

    def take_off(self):
        # Only changing the z vector to be able to take off
        self.altitude.vector.z = 3
        self.pub.publish(self.altitude)  # Vector3Stamped type variable

        # need to check and see if we are at the correct altitude before moving on to the next step
    #     # create and subscribe to the message /roomba/location
    #     rospy.Subscriber("/roomba/location", Vector3Stamped, self.callback_take_off)
    #     if self.over_roomba == 0:
    #         self.loop_roomba_prioritize()
    #
    # def callback_take_off(self, xy_roomba):

    def loop_roomba_prioritize(self):
        # while the node is still running loop
        while not rospy.is_shutdown():
            # Needs to choose what roomba to localize on when seeing more than one

            # sleep the ros rate
            self.rate.sleep()
            # Need something like this to tell it to land
            if self.need_to_land == 1:
                self.land()

    def land(self):
        # create and subscribe to the message /roomba/location
        rospy.Subscriber("/roomba/location", Vector3Stamped, self.callback_land)
        if self.over_roomba == 0:
            # Only changing the z vector for landing
            self.altitude.vector.z = 0
            self.pub.publish(self.altitude)  # Vector3Stamped type variable

    def callback_land(self, xy_roomba):
        # Moves the quad before landing so that it does not land on the roomba
        self.new_xy_to_land.vector.x = xy_roomba.vector.x + 2
        self.new_xy_to_land.vector.y = xy_roomba.vector.y + 2
        # publish the location to land vector to mavros
        self.pub.publish(self.new_xy_to_land)  # Vector3Stamped type variable

if __name__ == '__main__':
    # Initiate the node
    rospy.init_node('PID_main', anonymous=True)
    try:
        pid = TakeOffRoombaPrioritize()
    except rospy.ROSInterruptException:
        pass
