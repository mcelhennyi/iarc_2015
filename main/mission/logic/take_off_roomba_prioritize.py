import rospy
from geometry_msgs.msg import Vector3Stamped


# 1.Need to know when the quad is siting still over its current target(physical roomba or location we tell it),
    # will do this in here
# 2.Need to figure out how to change the PID to try to make the numbers go to something other than 0 so that we
    # can change the location when it is not looking at a target.
# 3.During take off and landing the X and Y needs to do nothing.
# 4.Subscribe to something to tell this if there is an obstacle in the way
# 5.Figure out when we need to land

# Need to take the roomba location
# Need to take the altitude
# Need to publish the error for the PID stuff


class TakeOffRoombaPrioritize:

    def __init__(self):
        # Create a publisher for acceleration data
        self.pub = rospy.Publisher('/master/control/error', Vector3Stamped, queue_size=10)
        # Create variable for use
        self.error_vector = Vector3Stamped()

        self.setpoint_x = 0
        self.setpoint_y = 0
        self.setpoint_z = 0

        self.measured_value_x = 0
        self.measured_value_y = 0
        self.measured_value_z = 0

        self.rate = rospy.Rate(10)  # 10hz

        self.state = 1
        self.at_target_location = 0  # 1. need to subscribe to this.

        self.obstacle_in_the_way = 0  # 4. Subscribe to this?
        self.ready_to_land = 0
        self.need_to_land = 0  # 5. calculate here?
        self.main()  # Runs the take off def

        # self.subscriber = rospy.Subscriber("/roomba/location", Vector3Stamped, self.callback)

    def main(self):
        while not rospy.is_shutdown():
            if self.state == 1:
                self.take_off()
            if self.state == 2:
                self.loop_roomba_prioritize()
            if self.state == 3:
                self.land()

            self.calculate_error()
            self.rate.sleep()

    def take_off(self):
        self.ready_to_land = 0  # Reset this value back to 0 if we had already landed and want to take off again

        # 2.This makes PID Z localize on 2 instead of 0
        self.setpoint_z = 2
        # while not rospy.is_shutdown():
        # self.calculate_error()  # Calculates error and publishes it

        # 3.Need to make sure the quad is not looking for a roomba until it is at the correct altitude

        # 1.if altitude is good
        if self.at_target_location == 1:
            self.state = 2

    def loop_roomba_prioritize(self):
        # while the node is still running loop
        # while not rospy.is_shutdown():
            # Needs to choose what roomba to localize on when seeing more than one

            # sleep the ros rate
            # self.rate.sleep()
            # Need something like this to tell it to land
        if self.need_to_land == 1:
            self.land()  # Will this work?? Or will it still be in this while loop forever??
            # Need to break loop??

    def land(self):
        self.need_to_land = 0  # Resets this variable for the next time
        # Moves the quad before landing so that it does not land on the roomba
        # 2.PID localizes on x=2,y=2 of the roomba location instead of 0,0
        self.setpoint_x = 2
        self.setpoint_y = 2
        # self.calculate_error()  # Calculates error and publishes it

        # 1.When the the quad is there then "ready_to_land" needs to go high
        while self.ready_to_land == 0:  # Probably can do this better?? cant think at the moment
            if self.at_target_location == 1:
                # 3.Need to make sure that it does not change X and Y location
                self.ready_to_land = 1

        if self.ready_to_land == 1:
            # 2.This needs to make the PID Z localize on 0 again instead of 2
            self.setpoint_z = 0
            self.calculate_error()  # Calculates error and publishes it

    def calculate_error(self):
        self.error_vector.x = self.setpoint_x - self.measured_value_x
        self.error_vector.y = self.setpoint_y - self.measured_value_y
        self.error_vector.z = self.setpoint_z - self.measured_value_z
        self.pub.publish(self.error_vector)  # Vector3Stamped type variable

if __name__ == '__main__':
    # Initiate the node
    rospy.init_node('master', anonymous=True)
    try:
        master = TakeOffRoombaPrioritize()
    except rospy.ROSInterruptException:
        pass


# for competition
    # def dodge_obstacle(self):
    #     self.obstacle_in_the_way = 0  # Resets this variable for the next time
    #     # 3.Needs to stop the X and Y movement
    #     # 2.Adjust the altitude to miss the object
    #     self.setpoint_z = 3  # Need to some how calculate the number to set as the new altitude
    #     self.calculate_error()
    #
    #     # 1.When the altitude is adjusted to miss the obstacle need to go back to looking for roombas
    #     # 1.if altitude is good
    #     if self.at_target_location == 1:
    #         self.loop_roomba_prioritize()