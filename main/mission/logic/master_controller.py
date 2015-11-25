import rospy
from geometry_msgs.msg import Vector3Stamped

# Taken care of now
# 1.Need to know when the quad is siting still over its current target(physical roomba or location we tell it),
    # will do this in here
# 2.Need to figure out how to change the PID to try to make the numbers go to something other than 0 so that we
    # can change the location when it is not looking at a target.
# 3.During take off and landing the X and Y needs to do nothing.
# Need to publish the error for the PID stuff

# Still needs to be fixed
# don't worry about this one now. 4.Subscribe to something to tell this if there is an obstacle in the way
# 5.Figure out when we need to land
# Need to subscribe to the roomba location
# Need to subscribe to the altitude



class TakeOffRoombaPrioritize:

    def __init__(self):
        # Create a publisher for acceleration data
        self.pub = rospy.Publisher('/master/control/error', Vector3Stamped, queue_size=10)
        # Create variable for use
        self.error_vector = Vector3Stamped()

        #  The offset of the quad from were it wants to be
        self.setpoint_x = 0  # Creating these values here in the master controller
        self.setpoint_y = 0
        self.setpoint_z = 0

        self.measured_value_x = 0  # Subscribe to these, x and y are the roomba location
        self.measured_value_y = 0
        self.measured_value_z = 0  # Subscribe to this, the altitude

        self.rate = rospy.Rate(10)  # 10hz

        self.state = 1  # starts in the take off state
        self.at_target_location = 0  # 1. need to produce this
        self.look_for_roomba = 0

        # self.obstacle_in_the_way = 0  # 4. Subscribe to this?
        self.need_to_land = 0  # 5. calculate here?

        self.main()  # Runs the main def

######################################################################################################################
    def main(self):
        while not rospy.is_shutdown():
            if self.state == 1:
                self.take_off()
            if self.state == 2:
                self.loop_roomba_prioritize()
            if self.state == 3:
                self.land()

            # subscribe to altitude
            # rospy.Subscriber("/roomba/location", Vector3Stamped, self.get_altitude)

            rospy.Subscriber("/roomba/location", Vector3Stamped, self.xy_location_control)
            # self.xy_location_control()  # Takes care of 3.
            self.calculate_error()  # Calculates error and can take care of 2.
            self.target_location()  # Takes care of 1.

            # self.need_to_dodge_things()  # 4

            self.rate.sleep()  # sleep the ros rate

######################################################################################################################
    def take_off(self):
        # 2.This makes PID Z localize on 2 instead of 0
        self.setpoint_z = 2

        # 3.Need to make sure the quad is not looking for a roomba until it is at the correct altitude
        # self.dont_look_for_roomba()
        self.look_for_roomba = 0  # Don't look for roomba

        # 1.if altitude is good
        if self.at_target_location == 1:
            self.state = 2  # moves to the flying state

    def loop_roomba_prioritize(self):
        self.look_for_roomba = 1  # Look for roomba
        # Needs to choose what roomba to localize on when seeing more than one

        # Need something like this to tell it to land
        if self.need_to_land == 1:
            self.state = 3  # moves to the landing state

    def land(self):
        self.need_to_land = 0  # Resets this variable for the next time
        # Moves the quad before landing so that it does not land on the roomba
        # 2.PID localizes on x=2,y=2 of the roomba location instead of 0,0
        self.setpoint_x = 2
        self.setpoint_y = 2

        # 1.When the the quad is there then the quad is ready to land
        if self.at_target_location == 1:
            # 2.This needs to make the PID Z localize on 0 again instead of 2
            self.setpoint_z = 0

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

#######################################################################################################################
    # Subscribe to the distance from an object
    # def find_object(self, ????)
        # stuff

    # Subscribes to the altitude
    def get_altitude(self, z_altitude):
        self.measured_value_z = z_altitude.vector.z  # probably need to change this, but basic idea

    # Subscribes to roomba location and chooses if we are looking for a roomba or not
    def xy_location_control(self, xy_roomba):
        if self.look_for_roomba == 1:
            self.measured_value_x = xy_roomba.vector.x
            self.measured_value_y = xy_roomba.vector.y
        else:
            if self.look_for_roomba == 0:
                self.measured_value_x = 0
                self.measured_value_y = 0

######################################################################################################################
    # Calculates the distance that the quad needs to move
    def calculate_error(self):
        self.error_vector.x = self.setpoint_x - self.measured_value_x
        self.error_vector.y = self.setpoint_y - self.measured_value_y
        self.error_vector.z = self.setpoint_z - self.measured_value_z
        self.pub.publish(self.error_vector)  # Vector3Stamped type variable

    # Checks to see if the quad is where we want it
    def target_location(self):
        if -.25 < self.error_vector.x < .25:
            if -.25 < self.error_vector.y < .25:
                if -.25 < self.error_vector.z < .25:
                    self.at_target_location = 1
                else:
                    self.at_target_location = 0
            else:
                self.at_target_location = 0
        else:
            self.at_target_location = 0

    # def need_to_dodge_things(self):
        # calculate how and when to dodge things

######################################################################################################################
if __name__ == '__main__':
    # Initiate the node
    rospy.init_node('master', anonymous=True)
    try:
        master = TakeOffRoombaPrioritize()
    except rospy.ROSInterruptException:
        pass


