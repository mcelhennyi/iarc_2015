#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float64
#from mavros_msgs.msgs import SetMode

# Taken care of now
# 1.Need to know when the quad is siting still over its current target(physical roomba or location we tell it),
    # will do this in here
# 2.Need to figure out how to change the PID to try to make the numbers go to something other than 0 so that we
    # can change the location when it is not looking at a target.
# 3.During take off and landing the X and Y needs to do nothing.
# Need to publish the error for the PID stuff
# 5.Figure out when we need to land

# Still needs to be fixed
# don't worry about this one now. 4.Subscribe to something to tell this if there is an obstacle in the way
# Need to subscribe to the roomba location
# Need to subscribe to the altitude



class TakeOffRoombaPrioritize:

    def __init__(self):
        # Create a publisher for acceleration data
        self.pub = rospy.Publisher("/master/control/error", Vector3Stamped, queue_size=10)

        ## Sets up service and initialize mode
        #rospy.wait_for_service('mavros/set_mode')
        #self.set_mode_serv = rospy.ServiceProxy('mavros/set_mode', SetMode)  # creates service for setting mode
        #self.off_board_mode = SetMode()  # Creates SetMode object for service
        #self.off_board_mode.custom_mode = "OFFBOARD"  # Sets mode to off board control
        #self.nav_guided_serv = rospy.ServiceProxy('/mavros/cmd/guided_enable', CommandBool)

        # Create variable for use
        self.error_vector = Vector3Stamped()

        #  The offset of the quad from where it wants to be
        self.setpoint_x = 0  # Creating these values here in the master controller
        self.setpoint_y = 0
        self.setpoint_z = 0

        self.measured_value_x = 0  # Subscribe to these, x and y are the roomba location
        self.measured_value_y = 0
        self.measured_value_z = 0  # Subscribe to this, the altitude

        # Set error for error measurement
        self.allowed_error = 0.1

        self.rate = rospy.Rate(10)  # 10hz

        self.state = 1  # starts in the take off state
        self.at_target_location = 0  # 1. need to produce this
        self.look_for_roomba = 0

        # flight mission variables
        self.mission_state = 0
        self.need_to_land = 0  # 5.

        # self.obstacle_in_the_way = 0  # 4. Subscribe to this?
        rospy.loginfo("init")

        self.main()  # Runs the main def

######################################################################################################################
    def main(self):
        rospy.loginfo("main")
        while not rospy.is_shutdown():
            if self.state == 1:
                self.take_off()
                rospy.loginfo("take off state")
            elif self.state == 2:
                self.flying_mission()
                rospy.loginfo("mission state")
            elif self.state == 3:
                self.land()
                rospy.loginfo("landing state")

            # Print out altitude in terminal
            rospy.loginfo(self.measured_value_z)

            # subscribe to altitude
            rospy.Subscriber("/mavros/global_position/rel_alt", Float64, self.get_altitude)

            # Subscribe to roomba location
            rospy.Subscriber("/roomba/location_meters", Vector3Stamped, self.xy_location_control)
            # self.xy_location_control()  # Takes care of 3.
            self.calculate_error()  # Calculates error and can take care of 2.
            self.target_location()  # Takes care of 1.

            # self.need_to_dodge_things()  # 4

            self.rate.sleep()  # sleep the ros rate

#####################################################################################################################
    def take_off(self):
        # 2.This makes PID Z localize on 2 instead of 0
        self.setpoint_z = 2

        # 3.Need to make sure the quad is not looking for a roomba until it is at the correct altitude
        # self.dont_look_for_roomba()
        self.look_for_roomba = 0  # Don't look for roomba

        # 1.if altitude is good
        if self.at_target_location == 1:
            self.state = 2  # moves to the flying state

    def flying_mission(self):
        self.look_for_roomba = 1  # Look for roomba

        # if we are hovering above the roomba
        if self.at_target_location == 1:

            if self.mission_state == 0:
                self.mission_state = 1  # Change mission state to next state
                self.setpoint_z = 1  # Move down

            elif self.mission_state == 1:
                self.mission_state = 2  # Change mission state to next state
                self.setpoint_z = 2  # Move up

            elif self.mission_state == 2:
                # self.mission_state = 3  # Change mission state to next state
                self.need_to_land = 1  # land

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
    #         go back to flying

#######################################################################################################################
    # Subscribe to the distance from an object
    # def find_object(self, ????)
        # stuff

    # Subscribes to the altitude
    def get_altitude(self,z_altittude):
        self.measured_value_z = z_altittude.data  # probably need to change this, but basic idea

    # Subscribes to roomba location and chooses if we are looking for a roomba or not
    # Need to later decide which roomba to focus on if there is more than one in sight
    def xy_location_control(self, xy_roomba):
        if self.look_for_roomba == 1:
            self.measured_value_x = xy_roomba.vector.x
            self.measured_value_y = xy_roomba.vector.y
        elif self.look_for_roomba == 0:
                self.measured_value_x = 0
                self.measured_value_y = 0

######################################################################################################################
    # Calculates the distance that the quad needs to move
    def calculate_error(self):
        self.error_vector.vector.x = self.setpoint_x - self.measured_value_x
        self.error_vector.vector.y = self.setpoint_y - self.measured_value_y
        self.error_vector.vector.z = self.setpoint_z - self.measured_value_z
        self.pub.publish(self.error_vector)  # Vector3Stamped type variable

    # Checks to see if the quad is where we want it
    def target_location(self):
        if  self.error_vector.vector.x > -self.allowed_error and self.error_vector.vector.x < self.allowed_error:
            if self.error_vector.vector.y > -self.allowed_error and self.error_vector.vector.y < self.allowed_error:
                if self.error_vector.vector.z > -self.allowed_error and self.error_vector.vector.z < self.allowed_error:
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
    rospy.loginfo("if main")
    try:
        master = TakeOffRoombaPrioritize()
    except rospy.ROSInterruptException:
        pass
