#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseArray
from mavros_msgs.msg import Altitude
from mavros_msgs.msg import TwistStamped
#from mavros.srv import *
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool

# NOTES:
    # About:
        # This is the master node for the competition
        # This will serve as the input for all of the filtered data
        # It will make decisions based on all of the data, and move the quad
        # This node uses a state machine type operation, with the states
        #   listed just below the class declaration
        # If a state can be broken into sub states, it should be broken down
        # OUTPUTS:
            # Error to the PID on the topic: "/master/control/error"
    # Assumptions:
        #

# State variable constants
INITIALIZE = -1  # This state is used for safety check before flight
TAKE_OFF = 0  # This state increases the altitude
GOTO_GRID_LOCATION = 1  # This state flys to a given location
FLY_SEARCH_PATTERN = 2  # This state flys a search pattern for roomba
SEARCH_FOR_ROOMBA = 3  # This state will check the roomba publisher for possible roombas
PICK_ROOMBA = 4  # This state decides the best roomba to interact with
LOCALIZE_TO_ROOMBA = 5  # This state localizes to the chosen roomba
HOVER_OVER_ROOMBA = 11  # This state allows the quad to hover over the roomba
LOWER_TO_ROOMBA = 6  # This state lowers to a safe altitude to interact with roomba
INTERACT_WITH_ROOMBA = 7  # This state interacts with the roomba
GOTO_SAFE_ALTITUDE = 8  # This state brings the quad up to a safe altitude
LAND = 9  # This state lands the quad in the current location
END = 10  # This state never changes to another state, all output values 0

# Constants
SEARCH_ALTITUDE = 1  # Meters (Also take off altitude) This is the altitude the program uses to fly at when searching, also the target when taking off
ROOMBA_INTERACTION_ALTITUDE = 0.305  # Meters (12 inches) This should be the height we need to hover to garuntee the magnet will hit the roomba
START_LOCATION = [0, 0]  # Variable for start location of the quad (x, y)
TIME_TO_HOVER = 5  # Seconds spent in hovering state
VELOCITY_VECTOR_MAG_MAX = 0.015  # This sets the initialize variable for the max allowed flow reading at startup


class TakeOffRoombaPrioritize:

    def __init__(self):
        # Set up publishers #
        #######################################################################
        self.pub = rospy.Publisher("/master/control/error", Float64MultiArray, queue_size=10)
        #######################################################################

        # SUBSCRIBERS #
        #######################################################################
        # subscribe to altitude
        rospy.Subscriber("/mavros/altitude", Altitude, self.get_altitude)
        # Subscribe to roomba location
        rospy.Subscriber("/roomba_interaction/trajectory", PoseArray, self.xy_roomba_callback)
        # Subscribe to flow velocity for initalization safety check
        self.flow_sub = rospy.Subscriber("mavros/local_position/velocity", TwistStamped, self.flow_callback)
        #######################################################################

        # Set up self variables #
        #######################################################################
        # Create variable for use in publishing the error for PID
        self.error_vector = Float64MultiArray()

        #  The offset of the quad from where it wants to be (Could be roomba or line)
        self.setpoint_x = 0
        self.setpoint_y = 0
        self.setpoint_z = 0

        # Measured value of x and y represent the location the quad is seeing (Could be roomba or line)
        self.measured_value_x = 0  # Set by subscribing - either lines or roombas
        self.measured_value_y = 0  # Set by subscribing - either lines or roombas
        self.measured_value_z = 0  # Subscribe to this, the altitude

        # Roomba(s) location (from callback)
        self.roombas = PoseArray()

        # Set error for error measurement
        self.allowed_error = 0.1

        # Sets the rate for the ros loop
        self.rate = rospy.Rate(10)  # 10hz

        # initializes the state variable
        self.state = INITIALIZE  # Starts in INITIALIZE state

        # Local to rooba, this is true whenn the quad is over the selected roomba and starts HOVERing
        self.is_hovering = False
        self.stop_hovering = False
        self.start_time = 0

        # variables for safety check
        self.velocity_vector_magnitude = 0
        self.has_flow_data = False
        self.stop_loop = False

        # Initialize grid_location for moving to location state
        self.grid_location = [0, 0]
        #######################################################################

        # Run main method
        self.main()

    def main(self):
        while not rospy.is_shutdown() or not self.stop_loop:
            ###################################################################
            # Check for obstacles
            # Subsribe to obstacle detection
            #rospy.Subscriber("/obstacle_detection/obstacle_alert", Float64, self.get_altitude)  # Still needs to be written
            ##################################################################

            # State Machine (This should happen before movement functions but after obstacle ovoidance and subscriber methods)
            if(self.state ==  INITIALIZE):
                self.initialize()
            elif(self.state ==  TAKE_OFF):
                self.take_off()
            elif(self.state == GOTO_GRID_LOCATION):
                self.goto_grid_location()
            elif(self.state == FLY_SEARCH_PATTERN):
                self.search()
            elif(self.state == SEARCH_FOR_ROOMBA):
                self.search_for_roomba()
            elif(self.state == PICK_ROOMBA):
                self.pick_roomba()
            elif(self.state == LOCALIZE_TO_ROOMBA):
                self.localize_roomba()
            elif(self.state == HOVER_OVER_ROOMBA):
                self.hover_over_roomba()
            elif(self.state == LOWER_TO_ROOMBA):
                self.lower_to_roomba()
            elif(self.state == INTERACT_WITH_ROOMBA):
                self.interact_with_roomba()
            elif(self.state == GOTO_SAFE_ALTITUDE):
                self.goto_safe_altitude()
            elif(self.state == LAND):
                self.land()
            elif(self.state == END):
                self.end()
            else:  # This will only execute if the state is not in the state machine above
                self.rospy.loginfo("ERROR: State does not exist")
                self.end()

            # Do movement methods
            self.calculate_error()  # Calculates error and can take care of 2.
            self.target_location()

            # Publishers
            self.pub.publish(self.error_vector)

            # Sleep the ros rate for the loop
            self.rate.sleep()  # sleep the ros rate

###############################################################################
    # State Methods
    def initialize(self):
        # wait for flow data
        if self.has_flow_data:
            # Once we have data, lets check it and change state
            if self.velocity_vector_magnitude > VELOCITY_VECTOR_MAG_MAX:
                rospy.loginfo("recalibration of FCU nessary: restart Pixhawk")
                rospy.loginfo("breaking loop, you must restart mavros after pixhawk")
                self.stop_loop = True
            else:
                self.state = TAKE_OFF

    def take_off(self):
        # Set the setpoint for error calculation to take off altitude
        self.setpoint_x = START_LOCATION[0]
        self.setpoint_y = START_LOCATION[1]
        self.setpoint_z = SEARCH_ALTITUDE

        #  Checks for the quad being at the location, changes state
        if self.at_target_location == 1:
            # self.grid_location = [10, 10]  # Center of field
            self.state = GOTO_GRID_LOCATION  # changes state to goto_grid_location

    def goto_grid_location(self):
        self.state = FLY_SEARCH_PATTERN

    def search(self):
        self.state = SEARCH_FOR_ROOMBA

    def search_for_roomba(self):
        self.state = PICK_ROOMBA

    def pick_roomba(self):
        self.state = LOCALIZE_TO_ROOMBA

    def localize_to_roomba(self):
        # Set setpoint location equal to roomba reading
        # self.setpoint_x = self.roombas[0].position.x
        # self.setpoint_y = self.roombas[0].position.y
        # add z to every state

        # Check for change to next state
        if self.at_target_location == 1:
            self.state = HOVER_OVER_ROOMBA

    def hover_over_roomba(self):
        # Get the start time for hovering
        if not self.is_hovering:
            self.is_hovering = True
            self.start_time = rospy.get_time()

        # Set setpoint location equal to roomba reading
        # self.setpoint_x = self.roombas[0].position.x
        # self.setpoint_y = self.roombas[0].position.y

        # Check for exit from hovering -> Check if the hover timer has expired
        if rospy.get_time() - self.start_time > TIME_TO_HOVER:
            self.is_hovering = False
            self.state = LOWER_TO_ROOMBA

    def lower_to_roomba(self):
        self.state = INTERACT_WITH_ROOMBA

    def interact_with_roomba(self):
        self.state = GOTO_SAFE_ALTITUDE

    def got_safe_altitude(self):
        self.state = LAND

    def land(self):
        # Set the setpoint to ground location
        self.setpoint_z = 0

        # Checks for the quad being at the location, changes state
        if self.at_target_location == 1:
            self.state = END

    def end(self):
        print "Ended"

###############################################################################

###############################################################################
    # Subscription callbacks #
    # Subscribes to the altitude
    def get_altitude(self, z_altittude):

        # Make sure the quad is in a safe zone to use the lidar
        if z_altittude.relative > 0.3 and z_altittude.relative < 20:
            self.measured_value_z = z_altittude.relative

        # Otherwise use the difference between ground baro reading and current baro reading
        else:
            self.measured_value_z = z_altittude.amsl - z_altittude.terrain

    # Roomba call back
    def xy_roomba_callback(self, roombas):
        self.rombas = roombas

    def flow_callback(self, flow_data):
        # Calculate the magnitude from the flow sensor
        self.velocity_vector_magnitude = (self.flow_data.twist.linear.x**2 +
                                          self.flow_data.twist.linear.y**2 +
                                          self.flow_data.twist.linear.z**2)**0.5
        # Notify the statemachine that the flow data has come in
        self.has_flow_data = True

        # Stop the subscriber
        self.flow_sub.unsubscribe()
###############################################################################

###############################################################################
    # Movement Functions #
    # Calculates the distance that the quad needs to move
    def calculate_error(self):
        self.error_vector[0] = 1  # Tells PID to use PID for x, not passs through velocity
        self.error_vector[1] = 1  # Tells PID to use PID for y, not passs through velocity
        self.error_vector[2] = 1  # Tells PID to use PID for z, not passs through velocity
        self.error_vector[3] = self.setpoint_x - self.measured_value_x
        self.error_vector[4] = self.setpoint_y - self.measured_value_y
        self.error_vector[5] = self.setpoint_z - self.measured_value_z

    # Checks to see if the quad is where we want it
    def target_location(self):
        if  self.error_vector[3] > -self.allowed_error and self.error_vector[3] < self.allowed_error:
            if self.error_vector[4] > -self.allowed_error and self.error_vector[4] < self.allowed_error:
                if self.error_vector[5] > -self.allowed_error and self.error_vector[5] < self.allowed_error:
                    self.at_target_location = 1
                else:
                    self.at_target_location = 0
            else:
                self.at_target_location = 0
        else:
            self.at_target_location = 0
###############################################################################

if __name__ == '__main__':
    # Initiate the node
    rospy.init_node('master', anonymous=False)
    rospy.loginfo("if main")
    try:
        master = TakeOffRoombaPrioritize()
    except rospy.ROSInterruptException:
        pass
