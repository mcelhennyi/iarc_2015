#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseArray
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
TAKE_OFF = 0  # This state increases the altitude
GOTO_GRID_LOCATION = 1  # This state flys to a given location
FLY_SEARCH_PATTERN = 2  # This state flys a search pattern for roomba
SEARCH_FOR_ROOMBA = 3  # This state will check the roomba publisher for possible roombas
PICK_ROOMBA = 4  # This state decides the best roomba to interact with
LOCALIZE_TO_ROOMBA = 5  # This state localizes to the chosen roomba
LOWER_TO_ROOMBA = 6  # This state lowers to a safe altitude to interact with roomba
INTERACT_WITH_ROOMBA = 7  # This state interacts with the roomba
GOTO_SAFE_ALTITUDE = 8  # This state brings the quad up to a safe altitude
LAND = 9  # This state lands the quad in the current location
END = 10  # This state never changes to another state, all output values 0

# Constants
SEARCH_ALTITUDE = 2.2  # Meters (Also take off altitude) This is the altitude the program uses to fly at when searching, also the target when taking off
ROOMBA_INTERACTION_ALTITUDE = 0.305  # Meters (12 inches) This should be the height we need to hover to garuntee the magnet will hit the roomba
START_LOCATION = [0, 0]  # Variable for start location of the quad (x, y)


class TakeOffRoombaPrioritize:

    def __init__(self):
        # Set up publishers #
        #######################################################################
        self.pub = rospy.Publisher("/master/control/error", Vector3Stamped, queue_size=10)
        #######################################################################

        # Set up self variables #
        #######################################################################
        # Create variable for use in publishing the error for PID
        self.error_vector = Vector3Stamped()

        #  The offset of the quad from where it wants to be (Could be roomba or line)
        self.setpoint_x = 0
        self.setpoint_y = 0
        self.setpoint_z = 0

        # Measured value of x and y represent the location the quad is seeing (Could be roomba or line)
        self.measured_value_x = 0
        self.measured_value_y = 0
        self.measured_value_z = 0  # Subscribe to this, the altitude

        # Set error for error measurement
        self.allowed_error = 0.1

        # Sets the rate for the ros loop
        self.rate = rospy.Rate(10)  # 10hz

        # initializes the state variable
        self.state = TAKE_OFF  # Starts in take off location

        # Initialize grid_location for moving to location state
        self.grid_location = [0, 0]
        #######################################################################

        # Variables/objects for setting mode #
        #######################################################################
        ## Sets up service and initialize mode
        #rospy.wait_for_service('mavros/set_mode')
        #self.set_mode_serv = rospy.ServiceProxy('mavros/set_mode', SetMode)  # creates service for setting mode
        #self.off_board_mode = SetMode()  # Creates SetMode object for service
        #self.off_board_mode.custom_mode = "OFFBOARD"  # Sets mode to off board control
        #self.nav_guided_serv = rospy.ServiceProxy('/mavros/cmd/guided_enable', CommandBool)

        ## Variable to check if offBoard is enabled
        #self.mode = SetMode()
        #self.mode.custom_mode = "OFFBOARD"
        #self.off_board_mode_enabled = True
        #self.armed = True
        #######################################################################

        # Run main method
        self.main()

    def main(self):
        while not rospy.is_shutdown():
            ###################################################################
            # Check for obstacles
            # Subsribe to obstacle detection
            #rospy.Subscriber("/obstacle_detection/obstacle_alert", Float64, self.get_altitude)  # Still needs to be written
            ###################################################################

            # SUBSCRIBERS #
            # subscribe to altitude
            rospy.Subscriber("/mavros/global_position/rel_alt", Float64, self.get_altitude)
            # Subscribe to roomba location
            rospy.Subscriber("/roomba_interaction/trajectory", PoseArray, self.get_roombas)

            # State Machine (This should happen before movement functions but after obstacle ovoidance and subscriber methods)
            if(self.state ==  TAKE_OFF):
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
            self.target_location()  #

            # Publishers
            self.pub.publish(self.error_vector)  # Vector3Stamped type variable

            # Sleep the ros rate for the loop
            self.rate.sleep()  # sleep the ros rate

###############################################################################
    # State Methods
    def take_off(self):
        # Set the setpoint for error calculation to take off altitude
        self.setpoint_x = START_LOCATION[0]
        self.setpoint_y = START_LOCATION[1]
        self.setpoint_z = SEARCH_ALTITUDE

        #  Checks for the quad being at the location, changes state
        if self.at_target_location == 1:
            self.grid_location = [10, 10]  # Center of field
            self.state = GOTO_GRID_LOCATION  # changes state to goto_grid_location



    def land(self):
        # Set the setpoint to ground location
        self.setpoint_z = 0

        # Checks for the quad being at the location, changes state
        if self.at_target_location == 1:
            self.state = END
###############################################################################

###############################################################################
    # Mode setting and arming method
    def set_off_board_mode(self):
        ready  = rospy.wait_for_service('mavros/set_mode')
        rospy.loginfo("setting off board mode")
        if ready:
            # If offboard is not enabled
            if not self.off_board_mode_enabled:
                try:
                    set_mode_srv = rospy.ServiceProxy('mavros/set_mode', SetMode)
                    self.off_board_mode_enabled = set_mode_srv.call(self.mode)
                    rospy.loginfo("Offboard Enabled")
                except rospy.ServiceException as e:
                    print ( ("offboard enable: Service call failed: %s" %e ) )

            if self.off_board_mode_enabled:
                try:
                    arm_srv = rospy.ServiceProxy('mavros/CommandBool', CommandBool)
                    self.armed = arm_srv.call(True)
                    rospy.loginfo("Armed")
                except rospy.ServiceException as e:
                    print ( ("Arming enable: Service call failed: %s"%e ) )

###############################################################################

###############################################################################
    # Subscription callbacks #
    # Subscribes to the altitude
    def get_altitude(self,z_altittude):
        self.measured_value_z = z_altittude.data

    # Roomba call back
    def xy_location_control(self, roombas):
        pass
###############################################################################

###############################################################################
    # Movement Functions #
    # Calculates the distance that the quad needs to move
    def calculate_error(self):
        self.error_vector.vector.x = self.setpoint_x - self.measured_value_x
        self.error_vector.vector.y = self.setpoint_y - self.measured_value_y
        self.error_vector.vector.z = self.setpoint_z - self.measured_value_z

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
###############################################################################

if __name__ == '__main__':
    # Initiate the node
    rospy.init_node('master', anonymous=False)
    rospy.loginfo("if main")
    try:
        master = TakeOffRoombaPrioritize()
    except rospy.ROSInterruptException:
        pass
