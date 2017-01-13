#!/usr/bin/env python
import rospy
from geometry_msgs.msg import *
from std_msgs.msg import *
from mavros_msgs.srv import *
from mavros_msgs.msg import *
import time
import PID

#Author: JJ Marcus
#Last Edit Date: 28/11/2016

class Master():

    def __init__(self):


        ################# Program variables
        self.program = [
            [ 0 ],
            [ 3 , 0.6 ],
            []
        ]

        self.program_index = 0
        ################

        ################ Stance variables
        self.stance = 0
        self.stance_names = [
            'Logical_State',
            'Land',
            'Takeoff',
            'uncoded',
            'uncoded',
            'uncoded',
            'uncoded',
            'uncoded',
            'uncoded',
            'uncoded'
        ]

        self.LOGICAL_STANCE = 0
        self.LAND = 1
        self.TAKOFF = 2
        ################

        ################ subscriber threads
        rospy.Subscriber("mavros/local_position/velocity", TwistStamped, self.local_position_velocity_callback)
        rospy.Subscriber("mavros/altitude", Altitude, self.altitude_callback)
        rospy.Subscriber("/mavros/state",State,self.state_callback)
        ################

        ################ publisher objects and variables
        self.master_to_pid_vector = rospy.Publisher("/master/control/error", Float64MultiArray, queue_size=10)
        self.master_to_pid_vector = Float64MultiArray
        self.PID_ON =  1
        self.PID_OFF = 0
        self.state_publisher = rospy.Publisher('mavros/state', State, queue_size = 10)
        self.state_variable = State()
        ################

    def main(self):

        count = 0

        while not rospy.is_shutdown():

            count += 1
            self.safty_checks(count)


            self.x_bool = self.PID_OFF
            self.y_bool = self.PID_OFF
            self.z_bool = self.PID_OFF

            self.x_linear = 0
            self.y_linear = 0
            self.z_linear = 0
            if self.stance == self.LOGICAL_STANCE:
                self.Logical_State()

            elif self.stance == self.LAND:
                self.Land()

            elif self.stance == self.TAKOFF: # Takeoff
                self.Takeoff(self.program[self.program_index][2])






            self.stance_previous = self.stance

            self.velocity_publisher.publish(self.velocity_vector)
    ##########################################################################################
    ################# state functions ########################################################
    ##########################################################################################

    ###################################
    def Logical_State(self):


        self.program_index += 1
        if self.program_index > len(self.program):
            self.stance = self.LAND
        else:
            self.stance = self.program[self.program_index][0]


        self.x_bool = self.PID_OFF
        self.y_bool = self.PID_OFF
        self.z_bool = self.PID_OFF

        self.x_linear = 0
        self.y_linear = 0
        self.z_linear = 0
    ###################################

    ###################################
    def Land(self):

        if not self.stance_previous == self.LAND:
            countdown = 5

        if self.altitude_current < 0.10 or self.altitude_current < 0:
            countdown -= 0.1
            rospy.loginfo("WARNING: low altitude, DISARMING in: " + str(countdown))
        else:
            countdown = 10

        if ( self.altitude_current < 0.05 or self.altitude_current < 0 ) and countdown < 0:
            rospy.loginfo("DISARM-DISARM-DISARM")
            self.state_variable.armed = False
            self.state_variable.guided = False
            self.state_publisher.publish(self.state_variable)

        self.x_bool = self.PID_OFF
        self.y_bool = self.PID_ON
        self.z_bool = self.PID_OFF

        self.x_linear = 0
        self.y_linear = 0.05
        self.z_linear = 0
    ###################################


    ###################################
    def takeoff(self, goal_altitude):

        self.x_bool = self.PID_OFF
        self.y_bool = self.PID_ON
        self.z_bool = self.PID_OFF

        self.x_linear = 0
        self.y_linear = self.maintain_altitude(goal_altitude)
        self.z_linear = 0
    ###################################


    ###################################
    def calibrate_fcu(self):

        self.x_bool = self.PID_OFF
        self.y_bool = self.PID_OFF
        self.z_bool = self.PID_OFF

        self.x_linear = 0
        self.y_linear = 0
        self.z_linear = 0
    ###################################


    ##########################################################################################
    ################# various functions ######################################################
    ##########################################################################################

    def safty_checsk(self, count):
        if not self.state_current.guided:
            self.stance = 0

        if count % 20 = 1:
            rospy.loginfo("Current stance: " + str(self.stance_names(self.stance)))
            rospy.loginfo("Current altitude: " + self.altitude_current)

        # Add a velocity magnitude HARD limit

    def maintain_altitude(self, altitude_goal):
        return self.altitude_current - altitude_goal



    ##########################################################################################
    ################# Subscriber Call Backs ##################################################
    ##########################################################################################

    def altitude_callback(self, IncomingData):

        if IncomingData.relative > 0.3 and IncomingData.relative < 20:
            self.altitude_current = ( IncomingData.relative )

        else:
            self.altitude_current = ( IncomingData.asml - IncomingData.terrain )

    def local_position_velocity_callback(self,velocity_current):
        self.velocity_current = velocity_current

    def state_callback(self, state):
        self.state_current = state

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
