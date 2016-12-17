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

        self.program = [
            [ 0 ],
            [ 3 , 0.6 ]
        ]

        self.program_index = 0

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


        rospy.Subscriber("mavros/local_position/velocity", TwistStamped, self.local_position_velocity_callback)
        rospy.Subscriber("mavros/altitude", Altitude, self.altitude_callback)
        rospy.Subscriber("/mavros/state",State,self.state_callback)

        self.state_publisher = rospy.Publisher('mavros/state', State, queue_size = 10)

        self.state_variable = State()

    def main(self):

        while not rospy.is_shutdown():

            self.x_bool = 1
            self.y_bool = 1
            self.z_bool = 1

            self.x_linear = 0
            self.y_linear = 0
            self.z_linear = 0

            print(self.stance_names(self.stance))

            self.safty_checks(self)

            if self.stance == 0:
                self.Logical_State()

            elif self.stance == 1:
                self.Land()

            elif self.stance == 2: # Takeoff
                self.Takeoff(self.program[self.program_index][2])

            elif self.stance == 3:
                pass

            elif self.stance == 4:
                pass

            elif self.stance == 5:
                pass







            self.stance_previous = self.stance

            velocity_publisher.publish(velocity_vector)


    ##########################################################################################
    ################# various functions ######################################################
    ##########################################################################################

    def safty_checsk(self):
        if not self.state_current.guided:
            self.stance = 0

        # Add a velocity magnitude HARD limit

    def maintain_altitude(self, altitude_goal):
        return self.altitude_current - altitude_goal


    ##########################################################################################
    ################# state functions ########################################################
    ##########################################################################################

    def Logical_State(self):


        self.program_index += 1
        if self.program_index > len(self.program):



        self.x_bool = 1
        self.y_bool = 1
        self.z_bool = 1

        self.x_linear = 0
        self.y_linear = 0
        self.z_linear = 0

    def Land(self):


        self.x_bool = 1
        self.y_bool = 0
        self.z_bool = 1

        self.x_linear = 0
        self.y_linear = 0.05
        self.z_linear = 0

    def takeoff(self, altitude):

        self.x_bool = 1
        self.y_bool = 1
        self.z_bool = 1

        self.x_linear = 0
        self.y_linear = self.maintain_altitude(0.6)
        self.z_linear = 0


    ##########################################################################################
    ################# Subscriber Call Backs ##################################################
    ##########################################################################################

    def altitude_callback(self, data):

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
