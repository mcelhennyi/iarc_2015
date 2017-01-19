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
        self.velocity_current = TwistStamped()
        rospy.Subscriber("mavros/local_position/velocity", TwistStamped, self.local_position_velocity_callback)
        time.sleep(0.1)
        velocity_current_linear_vector = [self.velocity_current.twist.linear.x,
                                          self.velocity_current.twist.linear.y,
                                          self.velocity_current.twist.linear.z]
        velocity_current_linear_vector_magnitude = ((velocity_current_linear_vector[0]**2 + velocity_current_linear_vector[1]**2+
               velocity_current_linear_vector[2]**2)**0.5)
        rospy.loginfo("Error from flow velocity: " + str(velocity_current_linear_vector_magnitude) )
        rospy.Subscriber("/mavros/state",State,self.state_callback)
        if     velocity_current_linear_vector_magnitude> 0.015:
           rospy.loginfo("recalibration of FCU nessary: restart Pixhawk")
           rospy.loginfo("breaking loop, you must restart mavros after pixhawk")

        else:
            rospy.loginfo("Starting main loop")
            self.main()

    def main(self):
        # Creates the publisher for ROS. (name of message, data type(from geometry_msgs.msg), queue size)
        velocity_publisher = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        velocity_vector = TwistStamped() # Ros data type for publisher

        x_linear = 0
        y_linear = 0
        z_linear = 0

        altitude_PID = PID(0.5,0.1,0)

        x_ang = 0
        y_ang = 0
        z_ang = 0


        # Creates a state publisher, for when we want to remotely disarm the system
        state_publisher = rospy.Publisher('mavros/state', State, queue_size = 10)
        state_variable = State()

        # sets the rate of the loop
        rate = rospy.Rate(10)
        # goal altitude
        altitude_goal = input('what altitude should I hover at?')
        self.altitude_current = 0

        # state variable
        stance = 0
        stance_previous = 0
        count = 0
        # 0 : logical element
        # 1 : waiting for offb controll
        # 2 : take off
        # 3 : land
        # 4 : search pattern

        while not rospy.is_shutdown():
            count += 1

            if count % 100 == 1:
                rospy.loginfo("State: " + str(stance))


##########################################################################################
################# stance switch ###########################################################
##########################################################################################
            if stance == 0:

                x_linear = 0
                y_linear = 0
                z_linear = 0

                x_ang = 0
                y_ang = 0
                z_ang = 0
                stance = 1

            # waiting for offb control
            elif stance == 1:
                rate.sleep()
                if self.state_current.guided:
                    rospy.loginfo("I'm taking control, takeoff in " + str(countdown))
                    countdown -= 0.1
                    if countdown < 0:
                        rospy.loginfo("Switching to Takeoff")
                        stance_previous = 1
                        stance = 2
                else:
                    countdown = 5

            # Take off:
            elif stance == 2:
                 altitude_goal = 2
                 if count % 10 == 1:
                      rospy.loginfo("Current altitude: " + str(self.altitude_current))

                 if self.altitude_current <= altitude_goal + 0.1 and self.altitude_current >= altitude_goal - 0.1:
                     countdown -= 0.1
                     if count % 10 == 1:
                          rospy.loginfo("At goal altitude, switching to logic state")
                          stance_previous = 2
                          stance = 0

                 else:
                     countdown = 5

                 if countdown < 0:
                     stance = 0


            # Landing
            elif stance == 3:
                 if count % 10 == 1:
                     rospy.loginfo(self.altitude)
                 z_linear = -0.02
                 if self.altitude_current < 0.10:
                     countdown -= 0.1
                     rospy.loginfo("WARNING: low altitude, DISARMING in: " + str(countdown))
                 else:
                     countdown = 10

                 if self.altitude_current < 0.05 and countdown < 0:
                     rospy.loginfo("DISARM-DISARM-DISARM")
                     state_variable.armed = False
                     state_variable.guided = False
                     state_publisher.publish(state_variable)

            elif stance == 4:
                pass
            elif stance == 5:
                pass
            elif stance == 6:
                pass




##########################################################################################
################# velocity finder ########################################################
##########################################################################################
            if not stance == 3: # If were not landing, this works
            # set the verticle speed

                z_linear = altitude_PID(self.altitude_current, altitude_goal)

            # need to do this for x_linear, y_linear, and possibly x,y, and z ang

##########################################################################################
################# velocity publisher #####################################################
##########################################################################################


            # set linear to value
            velocity_vector.twist.linear.x = x_linear
            velocity_vector.twist.linear.y = y_linear
            velocity_vector.twist.linear.z = z_linear

            # Set angular to zero
            velocity_vector.twist.angular.x = x_ang
            velocity_vector.twist.angular.y = y_ang
            velocity_vector.twist.angular.z = z_ang
            # Get the time now for the velocity commands
            time = rospy.Time.now()
            velocity_vector.header.stamp.secs = int(time.to_sec())
            velocity_vector.header.stamp.nsecs = int((time.to_sec() - int(time.to_sec())) * 1000000000)
            velocity_vector.header.seq = count

            # use the publisher
            velocity_publisher.publish(velocity_vector)

            rate.sleep()

##########################################################################################
################# Subscriber Call Backs ##################################################
##########################################################################################

    def altitude_callback(self, data):
        IncomingData = [0,0,0,0,0]
        IncomingData[0] = data.monotonic
        IncomingData[1] = data.amsl
        IncomingData[2] = data.relative
        IncomingData[3] = data.terrain
        IncomingData[4] = data.bottom_clearance

        if IncomingData[2] > 0.3 and IncomingData[2] < 20:
            self.altitude_current = ( IncomingData[2] )

        else:
            self.altitude_current = ( IncomingData[1] - IncomingData[3] )

        return

    def local_position_velocity_callback(self,velocity_current):
        self.velocity_current = velocity_current
        return

    def state_callback(self, state):
        self.state_current = state
        return


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
