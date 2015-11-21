import rospy
from geometry_msgs.msg import Vector3Stamped


# 1.Need to know when the quad is siting still over its current target(physical roomba or location we tell it)
# 2.Need to figure out how to change the PID to try to make the numbers go to something other than 0 so that we
    # can change the location when it is not looking at a target.
# 3.During take off and landing the X and Y needs to do nothing.
# 4.Subscribe to something to tell this if there is an obstacle in the way
# 5.Figure out when we need to land


class TakeOffRoombaPrioritize:

    def __init__(self):

        self.rate = rospy.Rate(10)  # 10hz

        self.at_target_location = 0  # 1. need to subscribe to this.

        self.obstacle_in_the_way = 0  # 4. Subscribe to this?
        self.ready_to_land = 0
        self.need_to_land = 0  # 5. calculate here?
        self.take_off()  # Runs the take off def

    def take_off(self):
        self.ready_to_land = 0  # Reset this value back to 0 if we had already landed and want to take off again

        # 2.This needs to make the PID Z localize on 2 instead of 0
        # May not need to change any kind of vectors only variables in the PID

        # 3.Need to make sure the quad is not looking for a roomba until it is at the correct altitude

        # if altitude is good
            self.loop_roomba_prioritize()


    def loop_roomba_prioritize(self):
        # while the node is still running loop
        while not rospy.is_shutdown():
            # Needs to choose what roomba to localize on when seeing more than one

            # sleep the ros rate
            self.rate.sleep()
            # Need something like this to tell it to land
            if self.need_to_land == 1:
                self.land()  # Will this work?? Or will it still be in this while loop forever??
                # Need to break loop??

            if self.obstacle_in_the_way == 1:
                self.dodge_obstacel()
                # Need to break loop??

    def dodge_obstacle(self):
        self.obstacle_in_the_way = 0  # Resets this variable for the next time
        # Needs to stop the X and Y movement
        # Needs to adjust the altitude to miss the object

        # When the altitude is adjusted to miss the obstacle need to go back to looking for roombas
        # if altitude is good
            self.loop_roomba_prioritize()


    def land(self):
        self.need_to_land = 1  # Resets this variable for the next time
        # Moves the quad before landing so that it does not land on the roomba
        # 2.Need to make the PID localize on x=2,y=2 of the roomba location instead of 0,0

        # When the the quad is there then "ready_to_land" needs to go high
         while not rospy.is_shutdown():
            if self.at_target_location == 0:
                self.ready_to_land = 0
            if self.at_target_location == 1:
                # 3.Need to make sure that it does not change X and Y location
                self.ready_to_land = 1

            if self.ready_to_land == 1:
                # 2.This needs to make the PID Z localize on 0 again instead of 2


if __name__ == '__main__':
    # Initiate the node
    rospy.init_node('PID_main', anonymous=True)
    try:
        pid = TakeOffRoombaPrioritize()
    except rospy.ROSInterruptException:
        pass
