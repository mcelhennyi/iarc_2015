#!/usr/bin/env python
import math
import rospy
from geometry_msgs.msg import PoseArray

class RoombaInteraction:

    def __init__(self):
        # Initiate publishers
        self.pub = rospy.Publisher("/roomba_interaction/trajectory", PoseArray, queue_size=10)

        # Initiate Global (self.XXX) variables
        self.rate = rospy.Rate(10)
        self.h_lines = [20]
        self.v_lines = [20]
        self.sub_roomba_array = PoseArray()
        self.pub_roomba_array = PoseArray()
        self.new_location = []
        self.old_location = []
        self.first_loop = True
        self.x_indx = 0
        self.y_indx = 1
        self.uuid_indx = 2

        self.loop()

# NOTES:
    # ABOUT:
        # This program will take in the roomba array from roomba feature tracking node, then compare old versions to new
        #   versions to get the trajectory in radians of the roombas. Then it will re publish all the data, including the
        #   the trajectory in the poses[x].orientation.x location.
    # ASSUMES THE FOLLOWING:
        # The roomba tracking publishes under the name "/roomba_tracking/roomba_array"
        # The x and y are x and y location in relation to the quad.
        # The z is a randomly generate id number that the tracker will associate with a roomba as it moves.
        # The h lines represent y and v lines represent x

    def loop(self):
        while not rospy.is_shutdown():
            # Sub to roomba data from feature tracking
            rospy.Subscriber("/roomba_tracking/roomba_array", PoseArray, self.roomba_callback)

            # Sub to line data from feature tracking
            rospy.Subscriber("/line_tracking/h_lines_array", PoseArray, self.h_line_callback)
            rospy.Subscriber("/line_tracking/v_lines_array", PoseArray, self.v_line_callback)

            # Compare the location of the roomba to the location of the lines
            self.compare_location_estimate_trajectory()

            # Publish the message
            self.pub.publish(self.pub_roomba_array)

            # Used to allow initialization of all variables
            self.first_loop = False

            # Sleeps for the rate set above
            self.rate.sleep()

    # Loops through the roomba array and compares the old location to new location to get trajectory
    def compare_location_estimate_trajectory(self):
        # Loop through roombas to get new location
        for i, roomba in enumerate(self.sub_roomba_array.poses):
            self.new_location[i] = self.get_roomba_grid_location(roomba)

        if self.first_loop:  # Since it is the first run the old location is set to the old location (This only runs once)
            self.old_location = self.new_location
        else:
            # Loop through the new locations
            for new_indx, new_roomba in enumerate(self.new_location):
                # Loop through the old locations
                for old_indx, old_roomba in enumerate(self.old_location):
                    # If the old roomba is the same as new roomba
                    if new_roomba[self.uuid_indx] == old_roomba[self.uuid_indx]:
                        # Assign all of the subscribed data to the publish array
                        self.pub_roomba_array.poses[old_indx].position.x = self.new_location[self.x_indx]
                        self.pub_roomba_array.poses[old_indx].position.y = self.new_location[self.y_indx]
                        self.pub_roomba_array.poses[old_indx].position.z = self.new_location[self.uuid_indx]

                        # Get the trajectory
                        # Assignes the trajectory to the orientation x in the pose array
                        self.pub_roomba_array.poses[old_indx].orientation.w = self.get_angle(new_roomba, old_roomba)


    # ##############################################
    # Call backs for subscriber
    # Type Pose array
    def roomba_callback(self, roombas):
        self.sub_roomba_array = roombas

    # Type Float64MultiArray
    def h_line_callback(self, h_lines):
        self.h_lines = h_lines

    # Type Float64MultiArray
    def v_line_callback(self, v_lines):
        self.v_lines = v_lines
    # ###############################################

    # Compare the location of the roomba in relation to the quad to the grid lines to return an array
    # containing the location of the roomba in the grid.
    # - Roomba_location is of type Pose
    # - Assumes the h lines represent y and v lines represent x
    def get_roomba_grid_location(self, roomba_location):
        # Initialize return array for roomba location
        roomba_grid_location = []  # 0 is X, 1 is Y

        # Where am I: Based on the absolute value of the 0th index of the  lines array
        quad_h_location = abs(self.h_lines[0])
        quad_v_location = abs(self.v_lines[0])

        # Get location of the roomba in relation to the grid: Add the quad location and the
        # roomba (referred to quad) location
        roomba_grid_location[self.x_indx] = quad_v_location + roomba_location.position.x
        roomba_grid_location[self.y_indx] = quad_h_location + roomba_location.position.y
        roomba_grid_location[self.uuid_indx] = roomba_location.position.z  # Represents a random number that the roomba
                                                                      # tracking generates to be able to track a
                                                                      # specific roomba

        # Make return of the roomba location
        return roomba_grid_location

    # Takes argument of [x, y, uuid] generated from roomba_grid_location does math to return trajectory
    def get_angle(self, new_location, old_location):
        x_old = old_location(self.x_indx)
        y_old = old_location(self.y_indx)

        x_new = new_location(self.x_indx)
        y_new = new_location(self.y_indx)

        return math.atan((y_new - y_old)/(x_new - x_old))

if __name__ == '__main__':
    # Initiate the node
    rospy.init_node('RoombaInteraction', anonymous=True)
    try:
        ri = RoombaInteraction()
    except rospy.ROSInterruptException:
        pass