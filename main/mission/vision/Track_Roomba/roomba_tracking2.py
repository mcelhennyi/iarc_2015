import rospy
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Float64MultiArray
import math


"""
Notes about the Pose representing the roomba
The parts of the pose and what they are used for:
- orientation.w = UUID
- orientation.z = Last time estimated
"""


class roomba_tracking:

    def __init__(self):
        # Initiate publishers
        self.pub = rospy.Publisher("/roomba_tracking/roomba_array", PoseArray, queue_size=10)
        self.sub_roomba_array = PoseArray()
        self.old_sub_roomba_array = PoseArray()
        self.pub_roomba_array = PoseArray()
        self.closest_new_roomba = Pose()
        self.rate = rospy.Rate(10)
        #roomba speed found in IARC rule doc
        self.speed_roomba = 0.33
        self.h_lines = [20]
        self.v_lines = [20]
        self.first_loop = True
        self.main()



    def main(self):
        while not rospy.is_shutdown():
            # subscribe roomba arrray from "Circle_detect.py". Use Pose.orientation.w as UUID
            rospy.Subscriber("/roomba/location_meters", PoseArray, self.call_back)

            # Sub to line data from feature tracking
            rospy.Subscriber("/line_tracking/h_lines_array", Float64MultiArray, self.h_line_callback)
            rospy.Subscriber("/line_tracking/v_lines_array", Float64MultiArray, self.v_line_callback)

            if not self.first_loop:
                # Loop through the array of poses
                for i, old_roomba in enumerate(self.old_sub_roomba_array.Pose):

                    # Get the old current roombas location in the grid
                    old_roomba_location = self.get_roomba_grid_location(old_roomba)

                    # Do the calculation to see where this "old_roomba" should be.
                    """
                    Do the calculation to determine future location for the old_roomba based on:
                     - the last time calculated (orientation.z)
                     - Known speed of roombas found in IARC rules doc
                     - its old grid location (old_roomba_location)
                    """
                    self.new_time = orientation.z
                    # Check for the new_roomba that has a current location closest to
                    for i, new_roomba in enumerate(self.sub_roomba_array.poses):
                        # If the first time, use as closest
                        if i is 0:
                            self.closest_new_roomba = new_roomba

                        # Get the new roomba location
                        new_roomba_location = self.get_roomba_grid_location(new_roomba)

                        """
                        Ideas on how to write this part:
                        - Use the first roomba and say it is the closest.
                        - Look at the difference between estimate future location of old_roomba vs closest,
                            if the next iteration of this for loop has a smaller diffence set it as the closest,
                            if it isnt smaller, pass and do not re assign the current new_roomba as the closest.

                            However you will need to consider the situation where there is not a roomba close to
                                it.....meaning that its a new one. This will need a new UUID generated.
                        """
                        # Save the current time as last time estimated (orientation.z)
                        self.new_roomba.orientation.z = time.time()

                    # Assign the "closest roomba" to an array to publish
                    if i < self.sub_roomba_array.poses.__len__:
                        # Assign the UUID to the new roomba
                        self.closest_new_roomba.orientation.w = old_roomba.orientation.w

                        # Put the roomba into the publish array
                        self.pub_roomba_array.poses[i] = self.closest_new_roomba
                    else:
                        # If there are no more roombas left in the current subcribed roombas stop searching
                        break


            # Save pub array as old array
            self.old_sub_roomba_array = self.pub_roomba_array

            # PUBLISH THE listed roomba location
            self.pub.publish(self.pub_roomba_array)
            rospy.loginfo(self.pub_roomba_array)

            # Sleep the ros rate for the loop
            self.rate.sleep()  # sleep the ros rate

    # This gets the subscribed array of poses and saves it as the current pose
    def call_back(self, pose_array):
        self.sub_roomba_array = pose_array

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
        roomba_grid_location[self.uuid_indx] = roomba_location.position.w # Represents a random number that the roomba
        # tracking generates to be able to track a
        # specific roomba

        # Make return of the roomba location
        return roomba_grid_location

    # Type Float64MultiArray
    def h_line_callback(self, h_lines):
        self.h_lines = h_lines

    # Type Float64MultiArray
    def v_line_callback(self, v_lines):
        self.v_lines = v_lines




if __name__ == '__main__':
    # Initiate the node
    rospy.init_node('roomba_tracking', anonymous=True)
    try:
        rt = roomba_tracking()
    except rospy.ROSInterruptException:
        pass