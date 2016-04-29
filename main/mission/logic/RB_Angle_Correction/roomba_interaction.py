import rospy

class RoombaInteraction():

    def __init__(self):
        # Initiate publishers

        # Initiate Global (self.XXX) variables

        pass


# NOTES:
    # Comments that are indented and contain a "# ##" at the beginning add more info to the above "# " type comment
    # Comments that are indented and contain a "# " at the beginning are sub methods or loops to be performed



    def loop(self):
        while not rospy.is_shutdown():
            # Sub to roomba data from feature tracking

            # Sub to line data from feature tracking

            # Compare the location of the roomba to the location of the lines

            # Save that location reference then compare to old reference from last iteration of loop
                # ##(Possibly save references in a queue like array that is shifted down one and added to every iteration)

            # Estimate the trajectory of the roomba based on array of past tracking locations

            # Save trajectory and compare to old trajectory to see if it has changed, gotten better, or worse
                # ##Make a claim to the accuracy of the trajectory

            # Publish trajectory in form (x, y) in reference to field
                # ## Where x and y (0, 0) are starting in the lower left corner (red and white intersection)
                # ## x and y max are (20, 20) being the top right of the field (green and white intersection)
                # ## x and y represent meters to a line, so when published they can be of type double for more accuracy

            self.rate.sleep()

if __name__ == '__main__':
    # Initiate the node
    rospy.init_node('RoombaInteraction', anonymous=True)
    try:
        ri = RoombaInteraction()
    except rospy.ROSInterruptException:
        pass
