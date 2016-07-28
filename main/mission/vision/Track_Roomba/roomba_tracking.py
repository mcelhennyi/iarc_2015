import rospy
from geometry_msgs.msg import Pose, PoseArray
#from std_msgs.msg import Header, Float64

#import math


class roomba_tracking:

    def __init__(self):
        # Initiate publishers
        self.pub = rospy.Publisher("/roomba_tracking/roomba_array", PoseArray, queue_size=10)
        self.sub_roomba_array = Pose()
        self.rate = rospy.Rate(10)
        self.h_lines = [20]
        self.v_lines = [20]
        self.uuid_indx = 0
        self.Header.seq += 1
        self.Header.stamp = rospy.get_time()
        self.roomba_track = ()
        self.loop()
        #subscribe roomba arrray from "Circle_detect.py"
        rospy.Subscriber("/roomba/location_meters", PoseArray, self.roomba_array, queue_size=10)

        while not rospy.is_shutdown():

            self.roomba_locations = list ( enumerate(self.roomba_array.Pose))

        #rospy.Subscriber("/mavros/global_position/rel_alt", Float64, self.callback)

        #PUBLISH THE listed roomba location
            self.pub.publish( self.roomba_locations)
            rospy.loginfo ( self.roomba_locations)

            #apply uuid to list index


            # Sleep the ros rate for the loop
            self.rate.sleep()  # sleep the ros rate



if __name__ == '__main__':
    # Initiate the node
    rospy.init_node('roomba_tracking', anonymous=True)
    try:
        rt = roomba_tracking()
    except rospy.ROSInterruptException:
        pass