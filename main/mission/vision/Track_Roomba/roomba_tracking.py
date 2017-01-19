import rospy
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Float64

#import math


class roomba_tracking:

    def __init__(self):
        # Initiate publishers
        self.pub = rospy.Publisher("/roomba_tracking/roomba_array", PoseArray, queue_size=10)
        self.sub_roomba_array = Pose()
        self.rate = rospy.Rate(10)
        #I set Pose.oreitation.w as UUID as Ian said
        self.uuid_indx = Pose.orientation.w
        #self.loop()

        #subscribe roomba arrray from "Circle_detect.py". Use Pose.orientation.w as UUID
        rospy.Subscriber("/roomba/location_meters", Float64, Pose.orientation.w, queue_size=10)

        while not rospy.is_shutdown():
            #enumerated UUID start at 0
            #there we have a pose includes index and UUID
            self.roomba_locations = list ( enumerate( Pose.orientation.w))

        #PUBLISH THE listed roomba location
            self.pub.publish( self.roomba_locations)
            rospy.loginfo ( self.roomba_locations)

        # Sleep the ros rate for the loop
        self.rate.sleep()  # sleep the ros rate



if __name__ == '__main__':
    # Initiate the node
    rospy.init_node('roomba_tracking', anonymous=True)
    try:
        rt = roomba_tracking()
    except rospy.ROSInterruptException:
        pass