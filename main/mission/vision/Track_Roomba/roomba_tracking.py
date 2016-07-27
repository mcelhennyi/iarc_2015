import rospy
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Header, Float64
#import math


class roomba_tracking:

    def __init__(self):
        # Initiate publishers
        self.pub = rospy.Publisher("/roomba_tracking/roomba_array", PoseArray, queue_size=10)
        self.sub_roomba_array = Pose()
        #self.sub_roomba_array = PoseArray()
        self.roomba_array = PoseArray()
        self.rate = rospy.Rate(10)
        self.h_lines = [20]
        self.v_lines = [20]
        self.uuid_indx = 0
        self.Header.seq += 1
        self.Header.stamp = rospy.get_time()
        self.roomba_track = ()
        self.loop()
        roomba_locations = PoseArray( self.Header,self.pose_array)
        #subscribe roomba arrray from "Circle_detect.py"
        rospy.Subscriber("/roomba/location_meters", PoseArray, self.callback, queue_size=10)

    def loop(self):
        #set uuid for detected roomba

        while not rospy.is_shutdown():

        roomba_track.append(sub_roomba_array)

        self.roomba_array = list ( enumerate(roomba_track))

        #rospy.Subscriber("/mavros/global_position/rel_alt", Float64, self.callback)

        self.pub.publish(roomba_locations)
        rospy.loginfo (roomba_locations)


if __name__ == '__main__':
    # Initiate the node
    rospy.init_node('roomba_tracking', anonymous=True)
    try:
        rt = roomba_tracking()
    except rospy.ROSInterruptException:
        pass