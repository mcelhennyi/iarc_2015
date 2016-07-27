# enumerate test
import rospy
#import __builtin__
from geometry_msgs.msg import PoseArray
#from std_msgs.msg import String

class enumberateTest():

    def __init__(self):
        pub = rospy.Publisher("/roomba/enumberateTest", PoseArray, queue_size=10)
        self.rate = rospy.Rate(10)  # 10hz
        pose = ['Spring', 'Summer', 'Fall', 'Winter']
        self.header.seq += 1
        self.header.stamp = rospy.get_time()
        while not rospy.is_shutdown():
            list1 = list ( enumerate(pose))


            roomba_locations = PoseArray( self.header,self.pose)

            pub.publish(list1)  # PoseArray type variable
            rospy.loginfo (list1)


if __name__ == '__main__':
    # Initiate the node
    rospy.init_node('enumberateTest', anonymous=True)
    try:
        et = enumberateTest()
    except rospy.ROSInterruptException:
        pass