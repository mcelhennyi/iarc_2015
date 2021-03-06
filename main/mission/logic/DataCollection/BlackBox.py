#!/usr/bin/env python
import rospy

from geometry_msgs.msg import *
from std_msgs.msg import *
from mavros_msgs.srv import *
from mavros_msgs.msg import *

import time

# Author: JJ Marcus
# Last Edited Date: 8th November 2016

# format: in the main it will write the time then a comma, then each subscriber's call back will
# write it's own data, in the format of "data, data, data," always ending in a comma
# at the end of the main, it will add a new line, and repeate

# keep the call back definitions in the same order as the callback calls

class Blackbox():

    def __init__(self):
<<<<<<< HEAD:main/mission/logic/Data Collection/DataCollector
        rospy.loginfo("init")
        self.rate = rospy.Rate(2)
        self.black_box_file = open(str(time.ctime()), "a"))
=======
        rospy.loginfo("Begin Recodring")
        self.rate = rospy.Rate(10)
        self.black_box_file = open(str(time.ctime()), "a")
>>>>>>> 3802239fab2209b4831feb8bf9f6de9f64f7922f:main/mission/logic/DataCollection/BlackBox.py
        self.main()

    def main(self):
        while not rospy.is_shutdown():

            self.black_box_file.write("\n")

            #self.black_box_file.write(str(time.ctime())+", ")
            rospy.Subscriber("/mavros/state", State, self.state_callback)
            rospy.Subscriber("/mavros/battery", BatteryStatus, self.battery_callback)
            rospy.Subscriber("/mavros/altitude", Altitude, self.altitude_callback)
            #rospy.Subscriber("/mavros/ ______", ______ ,self.imu_callback)

<<<<<<< HEAD:main/mission/logic/Data Collection/DataCollector
            self.black_box_file.write("/n")
            self.black_box_file.write("/n")

=======
>>>>>>> 3802239fab2209b4831feb8bf9f6de9f64f7922f:main/mission/logic/DataCollection/BlackBox.py
            self.rate.sleep()


    def state_callback(self, state):
        self.black_box_file.write((str(state.armed) + ", " + str(state.guided) + ", " + \
                                  state.mode + ", "))

    def battery_callback(self, battery):
        self.black_box_file.write((str(battery.voltage) + ", "))


    def altitude_callback(self, altitude):
        self.black_box_file.write((str(altitude.monotonic) + ", " + str(altitude.amsl) + ", " + \
                                  str(altitude.relative) + ", " + str(altitude.terrain) + ", " + \
                                  str(altitude.bottom_clearance) + ", "))

    def imu_callback(self, imu):
        pass




if __name__ == '__main__':
    # init the node
    rospy.init_node('Blackbox')
    try:
        box = Blackbox()
    except rospy.RosInterruptException:
        pass
