import cv2
import rospy
from geometry_msgs.msg import Vector3Stamped
import math


# pixel_distance = location_pixel - center_pixel
# camera_c = sqrt(camera_a^2 + camera_b^2)
# angle_per_pixel = camera_view_angle / camera_c
# angle_to_object = angle_per_pixel * pixel_distance
# distance = tan(angle_to_object) * height
# if distance changes too fast or jumps around don't publish

class ground_distance:

    def __init__(self):
        # Create a publisher for acceleration data
        self.pub = rospy.Publisher('/roomba/location', Vector3Stamped, queue_size=10)
        # set the rate of ros loop
        self.rate = rospy.Rate(10)  # 10hz
        self.xy_roomba = Vector3Stamped()
        self.xy_roomba = Vector3Stamped()

        self.camera_size_x = 600
        self.camera_size_y = 400
        self.camera_view_angle = 60  # In degrees

        self.location_pixel_x = 0  # subscribe for this value
        self.location_pixel_y = 0  # subscribe for this value
        self.height = 3  # subscribe for this value

        self.center_pixel_x = self.camera_size_x / 2
        self.center_pixel_y = self.camera_size_y / 2

    def loop_roomba_location(self):
        # while the node is still running loop
        while not rospy.is_shutdown():
            # create and subscribe to the message /roomba/location
            # rospy.Subscriber("/roomba/location", Vector3Stamped, callback="calculate",
            #                  callback_args= )
            # publish the the x and y distances to mavros
            self.pub.publish(self.xyz_roomba)  # Vector3Stamped type variable
            # sleep the ros rate
            self.rate.sleep()

    def calculate(self):
        pixel_distance_x = self.location_pixel_x - self.center_pixel_x
        pixel_distance_y = self.location_pixel_y - self.center_pixel_y
        camera_size_diagonal = math.sqrt(math.pow(self.camera_size_x, 2) + math.pow(self.camera_size_y, 2))
        angle_per_pixel = self.camera_view_angle / camera_size_diagonal
        angle_to_object_x = angle_per_pixel * pixel_distance_x
        angle_to_object_y = angle_per_pixel * pixel_distance_y

        self.xy_roomba.vector.x = math.tan(angle_to_object_x) * self.height  # publish
        self.xy_roomba.vector.y = math.tan(angle_to_object_y) * self.height  # publish
        rospy.loginfo(self.xyz_roomba)

if __name__ == '__main__':
     # Initiate the node
    rospy.init_node('PID_main', anonymous=True)
    try:
        pid = ground_distance()
    except rospy.ROSInterruptException:
        pass
