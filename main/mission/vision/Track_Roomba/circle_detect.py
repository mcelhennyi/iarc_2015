#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Vector3Stamped


class CircleDetect():

    def __init__(self):

        self.scaledown = 1.3
        self.cap = cv2.VideoCapture(0)
        self.width = 640
        self.height = 480
        self.size = (int(self.width/self.scaledown), int(self.height/self.scaledown))
        # Create a publisher for acceleration data
        self.pub = rospy.Publisher('/roomba/location/pixel', Vector3Stamped, queue_size=10)
        self.rate = rospy.Rate(10)  # 10hz

        self.location_new = Vector3Stamped()
        self.location_old = Vector3Stamped()

        self.loop_search()

    def loop_search(self):
        while not rospy.is_shutdown():
            # read frame from capture
            ret, img = self.cap.read()

            # img = cv2.resize(img, self.size)

            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            output = img.copy()

            # detect circles in the image
            circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 20,
                                       param1=50, param2=30, minRadius=0, maxRadius=0)

            # ensure at least some circles were found
            if circles is not None:
                # convert the (x, y) coordinates and radius of the circles to integers
                circles = np.uint16(np.around(circles))

                # loop over the (x, y) coordinates and radius of the circles
                for i in circles[0, :]:
                    # draw the circle in the output image, then draw a rectangle
                    # corresponding to the center of the circle
                    cv2.circle(output, (i[0], i[1]), i[2], (0, 255, 0), 4)  # i[n] n is from 0 to 2 same as x, y, r
                    x = i[0]
                    y = i[1]
                    r = i[2]
                    cv2.rectangle(output, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
                    self.location_new.vector.x = x
                    self.location_new.vector.y = y
                    self.location_new.vector.z = 0
            
            self.pub.publish(self.location_new)  # Vector3Stamped type variable
            rospy.loginfo(self.location_new)
            self.rate.sleep()
            ## show the output image
            #cv2.imshow("output", np.hstack([img, output]))

            ## exit condition to leave the loop
            #k = cv2.waitKey(30) & 0xff
            #if k == 27:
            #    break

        #cv2.destroyAllWindows()
        #self.cap.release()

            # if self.location_new.vector.x != self.location_old.vector.x and \
            #    self.location_new.vector.y != self.location_old.vector.y:
            #     self.pub.publish(self.location_new)  # Vector3Stamped type variable
            #
            # self.location_old = self.location_new

            # OR


if __name__ == '__main__':
    # Initiate the node
    rospy.init_node('Circle_Detect', anonymous=True)
    try:
        circle = CircleDetect()
    except rospy.ROSInterruptException:
        pass
