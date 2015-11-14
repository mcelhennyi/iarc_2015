#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Vector3Stamped
import math

class CircleDetect():

    def __init__(self):
        # Create a publisher for acceleration data
        self.pub = rospy.Publisher('/roomba/location/pixel', Vector3Stamped,
            queue_size=10)
        self.rate = rospy.Rate(10)  # 10hz
        # For finding radius
        self.a = 391.3
        self.b = -0.0238
        self.radius = 0
        self.alt = 32  # NEEDS TO CHANGE FROM HARD CODE TO SUBSCRIPTION
        self.location_new = Vector3Stamped()
        self.location_old = Vector3Stamped()
        self.cap = cv2.VideoCapture(0)
        self.loop_search()

    def loop_search(self):
        while not rospy.is_shutdown():
            # read frame from capture
            ret, img = self.cap.read()

            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            output = img.copy()

            # Gaussian Blur
            gaussian_blur = cv2.GaussianBlur(gray, (9, 9), 0)

            # Get the radius range based off of altitude (exponential fit)
            self.radius = int(self.a * math.exp(self.alt * self.b))

            # detect circles in the image
            circles = cv2.HoughCircles(gaussian_blur, cv2.cv.CV_HOUGH_GRADIENT,
                3, 100, minRadius=self.radius - 5, maxRadius=self.radius + 5)

            # ensure at least some circles were found
            if circles is not None:
                circles = np.round(circles[0, :]).astype("int")
                # loop over the (x, y) coordinates and radius of the circles
                for (x, y, r) in circles:
                    cv2.circle(output, (x, y), r, (0, 255, 0), 4)
                    cv2.rectangle(output, (x - 5, y - 5), (x + 5, y + 5),
                        (0, 128, 255), -1)
                    self.location_new.vector.x = x
                    self.location_new.vector.y = y
                    self.location_new.vector.z = 0
                    print r

            #########################
            # show the output image #
            #########################
            cv2.imshow("output", np.hstack([img, output]))
            # exit condition to leave the loop
            k = cv2.waitKey(30) & 0xff
            if k == 27:
                break

            ###############################################################
            ##############################Publisher########################
            ###############################################################
            self.pub.publish(self.location_new)  # Vector3Stamped type variable
            #rospy.loginfo(self.location_new)
            self.rate.sleep()


        cv2.destroyAllWindows()
        self.cap.release()


if __name__ == '__main__':
    # Initiate the node
    rospy.init_node('Circle_Detect', anonymous=True)
    try:
        circle = CircleDetect()
    except rospy.ROSInterruptException:
        pass
