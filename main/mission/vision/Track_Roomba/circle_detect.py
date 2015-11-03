#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Vector3Stamped


class CircleDetect():

    def __init__(self):
        # Create a publisher for acceleration data
        self.pub = rospy.Publisher('/roomba/location/pixel', Vector3Stamped, queue_size=10)
        self.rate = rospy.Rate(10)  # 10hz

        self.location_new = Vector3Stamped()
        self.location_old = Vector3Stamped()
        self.cap = cv2.VideoCapture(0)
        self.version = cv2.__version__.split('.')
        if self.version[0] is '2' and self.version[1] is '4':
            # is 2.4.X
            self.cv_version = 2
        if self.version[0] is '3':
            # is 3.0.0
            self.cv_version = 3
        self.loop_search()

    def loop_search(self):
        while not rospy.is_shutdown():
            # version of opencv is 3.X.X
            if self.cv_version is 3:
                # read frame from capture
                ret, img = self.cap.read()

                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                output = img.copy()

                # Gaussian Blur
                gaussian_blur = cv2.GaussianBlur(gray, (9,9), 0)

                # detect circles in the image
                circles = cv2.HoughCircles(gaussian_blur, cv2.cv.CV_HOUGH_GRADIENT, 3, 100,
                        minRadius=170, maxRadius=180)

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

            # version is 2.4.X
            if self.cv_version is 2:

                # read frame from capture
                ret, img = self.cap.read(cv2.CV_LOAD_IMAGE_GRAYSCALE)

                #gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                output = img.copy()

                # Gaussian Blur
                gaussian_blur = cv2.GaussianBlur(img, (9,9), 0)

                # detect circles in the image
                circles = cv2.HoughCircles(gaussian_blur, cv2.cv.CV_HOUGH_GRADIENT, 3, 100,
                        minRadius=170, maxRadius=180)

                # ensure at least some circles were found
                if circles is not None:
                    circles = np.round(circles[0, :]).astype("int")
                    # loop over the (x, y) coordinates and radius of the circles
                    for (x, y, r) in circles:
                        cv2.circle(output, (x, y), r, (0, 255, 0), 4)
                        cv2.rectangle(output, (x-5, y-5), (x+5, y+5), (0, 128, 255), -1)
                        self.location_new.vector.x = x
                        self.location_new.vector.y = y
                        self.location_new.vector.z = 0

            ###############################################################
            ##############################Publisher########################
            ###############################################################
            self.pub.publish(self.location_new)  # Vector3Stamped type variable
            rospy.loginfo(self.location_new)
            self.rate.sleep()

            #########################
            # show the output image #
            #########################
            cv2.imshow("output", np.hstack([img, output]))
            # exit condition to leave the loop
            k = cv2.waitKey(30) & 0xff
            if k == 27:
                break
        cv2.destroyAllWindows()
        self.cap.release()


if __name__ == '__main__':
    # Initiate the node
    rospy.init_node('Circle_Detect', anonymous=True)
    try:
        circle = CircleDetect()
    except rospy.ROSInterruptException:
        pass
