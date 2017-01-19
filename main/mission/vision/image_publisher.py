import rospy
import cv2

camera = cv2.VideoCapture(0)

class take_picture()
    def__init___(self):

        self.rate = rospy.Rate(10)
        self.loop

    def loop(self):
        main(self)

    def main(self):
        self.picture = self.get_image()

    def get_image(self):
        retval, im = camera.read()
        return im


if __name__ == '__main__':
    rospy.init_node('image_saver', anonymous = True)
    try:
        list = llist_Finder()
    except rospy.ROSInterruptException:
        pass
