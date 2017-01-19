import time
# from geometry_msgs.msg import PoseArray

# Global variables go here
time_global = 1


class Template:

    def __init__(self):
        # Self type variables go here
        self.time_self = time.time()

        # Run
        self.loop()

    def loop(self):
        self.print_variables(time_global, self.time_self)

    def print_variables(self, var1, var2):
        print var1
        print var2

if __name__ == '__main__':
    temp = Template()
    print "Done"
