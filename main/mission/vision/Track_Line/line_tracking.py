#!/usr/bin/env python

#import things for good reason
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int8MultiArray

class LineTracking():

    def __init__(self):
        # Create a subscriber for acceleration data
        rospy.Subscriber('/mavros/imu/data', Imu, self.process_imu_data)

        #declare variable types
        self.line_x_array = Float64MultiArray()
        self.line_y_array = Float64MultiArray()
        self.accel_x_array = Float64MultiArray()
        self.accel_y_array = Float64MultiArray()
        self.hlist = Float64MultiArray()
        self.vlist = Float64MultiArray()
        self.line_output_x = Float64MultiArray()
        self.line_output_y = Float64MultiArray()
        self.current_time = rospy.get_time()

        # set the rate of ros loop
        self.rate = rospy.Rate(10)  # 10hz

        # Create variables for use
        self.x_accel = 0.0
        self.y_accel = 0.0
        self.start_point_X = 0
        self.start_point_Y = 0
        self.x_velocity = 0.0
        self.y_velocity = 0.0
        self.rest_time = .1
        self.accuracy = .1
        self.predict_x_array = []
        self.predict_y_array = []
        #subscribe to start point
        rospy.Subscriber('/start/position', Int8MultiArray, self.get_start())

        #initialize the publisher
        self.X_line_pub = rospy.Publisher('/line_tracking/v_lines_array', Float64MultiArray, queue_size=10)
        self.Y_line_pub = rospy.Publisher('/line_tracking/h_lines_array', Float64MultiArray, queue_size=10)

        #create x and y array and use positions as counter
        self.accel_x_array=[]
        self.accel_y_array=[]
        self.get_start(Int8MultiArray)
        for i in range (21):
            self.accel_x_array.append(float(i)-self.start_point_X)
            self.accel_y_array.append(float(i)-self.start_point_Y)
        loop()
    #create loop for generating the line numbering
    def loop(self):
        while not rospy.is_shutdown():
            #subrscribe to imu data
            rospy.Subscriber('/mavros/imu/data', Imu, self.process_imu_data)

            #subscribe to the line vision
            rospy.Subscriber('/Track_line/Hllist', Float64MultiArray, self.Hlist_callback)
            rospy.Subscriber('/Track_line/Vllist', Float64MultiArray, self.Vlist_callback)

            #run the accel_array loop to generate accelerometer data array
            self.accel_array_x()
            self.accel_array_y()

            #pull array from camera and find the index from the accelerometer data where the drone is check within acceptable range +-.1meter(or may be a percentage)#  **publish boolean for error**
            self.check_array_x()
            self.check_array_y()

            #predict and store the next line_array to come from the camera for comparson to next iteration of the loop
            self.predict_array_x()
            self.predict_array_y()

            self.X_line_pub.publish(self.accel_x_array)
            self.Y_line_pub.publish(self.accel_y_array)

            # sleep the ros rate
            self.rate.sleep()

    def check_predict_array_x(self):
        if len(self.line_x_array) < len(self.predict_x_array):
            #get the length of the self.predict_x_array to determine length of the splice
            self.splice_predict_x_length=len(self.line_x_array)

            #run through the array to determine which line is closest to 0
            for j in range (len(self.predict_x_array)):
                if (self.predict_x_array[j] < .5 and self.predict_x_array[j] > 0.0) or (self.predict_x_array[j] > -.5 and self.predict_x_array[j] < 0.0):
                    self.splice_predict_x_start=j
                    break

            #use the line array and the accel array to generate an error value ad then use that errr to predict the next line array
            self.predict_x_splice=self.predict_x_array[(self.splice_predict_x_start-(self.splice_predict_x_length/2)):(self.splice_predict_x_start+(self.splice_predict_x_length/2))]

            for i in range(self.splice_predict_x_length):
                self.error_value_x_check+=self.predict_x_splice[i]-self.line_x_array[i]
            self.avg_x_error_check=self.error_value_x_check/self.splice_predict_x_length

        elif len(self.line_x_array) > len(self.predict_x_array):
            #get the length of the self.predict_x_array to determine length of the splice
            self.splice_line_x_length=len(self.predict_x_array)

            #run through the array to determine which line is closest to 0
            for j in range (len(self.line_x_array)):
                if (self.line_x_array[j] < .5 and self.line_x_array[j] > 0.0) or (self.line_x_array[j] > -.5 and self.line_x_array[j] < 0.0):
                    self.splice_line_x_start=j
                    break

            #use the line array and the accel array to generate an error value ad then use that errr to predict the next line array
            self.line_x_splice=self.line_x_array[(self.splice_line_x_start-(self.splice_line_x_length/2)):(self.splice_line_x_start+(self.splice_line_x_length/2))]

            for i in range(self.splice_line_x_length):
                self.error_value_x+check+=self.line_x_splice[i]-self.predict_x_array[i]
            self.avg_x_error_check=self.error_value_x_check/self.splice_line_x_length

        else:
            for i in range(len(self.line_x_length)):
                self.error_value_x_check+=self.line_x_array[i]-self.predict_x_array[i]
            self.avg_x_error_check=self.error_value_x_check/len(self.line_x_array)


    def check_predict_array_y(self):
        if len(self.line_y_array) < len(self.predict_y_array):
            #get the length of the self.predict_x_array to determine length of the splice
            self.splice_predict_y_length=len(self.line_y_array)

            #run through the array to determine which line is closest to 0
            for j in range (len(self.predict_y_array)):
                if (self.predict_y_array[j] < .5 and self.predict_y_array[j] > 0.0) or (self.predict_y_array[j] > -.5 and self.predict_y_array[j] < 0.0):
                    self.splice_predict_y_start=j
                    break

            #use the line array and the accel array to generate an error value ad then use that errr to predict the next line array
            self.predict_y_splice=self.predict_y_array[(self.splice_predict_y_start-(self.splice_predict_y_length/2)):(self.splice_predict_y_start+(self.splice_predict_y_length/2))]

            for i in range(self.splice_predict_y_length):
                self.error_value_y_check+=self.predict_y_splice[i]-self.line_y_array[i]
            self.avg_y_error_check=self.error_value_y_check/self.splice_predict_y_length

        elif len(self.line_y_array) > len(self.predict_y_array):
            #get the length of the self.predict_y_array to determine length of the splice
            self.splice_line_y_length=len(self.predict_y_array)

            #run through the array to determine which line is closest to 0
            for j in range (len(self.line_y_array)):
                if (self.line_y_array[j] < .5 and self.line_y_array[j] > 0.0) or (self.line_y_array[j] > -.5 and self.line_y_array[j] < 0.0):
                    self.splice_line_y_start=j
                    break

            #use the line array and the accel array to generate an error value ad then use that errr to predict the next line array
            self.line_y_splice=self.line_y_array[(self.splice_line_y_start-(self.splice_line_y_length/2)):(self.splice_line_y_start+(self.splice_line_y_length/2))]

            for i in range(self.splice_line_y_length):
                self.error_value_y_check+=self.line_y_splice[i]-self.predict_y_array[i]
            self.avg_y_error_check=self.error_value_y_check/self.splice_line_y_length

        else:
            for i in range(len(self.line_y_length)):
                self.error_value_y_check+=self.line_y_array[i]-self.predict_y_array[i]
            self.avg_y_error_check=self.error_value_y_check/len(self.line_y_array)

    def predict_array_x(self):
        #get the length of the self.line_x_array to determine length of the splice
        self.splice_x_length=len(self.line_x_array)

        #run through the array to determine which line is closest to 0
        for j in range (len(self.accel_x_array)):
            if (self.accel_x_array[j] < .5 and self.accel_x_array[j] > 0.0) or (self.accel_x_array[j] > -.5 and self.accel_x_array[j] < 0.0):
                self.splice_x_start=j
                break

        #use the line array and the accel array to generate an error value ad then use that errr to predict the next line array
        self.accel_x_splice=self.accel_x_array[(self.splice_x_start-(self.splice_x_length/2)):(self.splice_x_start+(self.splice_x_length/2))]
        for i in range(self.splice_x_length):
            self.error_value_x+=self.accel_x_splice[i]-self.line_x_array[i]
        self.avg_x_error_value=self.error_value_x/self.splice_x_length
        self.predict_x_array=[]
        for i in range (self.splice_x_length):
            self.predict_x_array.append(self.line_x_array[i]+self.avg_x_error_value)
        return self.predict_x_array
        
    def predict_array_y(self):
        #get the length of the self.line_y_array to determine length of the splice
        self.splice_y_length=len(self.line_y_array)

        #run through the array to determine which line is closest to 0
        for j in range (len(self.accel_y_array)):
            if (self.accel_y_array[j] < .5 and self.accel_y_array[j] > 0.0) or (self.accel_y_array[j] > -.5 and self.accel_y_array[j] < 0.0):
                self.splice_y_start=j
                break

        #use the line array and the accel array to generate an error value ad then use that errr to predict the next line array
        self.accel_y_splice=self.accel_y_array[(self.splice_y_start-(self.splice_y_length/2)):(self.splice_y_start+(self.splice_y_length/2))]
        for i in range(self.splice_y_length):
            self.error_value_y+=self.accel_y_splice[i]-self.line_y_array[i]
        self.avg_y_error_value=self.error_value_y/self.splice_y_length
        self.predict_y_array=[]
        for i in range (self.splice_y_length):
            self.predict_y_array.append(self.line_y_array[i]+self.avg_y_error_value)
        return self.predict_y_array

    def accel_array_x(self):
        #convert accel to speed data from IMU
        self.x_velocity =self.x_velocity+(self.x_accel*self.rest_time)

        #Determine distance moved per cycle of loop
        self.x_dist_moved=self.x_velocity/self.rest_time

        #Recalculate the array based on distance moved
        for i in range (21):
            self.accel_x_array[i]=self.accel_x_array[i]-self.x_dist_moved
            return self.accel_x_array

    def accel_array_y(self):
        self.y_velocity = self.y_velocity+(self.y_accel*self.rest_time)

        #Determine distance moved per cycle of loop
        self.y_dist_moved=self.y_velocity/self.rest_time

        #Recalculate the array based on distance moved
        for i in range (21):
            self.accel_y_array[i]=self.accel_y_array[i]-self.y_dist_moved
            return self.accel_y_array

    def process_imu_data(self, imu):
        self.current_time=imu.header.stamp
        self.x_accel=imu.linear_acceleration.x
        self.y_accel=imu.linear_acceleration.y

    def Hlist_callback(self, hlist):
        self.line_y_array=hlist.data

    def Vlist_callback(self, vlist):
        self.line_x_array=vlist.data

    def get_start(self, Start):
        self.start_point_X = Start.x
        self.start_point_Y = Start.y

    def check_array_x(self):
        for i in range (21):
            if (self.accel_x_array[i] < .5 and self.accel_x_array[i] > 0.0) or (self.accel_x_array[i] > -.5 and self.accel_x_array[i] < 0.0):
                for j in range (len(self.line_x_array)):
                    if (self.line_x_array[j] < .5 and self.line_x_array[j] > 0.0) or (self.line_x_array[j] > -.5 and self.line_x_array[j] < 0.0):
                        if self.accel_x_array[i] >= (self.line_x_array[j]+ self.accuracy) or self.accel_x_array[i] <= (self.line_x_array[j]-self.accuracy):
                            return False
                        else:
                            return True

    def check_array_y(self):
        for i in range (21):
            if self.accel_y_array[i] < .5 and self.accel_y_array[i] > 0.0 or (self.accel_y_array[i] > -.5 and self.accel_y_array[i] < 0.0):
                for j in range (len(self.line_y_array)):
                    if self.line_y_array[j] < .5 and self.line_y_array[j] > 0.0 or (self.line_y_array[j] > -.5 and self.line_y_array[j] < 0.0):
                        if self.accel_y_array[i] >= (self.line_y_array[j]+ self.accuracy) or self.accel_y_array[i] <= (self.line_y_array[j]-self.accuracy):
                            return False
                        else:
                            return True



if __name__ == '__main__':
    # Initiate the node
    rospy.init_node('LineTracking', anonymous=True)
    try:
        lt = LineTracking()
    except rospy.ROSInterruptException:
        pass
