import time
class PID:
    def __init__(self, P, I, D):
        self.P = P
        self.I = I
        self.D = D
        self.previous_error = 0
        self.integral = 0
        self.old_time = time.time()
        self.goal = 987


    def run(self, goal, current):
        dt = time.time() - self.old_time
        self.old_time = time.time()
        error = goal - current

        # Sets the  integral
        self.integral += error * dt
        # Sets the derivative
        derivative = (error - self.previous_error) / dt
        if not self.goal == goal:
            derivative = 0
            self.goal = goal

        # Adjusts the output by the constants P, I, and D
        print(str(error) + '  |  ' + str(derivative ) + '  |  ' + str(self.integral))
        output = ( self.P * error ) + ( self.I * self.integral ) + ( self.D * derivative )

        # Sets the previous error
        self.previous_error = error

        return output