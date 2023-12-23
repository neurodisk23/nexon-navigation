
import os
import numpy as np
class PIDController:
    def __init__(self, Kp, Ki, Kd, rate_min, rate_max):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.rate_min = rate_min
        self.rate_max = rate_max
        self.error =  0
        self.previous_error = 0
        self.integral = 0

    def update(self, desired_position, current_position):
        desired_position = np.array([desired_position])  # if needed
        current_position = np.array([current_position]) 
        self.error = desired_position - current_position
        self.integral += self.error
        derivative = self.error - self.previous_error
        output = self.Kp * self.error + self.Ki * self.integral + self.Kd * derivative
        self.previous_error = self.error
        rate = output
        rate = max(self.rate_min, min(rate, self.rate_max))

        return rate