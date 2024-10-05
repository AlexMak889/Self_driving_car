import RPi.GPIO as GPIO 
from time import sleep
import scipy as sp
from scipy.integrate import cumulative_trapezoid
from collections import deque
import time as time_module
import numpy as np


def pid():
    class pid_controller:
        def __init__(self, Kp, Ki, Kd, setpoint):
            self.Kp = Kp
            self.Ki = Ki
            self.Kd = Kd
            self.setpoint = setpoint
            self.previous_error = 0
            self.integral = 0
        
        def calculate_pid(self, current_angel, dt, ):
            senaste_tid = deque(maxlen=10)#makes it so that only the 10 latest error is in the aray
            senaste_error = deque(maxlen=10)

            start_t = time_module.time()
            nu_tid = time_module.time() - start_t
            error = self.setpoint - current_angel
            

            senaste_tid.append(nu_tid)
            senaste_error.append(error)
            #puts the latest error in senaste_error aray

            time_array = np.array(senaste_tid)
            error_array = np.array(senaste_error)

            p = self.kp * error
            
            integral = cumulative_trapezoid(error_array, time_array, initial=0)#calculates integral
            i = self.ki * self.integral  

            derivitive = (error - self.previous_error) / dt
            d = self.kd * derivitive

            self.previous_error = error #previos error

            resoult =  p + i + d
            return resoult

    def update_servo_angel(control_signal):
        
        angle = (control_signal + 1) * 90 
        angle = max(0, min(angle, 180))
        return angle

    if __name__ == '__main__': 
        #gains
        Kp = 0.1
        Ki = 0.01
        Kd = 0.01

        setpoint = 0  # Desired angel
        
        dt = 0.1
        pid = pid_controller(Kp, Ki, Kd, setpoint)
        current_angel = 0
        time = []
        

        servoPIN = 17
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(servoPIN, GPIO.OUT)
        a = GPIO.PWM(servoPIN, 50) 
        a.start(2.5)

        try:
            while True:            
                control_signal = pid.calculate_pid(current_angel, dt)

                # Update servo angel
                update_servo_angel(control_signal)
                angle = update_servo_angel(control_signal)

                a.ChangeDutyCycle(angle)
                sleep(0.1)
        except KeyboardInterrupt:
            GPIO.cleanup()  
            a.stop()
    return pid