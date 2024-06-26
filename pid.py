import RPi.GPIO as GPIO 
from time import sleep

GPIO.setmode(GPIO.BCM)
#Pins 18 22 24 GPIO 24 25 8
Motor1E = 24 #  Enable pin 1 of the controller IC
Motor1A = 25 #  Input 1 of the controller IC 
Motor1B = 8 #  Input 2 of the controller IC


GPIO.setup(Motor1A,GPIO.OUT)
GPIO.setup(Motor1B,GPIO.OUT)
GPIO.setup(Motor1E,GPIO.OUT)

class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.prev_error = 0
        self.integral = 0
    
    def compute(self, current_speed):
        error = self.setpoint - current_speed
        self.integral += error
        derivative = error - self.prev_error
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return output

def update_motor_speed(speed):
    # Update motor speed here
    # You can modify your existing code to set the motor speed based on the input 'speed'
    pass

if __name__ == '__main__': 
    # Constants for PID controller
    Kp = 0.1
    Ki = 0.01
    Kd = 0.01
    setpoint = 50  # Desired speed
    
    # Initialize PID controller
    pid = PIDController(Kp, Ki, Kd, setpoint)
    
    # Dummy current speed (replace with actual motor speed reading)
    current_speed = 2
    
    # Run the control loop
    try:
        while True:
            # Compute control signal
            control_signal = pid.compute(current_speed)

            # Update motor speed
            update_motor_speed(control_signal)

            # Delay for a short period (adjust as needed)
            sleep(0.1)
    except KeyboardInterrupt:
        # Clean up GPIO on keyboard interrupt
        GPIO.cleanup()