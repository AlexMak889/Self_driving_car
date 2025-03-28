from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import numpy as np
import math
from time import sleep
import time
import RPi.GPIO as GPIO 
import scipy as sp
from scipy.integrate import cumulative_trapezoid
from collections import deque
import time as time_module
import os


def canny(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    canny = cv2.Canny(blur, 100, 200)
    return canny

def region_of_interest(canny):
    height, width = canny.shape
    mask = np.zeros_like(canny)
    
    polygon = np.array([[
        (0, height), 
        (width, height),
        (width, height // 2), 
        (0, height // 2)
    ]], np.int32)
    
    cv2.fillPoly(mask, polygon, 255)
    masked_image = cv2.bitwise_and(canny, mask)
    
    return masked_image

def detect_line_segments(canny_masked):
    rho = 1  
    angle = np.pi / 180  
    min_threshold = 50  
    line_segments = cv2.HoughLinesP(canny_masked, rho, angle, min_threshold, np.array([]), minLineLength=50, maxLineGap=20)
    return line_segments

def average_slope_intercept(frame, line_segments):
    #This function combines line segments into one or two lane lines
    lane_lines = []

    height, width, _ = frame.shape
    left_fit = []
    right_fit = []

    #bounds the line to left or right side of screen
    boundary = 1/3
    left_region_boundary = width * (1 - boundary) 
    right_region_boundary = width * boundary 
    
    if line_segments is None:
        return lane_lines

    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                continue
            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = fit[0]
            intercept = fit[1]
            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit.append((slope, intercept))
            else:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_fit.append((slope, intercept))

    if len(left_fit) > 0:
        left_fit_average = np.average(left_fit, axis=0)#findes the average
        lane_lines.append(make_points(frame, left_fit_average)) #displays the lime


    if len(right_fit) > 0:
        right_fit_average = np.average(right_fit, axis=0)
        lane_lines.append(make_points(frame, right_fit_average))
    return lane_lines

def make_points(frame, line):
    height, width, _ = frame.shape
    slope, intercept = line
    y1 = height  # bottom of the frame
    y2 = int(y1 * 1 / 2)  # make points from middle of the frame down

    x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
    x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
    return [x1, y1, x2, y2]

def draw_lines(frame, lines):
#draws the 2 lane lines
    line_image = np.zeros_like(frame)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line
            cv2.line(line_image, (x1, y1), (x2, y2), (0, 255, 0), 5)
    combined_image = cv2.addWeighted(frame, 1, line_image, 0.8, 0)
    return combined_image

def draw_middle_line(frame, lane_lines):
    # Check if there are exactly two lane lines (left and right)
    if len(lane_lines) == 2:
        left_line = lane_lines[0]
        right_line = lane_lines[1]
        
        # Calculate middel points at the top and bottom of the frame
        x1_mid = (left_line[0] + right_line[0]) // 2
        y1_mid = (left_line[1] + right_line[1]) // 2
        x2_mid = (left_line[2] + right_line[2]) // 2
        y2_mid = (left_line[3] + right_line[3]) // 2
        
        # Draw the middle line
        cv2.line(frame, (x1_mid, y1_mid), (x2_mid, y2_mid), (255, 0, 0), 5)  # Blue line for the middle

    return frame

def angel(x1, y1, x2, y2):
    width = 640
    height = 480

    # calculates the coordinates of the midel line
    # Bottom coordinates
    x_bottom = width // 2
    y_bottom = height
    bottom_point = (x_bottom, y_bottom) 

    # Top coordinates
    x_top = width // 2
    y_top = height // 2
    top_point = (x_top, y_top)


    # midel_screen vector and midel_road vector
    midel_screen = np.array([x_top - x_bottom, y_top - y_bottom])
    midel_road_vector = np.array([x2 - x1, y2 - y1])

    # calculate the dot product
    dot_product = np.dot(midel_screen, midel_road_vector)

    # calculate the magnitude of the vectors
    magnitude_midel_screen = np.linalg.norm(midel_screen)
    magnitude_midel_road_vector = np.linalg.norm(midel_road_vector)

    # calculate the cosine of the angle
    cos_angle = dot_product / (magnitude_midel_screen * magnitude_midel_road_vector)
    cos_angle = np.clip(cos_angle, -1.0, 1.0)  # Clamp to valid range for acos  
    angel = math.degrees(math.acos(cos_angle))

    return angel

class pid_controller:
    def __init__(self, Kp, Ki, Kd, setpoint):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.previous_error = 0
        self.integral = 0
        self.senaste_tid = deque(maxlen=10)
        self.prev_time = 0
        self.current_time = 0

    def calculate_pid(self, current_angle, dt):
        error = self.setpoint - current_angle
        self.current_time = time.time()
        dt = self.current_time - self.prev_time if self.prev_time > 0 else 0.3 
        p = self.Kp * error
       
        self.integral += error*dt
        i = self.Ki * self.integral 

        derivative = (error - self.previous_error) / dt if dt > 0 else 0
        d = self.Kd * derivative

        self.previous_error = error
        self.prev_time = self.current_time

        result = p + i + d
        #print(result, "pid")
        return result


def update_servo_angle(control_signal):
    new_angle = (control_signal + 1) * 90  # Rescale control signal to 0 to 180] degrees
    print(new_angle, "before")
    new_angle = max(0, min(new_angle, 180))  # Clamp the angle to the valid servo range
    print(new_angle, "after")
    return new_angle

def move_servo(new_angle):
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    servo_pin = 18
    GPIO.setup(servo_pin, GPIO.OUT)
    pwm = GPIO.PWM(servo_pin, 50)  # 50Hz frequency
    pwm.start(0)
    print("start")

    try:
        while True:
            pwm.ChangeDutyCycle(new_angle)  # Move to 0 degrees
            print(f"{new_angle} degres")
            time.sleep(1)
    except KeyboardInterrupt:
        pass

    finally:
        pwm.stop()
        GPIO.cleanup()

camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))

# Initialize PID controller
Kp = 0.1
Ki = 0.01
Kd = 0.01

setpoint = 0  # Desired angle 
pid = pid_controller (Kp, Ki, Kd, setpoint)

# Setup GPIO for servo
servoPIN = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(servoPIN, GPIO.OUT)
servo = GPIO.PWM(servoPIN, 50) 
servo.start(7.5) 

previous_time = time_module.time()

for frame_data in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    frame = frame_data.array
    edges = canny(frame)
    mask = region_of_interest(edges)
    line_segments = detect_line_segments(mask)
    lane_lines = average_slope_intercept(frame, line_segments)
    line_image = draw_lines(frame, lane_lines)
    line_image = draw_middle_line(line_image, lane_lines)
    if lane_lines:
        for line in lane_lines:
            x1, y1, x2, y2 = line
            error = angel(x1, y1, x2, y2)
            print("Angle error:", error)
    
            # Calculate dt for pid controller
            current_time = time_module.time()
            dt = current_time - previous_time
            previous_time = current_time

            # Calculate PID control signal
            control_signal = pid.calculate_pid(error, dt)
            print(control_signal, "signal")
            # Update servo angle
            new_angle = update_servo_angle(control_signal)
            duty_cycle = (new_angle / 18) + 2.5  # Map angle to duty cycle for the servoi
            servo.ChangeDutyCycle(duty_cycle)
    move_servo(new_angle)
    cv2.imshow('Lane Lines', line_image)

    rawCapture.truncate(0) 

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
servo.stop()
GPIO.cleanup()
camera.close()
cv2.destroyAllWindows()
