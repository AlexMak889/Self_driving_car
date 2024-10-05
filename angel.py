import cv2
import math
import numpy as np
import Self_driving_car.computer_vision as computer_vision

radians = 0.5
pi = 3.14159265
degrees = radians * (180.0 / pi)

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

points = make_points(frame, line)
if len(points) > 0:
    x1, y1, x2, y2 = points[0]



def calculate_angle():
# to get the angel you use this formula:
#cos = dot product of 2 vectors devided by the magnitude of the 2 vectors: cos= a*b / ǀaǀǀbǀ

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

    # calculate the angle in degres
    angle = math.degrees(math.acos(cos_angle))
    return angle

# Calculate the angle
angle = calculate_angle()
