import scipy
from scipy.integrate import cumulative_trapezoid
import numpy as np
from collections import deque
import time as time_module
import random

senaste_tid = deque(maxlen=10)#makes it so that only the 10 latest error is in the aray
senaste_error = deque(maxlen=10)

start_t = time_module.time()

while True:
    nu_tid = time_module.time() - start_t
    num = random.randint(1, 10)#should be error

    senaste_tid.append(nu_tid)
    senaste_error.append(num)#num should be error
    #puts the latest error in senaste_error aray

    time_array = np.array(senaste_tid)
    error_array = np.array(senaste_error)

    if len(time_array) > 1:
        integral = cumulative_trapezoid(error_array, time_array, initial=0)#calculates integral

        print(integral)#prints the latest in the array
    
    time_module.sleep(1)
