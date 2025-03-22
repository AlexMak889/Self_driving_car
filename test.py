import unittest
import numpy as np
import math
from main_program_test import angel, pid_controller as pid
import time

class Test_angel(unittest.TestCase):
    def test_angels(self):
        self.assertAlmostEqual(angel(320, 480, 320, 240), 0, places=1)#test vertikal line

class Test_pid(unittest.TestCase):
    def setUp(self):
        self.kp = 0.1
        self.ki = 0.01
        self.kd = 0.01
        self.T = 0.033
        self.pid = pid(self.kp, self.ki, self.kd, self.T)

    def test_zero_error(self):
        current_angel = 0 
        
        output = self.pid.calculate_pid(current_angel, self.T)
        self.assertAlmostEqual(output, 0, places=5)

    def test_const_error(self):
        current_angel = 10
        output_values = []
        
        for i in range(20):
            output = self.pid.calculate_pid(current_angel)
            output_values.append(output)
            print(output_values) 
            
        for output in output_values:
                self.assertAlmostEqual(output, 4, delta=0.1)


    def test_step_chane(self):
        current_angel = 0
        time_step_aplied = False

        for step in range(20):
            if step == 9:
                current_angel = 20
                time_step_aplied = True

            output = self.pid.calculate_pid(current_angel, self.T)

            if time_step_aplied:
                self.assertTrue(output > 0)
            #print(output)    
            
            #print(f"Step {step}: Output = {output}")  
if __name__ == '__main__':
    unittest.main()
