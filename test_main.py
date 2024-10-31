import unittest
import math
from main_program_test import angel
from main_program_test import pid_controller as pid

class Test_angel(unittest.TestCase):
    def test_angle(self):
        self.assertAlmostEqual(angel(320, 480, 320, 240), 0, places=1)#test vertikal line
        self.assertAlmostEqual(angel(320, 480, 480, 320), 45, places=1)#test 45 degrea angel
        self.assertAlmostEqual(angel(320, 480, 160, 320), -45, places=1)#test -45 degrea angel

class Test_pid(unittest.TestCase):
    def setUp(self):
        self.kp = 0.1
        self.ki = 0.01
        self.kd = 0.01
        self.setpoint = 0
        self.pid = pid(self.kp, self.ki, self.kd, self.setpoint)
        self.dt = 0.1

    def zero_error(self):
        current_angel = self.setpoint

        output = self.pid.calculate_pid(current_angel, self.dt)
        self.assertAlmostEqual(output, 0, places=5)


if __name__ == '__main__':
    unittest.main()
