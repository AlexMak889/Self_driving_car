import unittest
import math
from main_t import angel

class Test_angel(unittest.TestCase):
    def test_angle(self):
        self.assertAlmostEqual(angel_t.angel(320, 480, 320, 240), 0, places=1)
        self.assertAlmostEqual(angel_t.angel(320, 480, 420, 240), 45, places=1)
 

if __name__ == '__main__':
    unittest.main()
