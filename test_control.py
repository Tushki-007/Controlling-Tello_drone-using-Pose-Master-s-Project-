"""
@brief      This is Control module unit test file
"""

import unittest

from Control import Control

control = Control(-100, 100)

class TestControlMethods(unittest.TestCase):

    def test_calculate_pid_p_only(self):
        control.update_setpoint(50)
        control.update_gain(1.0, 0.0, 0.0)

        pv = 0
        pid = 0
        for i in range(0, 3):
            pid = control.calculate_pid(pv, invert_output=True)
            pv += 10

        self.assertEqual(pid, 30)
        control.reset()
    
    def test_calculate_pid_pd_only(self):
        control.update_setpoint(50)
        control.update_gain(1.0, 0.0, 0.01)

        pv = 0
        pid = control.calculate_pid(pv, invert_output=True)

        self.assertEqual(pid, 55)
        control.reset()
    
    def test_clamp_output(self):
        self.assertEqual(control.clamp_output(200, -100, 100), 100)


if __name__ == '__main__':
    unittest.main()