import unittest
from ....raspi.component.motor import motor

class adc_unit_tests(unittest.TestCase):

    def test_MotorController_constructor(self):
        results = motor.MotorController(0, 0, 0, [0])
        self.assertEqual(results.pwm_pin, 0)


if __name__ == '__main__':
    unittest.main()