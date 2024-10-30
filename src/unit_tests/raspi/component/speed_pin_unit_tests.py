import unittest
from ....raspi.component.motor.pin import speed_pin
from ....raspi.state_management.utils import FakePWMOutputDevice

class speed_pin_unit_tests(unittest.TestCase):

    def test_get_WhenValue_GetsValue(self):
        test_value = 1.0
        results = speed_pin.get(FakePWMOutputDevice(0, test_value))
        self.assertEqual(results, test_value)

    
    def test_set_WhenPositiveValue_SetsValue(self):
        test_pin = FakePWMOutputDevice(0)
        test_value_positive = 1.0
        speed_pin.set(test_pin, test_value_positive)
        self.assertEqual(test_pin.value, test_value_positive)
        

    def test_set_WhenNegativeValue_SetsAbsoluteValue(self):
        test_pin = FakePWMOutputDevice(0)
        test_value_positive = 1.0
        test_value_negative = -1.0
        speed_pin.set(test_pin, test_value_negative)
        self.assertEqual(test_pin.value, test_value_positive)

    
    def test_stop_Always_SetsValueToZero(self):
        test_pin = FakePWMOutputDevice(0, 1.0)
        results = speed_pin.stop(test_pin)
        self.assertEqual(results, 0)


    def test_check_speed_WhenSameSpeed_ReturnsTrue(self):
        test_speed = 1.0
        test_pin = FakePWMOutputDevice(0, test_speed)
        results = speed_pin.check_speed(test_pin, test_speed)
        self.assertTrue(results)


    def test_check_speed_WhenDifferentSpeed_ReturnsFalse(self):
        test_speed = 1.0
        test_pin = FakePWMOutputDevice(0, 0.0)
        results = speed_pin.check_speed(test_pin, test_speed)
        self.assertFalse(results)


if __name__ == '__main__':
    unittest.main()