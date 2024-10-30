import unittest
from ....raspi.component.motor.pin import direction_pin
from ....raspi.state_management.utils import FakeDigitalOutputDevice

class direction_pin_unit_tests(unittest.TestCase):

    def test_check_direction_WhenInvalidDirection_ThrowsError(self):
        with self.assertRaises(ValueError):
            direction_pin.check_direction(FakeDigitalOutputDevice(0, 0), -1)

    
    def test_check_direction_WhenSameDirection_ReturnsTrue(self):
        results = direction_pin.check_direction(FakeDigitalOutputDevice(0, 0), 0)
        self.assertTrue(results)


    def test_check_direction_WhenDifferentDirection_ReturnsFalse(self):
        results = direction_pin.check_direction(FakeDigitalOutputDevice(0, 0), 1)
        self.assertFalse(results)

    
    def test_set_direction_WhenInvalidDirection_ThrowsError(self):
        with self.assertRaises(ValueError):
            direction_pin.set_direction(FakeDigitalOutputDevice(0, 0), -1)

    
    def test_set_direction_WhenValidDirection_SetsDirection(self):
        test_pin = FakeDigitalOutputDevice(0, 0)
        results = direction_pin.set_direction(test_pin, 1)
        self.assertTrue(results)
        self.assertEqual(test_pin.value, 1)


    def test_get_direction_WhenDirection_GetsDirection(self):
        test_pin = FakeDigitalOutputDevice(0, 0)
        results = direction_pin.get_direction(test_pin)
        self.assertEqual(results, 0)


if __name__ == '__main__':
    unittest.main()