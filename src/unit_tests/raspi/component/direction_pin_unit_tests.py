import unittest
from ....raspi.component.motor.pin import direction_pin
from ....raspi.state_management.utils import FakeDigitalOutputDevice

class adc_unit_tests(unittest.TestCase):

    def test_check_direction_WhenInvalidDirection_ThrowsError(self):
        with self.assertRaises(ValueError):
            direction_pin.check_direction(FakeDigitalOutputDevice(0, 0), -1)

    
    def test_check_direction_WhenSameDirection_ReturnsTrue(self):
        results = direction_pin.check_direction(FakeDigitalOutputDevice(0, 0), 0)
        self.assertTrue(results)

if __name__ == '__main__':
    unittest.main()