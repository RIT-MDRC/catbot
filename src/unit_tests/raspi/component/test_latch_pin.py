import unittest

from component.latch.pin import pin
from state_management.utils import FakeDigitalOutputDevice


class latch_pin_unit_tests(unittest.TestCase):

    def test_set_addr_WhenStateGreaterThanZero_SetValueToOne(self):
        test_state = 1
        test_device = FakeDigitalOutputDevice(0, 0)
        pin.set_addr(test_device, test_state)
        self.assertEqual(test_device.value, 1)

    def test_set_addr_WhenStateLessThanOrEqualToZero_SetValueToZero(self):
        test_state = 0
        test_device = FakeDigitalOutputDevice(0, 1)
        pin.set_addr(test_device, test_state)
        self.assertEqual(test_device.value, 0)

    def test_get_addr_Always_GetsValue(self):
        test_value = 1
        results = pin.get_addr(FakeDigitalOutputDevice(0, test_value))
        self.assertEqual(results, test_value)


if __name__ == "__main__":
    unittest.main()
