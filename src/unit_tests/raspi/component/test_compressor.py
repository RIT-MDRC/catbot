import unittest

from component.compressor import compressor
from state_management.utils import FakeDigitalOutputDevice


class compressor_unit_tests(unittest.TestCase):

    def test_turn_compressor_on_Always_SetsValueToOne(self):
        test_device = FakeDigitalOutputDevice(0, 0)

        compressor.turn_compressor_on(test_device)
        results = test_device.value

        self.assertEqual(results, 1)


    def test_turn_compressor_off_Always_SetsValueToZero(self):
        test_device = FakeDigitalOutputDevice(0, 1)

        compressor.turn_compressor_off(test_device)
        results = test_device.value

        self.assertEqual(results, 0)


    def test_turn_valve_WhenStateIsTrue_SetsValueToOne(self):
        test_device = FakeDigitalOutputDevice(0, 0)

        compressor.turn_valve(test_device, True)
        results = test_device.value

        self.assertEqual(results, 1)


    def test_turn_valve_WhenStateIsFalse_SetsValueToZero(self):
        test_device = FakeDigitalOutputDevice(0, 1)

        compressor.turn_valve(test_device, False)
        results = test_device.value

        self.assertEqual(results, 0)


    def test_toggle_valve_WhenValueIsOne_SetsValueToZero(self):
        test_device = FakeDigitalOutputDevice(0, 1)

        compressor.toggle_valve(test_device)
        results = test_device.value

        self.assertEqual(results, 0)


    def test_toggle_valve_WhenValueIsZero_SetsValueToOne(self):
        test_device = FakeDigitalOutputDevice(0, 0)

        compressor.toggle_valve(test_device)
        results = test_device.value

        self.assertEqual(results, 1)


    def test_get_valve_state_Always_GetsValue(self):
        test_device = FakeDigitalOutputDevice(0, 1)

        results = compressor.get_valve_state(test_device)

        self.assertEqual(results, 1)


if __name__ == "__main__":
    unittest.main()
