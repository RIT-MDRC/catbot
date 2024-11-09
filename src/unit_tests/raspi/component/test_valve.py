import unittest

from component.muscle.pneumatics.valve import valve
from state_management.utils import FakeDigitalOutputDevice


class valve_unit_tests(unittest.TestCase):

    def test_turn_valve_on_Always_SetsValueToOne(self):
        test_device = FakeDigitalOutputDevice(0, 0)

        valve.turn_valve_on(test_device)
        results = test_device.value

        self.assertEqual(results, 1)


    def test_turn_valve_off_Always_SetsValueToZero(self):
        test_device = FakeDigitalOutputDevice(0, 1)

        valve.turn_valve_off(test_device)
        results = test_device.value

        self.assertEqual(results, 0)


    def test_turn_valve_WhenStateIsTrue_TurnsValveOn(self):
        test_device = FakeDigitalOutputDevice(0, 0)

        valve.turn_valve(test_device, True)
        results = test_device.value

        self.assertEqual(results, 1)


    def test_turn_valve_WhenStateIsFalse_TurnsValveOff(self):
        test_device = FakeDigitalOutputDevice(0, 1)

        valve.turn_valve(test_device, False)
        results = test_device.value

        self.assertEqual(results, 0)

    
    def test_toggle_valve_WhenValveIsOn_TurnsValveOff(self):
        test_device = FakeDigitalOutputDevice(0, 1)

        valve.toggle_valve(test_device)
        results = test_device.value

        self.assertEqual(results, 0)


    def test_toggle_valve_WhenValveIsOff_TurnsValveOn(self):
        test_device = FakeDigitalOutputDevice(0, 0)

        valve.toggle_valve(test_device)
        results = test_device.value

        self.assertEqual(results, 1)


    def test_get_valve_state_WhenValveIsOff_ReturnsFalse(self):
        test_device = FakeDigitalOutputDevice(0, 0)

        results = valve.get_valve_state(test_device)

        self.assertFalse(results)


    def test_get_valve_state_WhenValveIsOn_ReturnsTrue(self):
        test_device = FakeDigitalOutputDevice(0, 1)

        results = valve.get_valve_state(test_device)

        self.assertTrue(results)


if __name__ == "__main__":
    unittest.main()
