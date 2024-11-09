import unittest

from component.muscle.pneumatics.pressure import pressure
from state_management.utils import FakeDigitalInputDevice


class pressure_unit_tests(unittest.TestCase):

    def test_is_pressure_ok_Always_GetsIsActive(self):
        test_device = FakeDigitalInputDevice(0, initial_is_state=True)

        results = pressure.is_pressure_ok(test_device)

        self.assertTrue(results)


    def test_on_pressure_active_Always_SetsWhenActivated(self):
        test_device = FakeDigitalInputDevice(0, initial_is_state=True)
        pressure.on_pressure_active(test_device, lambda: True)

        results = test_device.when_activated()

        self.assertTrue(results)


    def test_on_pressure_deactive_Always_SetsWhenDeactivated(self):
        test_device = FakeDigitalInputDevice(0, initial_is_state=True)
        pressure.on_pressure_deactive(test_device, lambda: True)

        results = test_device.when_deactivated()

        self.assertTrue(results)


if __name__ == "__main__":
    unittest.main()
