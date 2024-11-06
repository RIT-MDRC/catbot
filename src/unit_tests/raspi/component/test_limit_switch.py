import unittest

from component.limit_switch import switch
from state_management.utils import FakeDigitalInputDevice


class limit_switch_unit_tests(unittest.TestCase):

    def test_is_limit_switch_active_Always_ReturnsIsActive(self):
        test_device = FakeDigitalInputDevice(0, initial_is_state=True)

        self.assertTrue(switch.is_limit_switch_active(test_device))


    def test_on_limit_switch_activated_Always_SetsWhenActivatedAction(self):
        test_device = FakeDigitalInputDevice(0)
        switch.on_limit_switch_activated(test_device, lambda: True)

        self.assertTrue(test_device.when_activated)

    
    def test_on_limit_switch_deactivated_Always_SetsWhenDeactivatedAction(self):
        test_device = FakeDigitalInputDevice(0)
        switch.on_limit_switch_deactivated(test_device, lambda: True)

        self.assertTrue(test_device.when_deactivated)


if __name__ == "__main__":
    unittest.main()
