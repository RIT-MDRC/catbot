import unittest

from component.io_expander.pin import interrupt_pin
from state_management.utils import FakeDigitalInputDevice


class interrupt_pin_unit_tests(unittest.TestCase):

    def test_on_expander_activated_Always_SetsWhenActivated(self):
        test_device = FakeDigitalInputDevice(0, initial_is_state=True)

        results = interrupt_pin.on_expander_activated(test_device, lambda: True)
        
        self.assertTrue(results)
        self.assertTrue(test_device.when_activated())



if __name__ == "__main__":
    unittest.main()
