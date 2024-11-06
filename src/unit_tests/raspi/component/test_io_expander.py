import unittest
from component.io_expander import io_expander_actions
from state_management import *

class IOExpanderTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.io_expander = io_expander_actions.parse_io_expander(
            {
			"address": "0x20",
			"interrupt_pin": FakeDigitalInputDevice(0),
			"input_channels": {
				"channel_1": 0,
				"channel_2": 1,
				"channel_3": 2,
				"channel_4": 3,
				"channel_5": 4,
				"channel_6": 5,
				"channel_7": 6,
				"channel_8": 7
			}
            }, "io_expander"
        )

    