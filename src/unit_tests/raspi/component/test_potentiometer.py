import unittest
from component.adc import ADC_action
from component.potentiometer import potentiometer_actions
from state_management import *

class MockADC():
    def read_data(self,address):
        # address is a forced parameter as the analog input device always sends its own address
        # to the ADC to be read from when the analog input device attempts a .value call on itself,
        # and in this case potentiometer_actions.get_degree is just a .value call on itself, therefor it needs the address param
        return True

class PotentiomenterTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        configure_logger()

        # cls.adc = ADC_action.parse_adc({
        #     "i2c": "smbus",
		# 	"address": "0x48",
		# 	"power_down": 3,
		# 	"input_devices": {
        #         "pot1": 0,
        #     }
        # }, "adc",
        # )
        # log_states()
        cls.pot = ADC_action.parse_analog_input_device({
            "adc": MockADC(),
            "address": 0
        }, "pot1")

    def test_get_degree(self):
        self.assertTrue(potentiometer_actions.get_degree(self.pot))
        