import unittest
from component.adc import ADC_action
from component.potentiometer import potentiometer_actions
from state_management import *

class PotentiomenterTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        configure_logger()

        # can potentially make a mock adc class,
        # just needs a read_data function

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
            "adc": "adc",
            "address": 0
        }, "pot1")

    def test_get_degree(self):
        degree = potentiometer_actions.get_degree("adc.pot1") 
        