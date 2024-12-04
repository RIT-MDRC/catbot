import unittest
import sys
import os

sys.path.insert(1,os.path.join(f'{__file__}','..','..','..','..',)) # never could figure out how to directly export the python path correctly so did this
# os.environ["ENV"] = "dev" # tried to manually change environment
# print(sys.path)
# for ar in os.environ:
#     print(ar)

# print()
# print(os.environ)
# print(os.environ.get("ENV"))
# print(os.environ.get("ENV")=="dev")

# current python path I've got exported doesn't work lol
# export PYTHONPATH=../../../../../src/foundation/foundation

from foundation.foundation.state_management.utils.util import is_dev 

print(is_dev()) # true here but not in SMBUS

from foundation.foundation.component.adc import ADC_action
from foundation.foundation.state_management import identifier
from foundation.foundation.state_management.utils import FakeSMBus


class adc_unit_tests(unittest.TestCase):

    def test_ADCAnalogInputDevice_Constructor(self):
        test_adc = ADC_action.ADC(
            address=1, input_devices={}, power_down=0, _identifier=""
        )
        test_address = 1
        results = ADC_action.ADCAnalogInputDevice(test_adc, test_address)
        self.assertEqual(results.adc, test_adc)
        self.assertEqual(results.address, test_address)


    def test_ADCAnalogInputDevice_value_Always_GetsValue(self):
        test_adc = ADC_action.ADC(
            address=1, input_devices={}, power_down=0, _identifier=""
        )
        test_adc.i2c = FakeSMBus(0)

        test_address = 1
        test_adc_analog_input_device = ADC_action.ADCAnalogInputDevice(
            test_adc, test_address
        )
        
        # The FakeSMBus does not return a value currently.
        try:
            test_adc_analog_input_device.value
        except:
            self.fail("Unexpected error.")

    
    def test_parse_analog_input_device_WhenValidConfig_CreatesADCAnalogInputDevice(
        self,
    ):
        config = {
            "adc": ADC_action.ADC(
                address=1, input_devices={}, power_down=0, _identifier=""
            ),
            "address": 1,
        }
        
        results = ADC_action.parse_analog_input_device(config, _identifier="")
        self.assertEqual(results.adc, config["adc"])
        self.assertEqual(results.address, config["address"])

    def test_parse_adc_WhenInvalidPowerDown_ThrowsError(self):
        with self.assertRaises(ValueError):
            ADC_action.parse_adc({"power_down": 4}, _identifier="")

    def test_parse_adc_WhenNoItems_GetsADC(self):
        config = {"address": "0x1", "input_devices": {}, "power_down": 0}
        results = ADC_action.parse_adc(config, _identifier="")
        self.assertEqual(results.address, 1)
        self.assertEqual(results.input_devices, {})
        self.assertEqual(results.power_down, 0)

    def test_parse_adc_WhenOneItem_HydratesAnalogContext(self):
        input_device_name = "name"
        identifier = ""
        config = {
            "address": "0x1",
            "input_devices": {input_device_name: 1},
            "power_down": 0,
        }

        ADC_action.parse_adc(config, _identifier=identifier)
        results = ADC_action.analog_input_device_ctx.store[
            f"{identifier}.{input_device_name}"
        ]

        test_adc = ADC_action.ADC(**config)
        channel = ADC_action.channel_to_adc_addr(
            config["input_devices"][input_device_name]
        )

        self.assertIsInstance(results, ADC_action.ADCAnalogInputDevice)
        self.assertEqual(results.adc, test_adc)
        self.assertEqual(
            results.address, ((1 << 3 | channel) << 2 | config["power_down"]) << 2
        )

    def test_channel_to_adc_addr_WhenInvalidChannel_ThrowsError(self):
        with self.assertRaises(ValueError):
            ADC_action.channel_to_adc_addr(-1)
        with self.assertRaises(ValueError):
            ADC_action.channel_to_adc_addr(8)

    def test_channel_to_adc_addr_WhenValidChannel_GetsCorrectAddress(self):
        self.assertEqual(ADC_action.channel_to_adc_addr(0), 0)
        self.assertEqual(ADC_action.channel_to_adc_addr(1), 2)
        self.assertEqual(ADC_action.channel_to_adc_addr(2), 4)
        self.assertEqual(ADC_action.channel_to_adc_addr(3), 6)
        self.assertEqual(ADC_action.channel_to_adc_addr(4), 1)
        self.assertEqual(ADC_action.channel_to_adc_addr(5), 3)
        self.assertEqual(ADC_action.channel_to_adc_addr(6), 5)
        self.assertEqual(ADC_action.channel_to_adc_addr(7), 7)


if __name__ == "__main__":
    unittest.main()