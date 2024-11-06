import unittest

from component.latch import latch
from state_management import create_generic_context, DigitalOutputDevice
from state_management.utils import FakeDigitalOutputDevice


class latch_unit_tests(unittest.TestCase):

    def test_bitfield_Always_ReturnsBitField(self):
        results = latch.bitfield(7, 3)

        self.assertEqual(results, [1, 1, 1])


    def test_VirtualDigitalOutputDevice_Constructor(self):
        test_latch = latch.Latch(pins=[])
        test_addr = 0
        results = latch.VirtualDigitalOutputDevice(test_latch, test_addr)
        
        self.assertEqual(results.latch, test_latch)
        self.assertEqual(results.addr, test_addr)
        self.assertEqual(results._value, 0)
        self.assertEqual(results.pin, test_addr)


    def test_on_Always_SetsValueToOne(self):
        test_device = latch.VirtualDigitalOutputDevice(latch.Latch(pins=[]), 0)
        test_device._value = 0
        test_device.on()
        
        self.assertEqual(test_device._value, 1)

    
    def test_off_Always_SetsValueToOne(self):
        test_device = latch.VirtualDigitalOutputDevice(latch.Latch(pins=[]), 0)
        test_device._value = 1
        test_device.off()
        
        self.assertEqual(test_device._value, 0)


    def test_toggle_Always_TogglesValue(self):
        test_device = latch.VirtualDigitalOutputDevice(latch.Latch(pins=[]), 0)
        test_device._value = 0
        test_device.toggle()
        
        self.assertEqual(test_device._value, 1)

        test_device.toggle()

        self.assertEqual(test_device._value, 0)

    
    def test_set_value_Always_SetsValue(self):
        test_latch = latch.Latch(pins=[])
        test_latch.addr_1 = FakeDigitalOutputDevice(0)
        test_latch.addr_2 = FakeDigitalOutputDevice(0)
        test_latch.addr_3 = FakeDigitalOutputDevice(0)
        test_latch.data = FakeDigitalOutputDevice(0)
        test_latch.enab = FakeDigitalOutputDevice(0)
        test_device = latch.VirtualDigitalOutputDevice(test_latch, 7)

        test_value = 1
        test_device.set_value(test_value)
        
        self.assertEqual(test_device._value, test_value)
        self.assertEqual(test_device.latch.addr_1.value, 1)
        self.assertEqual(test_device.latch.addr_2.value, 1)
        self.assertEqual(test_device.latch.addr_3.value, 1)
        self.assertEqual(test_device.latch.data.value, test_value)
        self.assertEqual(test_device.latch.enab.value, 1)


    def test_value_Always_GetsValue(self):
        test_device = latch.VirtualDigitalOutputDevice(latch.Latch(pins=[]), 0)
        test_device._value = 1
        
        self.assertEqual(test_device.value, 1)


    def test_set_Always_SetsValues(self):
        test_latch = latch.Latch(pins=[])
        test_latch.addr_1 = FakeDigitalOutputDevice(0)
        test_latch.addr_2 = FakeDigitalOutputDevice(0)
        test_latch.addr_3 = FakeDigitalOutputDevice(0)
        test_latch.data = FakeDigitalOutputDevice(0)
        test_latch.enab = FakeDigitalOutputDevice(0)

        test_latch.set(7, 1)
        
        self.assertEqual(test_latch.addr_1.value, 1)
        self.assertEqual(test_latch.addr_2.value, 1)
        self.assertEqual(test_latch.addr_3.value, 1)
        self.assertEqual(test_latch.data.value, 1)
        self.assertEqual(test_latch.enab.value, 1)


    def test_process_queue_WhenItemInQueue_SetsValues(self):
        test_latch = latch.Latch(pins=[])
        test_latch.addr_1 = FakeDigitalOutputDevice(0)
        test_latch.addr_2 = FakeDigitalOutputDevice(0)
        test_latch.addr_3 = FakeDigitalOutputDevice(0)
        test_latch.data = FakeDigitalOutputDevice(0)
        test_latch.enab = FakeDigitalOutputDevice(0)

        test_latch.queue.append((7, 1))
        test_latch.process_queue()
        
        self.assertEqual(test_latch.addr_1.value, 1)
        self.assertEqual(test_latch.addr_2.value, 1)
        self.assertEqual(test_latch.addr_3.value, 1)
        self.assertEqual(test_latch.data.value, 1)
        self.assertEqual(test_latch.enab.value, 1)


    def test__set_one_device_Always_SetsValues(self):
        test_latch = latch.Latch(pins=[])
        test_latch.addr_1 = FakeDigitalOutputDevice(0)
        test_latch.addr_2 = FakeDigitalOutputDevice(0)
        test_latch.addr_3 = FakeDigitalOutputDevice(0)
        test_latch.data = FakeDigitalOutputDevice(0)
        test_latch.enab = FakeDigitalOutputDevice(0)

        test_latch._set_one_device(7, 1)
        
        self.assertEqual(test_latch.addr_1.value, 1)
        self.assertEqual(test_latch.addr_2.value, 1)
        self.assertEqual(test_latch.addr_3.value, 1)
        self.assertEqual(test_latch.data.value, 1)
        self.assertEqual(test_latch.enab.value, 1)


    def test_parse_latch_WhenInvalidContext_ThrowsError(self):
        latch.output_device_ctx = None

        with self.assertRaises(ValueError):
            latch.parse_latch({ "pins": {} }, _identifier="")

        latch.output_device_ctx = create_generic_context(
            "output_device", (DigitalOutputDevice, FakeDigitalOutputDevice)
        )


    def test_parse_latch_WhenValidContext_CreatesLatch(self):
        test_init_name = "pin1"
        test_identifier = ""
        test_name = f"{test_identifier}.{test_init_name}"
        test_pins = { test_init_name: 1 }
        test_data = { "pins": test_pins, "_identifier": test_identifier }
        results = latch.parse_latch(test_data, _identifier="")

        self.assertEqual(results.pins, test_pins)
        self.assertEqual(latch.output_device_ctx.store[test_name].pin, 1)
        self.assertEqual(latch.output_device_ctx.store[test_name].value, 0)


if __name__ == "__main__":
    unittest.main()
