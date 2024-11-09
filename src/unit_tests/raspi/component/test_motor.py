import unittest

from component.motor import motor
from state_management.utils import FakeDigitalOutputDevice


class motor_unit_tests(unittest.TestCase):

    def test_MotorController_constructor(self):
        test_pwm_pin = 0
        test_direction_address = 0
        test_direction_pin = 0
        test_address_pins = [0]
        test_device = FakeDigitalOutputDevice(test_address_pins[0])

        results = motor.MotorController(
            test_pwm_pin, 
            test_direction_address, 
            test_direction_pin, 
            test_address_pins
        )

        self.assertEqual(results.pwm_pin, test_pwm_pin)
        self.assertEqual(results.direction_address, test_direction_address)
        self.assertEqual(results.direction_pin.pin, test_direction_pin)
        self.assertEqual(results.address_output_devices[0].pin, test_device.pin)


    def test_set_direction_Always_SetsDirection(self):
        test_controller = motor.MotorController(0, 0, 0, [0])
        test_direction = 1

        test_controller.set_direction(test_direction)

        self.assertEqual(test_controller.direction_pin.value, test_direction)
        self.assertEqual(test_controller.address_output_devices[0].value, "0")

    
    def test_set_speed_Always_SetsSpeed(self):
        test_controller = motor.MotorController(0, 0, 0, [0])
        test_speed = 1

        test_controller.set_speed(test_speed)

        self.assertEqual(test_controller.motor.value, test_speed)


    def test_set_speed_dir_Always_SetsSpeedAndDirection(self):
        test_controller = motor.MotorController(0, 0, 0, [0])
        test_direction = 1
        test_speed = 1

        results = test_controller.set_speed_dir(test_speed, test_direction)

        self.assertTrue(results)
        self.assertEqual(test_controller.motor.value, test_speed)
        self.assertEqual(test_controller.direction_pin.value, test_direction)
        self.assertEqual(test_controller.address_output_devices[0].value, "0")
        self.assertEqual(test_controller.current_direction, test_direction)
        self.assertEqual(test_controller.current_speed, test_speed)


    def test_bitfield_Always_GetsBitField(self):
        results = motor.bitfield(1, 3)

        self.assertEqual(results, "001")


if __name__ == "__main__":
    unittest.main()
