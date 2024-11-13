import unittest

from component.motor import raw_motor
from state_management import identifier
from state_management.utils import FakePWMOutputDevice, FakeDigitalOutputDevice
from component.motor.pin import direction_pin_action, speed_pin_action


class raw_motor_unit_tests(unittest.TestCase):

    def test_RawMotorController_constructor(self):
        test_stop_duration = 1

        results = raw_motor.RawMotor(test_stop_duration)

        self.assertEqual(results.stop_duration, test_stop_duration)
        self.assertEqual(results._identifier, "latch")
        self.assertEqual(results.speed, identifier(speed_pin_action.ctx))
        self.assertEqual(results.direction, identifier(direction_pin_action.ctx))


    def test_parse_raw_motor_Always_CreatesRawMotor(self):
        test_stop_duration = 1

        results = raw_motor.parse_raw_motor(
            { 
                "stop_duration": test_stop_duration 
            },
            _identifier="latch"
        )

        self.assertEqual(results.stop_duration, test_stop_duration)
        self.assertEqual(results._identifier, "latch")
        self.assertEqual(results.speed, identifier(speed_pin_action.ctx))
        self.assertEqual(results.direction, identifier(direction_pin_action.ctx))


    def test_set_speed_Always_SetsSpeed(self):
        test_raw_motor = raw_motor.RawMotor(1)
        test_raw_motor.speed = FakePWMOutputDevice(0, 0)

        results = raw_motor.set_speed(test_raw_motor, 1)

        self.assertTrue(results)


    def test_get_speed_Always_GetsSpeed(self):
        test_speed = 1
        test_raw_motor = raw_motor.RawMotor(1)
        test_raw_motor.speed = FakePWMOutputDevice(0, test_speed)

        results = raw_motor.get_speed(test_raw_motor)

        self.assertEqual(results, test_speed)


    def test_check_direction_WhenInvalidDirection_ThrowsError(self):
        with self.assertRaises(ValueError):
            raw_motor.check_direction(raw_motor.RawMotor(1), 2)


    def test_check_direction_WhenMatchingDirection_ReturnsTrue(self):
        test_direction = 1
        test_raw_motor = raw_motor.RawMotor(1)
        test_raw_motor.direction = FakeDigitalOutputDevice(0, test_direction)

        results = raw_motor.check_direction(test_raw_motor, test_direction)

        self.assertTrue(results)


    def test_check_direction_WhenNotMatchingDirection_ReturnsFalse(self):
        test_direction = 1
        test_wrong_direction = 0
        test_raw_motor = raw_motor.RawMotor(1)
        test_raw_motor.direction = FakeDigitalOutputDevice(0, test_direction)

        results = raw_motor.check_direction(test_raw_motor, test_wrong_direction)

        self.assertFalse(results)


    def test_check_speed_WhenInvalidSpeed_ThrowsError(self):
        with self.assertRaises(ValueError):
            raw_motor.check_speed(raw_motor.RawMotor(1), 2)


    def test_check_speed_WhenMatchingSpeed_ReturnsTrue(self):
        test_speed = 1
        test_raw_motor = raw_motor.RawMotor(1)
        test_raw_motor.speed = FakePWMOutputDevice(0, test_speed)

        results = raw_motor.check_speed(test_raw_motor, test_speed)

        self.assertTrue(results)


    def test_check_speed_WhenNotMatchingSpeed_ReturnsFalse(self):
        test_speed = 1
        test_wrong_speed = 0
        test_raw_motor = raw_motor.RawMotor(1)
        test_raw_motor.speed = FakePWMOutputDevice(0, test_speed)

        results = raw_motor.check_speed(test_raw_motor, test_wrong_speed)

        self.assertFalse(results)


    def test_get_direction_Always_GetsDirection(self):
        test_direction = 1
        test_raw_motor = raw_motor.RawMotor(1)
        test_raw_motor.direction = FakeDigitalOutputDevice(0, test_direction)

        results = raw_motor.get_direction(test_raw_motor)

        self.assertEqual(results, test_direction)


if __name__ == "__main__":
    unittest.main()
