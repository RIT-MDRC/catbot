import unittest

from component.motor import raw_motor
from state_management import identifier
from state_management.utils import FakePWMOutputDevice
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


if __name__ == "__main__":
    unittest.main()
