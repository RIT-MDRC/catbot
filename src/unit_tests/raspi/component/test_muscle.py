import unittest
from unittest.mock import patch

# with patch("src.raspi.component.muscle.pneumatics"):
from component.muscle import muscle_actions
from state_management import *


def mock_pressure_okay(device: DigitalInputDevice):
    return True


def mock_pressure_not_okay(device: DigitalInputDevice):
    return False


class MuscleTest(unittest.TestCase):

    def test_create_muscle_object(self):
        muscle = muscle_actions.parse_muscle(
            {
                "pressure": 0,
                "valve": 24,
            },
            "test",
        )
        self.assertIsInstance(
            muscle, muscle_actions.MuscleObj, "Muscle built isn't isntance of MuscleObj"
        )
        print(DEVICE_CONTEXT_COLLECTION)
        clear_all_contexts()

    def test_muscle_object_contract_pressure_okay(self):
        muscle = muscle_actions.parse_muscle(
            {
                "pressure": 0,
                "valve": 24,
            },
            "test",
        )
        self.assertTrue(muscle_actions.contract(muscle, mock_pressure_okay))
        clear_all_contexts()

    def test_muscle_object_contract_pressure_not_okay(self):
        muscle = muscle_actions.parse_muscle(
            {
                "pressure": 0,
                "valve": 24,
            },
            "test",
        )
        self.assertFalse(muscle_actions.contract(muscle, mock_pressure_not_okay))
        clear_all_contexts()

    def test_muscle_object_relax_pressure_okay(self):
        muscle = muscle_actions.parse_muscle(
            {
                "pressure": 0,
                "valve": 24,
            },
            "test",
        )
        self.assertTrue(muscle_actions.relax(muscle, mock_pressure_okay))
        clear_all_contexts()

    def test_muscle_object_relax_pressure_okay(self):
        muscle = muscle_actions.parse_muscle(
            {
                "pressure": 0,
                "valve": 24,
            },
            "test",
        )
        self.assertFalse(muscle_actions.relax(muscle, mock_pressure_not_okay))
        clear_all_contexts()


if __name__ == "__main__":
    unittest.main()
