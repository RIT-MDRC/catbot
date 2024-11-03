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
    @classmethod
    def setUpClass(cls):
        configure_logger()
        cls.muscle = muscle_actions.parse_muscle(
            {
                "pressure": 0,
                "valve": 24,
            },
            "muscle",
        )

    # def test_create_muscle_object(self):
    #     self.assertIsInstance(
    #         self.muscle,
    #         muscle_actions.MuscleObj,
    #         "Muscle built isn't isntance of MuscleObj",
    #     )

    def test_muscle_object_contract_pressure_okay(self):
        self.assertTrue(muscle_actions.contract(self.muscle, mock_pressure_okay))

    def test_muscle_object_contract_pressure_not_okay(self):
        self.assertFalse(muscle_actions.contract(self.muscle, mock_pressure_not_okay))

    def test_muscle_object_relax_pressure_okay(self):
        self.assertTrue(muscle_actions.relax(self.muscle, mock_pressure_okay))

    def test_muscle_object_relax_pressure_okay(self):
        self.assertFalse(muscle_actions.relax(self.muscle, mock_pressure_not_okay))


if __name__ == "__main__":
    unittest.main()
