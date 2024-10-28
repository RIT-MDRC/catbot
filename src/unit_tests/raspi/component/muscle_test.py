import unittest
from unittest.mock import patch
from src.raspi.state_management import *
# with patch("src.raspi.component.muscle.pneumatics"):
from src.raspi.component.muscle import muscle_actions

def mock_pressure_okay(device: DigitalInputDevice):
    return True

def mock_pressure_not_okay(device: DigitalInputDevice):
    return False

class MuscleTest(unittest.TestCase):

    def test_create_muscle_object(self):
        muscle = muscle_actions.parse_muscle({
			"pressure": 0,
			"valve": 24,
		}, "test")
        self.assertIsInstance(muscle,muscle_actions.MuscleObj, "Muscle built isn't isntance of MuscleObj")
        muscle_actions.ctx.store = {} # clear store after every test
        muscle_actions.ctx.stored_keys = {} # clear stored keys after every test as well

    def test_muscle_object_contract_pressure_okay(self):
        muscle = muscle_actions.parse_muscle({
			"pressure": 0,
			"valve": 24,
		}, "test")
        self.assertTrue(muscle_actions.contract(muscle,mock_pressure_okay))
        muscle_actions.ctx.store = {}
        muscle_actions.ctx.stored_keys = {}

    def test_muscle_object_contract_pressure_not_okay(self):
        muscle = muscle_actions.parse_muscle({
			"pressure": 0,
			"valve": 24,
		}, "test")
        self.assertFalse(muscle_actions.contract(muscle,mock_pressure_not_okay))
        muscle_actions.ctx.store = {}
        muscle_actions.ctx.stored_keys = {}

    def test_muscle_object_relax_pressure_okay(self):
        muscle = muscle_actions.parse_muscle({
			"pressure": 0,
			"valve": 24,
		}, "test")
        self.assertTrue(muscle_actions.relax(muscle,mock_pressure_okay))
        muscle_actions.ctx.store = {}
        muscle_actions.ctx.stored_keys = {}

    def test_muscle_object_relax_pressure_okay(self):
        muscle = muscle_actions.parse_muscle({
			"pressure": 0,
			"valve": 24,
		}, "test")
        self.assertFalse(muscle_actions.relax(muscle,mock_pressure_not_okay))
        muscle_actions.ctx.store = {}
        muscle_actions.ctx.stored_keys = {}


if __name__ == '__main__':
    unittest.main()