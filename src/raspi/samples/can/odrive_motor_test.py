import logging

from component.can import can_bus as canbus_action
from component.motor import odrive_motor_action as odrive_action
from component.motor.odrive_enums import *
from state_management import configure_device

configure_device("src/raspi/pinconfig.json")

can_bus = "can_bus_1"
motor = "odrive_1"

odrive_action.request_set_state(motor, MotorState.FULL_CALIBRATION_SEQUENCE)

while odrive_action.get_state(motor) != MotorState.IDLE:
    print(f"Current state: {odrive_action.get_state(motor)}.")
    print(f"Current position: {odrive_action.get_current_position(motor)}.")
    print(f"Current velocity: {odrive_action.get_current_velocity(motor)}.")
    print("")
    pass

canbus_action.close(can_bus)