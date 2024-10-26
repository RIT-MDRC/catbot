import logging

from component.can import can_bus as canbus_action
from component.motor import odrive_motor_action
from component.motor.odrive_enums import *
from state_management import configure_device

configure_device("src/raspi/pinconfig.json")

can_bus = "can_bus_1"
motor = "odrive_1"

odrive_motor_action.request_set_state(motor, MotorState.FULL_CALIBRATION_SEQUENCE)

while odrive_motor_action.get_state(motor) != MotorState.IDLE:
    pass

canbus_action.close(can_bus)