import cantools
import math
import time
import logging
from dataclasses import dataclass
from component.motor.odrive_enums import *

import component.can.can_bus as can_bus

from state_management import (
    create_generic_context,
    device,
    device_action,
    device_parser,
    identifier
)

ODRIVE_CAN_DB = cantools.database.load_file("src/raspi/component/motor/odrive-cansimple.dbc")

@device
@dataclass
class ODriveMotor:
    axisID: int
    bus: can_bus.CanBus = identifier(can_bus.ctx)
    current_state : MotorState = MotorState.UNDEFINED
    current_position: float = None
    current_velocity: float = None

    def __post_init__(self):
        can_bus.add_listener(self.bus, self.axisID, lambda msg: read_message(self, msg))

#region Message Handling

EVENT_HANDLE = dict()
EVENT_IGNORE = ["Get_Encoder_Count", "Get_IQ", "Get_Sensorless_Estimates", "Get_Bus_Voltage_Current", "Get_ADC_Voltage"]

def read_message(motor, msg):
    msg_id = msg.arbitration_id & 0x1F
    msg_db = ODRIVE_CAN_DB.get_message_by_frame_id(msg_id)
    if msg_db.name in EVENT_HANDLE:
        EVENT_HANDLE[msg_db.name](motor, msg_db.decode(msg.data))
    elif msg_db.name not in EVENT_IGNORE:
        logging.warning(
            f'Unrecognized message "{msg_db.name}" recieved (id {hex(msg_id)})'
        )

def event_decorator(name):
    def wrapper(func):
        EVENT_HANDLE[name] = func
        return func
    return wrapper

@event_decorator("Heartbeat")
def read_heartbeat(motor, data):
    state = data['Axis_State'].value
    motor.current_state = state
    logging.info(f"Axis {motor.axisID} Heartbeat w/ state: " + str(state))
    axis_error = data['Axis_Error']
    if get_error_num(axis_error) != 0:
        logging.error(f"Axis {motor.axisID} Error w/ following data: " + str(axis_error))

@event_decorator("Get_Encoder_Estimates")
def update_estimates(motor, data):
    motor.current_position = data['Pos_Estimate']
    motor.current_velocity = data['Vel_Estimate']

@event_decorator("Get_Motor_Error")
def report_motor_error(motor, data):
    logging.error(f"Motor Error on axis {motor.axisID} w/ following data: " + str(data))

@event_decorator("Get_Encoder_Error")
def report_encoder_error(motor, data):
    logging.error(f"Encoder Error on axis {motor.axisID} w/ following data: " + str(data))

@event_decorator("Get_Sensorless_Error")
def report_sensorless_error(motor, data):
    logging.error(f"Sensorless Error on axis {motor.axisID} w/ following data: " + str(data))

@event_decorator("Get_Controller_Error")
def report_controller_error(motor, data):
    logging.error(f"Controller Error on axis {motor.axisID} w/ following data: " + str(data))

def get_error_num(error):
    #errors are flags, which cantools does not properly handle
    #if only 1 flag is present, it returns a "NamedSignalValue"
    #if multiple flags are present, it returns an int
    #"NamedSignalValue" can not be cast to int, the only way to get the number is to do "error.value", which is invalid if "error" is an int
    if error is int:
        return error
    else:
        return error.value

#endregion

ctx = create_generic_context("odrive_motor", [ODriveMotor])

@device_parser(ctx)
def parse_odrive(data: dict) -> ODriveMotor:
    return ODriveMotor(**data)

@device_action(ctx)
def set_target_position(motor: ODriveMotor, position: float, velocity_FF: float = 0.0, torque_FF: float = 0.0) -> bool:
    """Set the target position for this motor
    Only use this function if Control Mode is set to POSITION_CONTROL"""
    return send_message(motor, "Set_Input_Pos", {'Input_Pos': position, 'Vel_FF': velocity_FF, 'Torque_FF': torque_FF})

@device_action(ctx)
def set_target_velocity(motor: ODriveMotor, velocity: float, torque_FF: float = 0.0) -> bool:
    """Set the target velocity for this motor
    Only use this function if Control Mode is set to VELOCITY_CONTROL"""
    return send_message(motor, "Set_Input_Vel", {'Input_Vel': velocity, 'Input_Torque_FF': torque_FF})

@device_action(ctx)
def get_current_position(motor: ODriveMotor) -> float:
    return motor.current_position

@device_action(ctx)
def get_current_velocity(motor: ODriveMotor) -> float:
    return motor.current_velocity

@device_action(ctx)
def set_limits(motor: ODriveMotor, velocity_limit: float, current_limit: float) -> bool:
    """Set the velocity and current limits for this motor
    Consider changing the motor's config instead of using this function"""
    return send_message(motor, "Set_Limits", {'Velocity_Limit': velocity_limit, 'Current_Limit': current_limit})

@device_action(ctx)
def request_set_state(motor: ODriveMotor, state: MotorState) -> bool:
    """Set the state of the motor
    Motor will not recieve position/velocity input if not set to CLOSED_LOOP_CONTROL"""
    return send_message(motor, "Set_Axis_State", {'Axis_Requested_State': state})

@device_action(ctx)
def get_state(motor: ODriveMotor) -> MotorState:
    """Returns the most recent state reported by the motor."""
    return motor.current_state

@device_action(ctx)
def set_controller_mode(motor: ODriveMotor, control_mode: ControlMode, input_mode: InputMode) -> bool:
    return send_message(motor, "Set_Controller_Mode", {'Control_Mode': control_mode, 'Input_Mode': input_mode})

@device_action(ctx)
def send_message(motor: ODriveMotor, msg_name: str, data: dict) -> bool:
    msg = ODRIVE_CAN_DB.get_message_by_name(msg_name)
    msg_id = msg.frame_id | motor.axisID << 5
    data = msg.encode(data)
    return can_bus.send_message(motor.bus, msg_id, data)
