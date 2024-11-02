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

db = cantools.database.load_file("src/raspi/component/motor/odrive-cansimple.dbc")

@device
class ODriveMotor:
    axisID: int
    bus: can_bus.CanBus = identifier(can_bus.ctx)
    current_state : MotorState = MotorState.UNDEFINED
    current_position: float = None
    current_velocity: float = None

    def __post_init__(self):
        can_bus.add_listener(self.bus, self.axisID, lambda msg: read_message(self, msg))

def read_message(motor, msg):
    msg_id = msg.arbitration_id & 0x1F
    msg_db = db.get_message_by_frame_id(msg_id)
    match msg_db.name:
        case "Heartbeat": #0x01
            data = msg_db.decode(msg.data)
            state = data['Axis_State'].value
            motor.current_state = state
            logging.info("Heartbeat w/ state: " + state)
            axis_error = data['Axis_Error']
            if get_error_num(axis_error) != 0:
                logging.error("Axis Error w/ following data: " + axis_error)
        case "Get_Motor_Error": #0x03
            logging.error("Motor Error w/ following data: " + msg_db.decode(msg.data))
        case "Get_Encoder_Error": #0x04
            logging.error("Encoder Error w/ following data: " + msg_db.decode(msg.data))
        case "Get_Sensorless_Error": #0x05
            logging.error("Sensorless Error w/ following data: " + msg_db.decode(msg.data))
        case "Get_Encoder_Estimates": #0x09
            data = msg_db.decode(msg.data)
            motor.current_position = data['Pos_Estimate']
            motor.current_velocity = data['Vel_Estimate']
            pass
        case "Get_Encoder_Count": #0x0A
            pass
        case "Get_IQ": #0x14
            pass
        case "Get_Sensorless_Estimates": #0x15
            pass
        case "Get_Bus_Voltage_Current": #0x17
            pass
        case "Get_ADC_Voltage": #0x1C
            pass
        case "Get_Controller_Error": #0x1D
            logging.error("Controller Error w/ following data: " + msg_db.decode(msg.data))
        case _:
            logging.warning(f"Unrecognized message \"{msg_db.name}\" recieved (id {hex(msg_id)})")

def get_error_num(error):
    #errors are flags, which cantools does not properly handle
    #if only 1 flag is present, it returns a "NamedSignalValue"
    #if multiple flags are present, it returns an int
    #"NamedSignalValue" can not be cast to int, the only way to get the number is to do "error.value", which is invalid if "error" is an int
    if error is int:
        return error
    else:
        return error.value

ctx = create_generic_context("odrive_motor", [ODriveMotor])

@device_parser(ctx)
def parse_odrive(data: dict) -> ODriveMotor:
    motor = ODriveMotor(**data)
    return motor

@device_action(ctx)
def set_target_position(motor: ODriveMotor, position: float, velocity_FF: float = 0.0, torque_FF: float = 0.0) -> bool:
    return send_message(motor, "Set_Input_Pos", {'Input_Pos': position, 'Vel_FF': velocity_FF, 'Torque_FF': torque_FF})

@device_action(ctx)
def set_target_velocity(motor: ODriveMotor, velocity: float, torque_FF: float = 0.0) -> bool:
    return send_message(motor, "Set_Input_Vel", {'Input_Vel': velocity, 'Input_Torque_FF': torque_FF})

@device_action(ctx)
def stop(motor: ODriveMotor) -> bool:
    return set_target_velocity(motor, 0)

@device_action(ctx)
def get_current_position(motor: ODriveMotor) -> float:
    return motor.current_position

@device_action(ctx)
def get_current_velocity(motor: ODriveMotor) -> float:
    return motor.current_velocity

@device_action(ctx)
def set_limits(motor: ODriveMotor, velocity_limit: float, current_limit: float) -> bool:
    return send_message(motor, "Set_Limits", {'Velocity_Limit': velocity_limit, 'Current_Limit': current_limit})

@device_action(ctx)
def request_set_state(motor: ODriveMotor, state: MotorState) -> bool:
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
    msg = db.get_message_by_name(msg_name)
    msg_id = msg.frame_id | motor.axisID << 5
    data = msg.encode(data)
    return can_bus.send_message(motor.bus, msg_id, data)
