import cantools
import math
import time
import logging
from enum import IntEnum, IntFlag
from dataclasses import dataclass

import component.can.can_bus as can_bus

from state_management import (
    create_generic_context,
    device,
    device_action,
    device_parser,
    identifier
)

db = cantools.database.load_file("src/raspi/odrive-cansimple.dbc")

class MotorState(IntEnum):
    UNDEFINED = 0
    IDLE = 1
    STARTUP_SEQUENCE = 2
    FULL_CALIBRATION_SEQUENCE = 3
    MOTOR_CALIBRATION = 4
    ENCODER_INDEX_SEARCH = 6
    ENCODER_OFFSET_CALIBRATION = 7
    CLOSED_LOOP_CONTROL = 8
    LOCKIN_SPIN = 9
    ENCODER_DIR_FIND = 10
    HOMING = 11
    ENCODER_HALL_POLARITY_CALIBRATION = 12
    ENCODER_HALL_PHASE_CALIBRATION = 13

class InputMode(IntEnum):
    INACTIVE = 0
    PASSTHROUGH = 1
    VEL_RAMP = 2
    POS_FILTER = 3
    MIX_CHANNELS = 4
    TRAP_TRAJ = 5
    TORQUE_RAMP = 6
    MIRROR = 7
    TUNING = 8

class ControlMode(IntEnum):
    VOLTAGE_CONTROL = 0
    TORQUE_CONTROL = 1
    VELOCITY_CONTROL = 2
    POSITION_CONTROL = 3

class AxisError(IntFlag):
    NONE = 0
    INVALID_STATE = 1
    MOTOR_FAILED = 64
    SENSORLESS_ESTIMATOR_FAILED = 128
    ENCODER_FAILED = 256
    CONTROLLER_FAILED = 512
    WATCHDOG_TIMER_EXPIRED = 2048
    MIN_ENDSTOP_PRESSED = 4096
    MAX_ENDSTOP_PRESSED = 8192
    ESTOP_REQUESTED = 16384
    HOMING_WITHOUT_ENDSTOP = 131072
    OVER_TEMP = 262144
    UNKNOWN_POSITION = 524288

class MotorError(IntFlag):
    NONE = 0
    PHASE_RESISTANCE_OUT_OF_RANGE = 1
    PHASE_INDUCTANCE_OUT_OF_RANGE = 2
    DRV_FAULT = 8
    CONTROL_DEADLINE_MISSED = 16
    MODULATION_MAGNITUDE = 128
    CURRENT_SENSE_SATURATION = 1024
    CURRENT_LIMIT_VIOLATION = 4096
    MODULATION_IS_NAN = 65536
    MOTOR_THERMISTOR_OVER_TEMP = 131072
    FET_THERMISTOR_OVER_TEMP = 262144
    TIMER_UPDATE_MISSED = 524288
    CURRENT_MEASUREMENT_UNAVAILABLE = 1048576
    CONTROLLER_FAILED = 2097152
    I_BUS_OUT_OF_RANGE = 4194304
    BRAKE_RESISTOR_DISARMED = 8388608
    SYSTEM_LEVEL = 16777216
    BAD_TIMING = 33554432
    UNKNOWN_PHASE_ESTIMATE = 67108864
    UNKNOWN_PHASE_VEL = 134217728
    UNKNOWN_TORQUE = 268435456
    UNKNOWN_CURRENT_COMMAND = 536870912
    UNKNOWN_CURRENT_MEASUREMENT = 1073741824
    UNKNOWN_VBUS_VOLTAGE = 2147483648
    UNKNOWN_VOLTAGE_COMMAND = 4294967296
    UNKNOWN_GAINS = 8589934592
    CONTROLLER_INITIALIZING = 17179869184
    UNBALANCED_PHASES = 34359738368

class EncoderError(IntFlag):
    NONE = 0
    UNSTABLE_GAIN = 1
    CPR_POLEPAIRS_MISMATCH = 2
    NO_RESPONSE = 4
    UNSUPPORTED_ENCODER_MODE = 8
    ILLEGAL_HALL_STATE = 16
    INDEX_NOT_FOUND_YET = 32
    ABS_SPI_TIMEOUT = 64
    ABS_SPI_COM_FAIL = 128
    ABS_SPI_NOT_READY = 256
    HALL_NOT_CALIBRATED_YET = 512

class SensorlessError(IntFlag):
    NONE = 0
    UNSTABLE_GAIN = 1
    UNKNOWN_CURRENT_MEASUREMENT = 2

class ControllerError(IntFlag):
    NONE = 0
    OVERSPEED = 1
    INVALID_INPUT_MODE = 2
    UNSTABLE_GAIN = 4
    INVALID_MIRROR_AXIS = 8
    INVALID_LOAD_ENCODER = 16
    INVALID_ESTIMATE = 32
    INVALID_CIRCULAR_RANGE = 64
    SPINOUT_DETECTED = 128

@device
@dataclass(slots = True)
class ODriveMotor:
    bus: can_bus.CanBus = identifier(can_bus.ctx)
    axisID: int
    current_state = None
    current_position: float = None
    current_velocity: float = None
    update_interval: float = None
    update_task = None

    def __post_init__(self):
        can_bus.add_listener(self.bus, self.axisID, lambda msg: read_message(self, msg))
        set_estimate_update_interval(self, 0.02)

def read_message(motor, msg):
    msg_id = msg.arbitration_id & 0x1F
    msg_db = db.get_message_by_frame_id(msg_id)
    match msg_db.name:
        case "Heartbeat": #0x01
            data = msg_db.decode(msg.data)
            motor.current_state = data['Axis_State']
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

ctx = create_generic_context("odrive_motor", [ODriveMotor])

@device_parser(ctx)
def parse_odrive(data: dict) -> ODriveMotor:
    motor = ODriveMotor(data['bus'])
    motor.register(data['axisID'])
    return motor

@device_action(ctx)
def set_target_position(motor: ODriveMotor, position: float, velocity: float = 0.0) -> bool:
    msg = db.get_message_by_name("Set_Input_Pos")
    msg_id = msg.frame_id | motor.axisID << 5
    data = msg.encode({'Input_Pos': position, 'Vel_FF': velocity, 'Torque_FF': 0.0})
    return can_bus.send_message(motor.bus, msg_id, data)

@device_action(ctx)
def set_target_velocity(motor: ODriveMotor, velocity: float) -> bool:
    msg = db.get_message_by_name("Set_Input_Vel")
    msg_id = msg.frame_id | motor.axisID << 5
    data = msg.encode({'Input_Vel': velocity, 'Input_Torque_FF': 0.0})
    return can_bus.send_message(motor.bus, msg_id, data)

@device_action(ctx)
def set_estimate_update_interval(motor: ODriveMotor, interval: float) -> None:
    if motor.update_task is not None:
        motor.update_task.stop()
    msg = db.get_message_by_name("Get_Encoder_Estimates")
    msg_id = msg.frame_id | motor.axisID << 5
    data = msg.encode({'Pos_Estimate': 0.0, 'Vel_Estimate': 0.0})
    motor.update_interval = interval
    motor.update_task = can_bus.send_message_recurring(motor.bus, msg_id, data, interval)
    
@device_action(ctx)
def get_estimate_update_interval(motor: ODriveMotor) -> float:
    return motor.update_interval

@device_action(ctx)
def get_current_position(motor: ODriveMotor) -> float:
    return motor.current_position

@device_action(ctx)
def get_current_velocity(motor: ODriveMotor) -> float:
    return motor.current_velocity

@device_action(ctx)
def set_limits(motor: ODriveMotor, velocity_limit: float, current_limit: float) -> bool:
    msg = db.get_message_by_name("Set_Limits")
    msg_id = msg.frame_id | motor.axisID << 5
    data = msg.encode({'Velocity_Limit': velocity_limit, 'Current_Limit': current_limit})
    return can_bus.send_message(motor.bus, msg_id, data)

@device_action(ctx)
def set_state(motor: ODriveMotor, state: int | str) -> bool:
    msg = db.get_message_by_name("Set_Axis_State")
    msg_id = msg.frame_id | motor.axisID << 5
    data = msg.encode({'Axis_Requested_State': state})
    return can_bus.send_message(motor.bus, msg_id, data)

@device_action(ctx)
def get_state(motor: ODriveMotor) -> str:
    """Returns the recent state reported by the motor."""
    return motor.current_state.name if motor.current_state is not None else "UNDEFINED"

@device_action(ctx)
def get_state_numeric(motor: ODriveMotor) -> int:
    """Returns the recent state reported by the motor in numeric form."""
    return motor.current_state.value if motor.current_state is not None else 0
