from enum import IntEnum, IntFlag

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
