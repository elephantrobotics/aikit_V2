from enum import Enum

DEBUG = False

class DeviceAdress(Enum):
    IR_DETECT = 0x21
    STEPPER_MOTOR_42 = 0x30
    STEPPER_MOTOR_57 = 0x31

class Command(Enum):
    GET_TOF_DISTANCE = 0x51
    SET_DIR = 0xa0
    SET_SPEED = 0xa1
    GET_DIR = 0xa2
    GET_SPEED = 0xa3
    GET_DISTANCE = 0xa4
    WRITE_STEPS_BY_SWITCH = 0xa5
    WRITE_ANGLE = 0xa6
    WRITE_STEPS = 0xa7
    WRITE_DISTANCE_ZERO = 0xa8
    WRITE_DISTANCE = 0xa9

class MotorDirection(Enum):
    CLOCKWISE = 0x0
    COUNTCLOCKWISE = 0x1

class MotorInterfaceType(Enum):
    FUNCTION = 0
    DRIVER = 1
    FULL2WIRE = 2
    FULL3WIRE = 3
    FULL4WIRE = 4
    HALF3WIRE = 6
    HALF4WIRE = 8
    HALF8WIRE = 16

class StepperMotorType(Enum):
    STEPPER_MOTOR_42 = 1
    STEPPER_MOTOR_57 = 2

class SwitchMode(Enum):
    CLOSE = 0
    OPEN = 1