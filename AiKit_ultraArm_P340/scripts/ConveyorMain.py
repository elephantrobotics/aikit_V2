from serial import Serial
from common import *
import struct
import time

class ConveyorMain(object):
    read_delay = 0.5
    invalid_data = -1
    command_header = 0xff

    length = 0
    content = []
    cmd = 0
    check_digit_user = 0
    check_digit_ok = invalid_data
    check_digit_flag = False

    def __init__(self, port, baudrate="115200", timeout=0.1, debug=False):
        self._serial_port = Serial()
        self._serial_port.port = port
        self._serial_port.baudrate = baudrate
        self._serial_port.timeout = timeout
        self._serial_port.rts = False
        self._serial_port.dtr = False
        self._serial_port.open()

    def __destroy__(self):
        self._serial_port.close()

    def flush(self):
        self._serial_port.flush()

    def check_digit(self, cmd, content):
        ver = cmd
        if 0 < len(content):
            for item in content:
                ver += item
        ver &= 0xff
        self.check_digit_flag = True
        return ver
    
    def write(self, adress, content, command):
        data = [self.command_header, self.command_header, adress, len(content), *content, command, self.check_digit(command, content)]
        if DEBUG == True:
            print("write data: ", data)

        self._serial_port.flush()
        self._serial_port.write(data)

    def double_header_check(self, read_buff):
        if len(read_buff) >= 2:
            if self.command_header == read_buff[0] and self.command_header == read_buff[1]:
                return True
        return False

    def read(self):
        if self._serial_port.inWaiting() <= 0:
            return False
        
        time.sleep(self.read_delay)
        read_buff = self._serial_port.read(self._serial_port.inWaiting())
        if len(read_buff) < 6 or not self.double_header_check(read_buff):
            return False
        else:
            if DEBUG == True:
                print("Read timeout ! ")
            else:
                pass

        self.length = read_buff[3]
        self.content = []
        content_begin = 4
        if self.length == 0:
            self.cmd = read_buff[content_begin]
            self.check_digit_user = read_buff[content_begin + 1]
        else:
            content_end = content_begin + self.length
            self.content = read_buff[content_begin: content_end]
            self.cmd = read_buff[content_end]
            self.check_digit_user = read_buff[content_end + 1]
        self.check_digit_ok = self.check_digit(self.cmd, self.content)

        if DEBUG == True:
            print("content: ", self.content)

        if self.check_digit_flag:
            return True
        else:
            return False

    def get_data_from_slave(self):
        if not self.content:
            return []

        read_code = self.read()
        if read_code and self.check_digit_ok == self.check_digit_user:
            valid_data = self.content
            self.content = []
            return valid_data
        return []

    def unpack_args(self, *args):
        bits_pack_list = []
        args_list = list(args)
        for args in args_list:
            pair = struct.pack('>h', args)
            if len(pair) == 2:
                bits_pack_list.append(pair[0])
                bits_pack_list.append(pair[1])
                return bits_pack_list
        return []

    def control_command(self, adress, command, *args):
        unpack_list = self.unpack_args(*args)
        if DEBUG == True:
            print("unpack_list: ", unpack_list)

        new_data_buff = unpack_list

        if (adress == DeviceAdress.STEPPER_MOTOR_42.value or adress == DeviceAdress.STEPPER_MOTOR_57.value) \
            and (command == Command.SET_DIR.value or command == Command.SET_SPEED.value):
            new_data_buff.pop(0)

        if (adress == DeviceAdress.STEPPER_MOTOR_42.value or adress == DeviceAdress.STEPPER_MOTOR_57.value) \
            and (command == Command.WRITE_ANGLE.value):
            new_data_buff.pop(2)

        if (adress == DeviceAdress.STEPPER_MOTOR_42.value or adress == DeviceAdress.STEPPER_MOTOR_57.value) \
            and (command == Command.WRITE_STEPS.value):
            new_data_buff.pop(2)
            new_data_buff.pop(3)

        if (adress == DeviceAdress.STEPPER_MOTOR_57.value) and (command == Command.WRITE_STEPS_BY_SWITCH.value):
            new_data_buff.pop(0)
            new_data_buff.pop(1)

        if (adress == DeviceAdress.STEPPER_MOTOR_42.value) and (command == Command.WRITE_DISTANCE.value):
            new_data_buff.pop(2)
            new_data_buff.pop(3)

        if (adress == DeviceAdress.STEPPER_MOTOR_42.value) and (command == Command.WRITE_DISTANCE_ZERO.value):
            new_data_buff.pop(0)
        if DEBUG == True:
            print("new_data_buff: ", new_data_buff)

        if self.invalid_data < len(new_data_buff):
            self.write(adress, new_data_buff, command)

    def speed_range_check(self, speed, min=0, max=100):
        return min <= speed <= max

    def dir_range_check(self, dir):
        return dir == MotorDirection.CLOCKWISE.value or dir == MotorDirection.COUNTCLOCKWISE.value

    def get_single_data_from_slave(self, id, command):
        if id == StepperMotorType.STEPPER_MOTOR_42.value:
            self.control_command(DeviceAdress.STEPPER_MOTOR_42.value, command)
        elif id == StepperMotorType.STEPPER_MOTOR_57.value:
            self.control_command(DeviceAdress.STEPPER_MOTOR_57.value, command)

        valid_data = self.get_data_from_slave()
        if len(valid_data) == 1:
            return valid_data[0]
        return self.invalid_data

    def get_tof_distance(self):
        self.control_command(DeviceAdress.IR_DETECT.value, Command.GET_TOF_DISTANCE.value)

        distance = -1
        valid_data = self.get_data_from_slave()
        data_len = len(valid_data)
        if data_len > 0:
            if data_len == 1:
                distance = valid_data[0]
            elif data_len == 2:
                distance = (valid_data[0] << 8) | valid_data[1]
        if 20 <= distance:
            return distance
        return -1

    def get_dir(self, id):
        return self.get_single_data_from_slave(id, Command.GET_DIR.value)

    def get_speed(self, id):
        return self.get_single_data_from_slave(id, Command.GET_SPEED.value)

    def get_distance(self):
        return self.get_single_data_from_slave(StepperMotorType.STEPPER_MOTOR_42.value, Command.GET_DISTANCE.value)

    def set_dir(self, id, dir):
        if self.dir_range_check(dir):
            if id == StepperMotorType.STEPPER_MOTOR_42.value:
                self.control_command(DeviceAdress.STEPPER_MOTOR_42.value, Command.SET_DIR.value, dir)
            elif id == StepperMotorType.STEPPER_MOTOR_57.value:
                self.control_command(DeviceAdress.STEPPER_MOTOR_57.value, Command.SET_DIR.value, dir)

    def set_speed(self, id, speed):
        if self.speed_range_check(speed):
            if id == StepperMotorType.STEPPER_MOTOR_42.value:
                self.control_command(DeviceAdress.STEPPER_MOTOR_42.value, Command.SET_SPEED.value, speed)
            elif id == StepperMotorType.STEPPER_MOTOR_57.value:
                self.control_command(DeviceAdress.STEPPER_MOTOR_57.value, Command.SET_SPEED.value, speed)

    def write_angle(self, id, angle, speed):
        if self.speed_range_check(speed):
            if id == StepperMotorType.STEPPER_MOTOR_42.value:
                self.control_command(DeviceAdress.STEPPER_MOTOR_42.value, Command.WRITE_ANGLE.value, angle, speed)
            elif id == StepperMotorType.STEPPER_MOTOR_57.value:
                self.control_command(DeviceAdress.STEPPER_MOTOR_57.value, Command.WRITE_ANGLE.value, angle, speed)

    def write_steps(self, id, steps, speed, dir):
        if steps > 0 and self.speed_range_check(speed) and self.dir_range_check(dir):
            if id == StepperMotorType.STEPPER_MOTOR_42.value:
                self.control_command(DeviceAdress.STEPPER_MOTOR_42.value, Command.WRITE_STEPS.value, steps, speed, dir)
            elif id == StepperMotorType.STEPPER_MOTOR_57.value:
                self.control_command(DeviceAdress.STEPPER_MOTOR_57.value, Command.WRITE_STEPS.value, steps, speed, dir)

    def control_conveyor_by_switch(self, swicth, speed):
        if self.speed_range_check(speed) and (swicth == SwitchMode.CLOSE.value or swicth == SwitchMode.OPEN.value):
            self.control_command(DeviceAdress.STEPPER_MOTOR_57.value, Command.WRITE_STEPS_BY_SWITCH.value, swicth, speed)

    def oepn_conveyor(self, speed):
        if self.speed_range_check(speed):
            self.control_conveyor_by_switch(SwitchMode.OPEN.value, speed)

    def close_conveyor(self):
        self.control_conveyor_by_switch(SwitchMode.CLOSE.value, 0)

    def write_distance(self, distance, speed, tray_diameter_cm):
        if self.speed_range_check(speed) and (1 <= distance <= 10):
            self.control_command(DeviceAdress.STEPPER_MOTOR_42.value, Command.WRITE_DISTANCE.value, distance, speed, tray_diameter_cm)

    def move_zero(self, speed):
        if self.speed_range_check(speed):
            self.control_command(DeviceAdress.STEPPER_MOTOR_42.value, Command.WRITE_DISTANCE_ZERO.value, speed)