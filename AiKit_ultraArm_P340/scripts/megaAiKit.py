from enum import Enum
from serial import Serial
import struct

class DeviceAdress(Enum):
    """Communication protocol device sequence frame list, used to mark different hardware devices"""

    """IR ranging sensor"""
    IR_DETECT = 0x21

    """42 stepper motor"""
    STEPPER_MOTOR_42 = 0x30

    """57 stepper motor"""
    STEPPER_MOTOR_57 = 0x31

class Command(Enum):
    """Communication protocol instruction sequence frame list, used to mark different interfaces"""

    """Get TOF sensor ranging"""
    GET_TOF_DISTANCE = 0x51

    """Modify the direction of motion of the stepper motor"""
    SET_DIR = 0xa0

    """Modify the speed of the stepper motor"""
    SET_SPEED = 0xa1

    """Obtain the direction of motion of the stepper motor"""
    GET_DIR = 0xa2

    """Get the speed of the stepper motor"""
    GET_SPEED = 0xa3

    """Get the moving distance of the slide rail"""
    GET_DISTANCE = 0xa4

    """Control the slide rail movement according to the switch"""
    WRITE_STEPS_BY_SWITCH = 0xa5

    """Angular control of stepper motors"""
    WRITE_ANGLE = 0xa6

    """Control the stepper motor by the number of steps"""
    WRITE_STEPS = 0xa7

    """Control the stepper motor back to zero by distance"""
    WRITE_DISTANCE_ZERO = 0xa8

    """Control stepper motion by distance"""
    WRITE_DISTANCE = 0xa9

"""
    The interval between using this type of interface must be at least 1 millisecond
"""
class megaAikit(object):
    invalid_data = -1
    command_header = 255
    length = 0
    content = []
    cmd = 0
    check_digit_user = 0
    check_digit_ok = invalid_data
    check_digit_flag = False

    """Initialize all configurations"""
    def __init__(self, port, baudrate = "115200", timeout = 0.1, debug = False):
        """
        Args:
            port     : port string
            baudrate : baud rate string, default '115200'
            timeout  : default 0.1
            debug    : whether show debug info, default: False
        """
        import serial
        self._serial_port = Serial()
        self._serial_port.port = port
        self._serial_port.baudrate = baudrate
        self._serial_port.timeout = timeout
        # self._serial_port.rts = False
        # self._serial_port.dtr = False
        self._serial_port.open()


    """Destroy all memory"""
    def __destroy__(self):
        self._serial_port.close()

    """Send data to the slave to respond with the specified command communication protocol"""
    def write(self, adress, content, command):
        # print("write list: ", [self.command_header, self.command_header, adress, len(content), *content, command, self.check_digit(command, content)])
        self._serial_port.flush()
        self._serial_port.write([self.command_header, self.command_header, adress, len(content), *content, command, self.check_digit(command, content)]);

    """Calculate communication protocol check digit"""
    def check_digit(self, cmd, contents):
        ver = cmd
        if 0 < len(contents):
            for item in contents: ver += item
        self.check_digit_flag = True
        ver &= 0xff
        return ver

    """Communication protocol instruction double frame header inspection"""
    def double_header_check(self, read_buff):
        if 2 <= len(read_buff):
            if self.command_header == read_buff[0] and self.command_header == read_buff[1]:
                return True
        return False

    """Read the communication protocol command in the specified format from the slave"""
    def read(self):
        content_begin = 4
        while True:
            if self._serial_port.inWaiting() > 0:
                read_buff = self._serial_port.read(self._serial_port.inWaiting())
                # print("read list: ", read_buff)
                if 6 <= len(read_buff):
                    if self.double_header_check(read_buff):
                        self.length = read_buff[3]
                        if 0 == self.length:
                            self.content = []
                            self.cmd = read_buff[content_begin]
                            self.check_digit_user = read_buff[content_begin + 1]
                        else:
                            content_end = content_begin + self.length
                            self.content = read_buff[content_begin : content_end]
                            self.cmd = read_buff[content_end]
                            self.check_digit_user = read_buff[content_end + 1]

                        self.check_digit_ok = self.check_digit(self.cmd, self.content)
                        
                        if self.check_digit_flag:
                            return True
                        else:
                            self.content = []
                            return False
                    else:
                        self.content = []
                        return False
                else:
                    self.content = []
                    return False


    """Intercept the command returned from the slave except the frame header, frame tail...effective data"""
    def gets_data_from_slave(self):
        read_code = self.read()
        
        valid_data = []
        if [] == self.content:
            return []
        else:
            if read_code and self.check_digit_ok == self.check_digit_user:
                valid_data = self.content
        if [] != valid_data:
            self.content = []
        return valid_data

    """Convert the data of the variable parameter list into high and low lists in turn"""
    def unpack_args(self, *args):
        bits_pack_list = []
        args_list = list(args)
        for args in args_list:
            pair = struct.pack('>h', args)
            if 2 == len(pair):
                bits_pack_list.append(pair[0])
                bits_pack_list.append(pair[1])
            else:
                bits_pack_list = []
        return bits_pack_list

    """Interface for converting user data to communication protocol data"""
    def control_command(self, adress, command, *args):
        unpack_list = self.unpack_args(*args)
        # print("unpack_list: ", unpack_list)

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

        # print("new_data_buff: ", new_data_buff)
        
        if self.invalid_data < len(new_data_buff):
            self.write(adress, new_data_buff, command)

    """Velocity parameter range check"""
    def speed_range_check(self, speed, min = 0, max = 100):
        """
        Args:
            speed : int  (0 ~ 100)
            min   : int  (minimum speed)
            max   : int  (maximum speed)
        """
        return min <= speed <= max

    """Orientation parameter range check"""
    def dir_range_check(self, dir):
        """
        Args:
            dir: int  (0 - counterclockwise (left), 1 - clockwise (right))
        """
        return 0 == dir or 1 == dir;

    """An interface that handles the communication protocol that only returns one data to the user's request"""
    def get_single_data_from_read(self, id, command):
        """
        Args:
            id     : int  (1 - 42 stepper motors, 2 - 57 stepper motors)
            command: byte (instruction sequence frame)
        """
        if 1 == id:
            self.control_command(DeviceAdress.STEPPER_MOTOR_42.value, command)
        elif 2 == id:
            self.control_command(DeviceAdress.STEPPER_MOTOR_57.value, command)
        single_data = self.gets_data_from_slave()
        if 1 == len(single_data):
            return single_data[0]
        else:
            return self.invalid_data
        

    """User Interface""" 

    """Obtain the real-time ranging range of the TOF ranging sensor (0.02m ~ 2m)"""
    def get_tof_distance(self):
        self.control_command(DeviceAdress.IR_DETECT.value, Command.GET_TOF_DISTANCE.value)
        valid_content_list = self.gets_data_from_slave()

        distance = self.invalid_data
        data_length = len(valid_content_list)
        if 0 < data_length:
            if 1 == data_length:
                distance = valid_content_list[0]
            elif 2 == data_length:
                distance = (valid_content_list[0] << 8) | valid_content_list[1]
        if 20 <= distance:
            return distance
        return self.invalid_data


    """Obtain the movement direction of 42 and 57 type stepper motors"""
    def get_dir(self, id):
        """
        Args:
            id : int  (1 - 42 stepper motors, 2 - 57 stepper motors)
        """
        return self.get_single_data_from_read(id, Command.GET_DIR.value)


    """Obtain the movement speed of 42 and 57 type stepper motors"""
    def get_speed(self, id):
        """
        Args:
            id : int  (1 - 42 stepper motors, 2 - 57 stepper motors)
        """
        return self.get_single_data_from_read(id, Command.GET_SPEED.value)


    """Get the moving distance of the slide rail"""
    def get_distance(self):
        return self.get_single_data_from_read(1, Command.GET_DISTANCE.value)


    """Change the direction of movement of 42, 57 stepper motors"""
    def set_dir(self, id, dir):
        """
        Args:
            id : int  (1 - 42 stepper motors, 2 - 57 stepper motors)
            dir: int  (0 - counterclockwise (left), 1 - clockwise (right))
        """
        if self.dir_range_check(dir):
            if 1 == id:
                self.control_command(DeviceAdress.STEPPER_MOTOR_42.value, Command.SET_DIR.value, dir)
            elif 2 == id:
                self.control_command(DeviceAdress.STEPPER_MOTOR_57.value, Command.SET_DIR.value, dir)


    """Change the speed of movement of stepper motors 42 and 57"""
    def set_speed(self, id, speed):
        """
        Args:
            id   : int  (1 - 42 stepper motors, 2 - 57 stepper motors)
            speed: int  (0 ~ 100)
        """
        if self.speed_range_check(speed):
            if 1 == id:
                self.control_command(DeviceAdress.STEPPER_MOTOR_42.value, Command.SET_SPEED.value, speed)
            elif 2 == id:
                self.control_command(DeviceAdress.STEPPER_MOTOR_57.value, Command.SET_SPEED.value, speed)


    """Control the motor with an angle"""
    def write_angle(self, id, angle, speed):
        """
        Args:
            id   : int  (1 - 42 stepper motors, 2 - 57 stepper motors)
            angle: float  (no upper and lower limit, support negative numbers)
            speed: int  (0 ~ 100)
        """
        if self.speed_range_check(speed):
            if 1 == id:
                self.control_command(DeviceAdress.STEPPER_MOTOR_42.value, Command.WRITE_ANGLE.value, angle, speed)
            elif 2 == id:
                self.control_command(DeviceAdress.STEPPER_MOTOR_57.value, Command.WRITE_ANGLE.value, angle, speed)


    """Control the motor with pulses"""
    def write_steps(self, id, steps, speed, dir):
        """
        Args:
            id   : int  (1 - 42 stepper motors, 2 - 57 stepper motors)
            steps: int  (number of steps to send, the minimum number of steps is 1, and the range of negative numbers is not supported)
            speed: int  (0 ~ 100)
            dir  : int  (0 - counterclockwise (left), 1 - clockwise (right))
        """
        if 0 < steps and self.speed_range_check(speed) and self.dir_range_check(dir):
            if 1 == id:
                self.control_command(DeviceAdress.STEPPER_MOTOR_42.value, Command.WRITE_STEPS.value, steps, speed, dir)
            elif 2 == id:
                self.control_command(DeviceAdress.STEPPER_MOTOR_57.value, Command.WRITE_STEPS.value, steps, speed, dir)


    """Control slide rail movement with switch"""
    def control_conveyor_by_switch(self, swicth, speed):
        """
        Args:
            swicth : int  (0 - close rail, 1 - open rail)
            speed  : int  (0 ~ 100)
        """
        if (1 == swicth or 0 == swicth) and self.speed_range_check(speed):
            self.control_command(DeviceAdress.STEPPER_MOTOR_57.value, Command.WRITE_STEPS_BY_SWITCH.value, swicth, speed)
    

    """Control slide rail movement by distance"""
    def write_distance(self, distance, speed, tray_diameter_cm):
        """
        Args:
            distance     : int   (1cm ~ 10cm, negative range is not supported)
            speed        : int   (0 ~ 100)
            tray_diameter_cm: float (diameter of the object tray)
        """
        if (1 <= distance and 10 >= distance) and self.speed_range_check(speed):
            self.control_command(DeviceAdress.STEPPER_MOTOR_42.value, Command.WRITE_DISTANCE.value, distance, speed, tray_diameter_cm)


    """Return the control slide to zero position"""
    def move_zero(self, speed):
        """
        Args:
            speed    : int  (0 ~ 100)
        """
        if self.speed_range_check(speed):
            self.control_command(DeviceAdress.STEPPER_MOTOR_42.value, Command.WRITE_DISTANCE_ZERO.value, speed)
