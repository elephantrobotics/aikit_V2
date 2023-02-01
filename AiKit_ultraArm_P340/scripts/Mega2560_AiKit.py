from time import time
import serial
import struct
import time

class Mega2560_AiKit(object):
    recv_time_error_string = "Read serial port data timeout."
    invalid_data = -1
    read_timeout = 50
    command_header = 255

    header1 = None
    header2 = None
    adress = None
    length = None
    content = []
    slave_content_valid_data_buff = []     #从机返回的有效数据列表
    slave_content_valid_data_buff_tmp = [] #从机返回的有效数据缓存列表
    cmd = None
    check_digit_user = None
    check_digit_ok = None

    def __init__(self, port, baudrate = "115200", timeout = 0.1, debug = False):
        self._serial_port = serial.Serial()
        self._serial_port.port = port
        self._serial_port.baudrate = baudrate
        self._serial_port.timeout = timeout
        self._serial_port.rts = False
        self._serial_port.open()

    def close_serial(self):
        self._serial_port.close()

    def process_data(self, parity_data_buff):
        instruction_sequence_frame_len_diff = 1
        buff_len = len(parity_data_buff)
        if ((-1 < buff_len) and (buff_len == self.length + instruction_sequence_frame_len_diff)):
            ver = 0
            for item in parity_data_buff:
                ver += item
            return ver
        else:
            return self.invalid_data

    def get_command(self):
        entry_time = time.time()

        #用于存储需要计算校验位的数据列表
        parity_data_buff = []
        while True:
            self.header1 = int.from_bytes(self._serial_port.read(), 'big')
            if self.header1 == self.command_header:
                # print(self.header1)
                self.header2 = int.from_bytes(self._serial_port.read(), 'big')
                if self.header2 == self.command_header:
                    # print(self.header2)

                    #设备地址.
                    self.adress = int.from_bytes(self._serial_port.read(), 'big')
                    # print(self.adress)

                    #长度.
                    self.length = int.from_bytes(self._serial_port.read(), 'big')
                    # print(self.length)

                    #有效数据.
                    for item in range(self.length):
                        read = self._serial_port.read()
                        self.content = int.from_bytes(read, 'big')
                        self.slave_content_valid_data_buff.append(self.content)
                        parity_data_buff.append(self.content)
                    # print(self.content)
                    self.slave_content_valid_data_buff_tmp = self.slave_content_valid_data_buff
                    self.slave_content_valid_data_buff = []

                    #指令序列帧.
                    self.cmd = int.from_bytes(self._serial_port.read(), 'big')
                    parity_data_buff.append(self.cmd)
                    # print(self.cmd)

                    #用户校验码.
                    self.check_digit_user = int.from_bytes(self._serial_port.read(), 'big')
                    # print(self.check_digit_user)

                    #有效校验码.
                    self.check_digit_ok = self.process_data(parity_data_buff)
                    # print(self.check_digit_ok)

                    parity_data_buff = []

                    if (self.check_digit_user == self.check_digit_ok):
                        return 1
                    else:
                        return self.invalid_data
                    

            exit_time = time.time()
            if (exit_time - entry_time) > self.read_timeout:
                print(self.recv_time_error_string)
                return self.invalid_data

    def get_pcb_slave_data(self):
        self.get_command()
        return self.slave_content_valid_data_buff_tmp
        

    def unpack_args(self, *args):
        bits_pack_list = []
        args_list = list(args)
        
        '''
        valid_data_list = []
        # print(args_list)
        for args in args_list:
            pair = struct.pack('>h', args)
            # print(pair)
            if 2 == len(pair):
                valid_data_list.append(pair)
                # print(valid_data_list)
                valid_data_list.append(pair[0])
                valid_data_list.append(pair[1])
            if 2 == len(valid_data_list):
                bits_pack_list.append(valid_data_list)
                valid_data_list = []
            else:
                bits_pack_list = []
        '''

        for args in args_list:
            pair = struct.pack('>h', args)
            if 2 == len(pair):
                bits_pack_list.append(pair[0])
                bits_pack_list.append(pair[1])
            else:
                bits_pack_list = []
        return bits_pack_list
    
    def write_msg_to_slave(self, adress, content, command):
        # print(self.command_header)
        # print(self.command_header)
        # print(adress)
        # print(len(content))
        ver = command
        for item in content:
            ver += item
            # print(item)
        ver %= 255
        # print(command)
        # print(ver)
        self._serial_port.write([self.command_header, self.command_header, adress, len(content), *content, command, ver]);

    def control_command(self, adress, command, duplicate_flag, *args):
        unpack_list = self.unpack_args(*args)

        duplicate_data_buff = []
        if (duplicate_flag):
            for item in unpack_list:
                if 0 < item:
                    duplicate_data_buff.append(item)
        else:
            duplicate_data_buff = unpack_list

        if self.invalid_data < len(duplicate_data_buff):
            self.write_msg_to_slave(adress, duplicate_data_buff, command)

    def control_slide_rail(self, switching):
        # ff ff 31 01 01 a5 a6
        self.control_command(0x31, 0xa5, True, switching)

    def get_tof_dist(self):
        # ff ff 21 00 51 51
        distance = None
        self.control_command(0x21, 0x51, False)
        valid_content_list = self.get_pcb_slave_data()
        if [] != valid_content_list:
            valid_content_list_len = len(valid_content_list)
            if 1 == valid_content_list_len:
                distance = valid_content_list[0]
            elif 2 == valid_content_list_len:
                distance = (valid_content_list[0] << 8) | valid_content_list[1]
            self.slave_content_valid_data_buff_tmp = []
            if None != distance:
                return distance
