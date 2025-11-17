import time

import serial.tools.list_ports
from pymycobot.mycobot320 import MyCobot320
from common import limit_coords

# plist = [
#     str(x).split(" - ")[0].strip() for x in serial.tools.list_ports.comports()
# ]
#
# print(plist)
# mc = MyCobot320(plist[0], 115200)
# print(mc.get_angles())
#
# mc.set_gripper_mode(0)
# time.sleep(0.5)
#
#
# def gripper_on():
#     """start gripper"""
#     mc.set_gripper_state(0, 100)
#     time.sleep(1.5)
#
#
# def gripper_off():
#     """stop gripper"""
#     mc.set_gripper_state(1, 100)
#     time.sleep(1.5)
#
#
# gripper_off()
#
# time.sleep(3)
#
# gripper_on()
# time.sleep(4)
# gripper_off()
#
# gripper_on()

target1 = [500, -360, 230, -173.84, -0.14, -74.37]
target1 = limit_coords(target1)  # <--

print(target1)