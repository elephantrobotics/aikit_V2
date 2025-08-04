"""
280M5_wireless_keyboard_mouse_handle_control_raspi_linux.py
This script is suitable for using a wireless keyboard and mouse controller combination to control the mycobot 280 M5 machine with a controller.

Author: Wang Weijian
Date: 2025-07-02
"""
import argparse

# coding:utf-8
import pygame
import sys
import time
from pymycobot import MyCobot280
import platform
import threading
from pymycobot.utils import get_port_list

plist = get_port_list()
print(plist)
# Initialize MyCobot280 with serial port and baud rate
parser = argparse.ArgumentParser(description='MyCobot Joystick Controller')
parser.add_argument('--port', type=str, default='/dev/ttyACM0', help='设备串口号')
parser.add_argument('--baud', type=int, default=115200, help='波特率')
args = parser.parse_args()
print(f"使用串口: {args.port}, 波特率: {args.baud}")
mc = MyCobot280(args.port, args.baud)

init_angles = [0, 0, -90, 0, 0, 0]
go_home = [0, 0, 0, 0, 0, 0]

# Initialize Pygame and joystick modules
pygame.init()
pygame.joystick.init()
button_pressed = False
hat_pressed = False
previous_state = [0, 0, 0, 0, 0, 0]

joystick = None
last_joystick_check_time = time.time()

def init_joystick():
    global joystick
    pygame.joystick.quit()
    pygame.joystick.init()
    count = pygame.joystick.get_count()
    if count > 0:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        print(f"[Joystick] Connected: {joystick.get_name()}")
    else:
        joystick = None
        print("[Joystick] No joystick found.")


init_joystick()

# Function to turn on the vacuum pump (electromagnetic valve)
def pump_on():
    # 打开电磁阀
    mc.set_basic_output(5, 0)
    time.sleep(0.05)


# Function to turn off the vacuum pump (close electromagnetic valve and open release)
def pump_off():
    # Close valve
    mc.set_basic_output(5, 1)
    time.sleep(0.05)
    # Open release valve
    mc.set_basic_output(2, 0)
    time.sleep(1)
    mc.set_basic_output(2, 1)
    time.sleep(0.05)


# Function to safely stop the robot (used in a thread)
def safe_stop():
    try:
        mc.stop()
        time.sleep(0.02)
    except Exception as e:
        print("stop 出错：", e)


# Handler for joystick input events
def joy_handler():
    global button_pressed
    global hat_pressed
    global previous_state
    # Joystick axis movement (analog stick)
    if event.type == pygame.JOYAXISMOTION:
        axis = event.axis
        value = round(event.value, 2)
        if abs(value) > 0.1:
            flag = True
            previous_state[axis] = value
            # print('axis', axis, 'value', value)
            # jog_coord(index, direction, speed)
            # direction: 0 = negative direction, 1 = positive direction
            if axis == 0 and value == -1.00:
                mc.jog_coord(2, 1, 50)  # Y Axis increase
            elif axis == 0 and value == 1.00:
                mc.jog_coord(2, 0, 50)  # Y Axis decrease
            if axis == 1 and value == 1.00:
                mc.jog_coord(1, 0, 50)  # X Axis decrease
            elif axis == 1 and value == -1.00:
                mc.jog_coord(1, 1, 50)  # X Axis increase
            if axis == 4 and value == 1.00:
                mc.jog_coord(3, 0, 50)  # Z Axis decrease
            elif axis == 4 and value == -1.00:
                mc.jog_coord(3, 1, 50)  # Z Axis increase

            if axis == 3 and value == 1.00:
                mc.jog_coord(6, 0, 50)  # RZ Axis decrease
            elif axis == 3 and value == -1.00:
                mc.jog_coord(6, 1, 50)  # RZ Axis increase

            elif axis == 2 and value == 1.00:
                # print('release all servos')
                mc.release_all_servos()
                time.sleep(0.03)

            # Axis 5 to power on
            elif axis == 5 and value == 1.00:
                mc.power_on()
                time.sleep(0.03)
        else:
            if previous_state[axis] != 0:
                # mc.stop()
                threading.Thread(target=safe_stop).start()
                previous_state[axis] = 0
        # Joystick button pressed
    elif event.type == pygame.JOYBUTTONDOWN:
        # for i in range(joystick.get_numbuttons()):
        #     if joystick.get_button(i):
        #         print(f"按钮 {i} 被按下")
        if joystick.get_button(2) == 1:
            mc.set_gripper_state(0, 100)  # gripper on
        elif joystick.get_button(3) == 1:
            mc.set_gripper_state(1, 100)  # gripper off
        elif joystick.get_button(0) == 1:
            pump_on()  # pump on
        elif joystick.get_button(1) == 1:
            pump_off()  # pump off
        elif joystick.get_button(5) == 1:
            mc.send_angles(init_angles, 50)  # go init pos
            time.sleep(2)
        elif joystick.get_button(4) == 1:
            mc.send_angles(go_home, 50)  # go zero pos
            time.sleep(3)
    # D-Pad (HAT) movement
    elif event.type == pygame.JOYHATMOTION:

        hat_value = joystick.get_hat(0)
        # print('hat--->', hat_value)
        if hat_value == (0, -1):
            mc.jog_coord(4, 0, 50)  # RX Axis decrease
        elif hat_value == (0, 1):
            mc.jog_coord(4, 1, 50)  # RX Axis increase
        elif hat_value == (-1, 0):
            mc.jog_coord(5, 1, 50)  # RY Axis increase
        elif hat_value == (1, 0):
            mc.jog_coord(5, 0, 50)  # RY Axis decrease
        if hat_value != (0, 0):
            hat_pressed = True
        else:
            if hat_pressed:
                # mc.stop()
                threading.Thread(target=safe_stop).start()
                hat_pressed = False

# Initialize joystick if detected
# if pygame.joystick.get_count() > 0:
#     joystick = pygame.joystick.Joystick(0)
#     joystick.init()
#     time.sleep(1)
# else:
#     print("No controller detected")
#     pygame.quit()
#     sys.exit()
# # Main loop to process events
# running = True
# while running:
#     for event in pygame.event.get():
#         if event.type == pygame.QUIT:
#             running = False
#         joy_handler()
# pygame.quit()
running = True
while running:
    if (joystick is None or not joystick.get_init()) and time.time() - last_joystick_check_time > 5:
        init_joystick()
        last_joystick_check_time = time.time()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if joystick is not None:
            joy_handler()

pygame.quit()
