"""
270PI_wireless_keyboard_mouse_handle_control_raspi_linux.py
This script is suitable for using a wireless keyboard and mouse controller combination to control the MyPalletizer 260 machine with a controller.

Author: Wang Weijian
Date: 2025-07-31
"""
# coding:utf-8
import pygame
import sys
import time
from pymycobot import MyPalletizer260
import platform
import threading
from pymycobot.utils import get_port_list

if "linux" in platform.platform().lower():
    import RPi.GPIO as GPIO

    # Set GPIO mode
    GPIO.setmode(GPIO.BCM)
    # Set pin 20 (electromagnetic valve) and pin 21 (release valve) as output
    GPIO.setup(20, GPIO.OUT)
    GPIO.setup(21, GPIO.OUT)
plist = get_port_list()
print(plist)
# Initialize MyCobot280 with serial port and baud rate
mc = MyPalletizer260('/dev/ttyAMA0', 1000000)

init_angles = [0, 0, 0, 0]
go_home = [0, 0, 0, 0]

# Initialize Pygame and joystick modules
pygame.init()
pygame.joystick.init()
button_pressed = False
hat_pressed = False
previous_state = [0, 0, 0, 0, 0, 0]
STEP = 20
coord_speed = 10

axis_motion_flags = {
    0: False,  # Y
    1: False,  # X
    4: False,  # Z
    3: False,  # RX
}

axis_threads = {}

# Function to turn on the vacuum pump (electromagnetic valve)
def pump_on():
    # 打开电磁阀
    if platform.system() == "Linux":
        GPIO.output(20, 0)
    elif platform.system() == "Windows":
        mc.set_basic_output(5, 0)
        time.sleep(0.05)


# Function to turn off the vacuum pump (close electromagnetic valve and open release)
def pump_off():
    if platform.system() == "Linux":
        # Close valve
        GPIO.output(20, 1)
        time.sleep(0.05)
        # Open release valve
        GPIO.output(21, 0)
        time.sleep(1)
        GPIO.output(21, 1)
        time.sleep(0.05)
    elif platform.system() == "Windows":
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
        print("stop error：", e)


def move_loop(axis_id, direction):
    while axis_motion_flags[axis_id]:
        try:
            coords = mc.get_coords()
            if coords is None or len(coords) < 4:
                continue
            x, y, z, rx = coords
            # print('aixs-->', axis_id)
            if axis_id == 0:  # Y
                y += direction * STEP
                mc.send_coord(2, y, coord_speed)
            elif axis_id == 1:  # X
                x += direction * STEP
                mc.send_coord(1, x, coord_speed)
            elif axis_id == 4:  # Z
                z += direction * STEP
                mc.send_coord(3, z, coord_speed)
            elif axis_id == 3:  # RX
                rx += direction * STEP
                mc.send_coord(4, rx, coord_speed)

            time.sleep(0.01)
        except Exception as e:
            print("Mobile thread exception:", e)


# Handler for joystick input events
def joy_handler():
    global button_pressed
    global hat_pressed
    global previous_state
    # Joystick axis movement (analog stick)
    if event.type == pygame.JOYAXISMOTION:
        axis = event.axis
        value = round(event.value, 2)
        # print('axis-->', axis, value)
        # Axis 2 servo release
        if axis == 2 and value == 1.00:
            mc.release_all_servos()
            time.sleep(0.03)
        # Axis 5 to power on
        elif axis == 5 and value == 1.00:
            mc.power_on()
            time.sleep(0.03)
        if axis in [0, 1, 3, 4]:
            if abs(value) > 0.1:
                direction = -1 if value > 0 else 1
                # print('value', value, axis, direction)
                if not axis_motion_flags[axis]:
                    axis_motion_flags[axis] = True
                    axis_threads[axis] = threading.Thread(target=move_loop, args=(axis, direction))
                    axis_threads[axis].start()
            else:
                if axis_motion_flags[axis]:
                    axis_motion_flags[axis] = False  # 线程会自然退出
                if previous_state[axis] != 0:
                    threading.Thread(target=safe_stop).start()
                    previous_state[axis] = 0
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


# Initialize joystick if detected
if pygame.joystick.get_count() > 0:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    time.sleep(1)
else:
    print("No controller detected")
    pygame.quit()
    sys.exit()
# Main loop to process events
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        joy_handler()
pygame.quit()
