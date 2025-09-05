"""
gripper_block_demo.py
This module controls the robotic arm movements with adaptive waiting.

Author: Wang Weijian
Date: 2025-09-03
"""
import argparse
import time
from pymycobot import MyCobot280

class GripperBlockDemo:
    def __init__(self, port='COM39', baud=115200):
        self.mc = MyCobot280(port, baud)
        if self.mc.get_fresh_mode() != 0:
            self.mc.set_fresh_mode(0)
        # Common positions (angles and coordinates)
        self.HOME = [0, 0, 0, 0, 0, 0]
        self.INIT_POSE = [-3.33, 18.98, -54.4, -52.73, 0.52, -45.43]
        self.HOME_POSE = [0.79, -4.83, -94.13, 10.63, 0.35, -45]

        # Up and down movement
        self.UP_POINT = [92.3, -68.4, 278.9, -178.32, -0.94, -47.9]
        self.DOWN_POINT = [92.3, -68.4, 198.9, -178.32, -0.94, -47.9]

        # Point A
        # self.A_TOP = [183.6, -58.3, 198.1, -176.17, -3.09, -48.26]
        self.A_TOP = [150.8, -120.6, 191.8, -178.91, -1.63, -42.19]
        # self.A_GRAB = [183.6, -58.3, 128.1, -176.17, -3.09, -48.26]
        self.A_GRAB = [150.8, -120.6, 128.8, -178.91, -1.63, -42.19]

        # Point B
        # self.B_TOP_ANGLE = [72.42, -6.06, -98.43, 14.23, -0.87, -45]
        self.B_TOP_ANGLE = [71.27, -14.15, -99.14, 21.44, 0.08, -63.98]
        # self.B_PLACE = [117.2, 148.2, 130.3, 177.36, 1.45, 25.65]
        self.B_PLACE = [120.5, 158.3, 130.5, 179.26, 1.69, 45.25]

    def sleep(self, t: float = 2.0):
        """Pause execution for a given number of seconds (can be float)."""
        time.sleep(t)

    def go_home(self, speed=50):
        self.mc.send_angles(self.HOME, speed)
        self.sleep()

    def move_to_init(self, speed=50):
        self.mc.send_angles(self.INIT_POSE, speed)
        self.sleep(0.5)

    def up_down_gripper(self, repeat=3, speed=90):
        """Perform up and down movement with gripper open/close"""
        for _ in range(repeat):
            self.mc.send_coords(self.DOWN_POINT, speed, 1)
            self.sleep(1)
            self.mc.set_gripper_state(0, 80, 1)  # Close gripper

            self.mc.send_coords(self.UP_POINT, speed, 1)
            self.sleep(1)
            self.mc.set_gripper_state(1, 80, 1)  # Open gripper

    def grab_from_A(self, speed=90):
        """Grab block from point A"""
        self.mc.send_coords(self.A_TOP, speed, 1)
        self.sleep(1.5)
        self.mc.set_gripper_value(100, 80, 1)  # Open
        self.sleep(1)
        self.mc.send_coords(self.A_GRAB, speed, 1)
        self.sleep(1)
        self.mc.set_gripper_value(50, 80, 1)  # Close
        self.sleep(1)

        self.mc.send_coords(self.A_TOP, speed, 1)
        self.sleep(1)

    def place_to_B(self, speed=90):
        """Place block to point B"""
        self.mc.send_angles(self.B_TOP_ANGLE, 50)
        self.sleep()

        self.mc.send_coords(self.B_PLACE, speed, 1)
        self.sleep(1)
        self.mc.set_gripper_value(100, 80, 1)  # Open
        self.sleep(1)

        self.mc.send_angles(self.B_TOP_ANGLE, 50)
        self.sleep(1)

        self.mc.send_angles(self.HOME_POSE, speed)
        self.sleep()

    def run(self):
        """Execute full demo process"""
        print(">>> Go home")
        self.go_home(90)

        print(">>> Move to init pose")
        self.move_to_init(50)

        print(">>> Up-down movement with gripper actions")
        self.up_down_gripper(3, 90)

        print(">>> Grab block from A")
        self.grab_from_A(90)

        print(">>> Place block to B")
        self.place_to_B(90)

        print(">>> Back to home")
        self.go_home(50)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Adaptive gripper demonstration')
    parser.add_argument('--port', type=str, default='/dev/ttyACM0', help='Device serial port number')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate')
    args = parser.parse_args()
    print(f"Using serial port: {args.port}, baud rate: {args.baud}")
    demo = GripperBlockDemo(port=args.port, baud=args.baud)
    demo.run()

