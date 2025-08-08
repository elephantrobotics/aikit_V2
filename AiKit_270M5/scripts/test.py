from pymycobot.mecharm270 import MechArm270
from time import sleep

mc = MechArm270('COM5',115200)

a = mc.get_coords()
print(a)
mc.send_coords([2.2, 128.5, 171.6, 163.27, 10.58, -147.25], 30, 0)

sleep(4)
print(mc.get_coords())

new_move_coords_to_angles = [
    [-52.64, 35.06, -39.63, -2.28, 82.35, 55.45],  # D
    [-35.59, 61.78, -68.2, -1.14, 68.29, 88.33],  # C
    [32.34, 58.35, -62.13, 4.3, 61.52, 15.64],  # A
    [55.19, 42.71, -46.4, -0.96, 84.19, 15.99]  # B
]

z_down_values = [125, 130, 135, 125]  # D, C, A, B