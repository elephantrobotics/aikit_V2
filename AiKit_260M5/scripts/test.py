import os
import time
from pymycobot import *
path2 = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  

print(path2)

new_move_coords_to_angles = [
            [-54, 14.15, 16.34, 0],  # D Sorting area
            [-35, 53.61, -44.64, 0],  # C Sorting area
            [34, 51.15, -40.34, 0],  # A Sorting area
            [52.38, 14.67, 12.83, -0.43],  # B Sorting area
        ]

coord_down_D = 115
coord_down_A = 120
coord_down_C = 120
coord_down_B = 115