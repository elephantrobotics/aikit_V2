from pymycobot.mycobot280 import MyCobot280
import time
from pymycobot import PI_BAUD, PI_PORT
import serial
import serial.tools.list_ports

plist = [
    str(x).split(" - ")[0].strip() for x in serial.tools.list_ports.comports()
]

mc = MyCobot280(plist[0], 115200)

coords = [ 
        [135.0, -65.5, 280.1, 178.99, 5.38, -179.9],
        [136.1, -141.6, 243.9, 178.99, 5.38, -179.9]
    ]

angles = [0, 0, 0, 0, 0, 0]

new_move_coords_to_angles = [
            [-33.22, -10.28, -84.99, 4.83, 0.08, -7.99],  # D Sorting area
            [-21.79, -52.82, -26.45, -5.53, 0.08, -7.91],  # C Sorting area
            [47.81, -53.61, -27.15, -6.41, 0.08, -7.73],  # A Sorting area
            [72.42, -6.06, -98.43, 14.23, -0.87, -8.96],  # B Sorting area
        ]

coord_down_D = 138
coord_down_A = 147
coord_down_C = 145
coord_down_B = 135

mc.send_angles(new_move_coords_to_angles[0], 50)
