from pymycobot.mycobot import MyCobot
from time import sleep

mc = MyCobot('/dev/ttyAMA0',1000000)

a = mc.get_coords()
print(a)
