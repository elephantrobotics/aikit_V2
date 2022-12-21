from pymycobot.mycobot import MyCobot
from time import sleep

mc = MyCobot('/dev/ttyAMA0',1000000)

a = mc.get_coords()
print(a)
mc.send_coords([2.2, 128.5, 171.6, 163.27, 10.58, -147.25], 30, 0)

sleep(4)
print(mc.get_coords())
