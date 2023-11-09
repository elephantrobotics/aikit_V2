#!/usr/bin/python
# -*- coding:utf-8 -*-
# @File    : pump_test.py
# @Author  : Wang Weijian
# @Time    :  2023/11/07 11:59:02
# @function: the script is used to do something
# @version : V1

from pymycobot.mycobot import MyCobot
import time

# 初始化一个MyCobot对象
mc = MyCobot("COM7", 115200)

# # 开启吸泵
# def pump_on():
#     # 打开电磁阀
#     mc.set_basic_output(5, 0)
#     time.sleep(0.05)
#
# # 停止吸泵
# def pump_off():
#     # 关闭电磁阀
#     mc.set_basic_output(5, 1)
#     time.sleep(0.05)
#     # 泄气阀门开始工作
#     mc.set_basic_output(2, 0)
#     time.sleep(1)
#     mc.set_basic_output(2, 1)
#     time.sleep(0.05)

def pump_on():
    # 让2号位工作
    mc.set_basic_output(2, 0)
    # 让5号位工作
    mc.set_basic_output(5, 0)


# 停止吸泵 m5
def pump_off():
    # 让2号位停止工作
    mc.set_basic_output(2, 1)
    # 让5号位停止工作
    mc.set_basic_output(5, 1)

pump_off()
time.sleep(3)
pump_on()
time.sleep(3)
pump_off()
time.sleep(3)