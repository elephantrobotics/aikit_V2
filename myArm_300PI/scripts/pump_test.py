#!/usr/bin/python
# -*- coding:utf-8 -*-
# @File    : pump_test.py
# @Author  : Wang Weijian
# @Time    :  2023/11/07 11:59:02
# @function: the script is used to do something
# @version : V1

from pymycobot.mycobot import MyCobot
from pymycobot.myarm import MyArm
import time

# 初始化一个MyCobot对象
#mc = MyCobot("COM7", 115200)
mc = MyArm('/dev/ttyAMA0', 115200)

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
import RPi.GPIO as GPIO
GPIO.setwarnings(False)

GPIO.setmode(GPIO.BCM)
GPIO.setup(20, GPIO.OUT)
GPIO.setup(21, GPIO.OUT)
GPIO.output(20, 1)
GPIO.output(21, 1)

def gpio_status(flag):
    if flag:
        GPIO.output(20, 0)
        GPIO.output(21, 0)
    else:
        GPIO.output(20, 1)
        GPIO.output(21, 1)

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

#pump_off()
gpio_status(False)
time.sleep(3)
#pump_on()
gpio_status(True)
time.sleep(4)
#pump_off()
gpio_status(False)

time.sleep(3)
