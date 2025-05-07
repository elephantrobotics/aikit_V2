import time
from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Device
from gpiozero import LED

Device.pin_factory = LGPIOFactory(chip=0) # 显式指定/dev/gpiochip0
# 初始化 GPIO 控制的设备
valve = LED(72)  # 阀门


def gpio_status(flag):
    if flag:
        valve.off()
    else:
        valve.on()


# 打开吸泵
gpio_status(True)
time.sleep(3)

# 关闭吸泵
gpio_status(False)
time.sleep(2)

# 打开吸泵
gpio_status(True)
time.sleep(3)
# 关闭吸泵
gpio_status(False)