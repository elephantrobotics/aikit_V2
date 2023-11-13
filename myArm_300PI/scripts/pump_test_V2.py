from pymycobot.myarm import MyArm
import time
import RPi.GPIO as GPIO

mc = MyArm('/dev/ttyAMA0', 115200)
GPIO.setmode(GPIO.BCM)
GPIO.setup(20, GPIO.OUT)
GPIO.setup(21, GPIO.OUT)

def pump_on():
    GPIO.output(20, 0)
    

def pump_off():
    GPIO.output(20, 1)
    time.sleep(0.05)
    GPIO.output(21, 0)
    time.sleep(1)
    GPIO.output(21, 1)
    time.sleep(0.05)
    

pump_on()
time.sleep(4)
pump_off()
time.sleep(3)
