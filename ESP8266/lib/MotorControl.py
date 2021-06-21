import machine
from machine import Pin
import time

p15 = Pin(15, Pin.OUT)
p14 = Pin(14, Pin.OUT)
p13 = Pin(13, Pin.OUT)
p12 = Pin(12, Pin.OUT)
p2 = Pin(2, Pin.OUT)
p15.off()
p14.off()
p13.off()
p12.off()
p2.off()


def forward():
    p15.off()
    p14.on()
    p13.off()
    p12.on()


def turnleft():
    p15.on()
    p14.off()
    p13.off()
    p12.on()


def turnright():
    p15.off()
    p14.on()
    p13.on()
    p12.off()


def backward():
    p15.on()
    p14.off()
    p13.on()
    p12.off()


def stop():
    p15.off()
    p14.off()
    p13.off()
    p12.off()


def forward_offset():
    forward()
    time.sleep(5)
    stop()


def back_offset():
    time.sleep(6)
    backward()


def arm():
    p2.on()


def turn():
    turnleft()
    time.sleep(1.8)
    forward()
    time.sleep(8)


def circle():
    forward()
    time.sleep(8)
    turnright()
    time.sleep(1.8)
    forward()
    time.sleep(4)

    turn()
    turn()
    turn()

    turnleft()
    time.sleep(1.8)
    forward()
    time.sleep(4)
    stop()


def attack():
    forward()
    time.sleep(8)
    # MotorControl.arm()
    stop()
    p2.off()