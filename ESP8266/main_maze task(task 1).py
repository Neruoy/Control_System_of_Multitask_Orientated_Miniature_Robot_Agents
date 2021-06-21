# PC served as a server
import socket
import ConnectWiFi
import MotorControl
import machine
import time
import jy901
from amg88xx import AMG88XX
from machine import Pin, I2C, UART

# Indicator Blue LED for successful connection
led = Pin(2, Pin.OUT)
led.on()

i2c = I2C(scl=Pin(5), sda=Pin(4))
thermal = AMG88XX(i2c)
pxmaddr = 0x52

port = 10000
# Your PC ip
host = '192.168.1.2'


def imu():
    head_gyro = jy901.Gyro(i2c)
    print(" Acc:  " + repr(head_gyro.get_acc()) + "\n", "Gyro: " + repr(head_gyro.get_gyro()) + "\n",
          "Angle:" + repr(head_gyro.get_angle()) + "\n")
    time.sleep(0.5)
    ang = head_gyro.get_angle()[2]
    return ang


def proximity():
    i2c.writeto(pxmaddr, b'0x00')
    time.sleep(0.5)
    pxd = i2c.readfrom(pxmaddr, 2)
    pxd = int.from_bytes(pxd, "big")
    print(str(pxd) + "mm")
    time.sleep(1)
    return pxd


def thermalcam():
    thermal.refresh()
    for row in range(8):
        print()
        for col in range(8):
            print('{:4d}'.format(thermal[row, col]), end='')
    print("\n")
    time.sleep(1)


ConnectWiFi.connect()
rbSocket = socket.socket()
print("Finding...")
rbSocket.connect((host, port))
led.off()
print("TCP connected")


while True:
    data = rbSocket.recv(1)
    if len(data) == 0:
        rbSocket.sendall(b'No data received.')
        break
    led.on()
    decode_data = data.decode("utf8")

    if decode_data == 'w':
        rbSocket.send(b'Forward.')
        MotorControl.forward()
    elif decode_data == 'a':
        rbSocket.send(b'Turn left')
        MotorControl.turnleft()
    elif decode_data == 'd':
        MotorControl.turnright()
    elif decode_data == 's':
        MotorControl.backward()
    elif decode_data == 't':
        rbSocket.send(b'Stop')
        MotorControl.stop()
    elif decode_data == 'f':
        rbSocket.send(b'forward offset')
        MotorControl.forward_offset()
    elif decode_data == 'b':
        rbSocket.send(b'Backward offset')
        MotorControl.back_offset()
    elif decode_data == 'Successful Connection!':
        rbSocket.send(b'Successful Connection!')
    elif decode_data == 'm':
        rbSocket.send(b'Meet enemy. Start attack!')
        MotorControl.arm()
    # else:
        # rbSocket.sendall(b'Invalid input. Please enter again!')

