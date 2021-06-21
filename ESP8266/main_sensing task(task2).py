# PC served as a server
import math
import socket
import ConnectWiFi
import MotorControl
import machine
import time
# import jy901
from amg88xx import AMG88XX
from machine import Pin, I2C, UART
import json

# Indicator Blue LED for successful connection
led = Pin(2, Pin.OUT)
led.on()

i2c = I2C(scl=Pin(5), sda=Pin(4))
thermal = AMG88XX(i2c)
pxmaddr = 0x52

port = 10000
host = '192.168.1.12'

angle_list = []
pxd_list = []

delta_1 = 8  # detect the range to stop step turning
delta_2 = 3  # approaching accuracy
delta_pxd = 50  # nearest distance to the object

ConnectWiFi.connect()
rbSocket = socket.socket()
print("Finding...")
rbSocket.connect((host, port))
led.off()
print("TCP connected")


def imu():
    head_gyro = jy901.Gyro(i2c)
    az = head_gyro.get_angle()[2]
    return az


def proximity():
    i2c.writeto(pxmaddr, b'0x00')
    time.sleep(0.2)
    pxd = i2c.readfrom(pxmaddr, 2)
    pxd = int.from_bytes(pxd, "big")
    time.sleep(1)
    return pxd


def thermalcam():
    thermal_list = []
    thermal.refresh()
    for row in range(8):
        for col in range(8):
            thermal_list.append(int(thermal[row, col]))
    time.sleep(1)
    return thermal_list


def step():
    MotorControl.turnright()
    time.sleep(0.1)
    MotorControl.stop()


def get_distance_of_an_angle():
    d = proximity()
    rbSocket.sendall(b'Distance from PXY is ... ' + str(d).encode("utf8") + b' ')
    pxd_list.append(d)


def minimum_pxd_dis(alist, plist):
    min_distance = plist[0]
    if len(alist) != len(plist):
        print("Error data set!")
        rbSocket.send(b'Error data set!')
    else:
        for i in range(len(plist)):
            if plist[i] < min_distance:
                min_distance = plist[i]
                corresponding_angle = alist[i]
        rbSocket.send(str(corresponding_angle).encode("utf8"))
    return corresponding_angle


def scan(ai):
    while True:
        step()
        a_current = imu()
        angle_list.append(a_current)
        get_distance_of_an_angle()
        rbSocket.sendall(b'Current angle is ... ' + str(a_current).encode("utf8"))
        # print("Here-11", a_current)
        if len(angle_list) > 15 and ai - delta_1 < a_current < ai + delta_1:
            # print("Here-12")
            found_angle = minimum_pxd_dis(angle_list, pxd_list)
            rbSocket.sendall(b'\nFinal angle is ...' + str(found_angle).encode("utf8"))
            return found_angle


def approach(found_angle):
    while True:
        a_temp = imu()
        if found_angle - delta_2 >= a_temp or a_temp >= found_angle + delta_2:
            rbSocket.sendall(b'Turning(Approaching)..Now is:' + str(a_temp).encode("utf8"))
            step()
            time.sleep(0.5)

        else:
            while True:
                d_temp = proximity()
                if d_temp > delta_pxd:
                    MotorControl.forward()
                    rbSocket.sendall(b'Approaching target...' + str(d_temp).encode("utf8") + b'mm')
                else:
                    MotorControl.stop()
                    return "reach"


def avoid():
    MotorControl.turnright()
    time.sleep(1)
    MotorControl.stop()


def find_edge_length(ai, pi):
    while True:
        step()
        p_current = proximity()
        if p_current > 500:
            MotorControl.stop()
            af = imu()
            object_length = pi * math.tan(abs(af - ai) / (180 * math.pi))
            avoid()
            return str(object_length)


while True:
    data = rbSocket.recv(1)
    if len(data) == 0:
        print("No data")
        rbSocket.sendall(b'No data received.')
    print("Received:", data)
    led.on()
    decode_data = data.decode("utf8")

    if decode_data == 'o':
        rbSocket.sendall(b'Received command o.')
        ai = imu()
        found_angle = scan(ai)
        approach(found_angle)
        # print("Arrived.")
        rbSocket.sendall(b'Reach the object')

    elif decode_data == 'l':
        rbSocket.sendall(b'Received command l.')
        thermal_array = json.dumps(thermalcam())
        # print(thermal_array)
        rbSocket.sendall(thermal_array.encode('utf8'))

    elif decode_data == 'k':
        MotorControl.attack()

    elif decode_data == 'g':
        length = find_edge_length(imu(), proximity())
        rbSocket.sendall(length.encode('utf8'))

    elif decode_data == 'r':
        MotorControl.circle()
        rbSocket.sendall(b'Complete')
        # rbSocket.sendall(length.encode('utf8'))

    elif decode_data == 'w':
        MotorControl.forward()
    elif decode_data == 'a':
        MotorControl.turnleft()
    elif decode_data == 'd':
        MotorControl.turnright()
    elif decode_data == 's':
        MotorControl.backward()
    elif decode_data == 't':
        MotorControl.stop()

    else:
        rbSocket.send(b'--Waiting for next command--')
        MotorControl.stop()

    # time.sleep(2)

