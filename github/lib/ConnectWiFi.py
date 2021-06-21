import network
import time

hessid = "HMT-Freshmen"
hpwd = "stars15hmt"
lessid = "NETGEAR03"
lpwd = "kindzoo611"
cessid = "Cathrine"
cpwd = "aaaaaaaa"

sta = network.WLAN(network.STA_IF)
ap = network.WLAN(network.AP_IF)
ap.active(True)

def connect():
    if sta.isconnected() == True:
        print("Already connected")
        return

    while not(sta.isconnected()):
        print("Connecting...")
        sta.active(True)
        # sta.connect(hessid, hpwd)
        # time.sleep(3)
        sta.connect(lessid, lpwd)
        time.sleep(3)
        # sta.connect(cessid, cpwd)
        # time.sleep(3)

def findip():
    return sta.ifconfig()[0]
