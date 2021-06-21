import network
import webrepl
webrepl.start()

ap = network.WLAN(network.AP_IF)
ap.active(True)
ap.config(essid='ME-ESP8266-1')
ap.config(authmode=4, password='43499ME2021')
sta = network.WLAN(network.STA_IF)
sta.active(True)
print("Hi! I am RB8266-1, listening...")



