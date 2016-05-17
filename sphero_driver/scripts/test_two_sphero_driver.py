#!/usr/bin/python
import bluetooth
import struct
import time, threading
from sphero_driver import sphero_driver
import sys

SPHERO1_NAME = "Sphero-RBY"
SPHERO1_ADDR = "68:86:E7:07:85:E9"
SPHERO2_NAME = "Sphero-OGG"
SPHERO2_ADDR = "68:86:E7:02:8B:A0"

NO_WAIT = True

mutual_lock = threading.Lock()
sphero1 = sphero_driver.Sphero(SPHERO1_NAME, SPHERO1_ADDR, mutual_lock)
sphero2 = sphero_driver.Sphero(SPHERO2_NAME, SPHERO2_ADDR, mutual_lock)
sphero1.connect()
sphero2.connect()
sphero1.set_raw_data_strm(40, 1 , 0, False)
sphero2.set_raw_data_strm(40, 1 , 0, False)

sphero1.start()
if NO_WAIT == False:
    time.sleep(2)
sphero2.start()
if NO_WAIT == False:
    time.sleep(2)
print "disable stabilization"
sphero1.set_stablization(0, False)
if NO_WAIT == False:
    time.sleep(2)
sphero2.set_stablization(0, False)
if NO_WAIT == False:
    time.sleep(2)
print "set color to RED"
sphero1.set_rgb_led(255,0,0,0,False)
if NO_WAIT == False:
    time.sleep(2)
sphero2.set_rgb_led(255,0,0,0,False)
if NO_WAIT == False:
    time.sleep(2)
print "set color to GREEN"
sphero1.set_rgb_led(0,255,0,0,False)
if NO_WAIT == False:
    time.sleep(2)
sphero2.set_rgb_led(0,255,0,0,False)
time.sleep(2)
print "set color to BLUE"
sphero1.set_rgb_led(0,0,255,0,False)
if NO_WAIT == False:
    time.sleep(2)
sphero2.set_rgb_led(0,0,255,0,False)
if NO_WAIT == False:
    time.sleep(2)
print "set back led"
sphero1.set_rgb_led(255,255,255,0,False)
if NO_WAIT == False:
    time.sleep(2)
sphero2.set_rgb_led(255,255,255,0,False)
if NO_WAIT == False:
    time.sleep(2)
sphero1.set_back_led(255,False)
if NO_WAIT == False:
    time.sleep(2)
sphero2.set_back_led(255,False)
time.sleep(3)
print "enable stablization"
sphero1.set_stablization(1, False)
if NO_WAIT == False:
    time.sleep(2)
sphero2.set_stablization(1, False)
if NO_WAIT == False:
    time.sleep(2)
print "set aiming"
sphero1.roll(0, 90, 0, False)
if NO_WAIT == False:
    time.sleep(2)
sphero1.set_heading(90, False)
if NO_WAIT == False:
    time.sleep(2)
sphero2.roll(0, 90, 0, False)
if NO_WAIT == False:
    time.sleep(2)
sphero2.set_heading(90, False)
if NO_WAIT == False:
    time.sleep(2)
#sphero1.join()
#sphero2.join()a
time.sleep(5)
print "disconnet"
sphero1.disconnect()
if NO_WAIT == False:
    time.sleep(2)
sphero2.disconnect()
sys.exit(1)



