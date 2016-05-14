#!/usr/bin/python
import bluetooth
import struct
import time
from sphero_driver import sphero_driver
import sys

SPHERO1_NAME = "Sphero-RBY"
SPHERO1_ADDR = "68:86:E7:07:85:E9"
SPHERO2_NAME = "Sphero-ROB"
SPHERO2_ADDR = "68:86:E7:02:8D:CF"
SPHERO3_NAME = "Sphero-RGR"
SPHERO3_ADDR = "68:86:E7:06:E7:1F"
SPHERO4_NAME = "Sphero-OGG"
SPHERO4_ADDR = "68:86:E7:02:8B:A0"

sphero1 = sphero_driver.Sphero(SPHERO1_NAME, SPHERO1_ADDR)
sphero2 = sphero_driver.Sphero(SPHERO2_NAME, SPHERO2_ADDR)
sphero3 = sphero_driver.Sphero(SPHERO1_NAME, SPHERO1_ADDR)
sphero4 = sphero_driver.Sphero(SPHERO2_NAME, SPHERO2_ADDR)
sphero1.connect()
sphero2.connect()
sphero3.connect()
sphero4.connect()
sphero1.set_raw_data_strm(40, 1 , 0, False)
sphero2.set_raw_data_strm(40, 1 , 0, False)
sphero3.set_raw_data_strm(40, 1 , 0, False)
sphero4.set_raw_data_strm(40, 1 , 0, False)

sphero1.start()
sphero2.start()
sphero3.start()
sphero4.start()
time.sleep(2)
print "disable stabilization"
sphero1.set_stablization(0, False)
sphero2.set_stablization(0, False)
sphero3.set_stablization(0, False)
sphero4.set_stablization(0, False)
time.sleep(3)
print "set color to RED"
sphero1.set_rgb_led(255,0,0,0,False)
sphero2.set_rgb_led(255,0,0,0,False)
sphero3.set_rgb_led(255,0,0,0,False)
sphero4.set_rgb_led(255,0,0,0,False)
time.sleep(1)
print "set color to GREEN"
sphero1.set_rgb_led(0,255,0,0,False)
sphero2.set_rgb_led(0,255,0,0,False)
sphero3.set_rgb_led(0,255,0,0,False)
sphero4.set_rgb_led(0,255,0,0,False)
time.sleep(1)
print "set color to BLUE"
sphero1.set_rgb_led(0,0,255,0,False)
sphero2.set_rgb_led(0,0,255,0,False)
sphero3.set_rgb_led(0,0,255,0,False)
sphero4.set_rgb_led(0,0,255,0,False)
time.sleep(3)
print "set back led"
sphero1.set_rgb_led(255,255,255,0,False)
sphero2.set_rgb_led(255,255,255,0,False)
sphero3.set_rgb_led(255,255,255,0,False)
sphero4.set_rgb_led(255,255,255,0,False)
sphero1.set_back_led(255,False)
sphero2.set_back_led(255,False)
sphero3.set_back_led(255,False)
sphero4.set_back_led(255,False)
time.sleep(3)
time.sleep(3)
print "enable stablization"
sphero1.set_stablization(1, False)
sphero2.set_stablization(1, False)
sphero3.set_stablization(1, False)
sphero4.set_stablization(1, False)
time.sleep(5)
print "set aiming"
sphero1.roll(0, 90, 0, False)
sphero1.set_heading(90, False)
sphero2.roll(0, 90, 0, False)
sphero2.set_heading(90, False)
sphero3.roll(0, 90, 0, False)
sphero3.set_heading(90, False)
sphero4.roll(0, 90, 0, False)
sphero4.set_heading(90, False)
time.sleep(3)
time.sleep(15)
#sphero.join()
print "disconnet"
sphero1.disconnect()
sphero2.disconnect()
sphero3.disconnect()
sphero4.disconnect()
sys.exit(1)



