#!/usr/bin/python
import bluetooth
import struct
import time
from sphero_driver import sphero_driver
import sys
sphero = sphero_driver.Sphero()
sphero.connect()
sphero.set_raw_data_strm(40, 1 , 0, False)

sphero.start()
time.sleep(2)
print "disable stabilization"
sphero.set_stablization(0, False)
time.sleep(3)
print "set color to RED"
sphero.set_rgb_led(255,0,0,0,False)
time.sleep(1)
print "set color to GREEN"
sphero.set_rgb_led(0,255,0,0,False)
time.sleep(1)
print "set color to BLUE"
sphero.set_rgb_led(0,0,255,0,False)
time.sleep(3)
print "set back led"
sphero.set_rgb_led(255,255,255,0,False)
sphero.set_back_led(255,False)
time.sleep(3)
print "enable stablization"
sphero.set_stablization(1, False)
time.sleep(3)
print "set heading"
sphero.set_heading(180, False)
time.sleep(3)
time.sleep(15)
#sphero.join()
print "disconnet"
sphero.disconnect()
sys.exit(1)



