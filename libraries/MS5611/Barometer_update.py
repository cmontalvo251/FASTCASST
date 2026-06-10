"""
Provided to you by Emlid Ltd (c) 2014.
twitter.com/emlidtech || www.emlid.com || info@emlid.com

Example: Get pressure from MS5611 barometer onboard of Navio shield for Raspberry Pi

To run this example navigate to the directory containing it and run following commands:
sudo python Barometer_example.py

This is an example edited by Carlos Montalvo - 2025

"""

import time

import ms5611
import sys
sys.path.append('../Util')
import util

util.check_apm()
baro = ms5611.MS5611()
baro.calibrate()

while(True):
	baro.update()
	print("Temperature(C): %.6f" % (baro.TEMP), "Pressure(millibar): %.6f" % (baro.PRES), "Altitude (m): %.6f" % (baro.ALT))

