"""
Provided to you by Emlid Ltd (c) 2014.
twitter.com/emlidtech || www.emlid.com || info@emlid.com

Example: Get pressure from MS5611 barometer onboard of Navio shield for Raspberry Pi

To run this example navigate to the directory containing it and run following commands:
sudo python Barometer_example.py

This is an example edited by Carlos Montalvo - 2025

"""

##Check for APM
import sys
sys.path.append('../Util')
import util
util.check_apm()

##Import barometer
import ms5611
baro = ms5611.MS5611()

#Calibrate barometer
baro.calibrate() #if you don't calibrate (sea level defaults to 1013.25

##Initialize time
import time
StartTime = time.time()
RunTime = 0

while(True):
	RunTime = time.time() - StartTime
	baro.poll(RunTime)
	print("Time: %.3f" % (RunTime),"Pressure(millibar): %.3f" % (baro.PRES), "Altitude (m): %.3f" % (baro.ALT))
	#time.sleep(0.01) -- Because there are sleeps in the barometer you don't need this anymore

