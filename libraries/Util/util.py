import subprocess as sub
import sys
import os

def check_apm():
	# Bypass Linux/Raspberry Pi ArduPilot process check on macOS
	if sys.platform == 'darwin':
		print('macOS detected: Skipping APM check.')
		return
	print('Checking to make sure APM is off')
	ret = sub.call(["ps -AT | grep -c ap-timer > /dev/null"], shell = True)
	if ret <= 0:
		sys.exit("APM is running. Can't launch the example")
	else:
		print('APM is not running....')

"""
#Deprecating this function in lieu of adding a MODE flag
def isSIL():
	if os.uname().nodename == 'navio':
		return 0
	else:
		return 1
"""