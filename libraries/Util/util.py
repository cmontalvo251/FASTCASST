import subprocess as sub
import sys
import os

def check_apm():
	print('Checking to make sure APM is off')
	ret = sub.call(["ps -AT | grep -c ap-timer > /dev/null"], shell = True)
	if ret <= 0:
		sys.exit("APM is running. Can't launch the example")
	else:
		print('APM is not running....')

def isSIL():
	if os.uname().nodename == 'navio':
		return 0
	else:
		return 1