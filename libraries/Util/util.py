import subprocess as sub
import sys
import os

def check_apm():
    ret = sub.call(["ps -AT | grep -c ap-timer > /dev/null"], shell = True)
    if ret <= 0:
        sys.exit("APM is running. Can't launch the example")
    else:
    	print('APM is not running....')

def isSIL():
	if os.name == 'posix':
		return 1
	else:
		return 0