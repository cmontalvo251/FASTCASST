import sys, time
from rcio import RCInput

rcin = RCInput('SIL',992,2016,'../../RCInput/Calibration/XBOX_config.json',num_channels=6)

while (True):
    rcin.readALL()
    print(time.time(),rcin.rcsignals)
    time.sleep(0.1)
