from gps import GPS as G
import numpy as np
import matplotlib.pyplot as plt

##IMPORT DATA
data = np.loadtxt('example_data/raw_data_6_8_2022.txt',delimiter=',')

##CREATE THE GPS CLASS
myGPS = G()

##SEND DATA TO GPS
myGPS.latitude_vec = data[:,0]
myGPS.longitude_vec = data[:,1]
myGPS.time_vec = np.arange(0,len(data[:,0])*1.0,1.0)

###SET ORIGIN
myGPS.setOrigin(myGPS.latitude_vec[0],myGPS.longitude_vec[0])

###Convert to XY
myGPS.convertLATLONVEC2XY()

###Compute Velocity
myGPS.setFilterConstant(0.8)
myGPS.computeVelocityVec()

##Create PLots
myGPS.plot(plt)
plt.show()