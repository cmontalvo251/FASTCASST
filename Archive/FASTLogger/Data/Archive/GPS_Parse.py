import numpy as np
import matplotlib.pyplot as plt

##this function will parse an individual number into a float with text=float into just a float
def getfloat(instr):
    #print(instr)
    loc = instr.find('=')
    #print(loc)
    raw = instr[loc+1:]
    return np.float(raw)

##This function will parse a line
def parse(instr):
    #You'll notice that the line is separated by spaces so first use the line.split command
    list = instr.split(' ')
    #print(list)
    #Then we extract the relevant data sets
    lonstr = list[1]
    latstr = list[2]
    altstr = list[3]
    #print(lonstr,latstr,altstr)
    ##You'll then notice that each str is separated by an equal sign. We want everything after the equal
    #sign. There are a few ways to do this. We could use split again or we could find the = and grab everything
    #after it. I think I'll go with the grab everything after it.
    lon = getfloat(lonstr)/10000000.0
    lat = getfloat(latstr)/10000000.0  ##I'm dividing by random numbers until shit looks right
    alt = getfloat(altstr)/1000.0
    #print(lon,lat,alt)
    return lon,lat,alt

##Ok first import the file
file = open('GPS_File0.txt')
##Then loop through the file but only print every other line to skip that random gpsFix line=
skip = True
##Make empty arrays
latarray = []
lonarray = []
altarray = []
for line in file:
    if skip == False:
        ##Now that we have the line we want let's parse out some info using a fancy function
        #print(line)
        lon,lat,alt = parse(line)
        #Now that we have a function that will grab every line we can put it into an array
        lonarray.append(lon)
        latarray.append(lat)
        altarray.append(alt)
    skip = not skip

##Then convert the arrays to numpy
lonarray = np.array(lonarray)
latarray = np.array(latarray)
altarray = np.array(altarray)

##Then we just plot. I'll just plot latitude and longitude
fig = plt.figure()
plti = fig.add_subplot(1,1,1)
plti.plot(lonarray,latarray)
plti.grid()
plti.set_xlabel('Longitude (deg)')
plti.set_ylabel('Latitude (deg)')
plti.get_yaxis().get_major_formatter().set_useOffset(False)
plti.get_xaxis().get_major_formatter().set_useOffset(False)
plt.show()