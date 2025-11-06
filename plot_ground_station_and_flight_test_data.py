import numpy as np
import matplotlib.pyplot as plt

###############PREAMBLE##############################33

###THIS IS FOR 2/16/2023 - IO RPI FC QUAD TEST
xstart = 394.
xend = 460.
#Do you want to plot ground station data?
PLOTGROUNDSTATIONDATA = False
#Do you want to plot flight test data as well?
PLOTFLIGHTTESTDATA = False

#######################################################

###THIS IS THE GROUND STATION FILE
if PLOTGROUNDSTATIONDATA:
    data = np.loadtxt('logs/Ground_Station_RPI_FC_IO_TEST_2_16_2023',delimiter=',')
    ###EXTRACT GROUND STATION DATA
    time = data[:,0]
    imuroll = data[:,1]
    imupitch = data[:,2]
    compass = data[:,3]
    lat_gnd = data[:,4]
    lon_gnd = data[:,5]
    baro_altitude = data[:,6]
    gps_speed = data[:,7]

    ##Truncate
    timetruncated = time[time < xend]
    barotruncated = baro_altitude[time < xend]
    barotruncated = barotruncated[timetruncated > xstart]
    timetruncated = timetruncated[timetruncated > xstart]

#Open the flight test data
if PLOTFLIGHTTESTDATA:
    datafile = open('../data/RPI_FC_IO_TEST_2_16_2023','r')
    dataheaders = datafile.readline().split(',')
    numVars = len(dataheaders)
    print('Number of Vars = ',numVars)
    counter = 0
    for header in dataheaders:
        print(header,counter)
        counter = counter + 1
    sense_data = []
    for line in datafile:
        row = line.split(',')
        numarray = [np.float(x) for x in row]
        sense_data.append(numarray)
    sense_data = np.array(sense_data)
    sense_time = sense_data[:,0]
    if xstart > 0:
        istart_sense = np.where(sense_time>xstart)[0][0]
    else:
        istart_sense = 0
    if xend > 0:
        iend_sense = np.where(sense_time>xend)[0][0]
    else:
        iend_sense = -1
    ###EXTRACT FLIGHT TEST DATA
    sense_baro = sense_data[:,28]
    gps_altitude = sense_data[:,18]
    latitude = sense_data[:,16]
    longitude = sense_data[:,17]
    sense_time_truncated = sense_time[istart_sense:iend_sense]
    sense_baro_truncated = sense_baro[istart_sense:iend_sense]
    gps_altitude_truncated = gps_altitude[istart_sense:iend_sense]

    ##Pull out all bad GPS data
    lat = []
    lon = []
    for x in range(0,len(latitude)):
        if latitude[x] != 0 and longitude[x] != 0:
            lat.append(latitude[x])
            lon.append(longitude[x])

    print(lat,lon)

##########GENERATE PLOTS        
plt.figure()
if PLOTGROUNDSTATIONDATA:
    plt.plot(lon_gnd,lat_gnd,label='Ground Station GPS')
if PLOTFLIGHTTESTDATA:
    plt.plot(lat,lon,label='Flight Test GPS')
plt.grid()
plt.xlabel('Longitude (Deg)')
plt.ylabel('Latitude (Deg)')

plt.figure()
if PLOTGROUNDSTATIONDATA:
    plt.plot(timetruncated,barotruncated,'b-*',label='Ground Station Barometer')
if PLOTFLIGHTTESTDATA:
    plt.plot(sense_time_truncated,sense_baro_truncated,'r--*',label='Flight Test Barometer')
    plt.plot(sense_time_truncated,gps_altitude_truncated,'g--*',label='Flight Test GPS')
plt.grid()
plt.xlabel('Time (sec)')
plt.ylabel('Altitude (m)')
plt.legend()

###Print my timestep
if PLOTGROUNDSTATIONDATA:
    dt = timetruncated[1:]-timetruncated[0:-1]
    print('Mean Timestep = ',np.mean(dt))
    #Take Derivative of Barometer for vertical flight speed
    dz = barotruncated[1:]-barotruncated[0:-1]
    vz = dz/dt
    plt.figure()
    plt.plot(timetruncated[0:-1],vz)
    plt.grid()
    plt.xlabel('Time (sec)')
    plt.ylabel('Vz (m/s)')

plt.show()

