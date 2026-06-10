from pdf import *
import sys
#try:
import quad as Q
#except SyntaxError:
#	print('You have to use python for quad.py. Unfortunately I have not updated this for python3')
#	sys.exit()

pp = PDF(0,plt)

fileName = 'Drone_Test/example_quad_data.log'
#fileName = 'Drone_Test/Drone_Test_IRIS_Takeoff_7_21_2021.txt'
quad_data = Q.get_quad_data(fileName)
if not quad_data:
    sys.exit()

print("Quad Data Directory = ",quad_data[-1])
baroRead = quad_data[0]
gpsRead = quad_data[1]
rcinoutRead = quad_data[2]
IMURead = quad_data[3]
ATTRead = quad_data[4]
BATTRead = quad_data[5]

#Defaults. DO NOT TOUCH
x0 = baroRead.timeSec[0]
xf = baroRead.timeSec[-1]

# x0 = 530
# xf = 550
# x0 = 40
# xf = 50

figure1 = plt.figure()#figsize = (11, 8))
print('Plotting Barometer')
baroRead.plot_barometer(figure1,x0,xf,pp)

if gpsRead.GPSINIT == 1:
    print('Plotting GPS')
    figure2 = plt.figure()
    gpsRead.plot_latlon(figure2,pp)
    gpsRead.plotGPS_Speed(x0,xf,pp)
    gpsRead.plot_Time(pp)
else:
    print("GPSINIT NOT FOUND")

print('Plotting RC')
rcinoutRead.plot_rc(x0,xf,pp)

print('Plotting IMU')
IMURead.plot_IMU(x0,xf,pp)

print('Plotting Attitude')
ATTRead.plot_ATTITUDE(x0,xf,pp)

print('Plotting Altitude')
Q.plot_altitude(quad_data,x0,xf,pp)

print('Plot battery')
BATTRead.plot_battery(pp)

pp.close()

