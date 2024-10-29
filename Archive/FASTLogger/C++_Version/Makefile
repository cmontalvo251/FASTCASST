CC=g++
##CHANGE THIS TO YOUR PATH
#RASPBERRYPI
CPATH=/home/pi/C
HPATH=/home/pi/HIL
#T540P Work Computer
#CPATH=/home/carlos/Files/Git_Repos/Github/C++
#X1 Carbon Work Computer
#CPATH=/home/carlos/Documents/Git_Repos/Github/C++
###########################
EXECUTABLE=FASTLogger.exe
CPPSOURCES=$(wildcard *.cpp ${CPATH}/Datalogger/Datalogger.cpp ${CPATH}/MATLAB/MATLAB.cpp ${CPATH}/Mathp/mathp.cpp ${CPATH}/Timer/timer.cpp ${HPATH}/GPS/GPS.cpp ${HPATH}/GPS/Ublox.cpp ${HPATH}/Baro/BaroTemp.cpp ${HPATH}/Baro/MS5611.cpp ${HPATH}/Serial/I2Cdev.cpp ${HPATH}/IMU/IMU.cpp ${HPATH}/IMU/MPU9250.cpp ${HPATH}/IMU/AHRS.cpp ${HPATH}/ADC/ADC.cpp ${HPATH}/ADC/ADC_Navio2.cpp ${CPATH}/Sensors/temperature.cpp ${HPATH}/IMU/LSM9DS1.cpp ${HPATH}/Serial/Telemetry.cpp)
CPPOBJECTS=$(CPPSOURCES:.cpp=.o)
OBJECTS=$(CPPOBJECTS)
COMPILE=-c -w -std=c++11 -Wno-psabi #-c means precompile, -w is no warnings -O3 all the optimizers to compile
LIB=-lwiringPi #You need -lwiringPi for serial stuff //sudo apt-get install wiringpi
INCLUDE=-I${CPATH}/ -I${HPATH}/
#Put Desktop if compiling on desktop
#DEBUG to turn on STDOUT prints
#TELEMETRY To turn on telemetry to a ground station
#DESKTOP if compiling on your desktop and you want to run SIL
#The RPI directive is to make sure the Telemetry class uses the RPi flags
DIRECTIVES=-DDEBUG -DTELEMETRY -DRPI 

all: $(CPPSOURCES) $(EXECUTABLE)
$(EXECUTABLE): $(OBJECTS)
	$(CC) $(OBJECTS) -o $@ $(INCLUDE) $(LIB) $(DIRECTIVES)
.cpp.o:
	$(CC) $(COMPILE) $< -o $@ $(LIB) $(INCLUDE) $(DIRECTIVES)
clean:	
	echo ' ' > ${CPATH}/Datalogger/d.o
	rm ${CPATH}/Datalogger/*.o
	echo ' ' > ${CPATH}/MATLAB/d.o
	rm ${CPATH}/MATLAB/*.o
	echo ' ' > ${CPATH}/Timer/d.o
	rm ${CPATH}/Timer/*.o
	echo ' ' > ${CPATH}/Sensors/d.o
	rm ${CPATH}/Sensors/*.o
	echo ' ' > ${CPATH}/Mathp/d.o
	rm ${CPATH}/Mathp/*.o
	echo ' ' > ${HPATH}/ADC/d.o
	rm ${HPATH}/ADC/*.o
	echo ' ' > ${HPATH}/GPS/d.o
	rm ${HPATH}/GPS/*.o
	echo ' ' > ${HPATH}/IMU/d.o
	rm ${HPATH}/IMU/*.o
	echo ' ' > ${HPATH}/Baro/d.o
	rm ${HPATH}/Baro/*.o
	echo ' ' > ${HPATH}/Serial/d.o
	rm ${HPATH}/Serial/*.o
	echo ' ' > data/d.txt
	rm data/*.txt
	echo ' ' > d.o
	rm *.o
	echo ' ' > d.exe
	rm *.exe
rebuild:
	make clean
	make
