#########FAST MAKEFILE DEFAULTS##########
MODEL=airplane
RX=KEYBOARD
PLATFORM=DESKTOP
TYPE=SIMONLY
EXECUTABLE=simonly.exe
MAIN=src/fast.o
OPENGLSOURCES=
RENDER=
THREAD=
WIRINGPI=
CC=g++
COMPILE=-g -c -w -std=c++11 -Wno-psabi
FLAGS=-DDEBUG -DJSON_FILE_NAME=\"$(RX)_config.json\" -DFORCES_HEADER_FILE=\"$(MODEL)_forces.h\" -DCONTROLLER_HEADER_FILE=\"$(MODEL)_controller.h\"
LIB=-L/usr/local/lib -L./
MODELPATH=libraries/V_$(MODEL)/
INCLUDE=-Ilibraries/ -I${MODELPATH} -I./

###HELPER
HELPERSOURCES=libraries/Datalogger/Datalogger.cpp libraries/MATLAB/MATLAB.cpp libraries/Mathp/mathp.cpp libraries/Timer/timer.cpp
HELPEROBJECTS=$(HELPERSOURCES:.cpp=.o)

###ALGORITHMS OR SENSOR HELPERS
AHRSSOURCES=$(wildcard libraries/AHRS/*.cpp)
UBLOXSOURCES=$(wildcard libraries/Ublox/*.cpp)
UTILSOURCES=$(wildcard libraries/Util/*.cpp)
ALGSOURCES=$(AHRSSOURCES) $(UBLOXSOURCES) $(UTILSOURCES)
ALGOBJECTS=$(ALGSOURCES:.cpp=.o)

##RCSOURCES
RCIOSOURCES=$(wildcard libraries/RCIO/*.cpp)
RCINPUTSOURCES=$(wildcard libraries/RCInput/*.cpp)
RCOUTPUTSOURCES=$(wildcard libraries/RCOutput/*.cpp)
PWMSOURCES=$(wildcard libraries/PWMSIGNALS/*.cpp)
RCSOURCES=$(RCIOSOURCES) $(RCINPUTSOURCES) $(RCOUTPUTSOURCES) $(PWMSOURCES)
RCOBJECTS=$(RCSOURCES:.cpp=.o)

##COMMS
COMMSSOURCES=$(wildcard libraries/Comms/*.cpp)
SERIALSOURCES=$(wildcard libraries/SerialComms/*.cpp)
I2CSOURCES=$(wildcard libraries/I2Cdev/*.cpp)
COMMSOURCES=$(COMMSSOURCES) $(SERIALSOURCES) $(I2CSOURCES)
COMMOBJECTS=$(COMMSOURCES:.cpp=.o)

###SENSORS
IMUSOURCES=$(wildcard libraries/IMU/*.cpp)
MPUSOURCES=$(wildcard libraries/MPU9250/*.cpp)
LSMSOURCES=$(wildcard libraries/LSM9DS1/*.cpp)
GPSSOURCES=$(wildcard libraries/GPS/*.cpp)
STARSOURCES=$(wildcard libraries/startracker/*.cpp)
PTHSOURCES=$(wildcard libraries/PTH/*.cpp)
MS5611SOURCES=$(wildcard libraries/MS5611/*.cpp)
ADCSOURCES=$(wildcard libraries/ADC/*.cpp)
HWSOURCES=libraries/sensors/sensors.cpp libraries/hardware/hardware.cpp
HARDWARESOURCES=$(MPUSOURCES) $(LSMSOURCES) $(IMUSOURCES) $(GPSSOURCES) $(PTHSOURCES) $(ADCSOURCES) $(HWSOURCES) $(MS5611SOURCES) $(STARSOURCES)
HARDWAREOBJECTS=$(HARDWARESOURCES:.cpp=.o)

##MODELING
ENVSOURCES=$(wildcard libraries/Environment/*.cpp)
GEOSOURCES=$(wildcard libraries/GeographicLib/*.cpp)
RK4SOURCES=$(wildcard libraries/RK4/*.cpp)
ROTSOURCES=$(wildcard libraries/Rotation/*.cpp)
WRFSOURCES=$(wildcard libraries/wrf/*.cpp)
DRYDENSOURCES=$(wildcard libraries/Dryden/*.cpp)
MODELINGSOURCES=$(ENVSOURCES) $(GEOSOURCES) $(RK4SOURCES) $(ROTSOURCES) $(WRFSOURCES) $(DRYDENSOURCES) libraries/modeling/modeling.cpp
MODELINGOBJECTS=$(MODELINGSOURCES:.cpp=.o)

###MODEL
MODELSOURCES=$(wildcard libraries/V_$(MODEL)/*.cpp)
MODELOBJECTS=$(MODELSOURCES:.cpp=.o)

###COMBINE ALL OBJECTS
#SOURCES=$(ALGSOURCES) $(RCSOURCES) $(COMMSOURCES) $(HELPERSOURCES) $(HARDWARESOURCES) $(MODELSOURCES) $(OPENGLSOURCES) $(MODELINGSOURCES)
OBJECTS=$(ALGOBJECTS) $(RCOBJECTS) $(COMMOBJECTS) $(HELPEROBJECTS) $(HARDWAREOBJECTS) $(MODELOBJECTS) $(MODELINGOBJECTS) $(OPENGLSOURCES:.cpp=.o)

#Simonly is just a repeat of all but it is written here to be thorough
simonly:
	make all

#SIL runs the main routine with openGL 
sil:
	make all TYPE="SIL" EXECUTABLE="sil.exe" RENDER="-lGL -lGLU -lglut" OPENGLSOURCES="libraries/opengl/opengl.cpp" THREAD="-lpthread -lboost_thread"

#Auto mode. Fully deployed on platform. Modeling routine is off
auto:
	make all TYPE="AUTO" EXECUTABLE="auto.exe" PLATFORM="RPI" RX="FLYSKY" WIRINGPI="-lwiringPi"

hil_desktop:
	make all TYPE="HIL" EXECUTABLE="hil.exe" PLATFORM="DESKTOP" RENDER="-lGL -lGLU -lglut" OPENGLSOURCES="libraries/opengl/opengl.cpp" THREAD="-lpthread -lboost_thread"

hil_rpi:
	make all TYPE="HIL" EXECUTABLE="hil.exe" PLATFORM="RPI" RX="FLYSKY" WIRINGPI="-lwiringPi"

##Target to make all depends on the MAIN, OBJECTS and the EXECUTABLE
all: $(OBJECTS) $(MAIN) $(EXECUTABLE)

###What or who
what:
	make who
who:
	# make (simonly, sil, hil_rpi, hil_desktop, auto)
	# PLATFORM -> DESKTOP, RPI
	# RX -> RCTECH, KEYBOARD, FLYSKY, XBOX
	# MODEL -> airplane, portalcube, quadcopter, satellite, tank, x8

##Rule for executable depends on the OBJECTS and MAIN
$(EXECUTABLE): $(OBJECTS) $(MAIN)
	$(CC) $(OBJECTS) $(MAIN) -o $(EXECUTABLE) $(LIB) $(INCLUDE) $(RENDER) $(THREAD) $(WIRINGPI)

##Target MAIN depends on it's respective cpp file
$(MAIN): $(MAIN:.o=.cpp)
	$(CC) $(COMPILE) $(FLAGS) $(INCLUDE) -D$(MODEL) -D$(RX) -D$(PLATFORM) -D$(TYPE) $(MAIN:.o=.cpp) -o $(MAIN) $(WIRINGPI)

##The rule for the objects depends on the sources
%.o: %.cpp $(ALGSOURCES)
	$(CC) $(COMPILE) $(FLAGS) $(INCLUDE) -D$(MODEL) -D$(RX) -D$(PLATFORM) -D$(TYPE) $(LIB) $(WIRINGPI) $< -o $@
%.o: %.cpp $(RCSOURCES)
	$(CC) $(COMPILE) $(FLAGS) $(INCLUDE) -D$(MODEL) -D$(RX) -D$(PLATFORM) -D$(TYPE) $(LIB) $(WIRINGPI) $< -o $@
%.o: %.cpp $(COMMSOURCES)
	$(CC) $(COMPILE) $(FLAGS) $(INCLUDE) -D$(MODEL) -D$(RX) -D$(PLATFORM) -D$(TYPE) $(LIB) $(WIRINGPI) $< -o $@
%.o: %.cpp $(HELPERSOURCES)
	$(CC) $(COMPILE) $(FLAGS) $(INCLUDE) -D$(MODEL) -D$(RX) -D$(PLATFORM) -D$(TYPE) $(LIB) $(WIRINGPI) $< -o $@
%.o: %.cpp $(HARDWARESOURCES)
	$(CC) $(COMPILE) $(FLAGS) $(INCLUDE) -D$(MODEL) -D$(RX) -D$(PLATFORM) -D$(TYPE) $(LIB) $(WIRINGPI) $< -o $@
%.o: %.cpp $(MODELSOURCES)
	$(CC) $(COMPILE) $(FLAGS) $(INCLUDE) -D$(MODEL) -D$(RX) -D$(PLATFORM) -D$(TYPE) $(LIB) $(WIRINGPI) $< -o $@
%.o: %.cpp $(OPENGLSOURCES)
	$(CC) $(COMPILE) $(FLAGS) $(INCLUDE) -D$(MODEL) -D$(RX) -D$(PLATFORM) -D$(TYPE) $(LIB) $(WIRINGPI) $< -o $@
%.o: %.cpp $(MODELINGSOURCES)
	$(CC) $(COMPILE) $(FLAGS) $(INCLUDE) -D$(MODEL) -D$(RX) -D$(PLATFORM) -D$(TYPE) $(LIB) $(WIRINGPI) $< -o $@

##Clean function
clean:
	echo ' ' > d.exe
	rm *.exe
	echo ' ' > src/d.o
	rm src/*.o
	echo ' ' > libraries/MATLAB/d.o
	rm libraries/*/*.o
