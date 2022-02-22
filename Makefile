#########FAST MAKEFILE DEFAULTS##########
MODEL=portalcube
RX=KEYBOARD
PLATFORM=DESKTOP
TYPE=SIMONLY
EXECUTABLE=simonly.exe
MAIN=src/main.o
OPENGLSOURCES=
RENDER=
THREAD=
WIRINGPI=

###COMPILER AND OTHER FLAGS
CC=g++
HELPER=helper
HARDWARE=hardware
MODELING=modeling
COMPILE=-c -w -std=c++11 -Wno-psabi
FLAGS=-DDEBUG
LIB=-L/usr/local/lib -L./
#FAST is using boost for threading #sudo apt-get install libboost-all-dev
THREAD=-lpthread -lboost_system -lboost_thread -lboost_date_time 
MODELPATH=vehicles/$(MODEL)/src
INCLUDE=-I${HELPER} -I${HARDWARE} -I${MODELPATH} -I${MODELING} -I./
###HELPER
HELPERSOURCES=$(wildcard $(HELPER)/*/*.cpp)
###HARDWARE
HARDWARESOURCES=$(wildcard $(HARDWARE)/*/*.cpp)
##MOELING
MODELINGSOURCES=$(wildcard $(MODELING)/*/*.cpp)
###MODEL
MODELSOURCES=$(wildcard vehicles/$(MODEL)/src/*.cpp)

###COMBINE ALL SOURCES
SOURCES=$(HELPERSOURCES) $(HARDWARESOURCES) $(MODELSOURCES) $(OPENGLSOURCES) $(MODELINGSOURCES)
OBJECTS=$(SOURCES:.cpp=.o)

##Logger SIL is on DESKTOP and basically takes and logs fictitious data
loggersil:
	make all TYPE="SIL" EXECUTABLE="loggersil.exe" MAIN="src/logger.o" MODELINGSOURCES=

##Logger is on RPI but all it does is take data
logger:
	make all TYPE="AUTO" EXECUTABLE="logger.exe" PLATFORM="RPI" MAIN="src/logger.o" WIRINGPI="-lwiringPi"

#Demo sil is on DESKTOP but only does RCIN>IMU>CONTROL>RCOUT
demosil:
	make all TYPE="SIL" EXECUTABLE="demosil.exe" MAIN="src/demo.o" MODELINGSOURCES=

#Demo is demosil but on rpi
demo:
	make all TYPE="AUTO" EXECUTABLE="demo.exe" PLATFORM="RPI" RX="FLYSKY" MAIN="src/demo.o" WIRINGPI="-lwiringPi"

#Simonly is just a repeat of all but it is written here to be thorough
simonly:
	make all

#SIL runs the main routine with openGL 
#sudo apt-get install freeglut3-dev
sil:
	make all TYPE="SIL" EXECUTABLE="sil.exe" RENDER="-lGL -lGLU -lglut" OPENGLSOURCES="opengl/opengl.cpp" THREAD="-lpthread -lboost_system -lboost_thread -lboost_date_time"

#Auto mode. Fully deployed on platform. Modeling routine is off
auto:
	make all TYPE="AUTO" EXECUTABLE="auto.exe" PLATFORM="RPI" RX="FLYSKY" WIRINGPI="-lwiringPi"

hil:
	echo 'WIP'

##Target to make all depends on the MAIN, OBJECTS and the EXECUTABLE
all: $(OBJECTS) $(MAIN) $(EXECUTABLE)

##Rule for executable depends on the OBJECTS and MAIN
$(EXECUTABLE): $(OBJECTS) $(MAIN)
	$(CC) $(OBJECTS) $(MAIN) -o $(EXECUTABLE) $(LIB) $(INCLUDE) $(RENDER) $(THREAD) $(WIRINGPI)

##Target MAIN depends on it's respective cpp file
$(MAIN): $(MAIN:.o=.cpp)
	$(CC) $(COMPILE) $(FLAGS) $(INCLUDE) -D$(RX) -D$(PLATFORM) -D$(TYPE) $(MAIN:.o=.cpp) -o $(MAIN) $(WIRINGPI)

##The rule for the objects depends on the sources
.cpp.o: $(SOURCES)
	$(CC) $(COMPILE) $(FLAGS) $(INCLUDE) -D$(RX) -D$(PLATFORM) -D$(TYPE) $(LIB) $(WIRINGPI) $< -o $@

##Clean function
clean:
	echo ' ' > d.exe
	rm *.exe
	echo ' ' > src/d.o
	rm src/*.o
	echo ' ' > $(HELPER)/MATLAB/d.o
	rm $(HELPER)/*/*.o
	echo ' ' > $(HARDWARE)/ADC/d.o
	rm $(HARDWARE)/*/*.o
	echo ' ' > $(MODELING)/RK4/d.o
	rm $(MODELING)/*/*.o
	echo ' ' > vehicles/portalcube/src/d.o
	rm vehicles/portalcube/src/*.o
