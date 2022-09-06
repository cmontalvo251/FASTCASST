#########FAST MAKEFILE DEFAULTS##########
PLATFORM=RPI
EXECUTABLE=a.exe
MAIN=main.o
WIRINGPI="-lwiringPi"

###COMPILER AND OTHER FLAGS
CC=g++
COMPILE=-c -w -std=c++11 -Wno-psabi
FLAGS=-DDEBUG
LIB=-L/usr/local/lib -L./
#FAST is using boost for threading #sudo apt-get install libboost-all-dev
INCLUDE=-Ilibraries/ -I./
###HELPER
HELPERSOURCES=libraries/MATLAB/MATLAB.cpp libraries/Mathp/mathp.cpp libraries/Timer/timer.cpp
###HARDWARE
UARTSOURCES=$(wildcard libraries/UART/*.cpp)
UTILSOURCES=$(wildcard libraries/Util/*.cpp)
HARDWARESOURCES=$(UARTSOURCES) $(UTILSOURCES)
###COMBINE ALL SOURCES
SOURCES=$(HELPERSOURCES) $(HARDWARESOURCES)
OBJECTS=$(SOURCES:.cpp=.o)

##Target to make all depends on the MAIN, OBJECTS and the EXECUTABLE
all: $(OBJECTS) $(MAIN) $(EXECUTABLE)

##Rule for executable depends on the OBJECTS and MAIN
$(EXECUTABLE): $(OBJECTS) $(MAIN)
	$(CC) $(OBJECTS) $(MAIN) -o $(EXECUTABLE) $(LIB) $(INCLUDE) $(WIRINGPI)

##Target MAIN depends on it's respective cpp file
$(MAIN): $(MAIN:.o=.cpp)
	$(CC) $(COMPILE) $(FLAGS) $(INCLUDE) -D$(PLATFORM) $(MAIN:.o=.cpp) -o $(MAIN) $(WIRINGPI)

##The rule for the objects depends on the sources
.cpp.o: $(SOURCES)
	$(CC) $(COMPILE) $(FLAGS) $(INCLUDE) -D$(PLATFORM) $(LIB) $(WIRINGPI) $< -o $@

##Clean function
clean:
	echo ' ' > d.exe
	rm *.exe
	rm *.o
	echo ' ' > libraries/MATLAB/d.o
	rm libraries/*/*.o
