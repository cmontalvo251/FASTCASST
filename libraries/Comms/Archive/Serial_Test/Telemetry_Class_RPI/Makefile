CC=g++
EXECUTABLE=telemetry.exe
CPPSOURCES=$(wildcard *.cpp ../../Serial/Telemetry.cpp)
CPPOBJECTS=$(CPPSOURCES:.cpp=.o)
OBJECTS=$(CPPOBJECTS)
COMPILE=-c -w -O3
DIRECTIVES=-DRPI ##You need this to make sure the computer uses RPI
LIB=-L/usr/local/lib -lwiringPi #You need -lwiringPi for serial stuff
INCLUDE=-I../../

all: $(CPPSOURCES) $(EXECUTABLE)
$(EXECUTABLE): $(OBJECTS)
	$(CC) $(OBJECTS) -o $@ $(LIB)
.cpp.o:
	$(CC) $(COMPILE) $(DIRECTIVES) $< -o $@ $(LIB) $(INCLUDE)
clean:
	rm ../../Serial/*.o
	rm -rf $(EXECUTABLE) *.o
rebuild:
	make clean
	make
