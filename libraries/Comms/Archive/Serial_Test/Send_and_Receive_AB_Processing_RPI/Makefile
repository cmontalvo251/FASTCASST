CC=g++
EXECUTABLE=run.exe
CSOURCES=$(wildcard *.c)
CPPSOURCES=$(wildcard *.cpp)
COBJECTS=$(CSOURCES:.c=.o)
CPPOBJECTS=$(CPPSOURCES:.cpp=.o)
OBJECTS=$(COBJECTS) $(CPPOBJECTS)
COMPILE=-c -w -O3
DIRECTIVES= -DRPI #-DHIL #Comment this out if you don't want HIL and you're just using simulation
#define HIL // comment = no HIL (read sensors on ADCS board), uncomment HIL (serial data transmission from desktopcomputer)
#//Subroutines. these routines ideally would be in a separate cpp file or even a class but I'm putting them here for reference
LIB= -L/usr/local/lib -lwiringPi

all: $(CSOURCES) $(CPPSOURCES) $(EXECUTABLE)
$(EXECUTABLE): $(OBJECTS)
	$(CC) $(OBJECTS) -o $@ $(LIB)
.c.o:
	$(CC) $(COMPILE) $(DIRECTIVES) $< -o $@ $(LIB)
.cpp.o:
	$(CC) $(COMPILE) $(DIRECTIVES) $< -o $@ $(LIB)
clean:
	rm -rf $(EXECUTABLE) *.o
rebuild:
	make clean
	make
