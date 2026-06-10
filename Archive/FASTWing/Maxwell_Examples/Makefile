CXX ?= g++
NAVIO = ../../Navio/C++/Navio
PROP = links

PIGPIO_PATH := $(PIGPIO_PATH)

LIB = -L$(PIGPIO_PATH) -L$(NAVIO) -L$(PROP)

EXECUTABLE = Servo2.exe

INCLUDES = -I ../../Navio/C++/Navio -I$(PIGPIO_PATH)

all:
	$(MAKE) -C links all
	$(MAKE) -C ../../Navio/C++/Navio all
	$(CXX) -std=gnu++11 -w  $(INCLUDES) $(LIB) RCInputServo2.cpp -lmatlab -lnavio -o $(EXECUTABLE) -lrt -lpthread -lpigpio || $(MAKE) pigpio

clean:
	rm ../../Navio/C++/Navio/Navio2/*.o
	rm ../../Navio/C++/Navio/Navio+/*.o
	rm links/*.o
	rm -f $(EXECUTABLE)
rebuild:
	make clean
	make
