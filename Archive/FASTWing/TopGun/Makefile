CXX ?= g++
NAVIO = ../../Navio
PROP = links

PIGPIO_PATH := $(PIGPIO_PATH)

LIB = -L$(PIGPIO_PATH) -L$(NAVIO) -L$(PROP)

EXECUTABLE = TopGun.exe

INCLUDES = -I ../../Navio -I$(PIGPIO_PATH)

all:
	$(MAKE) -C links all
	$(MAKE) -C ../../Navio all
	$(CXX) -std=gnu++11 -w $(INCLUDES) $(LIB) Autopilot.cpp TopGun.cpp -lmatlab -lnavio -o $(EXECUTABLE) -lrt -lpthread -lpigpio || $(MAKE) pigpio

clean:
	rm ../../Navio/Navio2/*.o
	rm ../../Navio/Navio+/*.o
	rm ../../Navio/Common/*.o
	rm links/*.o
	rm -f $(EXECUTABLE)
rebuild:
	make clean
	make
