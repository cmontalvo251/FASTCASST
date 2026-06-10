CXX ?= g++
NAVIO = ../../Navio

PIGPIO_PATH := $(PIGPIO_PATH)

LIB = -L$(PIGPIO_PATH)

INCLUDES = -I ../../Navio -I$(PIGPIO_PATH)

all:
	rm ../../Navio/Navio2/*.o
	rm ../../Navio/Navio+/*.o
	$(MAKE) -C ../../Navio all
	$(CXX) -std=gnu++11 $(INCLUDES) $(LIB) Arming.cpp -L$(NAVIO) -lnavio -o Arming -lrt -lpthread -lpigpio || $(MAKE) pigpio

clean:
	rm -f Arming
