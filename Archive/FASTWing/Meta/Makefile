CXX ?= g++
NAVIO = ../../Navio_Emlid/C++/Navio


PIGPIO_PATH := $(PIGPIO_PATH)

LIB = -L$(PIGPIO_PATH)

INCLUDES = -I ../../Navio_Emlid/C++/Navio -I$(PIGPIO_PATH)


all:
	rm ../../Navio_Emlid/C++/Navio/Navio2/*.o
	rm ../../Navio_Emlid/C++/Navio/Navio+/*.o
	$(MAKE) -C ../../Navio_Emlid/C++/Navio all
	$(CXX) -std=gnu++11 $(INCLUDES) $(LIB) Meta.cpp -L$(NAVIO) -lnavio -o Meta -lrt -lpthread -lpigpio || $(MAKE) pigpio

clean:
	rm -f MainLoop
