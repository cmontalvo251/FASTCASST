CXX ?= g++
NAVIO = ../../
INCLUDES = -I${NAVIO}

all:
	$(MAKE) -C ${NAVIO} all
	$(CXX) -std=c++11 $(INCLUDES) AHRS.cpp -L$(NAVIO) -lnavio -o AHRS -lrt -lpthread

clean:
	rm -f ADC
	rm ../../Common/*.o
	rm ../../Navio2/*.o
rebuild:
	make clean
	make all
