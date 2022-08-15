CXX ?= g++
NAVIO =../../
INCLUDES = -I${NAVIO}

all:
	$(MAKE) -C ${NAVIO} all
	$(CXX) $(INCLUDES) Barometer.cpp -L$(NAVIO) -lnavio -o Barometer

clean:
	rm -f Barometer
	rm ../../Common/*.o
	rm ../../Navio2/*.o
rebuild:
	make clean
	make all


