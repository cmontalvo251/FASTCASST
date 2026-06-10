CXX ?= g++
NAVIO = ../../
INCLUDES = -I${NAVIO}

all:
	$(MAKE) -C ${NAVIO} all
	$(CXX) -std=gnu++11 $(INCLUDES)  ADC.cpp -L$(NAVIO) -lnavio -o ADC

clean:
	rm -f ADC
	rm ../../Common/*.o
	rm ../../Navio2/*.o
rebuild:
	make clean
	make all
