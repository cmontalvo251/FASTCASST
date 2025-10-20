CXX ?= g++
NAVIO=../../
INCLUDES=-I${NAVIO}

all:
	$(MAKE) -C ${NAVIO} all
	$(CXX) -std=c++11 $(INCLUDES) -L$(NAVIO) AccelGyroMag.cpp -lnavio -o AccelGyroMag

clean:
	rm -f AccelGyroMag
	rm ../../Common/*.o
	rm ../../Navio2/*.o
rebuild:
	make clean
	make all
