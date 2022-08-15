CXX ?=g++
CFLAGS=-c -Wall
NAVIO=../../
INCLUDES = -I${NAVIO}

all:
	$(MAKE) -C ${NAVIO} all
	$(CXX) -std=c++11 $(INCLUDES) gps.cpp -L$(NAVIO) -lnavio -o gps
	rm -rf *.o

clean:
	rm -rf *.o gps
	rm ../../Common/*.o
	rm ../../Navio2/*.o
rebuild:
	make clean
	make all
