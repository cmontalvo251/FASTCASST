CXX=g++
CGIT=/home/pi/C/
NAVIO=$(CGIT)Sensors/
LIB=-L$(NAVIO) -lwiringPi -L$(CGIT) -Llinks/ -L$(CGIT)MATLAB/
EXECUTABLE=FASTLogger.exe
INCLUDES=-I$(NAVIO) -I$(CGIT) -Ilinks/ -I$(CGIT)MATLAB/

all:
	$(MAKE) -C links all
	$(MAKE) -C $(NAVIO) all
	$(CXX) -std=gnu++11 -w $(INCLUDES) Sensors.cpp FASTLogger.cpp -o $(EXECUTABLE) $(LIB) -lmatlab -lnavio -lrt -lpthread -lpigpio || $(MAKE) pigpio
clean:
	rm $(NAVIO)/Navio2/*.o
	rm $(NAVIO)/Navio+/*.o
	rm $(NAVIO)/Common/*.o
	rm links/*.o
	rm -f $(EXECUTABLE)
rebuild:
	make clean
	make
