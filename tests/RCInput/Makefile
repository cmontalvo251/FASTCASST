CC=g++
CFLAGS=-c -w 
LIB_DIRS=-L../../ 
INCLUDES=-I../../
SOURCES=main.cpp ../../RCIO/RCInput.cpp
####ON DESKTOP DO THE FOLLOWING
#DFLAGS=-DDESKTOP -DSIL -DXBOX
DFLAGS=-DDESKTOP -DSIL -DRCTECH
###ON RASPBERRY PI DO THE FOLLOWING
#DFLAGS=-DRPI -DAUTO -DFLYSKY
#This means all the objects are the sources except replace .cpp with .o
OBJECTS=$(SOURCES:.cpp=.o) 
EXECUTABLE=RCInput.exe

##Tells make to compile all the sources and executable
all: $(SOURCES) $(EXECUTABLE)
#Rule for executable, will only do this if the objects have changed
#The $@ symbol tells the makefile to compile the thing in blue
$(EXECUTABLE): $(OBJECTS)
	$(CC) $(INCLUDES) $(OBJECTS) -o $@ $(LIB_DIRS)
#rule for all of the .cpp files 
.cpp.o:
	$(CC) $(CFLAGS) $(DFLAGS) $(LIB_DIRS) $(INCLUDES) $(LIB_DIRS) $< -o $@
clean:
	echo ' ' > ../../RCIO/d.o
	rm ../../RCIO/*.o
	rm -rf *.o $(EXECUTABLE)
rebuild:
	make clean
	make
