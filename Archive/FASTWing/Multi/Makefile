CC=g++ #Define compiler (CC) to be gfortran compiler
CFLAGS=-c -w #-O1 #-g for debug mode
DFLAGS= -DRUN_VISUALIZER -DJOYSTICK -DOPENGL_FLAG # Directives for #ifdefs
LIB_DIRS = -L/usr/local/lib #used to include boost and glut directories
BOOST_LIBS = -lpthread -lboost_system -lboost_thread -lboost_date_time
OPENGL_LIBS = -lGL -lGLU -lglut
SOURCES= MultiMMACSV10.cpp timer.cpp opengl.cpp MATLAB.cpp mathp.cpp
#This means all the objects are the sources except replace .cpp with .o
OBJECTS = $(SOURCES:.cpp=.o) 
EXECUTABLE=Run.exe

##Tells make to compile all the sources and executable
all: $(SOURCES) $(EXECUTABLE)

#Rule for executable, will only do this if the objects have changed
#The $@ symbol tells the makefile to compile the thing in blue
$(EXECUTABLE): $(OBJECTS)
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@ $(LIB_DIRS) $(BOOST_LIBS) $(OPENGL_LIBS)

#rule for all of the .cpp files 
.cpp.o:
	$(CC) $(CFLAGS) $(DFLAGS) $< -o $@
clean:
	rm -rf *.o $(EXECUTABLE)
rebuild:
	make clean
	make

