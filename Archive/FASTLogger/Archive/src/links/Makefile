CXX = g++
CFLAGS = -w -c -I .
DIRECTIVES=-DRPI
SOURCES=$(wildcard *.cpp)
INCLUDES=
OBJECTS=$(SOURCES:.cpp=.o)

all: $(SOURCES) $(OBJECTS)
	ar rcs libmatlab.a $(OBJECTS)
.cpp.o: 
	$(CXX) $(INCLUDES) $(CFLAGS) $(DIRECTIVES) $< -o $@ 
clean:
	rm -f *.o *.a
rebuild:
	make clean
	make
