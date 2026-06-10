CXX = g++
CFLAGS = -w -c -I .
INCLUDES= -I ../../../Navio
SOURCES=$(wildcard *.cpp)
OBJECTS=$(SOURCES:.cpp=.o)

all: $(SOURCES) $(OBJECTS)
	ar rcs libpropulsion.a $(OBJECTS)
.cpp.o: 
	$(CXX) $(INCLUDES) $(CFLAGS) $< -o $@ 
clean:
	rm -f *.o *.a
rebuild:
	make clean
	make
