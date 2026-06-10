CXX = g++
CFLAGS = -w -c -I .

SOURCES=$(wildcard *.cpp)
OBJECTS=$(SOURCES:.cpp=.o)

all: $(SOURCES) $(OBJECTS)
	ar rcs libmatlab.a $(OBJECTS)
.cpp.o: 
	$(CXX) $(CFLAGS) $< -o $@ 
clean:
	rm -f *.o *.a
rebuild:
	make clean
	make
