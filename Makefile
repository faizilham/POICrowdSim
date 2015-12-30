# generic makefile

CC = g++
EXT = cpp
HEXT = h

# flags
LIB = -lm
FLAGS = -Wall -std=c++11 -fopenmp

MODE = debug
# release or debug

ifeq ($(MODE), release)
FLAGS += -O2
else
FLAGS += -O0 -g
endif

# modules
INCLUDE_DIR=includes
THIRD_PARTY_DIR=tinyxml2 polypartition

MODULES = 	shapes.cpp mapobject.cpp tinyxml2.cpp mapreader.cpp \
			polypartition.cpp compiledmap.cpp graph.cpp pathfinder.cpp image.cpp imageio.cpp

# Everything after this is generic, no need to edit
VPATH = src $(addprefix src/,$(THIRD_PARTY_DIR)) $(INCLUDE_DIR)
INCLUDE = -Isrc $(addprefix -I,$(INCLUDE_DIR))
SOURCES = $(MODULES)
OBJS = $(SOURCES:%.$(EXT)=bin/%.o)

.PHONY: all run clean obj
  
all: obj bin/main.o
	$(CC) -o bin/main bin/main.o $(OBJS) $(FLAGS) $(LIB)

obj: $(OBJS)

bin/main.o: src/main.cpp
	$(CC) -c -o bin/main.o src/main.cpp $(FLAGS) $(INCLUDE)

run:
	bin/main
	
clean:
	rm -f bin/main bin/*.o

#dependency builder
#depend: .depend

#.depend: $(SOURCES)
#	rm -f ./.depend
#	$(CC) $(CFLAGS) -MM $^ -MF  ./.depend;

#include .depend
#dependency

bin/%.o : %.$(EXT) %.$(HEXT)
	$(CC) -c $< -o $@ $(FLAGS) $(INCLUDE)
