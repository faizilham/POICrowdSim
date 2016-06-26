# generic makefile

CC = g++
EXT = cpp
HEXT = h

# flags
LIB = -lm
MAINLIB = -lsfml-graphics -lsfml-window -lsfml-system
FLAGS = -Wall -std=c++11 -fopenmp
DLL_FLAGS = -fPIC -DBUILD_POICS_DLL

MODE = debug
# release or debug

ifeq ($(MODE), release)
FLAGS += -O2
else
FLAGS += -O0 -g
endif

# modules
INCLUDE_DIR=includes
THIRD_PARTY_DIR=tinyxml2 polypartition RVO2 clipper

MODULES = 	tinyxml2 polypartition image imageio clipper \
			Agent KdTree Obstacle RVOSimulator \
			shapes mapobject xmlreader navmesh planmanager \
			graph pathfinder agentbuilder gop simulator imagehelper rng

TEST = traffictest

# Everything after this is generic, no need to edit
VPATH = src $(addprefix src/,$(THIRD_PARTY_DIR)) $(INCLUDE_DIR) test
INCLUDE = -Isrc $(addprefix -I,$(INCLUDE_DIR))
SOURCES = $(addsuffix .$(EXT),$(MODULES))
OBJS = $(SOURCES:%.$(EXT)=bin/%.o)

.PHONY: all run clean test
  
all: bin/main

lib/libpoics.so: $(OBJS)
	$(CC) -shared -o lib/libpoics.so $(OBJS) $(DLL_FLAGS) $(FLAGS) $(LIB)

bin/main: lib/libpoics.so bin/main.o
	$(CC) -o bin/main bin/main.o $(FLAGS) $(LIB) -Llib -Wl,-rpath=lib -lpoics $(MAINLIB)

bin/main.o: src/main.cpp
	$(CC) -c -o bin/main.o src/main.cpp $(FLAGS) $(INCLUDE)

### tests

test: bin/traffictest bin/routevalidate

bin/traffictest: lib/libpoics.so test/traffictest.cpp
	$(CC) -o bin/traffictest test/traffictest.cpp $(FLAGS) $(INCLUDE) $(LIB) -Llib -Wl,-rpath=lib -lpoics

bin/routevalidate: lib/libpoics.so test/routevalidate.cpp
	$(CC) -o bin/routevalidate test/routevalidate.cpp $(FLAGS) $(INCLUDE) $(LIB) -Llib -Wl,-rpath=lib -lpoics

###

run:
	bin/main example/test
	
clean:
	rm -f bin/main bin/*.o lib/libpoics.so

#dependency builder
#depend: .depend

#.depend: $(SOURCES)
#	rm -f ./.depend
#	$(CC) $(CFLAGS) -MM $^ -MF  ./.depend;

#include .depend
#dependency

bin/%.o : %.$(EXT) %.$(HEXT)
	$(CC) -c $< -o $@ $(DLL_FLAGS) $(FLAGS) $(INCLUDE)
