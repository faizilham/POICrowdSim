# generic makefile

CC = g++
EXT = cpp
HEXT = h

# modules
INCLUDE_DIR=includes
THIRD_PARTY_DIR=tinyxml2

MODULES = shapes.cpp mapobject.cpp tinyxml2.cpp

LIB = -lm
INCLUDE = $(addprefix -I,$(INCLUDE_DIR))

MODE = debug
# release or debug

FLAGS = -Wall -std=c++11

ifeq ($(MODE), release)
FLAGS += -O2
else
FLAGS += -O0 -g
endif

# Everything after this is generic, no need to edit
VPATH = src $(addprefix src/,$(THIRD_PARTY_DIR)) $(INCLUDE_DIR)
SOURCES = $(MODULES)
OBJS = $(SOURCES:%.$(EXT)=bin/%.o)

.PHONY: all run clean obj
  
all: obj
	$(CC) -o bin/main $(OBJS) $(LIB)

obj: $(OBJS)

run:
	bin/main
	
clean:
	rm -f bin/main bin/*.o

bin/%.o : %.$(EXT) %.$(HEXT)
	$(CC) -c $< -o $@ $(FLAGS) $(INCLUDE)

