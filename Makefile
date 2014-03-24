include ../common.mk

# flags for building the gtk library
CFLAGS = $(CFLAGS_STD) $(CFLAGS_COMMON) $(CFLAGS_VX) $(CFLAGS_GTK) -fPIC -O2
LDFLAGS =  $(LDFLAGS_VX_GTK) $(LDFLAGS_VX) $(LDFLAGS_GTK) $(LDFLAGS_IMAGESOURCE) $(LDFLAGS_COMMON) $(LDFLAGS_STD) $(LDFLAGS_LCMTYPES) -lstdc++
CXXFLAGS_STD := -g -D FILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE -D REENTRANT \
	-Wall -Wno-unused-parameter -pthread -Wno-write-strings 
CFLAGS_CXX = $(CXXFLAGS_STD) $(CFLAGS_COMMON) $(CFLAGS_VX) $(CFLAGS_GTK) -fPIC -O2 -std=c++0x

LIB = ../../lib

all: 

main: main.o
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)


clean:
	@rm -rf  $(VX_BASE) *.o *~ main
