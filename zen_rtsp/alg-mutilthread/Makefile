CPP     = g++
RM      = rm -rf
## debug flag
DBG_ENABLE   = 1
## source file path
#SRC_PATH   := $(PWD)
#SRC_PATH=$(PWD)
SRC_PATH=./

## target exec file name
TARGET     := libzenalg.so

## get all source files
SRCS         += $(wildcard $(SRC_PATH)/*.cpp)

## all .o based on all .c
OBJS        := $(SRCS:.cpp=.o)

## need libs, add at here
LIBS :=

## used headers  file path
INCLUDE_PATH += /usr/include/python3.5/
INCLUDE_PATH += $(SRC_PATH)/include
INCLUDE_PATH += $(SRC_PATH)

## used include librarys file path
LIBRARY_PATH := /lib

## debug for debug info, when use gdb to debug
ifeq (1, ${DBG_ENABLE})
	CFLAGS += -D_DEBUG -O0 -g -DDEBUG=1
endif

## get all include path
CFLAGS  += $(foreach dir, $(INCLUDE_PATH), -I$(dir))  -g
CFLAGS += -fopenmp

## get all library path
LDFLAGS += $(foreach lib, $(LIBRARY_PATH), -L$(lib)) -g DDEBUG=1

## get all librarys
LDFLAGS += $(foreach lib, $(LIBS), -l$(lib))
LDFLAGS += -lgomp -lpthread 
LDFLAGS += -L../lib -lobject_detection_yolov3_async


all:build
build:
	$(CC) -fPIC -c $(CFLAGS) $(SRCS) -g
	$(CC) -fPIC -shared -o $(TARGET) $(OBJS)
.PHONY:clean
clean:
	$(RM) $(OBJS) $(TARGET)
