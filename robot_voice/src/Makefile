#common makefile header

DIR_INC = /home/raspberry/spotmicro/src/spotMicro/robot_voice/include
DIR_BIN = /home/raspberry/spotmicro/src/spotMicro/robot_voice/bin
DIR_LIB = /home/raspberry/spotmicro/src/spotMicro/robot_voice/libs

TARGET	= iat_publish
BIN_TARGET = $(DIR_BIN)/$(TARGET)

CROSS_COMPILE = 
CFLAGS = -g -Wall -I$(DIR_INC)

#可根据实际需要修改，选择所需平台SDK，默认为x64或x86平台。

LDFLAGS := -L$(DIR_LIB)/RaspberryPi


LDFLAGS += -lmsc -lrt -ldl -lpthread -lasound -lstdc++

OBJECTS := $(patsubst %.c,%.o,$(wildcard *.c))

$(BIN_TARGET) : $(OBJECTS)
	$(CROSS_COMPILE)gcc $(CFLAGS) $^ -o $@ $(LDFLAGS)

%.o : %.c
	$(CROSS_COMPILE)gcc -c $(CFLAGS) $< -o $@
clean:
	@rm -f *.o $(BIN_TARGET)

.PHONY:clean

#common makefile foot
