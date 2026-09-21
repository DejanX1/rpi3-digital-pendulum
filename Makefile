# Build the digital pendulum user-space application (the four RT threads).
# The kernel drivers are built separately, each from its own Makefile under
# drivers/haptic_feedback/ and drivers/proximity_warning/.
#
#   make                      - build ./digital_pendulum
#   sudo ./digital_pendulum   - run it (RT scheduling needs privileges)
#   make clean                - remove build output

CC       ?= gcc
CFLAGS   ?= -Wall -Wextra -std=c11 -O2
CPPFLAGS += -Iinclude
LDLIBS   += -lm

TARGET  := digital_pendulum
SOURCES := src/main.c \
           src/task1_imu.c \
           src/task2_joystick.c \
           src/task3_physics.c \
           src/task4_display.c \
           src/lsm9ds1.c
OBJECTS := $(SOURCES:.c=.o)

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CC) $(CFLAGS) -pthread -o $@ $^ $(LDFLAGS) $(LDLIBS)

%.o: %.c
	$(CC) $(CFLAGS) $(CPPFLAGS) -pthread -c -o $@ $<

clean:
	rm -f $(OBJECTS) $(TARGET)

.PHONY: all clean
