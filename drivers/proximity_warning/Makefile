# Out-of-tree kbuild Makefile for the proximity_warning driver.
#
# Build (on the RPi, against its running kernel's headers):
#   make
# Load / unload:
#   sudo insmod proximity_warning_drv.ko
#   sudo rmmod proximity_warning_drv
# Try it:
#   cat  /sys/kernel/pendulum/proximity_warning
#   echo "EAST 1" | sudo tee /sys/kernel/pendulum/proximity_warning
# Clean:
#   make clean

obj-m := proximity_warning_drv.o

# Shared uapi header (direction/risk wire format), agreed on in the
# Driver Communication Formats task - keep this in sync, don't duplicate.
ccflags-y := -I$(src)/../../include

KDIR ?= /lib/modules/$(shell uname -r)/build
PWD  := $(shell pwd)

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
