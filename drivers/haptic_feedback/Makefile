# Out-of-tree kbuild Makefile for the haptic_feedback driver.
#
# Build (on the RPi, against its running kernel's headers):
#   make
# Load / unload:
#   sudo insmod haptic_feedback_drv.ko [gpio_pin=<n>]
#   sudo rmmod haptic_feedback_drv
# Clean:
#   make clean

obj-m := haptic_feedback_drv.o

# The shared uapi header (struct haptic_pulse, ioctl numbers) lives in
# include/ at the repo root, agreed on in the Driver Communication Formats
# task - keep this in sync with that header, don't duplicate it here.
ccflags-y := -I$(src)/../../include

KDIR ?= /lib/modules/$(shell uname -r)/build
PWD  := $(shell pwd)

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
