#ifndef TASK2_JOYSTICK_H
#define TASK2_JOYSTICK_H

/*
 * Task 2: Joystick interrupt handling (SCHED_FIFO 90, hardware interrupt).
 *
 * The Sense HAT joystick is not a bare GPIO button we can register our own
 * request_threaded_irq() on: it sits behind the ATtiny88 controller on
 * I2C (address 0x46) and is already owned by the kernel's sensehat_joystick
 * driver, which exposes it as a standard Linux input device
 * ("Raspberry Pi Sense HAT Joystick", /dev/input/eventN - verified on
 * hardware: event2). That existing driver is what actually services the
 * chip's interrupt line; a second, competing GPIO IRQ handler on the same
 * line from user space isn't possible (and isn't how this hardware is
 * meant to be consumed).
 *
 * The correct interrupt-driven consumption path for an evdev device is
 * poll()/read() on its file descriptor - the thread sleeps until the
 * kernel driver posts a real event (per Vjezba 7, "Најефикасније да се
 * не користи петља него функција poll"), it does not poll the joystick's
 * value in a busy loop.
 *
 * Created by application bootstrap; scheduling attributes are set
 * externally via pthread_attr_t. Runs until shared_state_is_running()
 * becomes false (checked on every poll() timeout, see task2_joystick.c).
 */

#include "shared_state.h"

/* arg must point to the shared_state_t the whole application uses. */
void *task2_joystick_thread(void *arg);

#endif
