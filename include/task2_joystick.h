#ifndef TASK2_JOYSTICK_H
#define TASK2_JOYSTICK_H

/*
 * Joystick interrupt handling
 *
 * Created by application bootstrap; scheduling attributes are set
 * externally via pthread_attr_t. Runs until shared_state_is_running()
 * becomes false (checked on every poll() timeout, see task2_joystick.c).
 */

#include "shared_state.h"

/* arg must point to the shared_state_t the whole application uses. */
void *task2_joystick_thread(void *arg);

#endif
