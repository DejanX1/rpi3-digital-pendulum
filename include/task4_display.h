#ifndef TASK4_DISPLAY_H
#define TASK4_DISPLAY_H

/*
 * Task 4: Display & telemetry thread (SCHED_OTHER, 40 ms / 25 FPS).
 *
 * Created by application bootstrap; scheduling attributes are set
 * externally via pthread_attr_t (SCHED_OTHER needs no special priority).
 * Runs until shared_state_is_running() becomes false.
 */

#include "shared_state.h"

/* arg must point to the shared_state_t the whole application uses. */
void *task4_display_thread(void *arg);

#endif
