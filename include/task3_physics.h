#ifndef TASK3_PHYSICS_H
#define TASK3_PHYSICS_H

/*
 * Physics thread interface for updating ball position and state.
 *
 * Created by application bootstrap; scheduling attributes are set
 * externally via pthread_attr_t. Runs until shared_state_is_running()
 * becomes false.
 *
 */

#include "shared_state.h"

/* arg must point to the shared_state_t the whole application uses. */
void *task3_physics_thread(void *arg);

#endif
