#ifndef TASK1_IMU_H
#define TASK1_IMU_H

/*
 * Task 1: IMU Acquisition (SCHED_FIFO 99, 5 ms / 200 Hz).
 *
 * Created by application bootstrap.
 * Scheduling attributes are set externally via pthread_attr_t.
 * Runs until shared_state_is_running() becomes false.
 */

#include "shared_state.h"

/* arg must point to the shared_state_t the whole application uses. */
void *task1_imu_thread(void *arg);

#endif
