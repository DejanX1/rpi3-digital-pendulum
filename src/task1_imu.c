#define _POSIX_C_SOURCE 200809L

/*
 * Task 1: IMU Acquisition (SCHED_FIFO 99, 5 ms / 200 Hz).
 *
 * Periodically reads LSM9DS1 over I2C, computes accelerometer-only tilt, and updates shared_state.
 * Uses clock_nanosleep (CLOCK_MONOTONIC, TIMER_ABSTIME) to prevent drift.
 */

#include "task1_imu.h"
#include "lsm9ds1.h"

#include <math.h>
#include <stdio.h>
#include <time.h>

#define I2C_DEVICE "/dev/i2c-1"
#define PERIOD_NS  (5L * 1000L * 1000L) /* 5 ms */
#define NSEC_PER_SEC 1000000000L
#define RAD_TO_DEG_F 57.29577951308232f

static void timespec_add_ns(struct timespec *t, long ns)
{
    t->tv_nsec += ns;
    while (t->tv_nsec >= NSEC_PER_SEC) {
        t->tv_nsec -= NSEC_PER_SEC;
        t->tv_sec += 1;
    }
}

static uint64_t timespec_to_ns(const struct timespec *t)
{
    return (uint64_t)t->tv_sec * (uint64_t)NSEC_PER_SEC + (uint64_t)t->tv_nsec;
}

/* Accelerometer-only pitch/roll. Verify axis orientation during 
 * display testing; signs may need adjustment depending on mounting. */
static void tilt_from_accel(const lsm9ds1_vector_t *g, float *pitch_deg, float *roll_deg)
{
    *pitch_deg = atan2f(-g->x, sqrtf(g->y * g->y + g->z * g->z)) * RAD_TO_DEG_F;
    *roll_deg  = atan2f(g->y, g->z) * RAD_TO_DEG_F;
}

void *task1_imu_thread(void *arg)
{
    shared_state_t *state = (shared_state_t *)arg;
    struct timespec next;
    int fd;

    fd = lsm9ds1_open(I2C_DEVICE);
    if (fd < 0) {
        perror("task1_imu: lsm9ds1_open");
        return NULL;
    }

    if (lsm9ds1_init(fd) < 0) {
        perror("task1_imu: lsm9ds1_init");
        lsm9ds1_close(fd);
        return NULL;
    }

    clock_gettime(CLOCK_MONOTONIC, &next);

    while (shared_state_is_running(state)) {
        lsm9ds1_sample_t sample;

        if (lsm9ds1_read_sample(fd, &sample) == 0) {
            imu_tilt_t tilt;
            struct timespec now;

            tilt_from_accel(&sample.accel_g, &tilt.pitch_deg, &tilt.roll_deg);
            clock_gettime(CLOCK_MONOTONIC, &now);
            tilt.timestamp_ns = timespec_to_ns(&now);

            shared_state_set_tilt(state, tilt);
        } else {
            perror("task1_imu: lsm9ds1_read_sample");
        }

        timespec_add_ns(&next, PERIOD_NS);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
    }

    lsm9ds1_close(fd);
    return NULL;
}
