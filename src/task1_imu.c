#define _POSIX_C_SOURCE 200809L

#include "task1_imu.h"
#include "lsm9ds1.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <time.h>

#define I2C_DEVICE "/dev/i2c-1"
#define PERIOD_NS  (5L * 1000L * 1000L) /* 5 ms */
#define NSEC_PER_SEC 1000000000L
#define RAD_TO_DEG_F 57.29577951308232f

/*
 * Filter time constant: changes slower than this are set by the
 * accelerometer, faster ones by the gyroscope. Half a second is generous
 * towards the gyroscope, which the bias calibration makes affordable -
 * the leftover drift measured on this board is under 0.07 deg/s.
 */
#define FILTER_TAU_S 0.5f

#define GYRO_PITCH_SIGN (-1.0f)
#define GYRO_ROLL_SIGN  (-1.0f)

/* A cycle longer than this means something stalled; integrating over it
 * would throw the angle off, so fall back to the accelerometer for it. */
#define MAX_SANE_DT_S 0.1f

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

/* Computes pitch and roll angles from raw accelerometer data. */
static void tilt_from_accel(const lsm9ds1_vector_t *g, float *pitch_deg, float *roll_deg)
{
    *pitch_deg = atan2f(-g->x, sqrtf(g->y * g->y + g->z * g->z)) * RAD_TO_DEG_F;
    *roll_deg  = atan2f(g->y, g->z) * RAD_TO_DEG_F;
}

void *task1_imu_thread(void *arg)
{
    shared_state_t *state = (shared_state_t *)arg;
    struct timespec next;
    lsm9ds1_vector_t gyro_bias = { 0.0f, 0.0f, 0.0f };
    float pitch_deg = 0.0f;
    float roll_deg = 0.0f;
    uint64_t previous_ns = 0;
    bool seeded = false;
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

    /* Takes about a second, and only works if the board is left alone. If it
     * is not, we carry on with a zero bias rather than a wrong one. */
    if (lsm9ds1_calibrate_gyro(fd, &gyro_bias) < 0)
        perror("task1_imu: gyro calibration skipped");

    clock_gettime(CLOCK_MONOTONIC, &next);

    while (shared_state_is_running(state)) {
        lsm9ds1_sample_t sample;

        if (lsm9ds1_read_sample(fd, &sample) == 0) {
            imu_tilt_t tilt;
            struct timespec now;
            uint64_t now_ns;
            float accel_pitch;
            float accel_roll;

            clock_gettime(CLOCK_MONOTONIC, &now);
            now_ns = timespec_to_ns(&now);

            tilt_from_accel(&sample.accel_g, &accel_pitch, &accel_roll);

            if (seeded) {
                /* Measured, not nominal: a late cycle must integrate over the
                 * time that actually passed, or the angle comes out short. */
                float dt = (float)(now_ns - previous_ns) / (float)NSEC_PER_SEC;

                if (dt > 0.0f && dt < MAX_SANE_DT_S) {
                    float alpha = FILTER_TAU_S / (FILTER_TAU_S + dt);
                    float pitch_rate = GYRO_PITCH_SIGN *
                        (sample.gyro_rad_s.y - gyro_bias.y) * RAD_TO_DEG_F;
                    float roll_rate = GYRO_ROLL_SIGN *
                        (sample.gyro_rad_s.x - gyro_bias.x) * RAD_TO_DEG_F;

                    pitch_deg = alpha * (pitch_deg + pitch_rate * dt) +
                                (1.0f - alpha) * accel_pitch;
                    roll_deg  = alpha * (roll_deg + roll_rate * dt) +
                                (1.0f - alpha) * accel_roll;
                } else {
                    pitch_deg = accel_pitch;
                    roll_deg = accel_roll;
                }
            } else {
                /* Start from the accelerometer, otherwise the filter spends
                 * its first few hundred milliseconds crawling up from zero. */
                pitch_deg = accel_pitch;
                roll_deg = accel_roll;
                seeded = true;
            }

            previous_ns = now_ns;

            tilt.pitch_deg = pitch_deg;
            tilt.roll_deg = roll_deg;
            tilt.timestamp_ns = now_ns;

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
