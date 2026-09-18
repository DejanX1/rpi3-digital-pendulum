#define _POSIX_C_SOURCE 200809L

/*
 * Physics thread handling ball movement based on tilt (gyro mode) 
 * or manual direction input, including boundary collision detection.
 */

#include "task3_physics.h"

#include <stdbool.h>
#include <time.h>

#define PERIOD_NS (20L * 1000L * 1000L) /* 20 ms */
#define NSEC_PER_SEC 1000000000L

#define GRID_MIN 0.0f
#define GRID_MAX 7.0f

/* Mode 1 (tilt): how strongly tilt accelerates the ball. Placeholder
 * value, not derived from a real physical model */
#define TILT_ACCEL_PER_DEG 0.05f

/* Mode 2 (manual): constant speed */
#define MANUAL_SPEED 2.0f

static void timespec_add_ns(struct timespec *t, long ns)
{
    t->tv_nsec += ns;
    while (t->tv_nsec >= NSEC_PER_SEC) {
        t->tv_nsec -= NSEC_PER_SEC;
        t->tv_sec += 1;
    }
}

/* Clamps position to grid boundaries and stops velocity on impact. */
static bool clamp_axis(float *pos, float *vel)
{
    if (*pos < GRID_MIN) { *pos = GRID_MIN; *vel = 0.0f; return true; }
    if (*pos > GRID_MAX) { *pos = GRID_MAX; *vel = 0.0f; return true; }
    return false;
}

void *task3_physics_thread(void *arg)
{
    shared_state_t *state = (shared_state_t *)arg;
    struct timespec next;
    ball_state_t ball = { .pos_x = 3.5f, .pos_y = 3.5f, .vel_x = 0.0f, .vel_y = 0.0f };
    const float dt = PERIOD_NS / (float)NSEC_PER_SEC;

    clock_gettime(CLOCK_MONOTONIC, &next);

    while (shared_state_is_running(state)) {
        system_mode_t mode = shared_state_get_mode(state);

        if (mode == MODE_GYRO) {
            imu_tilt_t tilt = shared_state_get_tilt(state);

            ball.vel_x +=  tilt.pitch_deg * TILT_ACCEL_PER_DEG * dt;
            ball.vel_y += -tilt.roll_deg  * TILT_ACCEL_PER_DEG * dt;
        } else {
            direction_t dir = shared_state_get_manual_direction(state);

            switch (dir) {
            	case DIR_NORTH: ball.vel_x = 0.0f;          ball.vel_y = -MANUAL_SPEED; 
            		break;
            	case DIR_SOUTH: ball.vel_x = 0.0f;          ball.vel_y =  MANUAL_SPEED; 
            		break;
            	case DIR_EAST:  ball.vel_x =  MANUAL_SPEED; ball.vel_y = 0.0f;
            		break;
            	case DIR_WEST:  ball.vel_x = -MANUAL_SPEED; ball.vel_y = 0.0f;
            		break;
            	default:        ball.vel_x = 0.0f;          ball.vel_y = 0.0f;
            		break;
            }
        }

        ball.pos_x += ball.vel_x * dt;
        ball.pos_y += ball.vel_y * dt;

        bool hit_x = clamp_axis(&ball.pos_x, &ball.vel_x);
        bool hit_y = clamp_axis(&ball.pos_y, &ball.vel_y);

        if (hit_x || hit_y) {
            direction_t hit_dir;

            if (hit_x)
                hit_dir = (ball.pos_x <= GRID_MIN) ? DIR_WEST : DIR_EAST;
            else
                hit_dir = (ball.pos_y <= GRID_MIN) ? DIR_NORTH : DIR_SOUTH;

            shared_state_set_collision(state, hit_dir);
        }

        shared_state_set_ball(state, ball);

        timespec_add_ns(&next, PERIOD_NS);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
    }

    return NULL;
}
