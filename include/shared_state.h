#ifndef SHARED_STATE_H
#define SHARED_STATE_H

/*
 * Shared state connecting the 4 real-time threads of the digital pendulum:
 *
 *   Task 1 (IMU, SCHED_FIFO 99)          -> writes tilt
 *   Task 2 (Joystick IRQ, SCHED_FIFO 90) -> writes mode, manual_direction
 *   Task 3 (Physics, SCHED_RR 50)        -> reads tilt/mode/manual_direction,
 *                                           writes ball and collision_direction
 *   Task 4 (Display, SCHED_OTHER)        -> reads ball, collision_direction and mode
 *
 */

#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* Toggled by the joystick center button (Task 2). */
typedef enum {
    MODE_GYRO = 0,   /* Mode 1: tilt-controlled ball */
    MODE_MANUAL = 1  /* Mode 2: joystick-directional ball */
} system_mode_t;

/*
 * Discrete direction. Used for manual-mode heading (Task 2 -> Task 3) and for reporting
 * which wall was/will be hit (Task 3 -> Task 4 and the proximity_warning kernel interface).
 */
typedef enum {
    DIR_NONE  = 0,
    DIR_NORTH = 1, /* -Y */
    DIR_SOUTH = 2, /* +Y */
    DIR_EAST  = 3, /* +X */
    DIR_WEST  = 4  /* -X */
} direction_t;

/* IMU-derived tilt angles, produced by Task 1's sensor fusion filter. */
typedef struct {
    float pitch_deg;
    float roll_deg;
    uint64_t timestamp_ns; /* CLOCK_MONOTONIC timestamp of the sample */
} imu_tilt_t;

/* Ball kinematics on the 8x8 LED grid, produced by Task 3's physics step. */
typedef struct {
    float pos_x; /* 0.0 .. 7.0 */
    float pos_y; /* 0.0 .. 7.0 */
    float vel_x; /* LED cells / s */
    float vel_y; /* LED cells / s */
} ball_state_t;

typedef struct {
    pthread_mutex_t lock;

    system_mode_t mode;
    direction_t manual_direction;      /* set by Task 2 in MODE_MANUAL */
    imu_tilt_t tilt;                   /* set by Task 1 */
    ball_state_t ball;                 /* set by Task 3 */
    direction_t collision_direction;   /* set by Task 3 on wall hit */

    atomic_bool running;               /* clean-shutdown flag, lock-free */
} shared_state_t;

/* Zero-initializes state and creates the mutex with PRIO_INHERIT. Returns 0
 * on success, or a negative errno-style value on failure. */
static inline int shared_state_init(shared_state_t *s)
{
    pthread_mutexattr_t attr;
    int rc;

    memset(s, 0, sizeof(*s));

    rc = pthread_mutexattr_init(&attr);
    if (rc != 0)
        return -rc;

    rc = pthread_mutexattr_setprotocol(&attr, PTHREAD_PRIO_INHERIT);
    if (rc != 0) {
        pthread_mutexattr_destroy(&attr);
        return -rc;
    }

    rc = pthread_mutex_init(&s->lock, &attr);
    pthread_mutexattr_destroy(&attr);
    if (rc != 0)
        return -rc;

    s->mode = MODE_GYRO;
    s->manual_direction = DIR_NONE;
    s->collision_direction = DIR_NONE;
    atomic_init(&s->running, true);

    return 0;
}

static inline void shared_state_destroy(shared_state_t *s)
{
    pthread_mutex_destroy(&s->lock);
}

/* --- running flag: lock-free --- */

static inline bool shared_state_is_running(const shared_state_t *s)
{
    return atomic_load_explicit(&((shared_state_t *)s)->running,
                                 memory_order_acquire);
}

static inline void shared_state_request_stop(shared_state_t *s)
{
    atomic_store_explicit(&s->running, false, memory_order_release);
}

/* --- tilt: Task 1 writes, Task 3 reads --- */

static inline void shared_state_set_tilt(shared_state_t *s, imu_tilt_t tilt)
{
    pthread_mutex_lock(&s->lock);
    s->tilt = tilt;
    pthread_mutex_unlock(&s->lock);
}

static inline imu_tilt_t shared_state_get_tilt(shared_state_t *s)
{
    imu_tilt_t out;
    pthread_mutex_lock(&s->lock);
    out = s->tilt;
    pthread_mutex_unlock(&s->lock);
    return out;
}

/* --- mode: Task 2 writes, Task 3/Task 4 read --- */

static inline void shared_state_set_mode(shared_state_t *s, system_mode_t mode)
{
    pthread_mutex_lock(&s->lock);
    s->mode = mode;
    pthread_mutex_unlock(&s->lock);
}

static inline system_mode_t shared_state_get_mode(shared_state_t *s)
{
    system_mode_t out;
    pthread_mutex_lock(&s->lock);
    out = s->mode;
    pthread_mutex_unlock(&s->lock);
    return out;
}

/* --- manual_direction: Task 2 writes, Task 3 reads --- */

static inline void shared_state_set_manual_direction(shared_state_t *s,
                                                       direction_t dir)
{
    pthread_mutex_lock(&s->lock);
    s->manual_direction = dir;
    pthread_mutex_unlock(&s->lock);
}

static inline direction_t shared_state_get_manual_direction(shared_state_t *s)
{
    direction_t out;
    pthread_mutex_lock(&s->lock);
    out = s->manual_direction;
    pthread_mutex_unlock(&s->lock);
    return out;
}

/* --- ball: Task 3 writes, Task 4 reads --- */

static inline void shared_state_set_ball(shared_state_t *s, ball_state_t ball)
{
    pthread_mutex_lock(&s->lock);
    s->ball = ball;
    pthread_mutex_unlock(&s->lock);
}

static inline ball_state_t shared_state_get_ball(shared_state_t *s)
{
    ball_state_t out;
    pthread_mutex_lock(&s->lock);
    out = s->ball;
    pthread_mutex_unlock(&s->lock);
    return out;
}

/* --- collision_direction: Task 3 writes on wall hit, Task 4 / warning
 * logic reads and clears it --- */

static inline void shared_state_set_collision(shared_state_t *s,
                                               direction_t dir)
{
    pthread_mutex_lock(&s->lock);
    s->collision_direction = dir;
    pthread_mutex_unlock(&s->lock);
}

static inline direction_t shared_state_consume_collision(shared_state_t *s)
{
    direction_t out;
    pthread_mutex_lock(&s->lock);
    out = s->collision_direction;
    s->collision_direction = DIR_NONE;
    pthread_mutex_unlock(&s->lock);
    return out;
}

#endif /* SHARED_STATE_H */
