#define _POSIX_C_SOURCE 200809L

/*
 * Physics thread handling ball movement based on tilt (gyro mode)
 * or manual direction input, with air-cushion damping and wall collisions.
 */

#include "task3_physics.h"
#include "driver_io.h"
#include "proximity_warning.h"

#include <math.h>
#include <stdbool.h>
#include <syslog.h>
#include <time.h>

#define PERIOD_NS (20L * 1000L * 1000L) /* 20 ms */
#define NSEC_PER_SEC 1000000000L

#define GRID_MIN 0.0f
#define GRID_MAX 7.0f

/*
 * Mode 1 (tilt): acceleration from tilt, and the air-cushion drag that
 * opposes motion. Together they decide the steady speed a held tilt
 * settles at:
 *
 *     steady speed [cells/s] = TILT_ACCEL_PER_DEG / AIR_DRAG_PER_SEC * tilt
 *
 * With the values below that is 0.2 cells/s per degree, so a 20 degree
 * tilt glides at about 4 cells/s. Drag also sets how quickly the ball
 * coasts to a stop once the board is levelled (about 1 second here).
 */
#define TILT_ACCEL_PER_DEG 0.20f
#define AIR_DRAG_PER_SEC   1.00f

/* Mode 2 (manual): constant speed, so drag does not apply there. */
#define MANUAL_SPEED 2.0f

/*
 * Wall response. WALL_RESTITUTION is the fraction of speed that survives
 * an impact: 0.0 stops the ball dead, 1.0 is a perfect bounce. Bouncing
 * is used in gyro mode only - in manual mode the heading is the user's
 * choice, so a bounce would immediately be overridden and the ball would
 * just rattle against the wall.
 */
#define WALL_RESTITUTION 0.5f

/* Below this speed a bounce is not worth it - the ball would only jitter
 * against the wall and re-trigger the impact. */
#define BOUNCE_MIN_SPEED 0.3f

/*
 * Impact speed to buzzer pulse. The upper bound comes from measurements on
 * the mounted board, where hits ran from a gentle touch up to about
 * 10 cells/s. Speed drives pitch and duration, as the assignment asks,
 * and volume along with them.
 */
#define IMPACT_SPEED_MAX    10.0f
#define HAPTIC_MIN_IMPACT   0.3f /* slower than this is the ball settling, not a hit */
#define HAPTIC_FORCE_MIN    30
#define HAPTIC_FORCE_MAX    100
#define HAPTIC_FREQ_MIN     500
#define HAPTIC_FREQ_MAX     2500
#define HAPTIC_DURATION_MIN 50
#define HAPTIC_DURATION_MAX 200

/* The assignment puts the early warning at exactly one LED cell from a wall. */
#define PROXIMITY_DISTANCE 1.0f

static void timespec_add_ns(struct timespec *t, long ns)
{
    t->tv_nsec += ns;
    while (t->tv_nsec >= NSEC_PER_SEC) {
        t->tv_nsec -= NSEC_PER_SEC;
        t->tv_sec += 1;
    }
}

/* Turns an impact into a pulse: harder hits are louder, higher pitched and
 * longer. Gentle contacts are skipped so the buzzer does not click every
 * time the ball drifts into a wall it is already resting against. */
static void sound_impact(const driver_io_t *io, float impact_speed)
{
    float ratio;

    if (impact_speed < HAPTIC_MIN_IMPACT)
        return;

    ratio = impact_speed / IMPACT_SPEED_MAX;
    if (ratio > 1.0f)
        ratio = 1.0f;

    driver_io_haptic_pulse(io,
        HAPTIC_FORCE_MIN    + (int)(ratio * (HAPTIC_FORCE_MAX - HAPTIC_FORCE_MIN)),
        HAPTIC_FREQ_MIN     + (int)(ratio * (HAPTIC_FREQ_MAX - HAPTIC_FREQ_MIN)),
        HAPTIC_DURATION_MIN + (int)(ratio * (HAPTIC_DURATION_MAX - HAPTIC_DURATION_MIN)));
}

/* Which wall the ball is dangerously close to, or DIR_NONE when it is clear
 * of all of them. In a corner the nearer wall wins. */
static direction_t threat_direction(const ball_state_t *ball)
{
    float nearest = ball->pos_x - GRID_MIN; /* distance to the west wall */
    direction_t dir = DIR_WEST;
    float distance;

    distance = GRID_MAX - ball->pos_x;
    if (distance < nearest) { nearest = distance; dir = DIR_EAST; }

    distance = ball->pos_y - GRID_MIN;
    if (distance < nearest) { nearest = distance; dir = DIR_NORTH; }

    distance = GRID_MAX - ball->pos_y;
    if (distance < nearest) { nearest = distance; dir = DIR_SOUTH; }

    return (nearest <= PROXIMITY_DISTANCE) ? dir : DIR_NONE;
}

static const char *direction_name(direction_t dir)
{
    switch (dir) {
    case DIR_NORTH: return "NORTH";
    case DIR_SOUTH: return "SOUTH";
    case DIR_EAST:  return "EAST";
    case DIR_WEST:  return "WEST";
    default:        return "NONE";
    }
}

/*
 * Keeps the ball inside the grid and applies the wall response. Returns
 * true when this axis is against a wall, and reports through impact_speed
 * how fast the ball arrived - captured before the velocity is changed,
 * because that is the number the buzzer strength is derived from.
 */
static bool collide_axis(float *pos, float *vel, bool bounce, float *impact_speed)
{
    /* Note the >= / <=: a ball resting exactly on the boundary still counts
     * as touching. With a strict > it would read as "clear of the wall" for
     * one step, and the next nudge past the edge would be reported as a
     * fresh impact over and over while the ball just sits there. */
    if (*pos <= GRID_MIN)
        *pos = GRID_MIN;
    else if (*pos >= GRID_MAX)
        *pos = GRID_MAX;
    else
        return false;

    *impact_speed = fabsf(*vel);

    if (bounce && *impact_speed >= BOUNCE_MIN_SPEED)
        *vel = -(*vel) * WALL_RESTITUTION;
    else
        *vel = 0.0f;

    return true;
}

void *task3_physics_thread(void *arg)
{
    shared_state_t *state = (shared_state_t *)arg;
    struct timespec next;
    ball_state_t ball = { .pos_x = 3.5f, .pos_y = 3.5f, .vel_x = 0.0f, .vel_y = 0.0f };
    const float dt = PERIOD_NS / (float)NSEC_PER_SEC;
    bool touching_x = false;
    bool touching_y = false;
    direction_t reported_threat = DIR_NONE;
    driver_io_t io;

    /* Warns once per interface if a module is not loaded, and the calls below
     * then do nothing - the simulation still runs, just silently. */
    driver_io_open(&io);

    /* Start from a known state, in case a previous run left a stale warning. */
    driver_io_proximity_report(&io, DIR_NONE, PROXIMITY_RISK_SAFE);

    clock_gettime(CLOCK_MONOTONIC, &next);

    while (shared_state_is_running(state)) {
        system_mode_t mode = shared_state_get_mode(state);
        bool gyro_mode = (mode == MODE_GYRO);
        float impact_x = 0.0f;
        float impact_y = 0.0f;
        bool hit_x;
        bool hit_y;

        if (gyro_mode) {
            imu_tilt_t tilt = shared_state_get_tilt(state);

            /* Mapping confirmed on the mounted board: tilting the right edge
             * down moves the ball east (pitch -> x), tilting the top edge down
             * moves it north (-roll -> y). Not the pitch/roll convention the
             * names suggest, so do not "fix" the axes without retesting. */
            ball.vel_x +=  tilt.pitch_deg * TILT_ACCEL_PER_DEG * dt;
            ball.vel_y += -tilt.roll_deg  * TILT_ACCEL_PER_DEG * dt;

            /* Air cushion: drag pulls the speed back towards zero, so a held
             * tilt reaches a steady glide instead of accelerating forever. */
            ball.vel_x -= ball.vel_x * AIR_DRAG_PER_SEC * dt;
            ball.vel_y -= ball.vel_y * AIR_DRAG_PER_SEC * dt;
        } else {
            direction_t dir = shared_state_get_manual_direction(state);

            switch (dir) {
            
            	case DIR_NORTH: 
            		ball.vel_x = 0.0f;
            		ball.vel_y = -MANUAL_SPEED;
            		break;
            	
            	case DIR_SOUTH:
            		ball.vel_x = 0.0f;
            		ball.vel_y =  MANUAL_SPEED;
            		break;
            	
            	case DIR_EAST:
            		ball.vel_x =  MANUAL_SPEED;
            		ball.vel_y = 0.0f;
            		break;
            
            	case DIR_WEST:
            		ball.vel_x = -MANUAL_SPEED;
            		ball.vel_y = 0.0f;
            		break;
            	
            	default:
            		ball.vel_x = 0.0f;
            		ball.vel_y = 0.0f;
            		break;
            }
        }

        ball.pos_x += ball.vel_x * dt;
        ball.pos_y += ball.vel_y * dt;

        hit_x = collide_axis(&ball.pos_x, &ball.vel_x, gyro_mode, &impact_x);
        hit_y = collide_axis(&ball.pos_y, &ball.vel_y, gyro_mode, &impact_y);

        /*
         * Report an impact only when the ball first reaches a wall, not on
         * every step it spends resting against one - otherwise holding a
         * direction into a wall would fire a collision 50 times a second.
         */
        if (hit_x && !touching_x) {
            direction_t dir = (ball.pos_x <= GRID_MIN) ? DIR_WEST : DIR_EAST;

            shared_state_set_collision(state, dir);
            sound_impact(&io, impact_x);
            syslog(LOG_INFO, "collision %s at %.2f cells/s",
                   direction_name(dir), impact_x);
        }

        if (hit_y && !touching_y) {
            direction_t dir = (ball.pos_y <= GRID_MIN) ? DIR_NORTH : DIR_SOUTH;

            shared_state_set_collision(state, dir);
            sound_impact(&io, impact_y);
            syslog(LOG_INFO, "collision %s at %.2f cells/s",
                   direction_name(dir), impact_y);
        }

        touching_x = hit_x;
        touching_y = hit_y;

        /* Early warning: report only when the threatened wall changes, so the
         * sysfs attribute is written on transitions instead of 50 times a
         * second while the ball lingers near an edge. */
        {
            direction_t threat = threat_direction(&ball);

            if (threat != reported_threat) {
                driver_io_proximity_report(&io, threat,
                                           threat == DIR_NONE ? PROXIMITY_RISK_SAFE
                                                              : PROXIMITY_RISK_WARNING);
                reported_threat = threat;
            }
        }

        shared_state_set_ball(state, ball);

        timespec_add_ns(&next, PERIOD_NS);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
    }

    /* Leave the warning cleared on the way out, the way the display thread
     * clears the matrix - otherwise an external reader would keep seeing a
     * threat long after the program stopped. */
    driver_io_proximity_report(&io, DIR_NONE, PROXIMITY_RISK_SAFE);
    driver_io_close(&io);

    return NULL;
}
