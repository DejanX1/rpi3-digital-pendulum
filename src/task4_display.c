#define _POSIX_C_SOURCE 200809L

/*
 * Task 4: Display & telemetry thread (SCHED_OTHER, 40 ms / 25 FPS).
 *
 * Renders shared_state's ball position as a single lit pixel on the
 * Sense HAT LED matrix, and periodically logs the coordinates to the
 * system log (assignment: "слање тренутних координата на системски
 * лог"). Not the final polished animation (no trail, no color coding by
 * speed/mode etc.) - just a real, correctly positioned dot, since the
 * point of this task was proving the render path works against the
 * live ball state (see Task #16's physics thread).
 *
 * Framebuffer auto-discovery mirrors test-hardver/led_matrix/led_sos.c:
 * match by name via /sys/class/graphics/fbN/name rather than hardcoding
 * "/dev/fb0" - already confirmed on this RPi that the Sense HAT isn't
 * always fb0 (depends on what else is registered first).
 */

#include "task4_display.h"

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <syslog.h>
#include <time.h>
#include <unistd.h>

#define PERIOD_NS (40L * 1000L * 1000L) /* 40 ms -> 25 FPS */
#define NSEC_PER_SEC 1000000000L

#define MATRIX_SIZE 8
#define COLOR_OFF   0x0000
#define BALL_COLOR  0xFFFF /* white, RGB565 */

/* Log coordinates roughly once a second instead of every 40ms frame -
 * still satisfies "send current coordinates to system log" without
 * flooding it at 25 lines/sec for the app's whole runtime. */
#define TELEMETRY_EVERY_N_FRAMES 25

static void timespec_add_ns(struct timespec *t, long ns)
{
    t->tv_nsec += ns;
    while (t->tv_nsec >= NSEC_PER_SEC) {
        t->tv_nsec -= NSEC_PER_SEC;
        t->tv_sec += 1;
    }
}

static int open_sense_hat_framebuffer(void)
{
    for (int index = 0; index < 10; ++index) {
        char name_path[64];
        char dev_path[32];
        char name[128];
        FILE *name_file;
        int fd;

        snprintf(name_path, sizeof(name_path),
                 "/sys/class/graphics/fb%d/name", index);

        name_file = fopen(name_path, "r");
        if (name_file == NULL)
            continue;

        if (fgets(name, sizeof(name), name_file) == NULL) {
            fclose(name_file);
            continue;
        }
        fclose(name_file);

        if (strstr(name, "RPi-Sense FB") == NULL &&
            strstr(name, "rpisense") == NULL)
            continue;

        snprintf(dev_path, sizeof(dev_path), "/dev/fb%d", index);
        fd = open(dev_path, O_RDWR);
        if (fd >= 0)
            return fd;
    }

    errno = ENODEV;
    return -1;
}

static int clamp_cell(float pos)
{
    int cell = (int)(pos + 0.5f); /* round to nearest cell */

    if (cell < 0)
        cell = 0;
    if (cell > MATRIX_SIZE - 1)
        cell = MATRIX_SIZE - 1;
    return cell;
}

static int write_frame(int fb, const uint16_t pixels[MATRIX_SIZE][MATRIX_SIZE])
{
    const uint8_t *data = (const uint8_t *)pixels;
    size_t remaining = MATRIX_SIZE * MATRIX_SIZE * sizeof(uint16_t);

    if (lseek(fb, 0, SEEK_SET) < 0)
        return -1;

    while (remaining > 0) {
        ssize_t written = write(fb, data, remaining);

        if (written < 0)
            return -1;

        data += written;
        remaining -= (size_t)written;
    }

    return 0;
}

void *task4_display_thread(void *arg)
{
    shared_state_t *state = (shared_state_t *)arg;
    struct timespec next;
    uint16_t pixels[MATRIX_SIZE][MATRIX_SIZE];
    int fb;
    unsigned long frame = 0;

    fb = open_sense_hat_framebuffer();
    if (fb < 0) {
        perror("task4_display: Sense HAT framebuffer not found");
        return NULL;
    }

    openlog("digital-pendulum", LOG_PID, LOG_USER);

    clock_gettime(CLOCK_MONOTONIC, &next);

    while (shared_state_is_running(state)) {
        ball_state_t ball = shared_state_get_ball(state);
        int cell_x = clamp_cell(ball.pos_x);
        int cell_y = clamp_cell(ball.pos_y);

        memset(pixels, 0, sizeof(pixels));
        pixels[cell_y][cell_x] = BALL_COLOR;

        if (write_frame(fb, pixels) < 0)
            perror("task4_display: write_frame");

        if (frame % TELEMETRY_EVERY_N_FRAMES == 0) {
            syslog(LOG_INFO, "ball position: x=%.2f y=%.2f (cell %d,%d)",
                   ball.pos_x, ball.pos_y, cell_x, cell_y);
        }
        frame++;

        timespec_add_ns(&next, PERIOD_NS);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
    }

    /* Clear the matrix before exiting, matching led_sos.c's convention. */
    memset(pixels, 0, sizeof(pixels));
    write_frame(fb, pixels);

    closelog();
    close(fb);
    return NULL;
}
