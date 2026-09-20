#define _POSIX_C_SOURCE 200809L


#include "task2_joystick.h"

#include <errno.h>
#include <fcntl.h>
#include <linux/input.h>
#include <poll.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#define JOYSTICK_DEVICE_NAME "Sense HAT Joystick"
#define POLL_TIMEOUT_MS 200 /* how often we re-check the running flag */

/* Same discovery pattern as test-hardver/led_matrix/led_sos.c: match by
 * name via sysfs rather than hardcoding "event2", since the index isn't
 * guaranteed stable across boots/kernels. */
static int open_joystick_device(void)
{
    for (int index = 0; index < 32; ++index) {
        char name_path[64];
        char dev_path[32];
        char name[128];
        FILE *name_file;
        int fd;

        snprintf(name_path, sizeof(name_path),
                 "/sys/class/input/event%d/device/name", index);

        name_file = fopen(name_path, "r");
        if (name_file == NULL)
            continue;

        if (fgets(name, sizeof(name), name_file) == NULL) {
            fclose(name_file);
            continue;
        }
        fclose(name_file);

        if (strstr(name, JOYSTICK_DEVICE_NAME) == NULL)
            continue;

        snprintf(dev_path, sizeof(dev_path), "/dev/input/event%d", index);
        fd = open(dev_path, O_RDONLY);
        if (fd >= 0)
            return fd;
    }

    errno = ENODEV;
    return -1;
}

static direction_t direction_from_key(unsigned int code)
{
    switch (code) {
    case KEY_UP:    return DIR_NORTH;
    case KEY_DOWN:  return DIR_SOUTH;
    case KEY_RIGHT: return DIR_EAST;
    case KEY_LEFT:  return DIR_WEST;
    default:        return DIR_NONE;
    }
}

void *task2_joystick_thread(void *arg)
{
    shared_state_t *state = (shared_state_t *)arg;
    struct pollfd pfd;
    int fd;

    fd = open_joystick_device();
    if (fd < 0) {
        perror("task2_joystick: Sense HAT joystick device not found");
        return NULL;
    }

    pfd.fd = fd;
    pfd.events = POLLIN;

    while (shared_state_is_running(state)) {
        int ready = poll(&pfd, 1, POLL_TIMEOUT_MS);

        if (ready < 0) {
            if (errno == EINTR)
                continue;
            perror("task2_joystick: poll");
            break;
        }
        if (ready == 0)
            continue; /* timeout: no event yet, loop back to re-check running */

        struct input_event ev;
        ssize_t n = read(fd, &ev, sizeof(ev));
        if (n != (ssize_t)sizeof(ev))
            continue;

        /* Only react to key-down edges, ignore release (value=0) and any
         * autorepeat (value=2). */
        if (ev.type != EV_KEY || ev.value != 1)
            continue;

        if (ev.code == KEY_ENTER) {
            system_mode_t current = shared_state_get_mode(state);
            shared_state_set_mode(state, current == MODE_GYRO ? MODE_MANUAL : MODE_GYRO);
        } else {
            direction_t dir = direction_from_key(ev.code);
            if (dir != DIR_NONE)
                shared_state_set_manual_direction(state, dir);
        }
    }

    close(fd);
    return NULL;
}
