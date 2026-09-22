#define _POSIX_C_SOURCE 200809L

#include "driver_io.h"
#include "haptic_feedback.h"
#include "proximity_warning.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

/* Ranges documented in haptic_feedback.h - the driver rejects anything
 * outside them with -EINVAL, so clamp here instead of failing the call. */
#define FORCE_MIN    0
#define FORCE_MAX    100
#define FREQ_MIN     200
#define FREQ_MAX     4000
#define DURATION_MIN 1
#define DURATION_MAX 1000

static int clamp_int(int value, int low, int high)
{
    if (value < low)
        return low;
    if (value > high)
        return high;
    return value;
}

static const char *direction_token(direction_t direction)
{
    switch (direction) {
    case DIR_NORTH: return PROXIMITY_DIR_NORTH;
    case DIR_SOUTH: return PROXIMITY_DIR_SOUTH;
    case DIR_EAST:  return PROXIMITY_DIR_EAST;
    case DIR_WEST:  return PROXIMITY_DIR_WEST;
    default:        return PROXIMITY_DIR_NONE;
    }
}

int driver_io_open(driver_io_t *io)
{
    io->haptic_fd = open(HAPTIC_DEVICE_PATH, O_RDWR);
    if (io->haptic_fd < 0)
        fprintf(stderr, "driver_io: %s unavailable (%s) - continuing without sound\n",
                HAPTIC_DEVICE_PATH, strerror(errno));

    io->proximity_fd = open(PROXIMITY_WARNING_SYSFS_PATH, O_WRONLY);
    if (io->proximity_fd < 0)
        fprintf(stderr, "driver_io: %s unavailable (%s) - continuing without proximity reports\n",
                PROXIMITY_WARNING_SYSFS_PATH, strerror(errno));

    return (io->haptic_fd >= 0 && io->proximity_fd >= 0) ? 0 : -1;
}

void driver_io_close(driver_io_t *io)
{
    if (io->haptic_fd >= 0) {
        close(io->haptic_fd);
        io->haptic_fd = -1;
    }

    if (io->proximity_fd >= 0) {
        close(io->proximity_fd);
        io->proximity_fd = -1;
    }
}

int driver_io_haptic_pulse(const driver_io_t *io, int force_percent,
                           int frequency_hz, int duration_ms)
{
    struct haptic_pulse pulse;

    if (io->haptic_fd < 0)
        return -1;

    pulse.force_percent = (__u8)clamp_int(force_percent, FORCE_MIN, FORCE_MAX);
    pulse.frequency_hz  = (__u16)clamp_int(frequency_hz, FREQ_MIN, FREQ_MAX);
    pulse.duration_ms   = (__u16)clamp_int(duration_ms, DURATION_MIN, DURATION_MAX);

    return (ioctl(io->haptic_fd, HAPTIC_IOC_PULSE, &pulse) == 0) ? 0 : -1;
}

int driver_io_haptic_stop(const driver_io_t *io)
{
    if (io->haptic_fd < 0)
        return -1;

    return (ioctl(io->haptic_fd, HAPTIC_IOC_STOP) == 0) ? 0 : -1;
}

int driver_io_proximity_report(const driver_io_t *io, direction_t direction,
                               int risk_level)
{
    char line[PROXIMITY_WARNING_MAX_LEN];
    int length;

    if (io->proximity_fd < 0)
        return -1;

    length = snprintf(line, sizeof(line), "%s %d\n",
                      direction_token(direction), risk_level);
    if (length < 0 || (size_t)length >= sizeof(line))
        return -1;

    /* A sysfs store() takes the whole value from the start of the buffer,
     * so rewind before each report rather than appending to the previous one. */
    if (lseek(io->proximity_fd, 0, SEEK_SET) < 0)
        return -1;

    if (write(io->proximity_fd, line, (size_t)length) != (ssize_t)length)
        return -1;

    return 0;
}
