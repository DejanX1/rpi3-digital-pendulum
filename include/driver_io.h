#ifndef DRIVER_IO_H
#define DRIVER_IO_H

#include "shared_state.h" /* direction_t */

typedef struct {
    int haptic_fd;    /* -1 when /dev/haptic_feedback is unavailable */
    int proximity_fd; /* -1 when the proximity_warning attribute is unavailable */
} driver_io_t;

/* Opens both interfaces. Returns 0 if both are available, -1 if at least
 * one is missing - the struct is usable either way. */
int driver_io_open(driver_io_t *io);

void driver_io_close(driver_io_t *io);

/*
 * Fires one buzzer pulse. The three values are clamped to the ranges
 * documented in haptic_feedback.h, so a caller can pass a number derived
 * straight from impact speed without range-checking it first.
 * Returns 0 on success, -1 if the device is unavailable or the ioctl failed.
 */
int driver_io_haptic_pulse(const driver_io_t *io, int force_percent,
                           int frequency_hz, int duration_ms);

/* Silences the buzzer immediately, cancelling any pulse in progress. */
int driver_io_haptic_stop(const driver_io_t *io);

/*
 * Reports threat direction and risk level through proximity_warning.
 * Every call overwrites the stored value, so the caller should report on
 * state changes rather than on every physics step.
 * Returns 0 on success, -1 if the interface is unavailable or the write failed.
 */
int driver_io_proximity_report(const driver_io_t *io, direction_t direction,
                               int risk_level);

#endif
