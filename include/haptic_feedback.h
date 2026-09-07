#ifndef HAPTIC_FEEDBACK_H
#define HAPTIC_FEEDBACK_H


#include <linux/ioctl.h>
#include <linux/types.h>

#define HAPTIC_DEVICE_PATH "/dev/haptic_feedback"

/* Force/duration/tone for one impact pulse. Sent as-is to the kernel,
 * which turns it into a PWM waveform via hrtimer. */
struct haptic_pulse {
    __u8  force_percent;  /* 0-100: PWM duty cycle, proportional to impact speed */
    __u16 frequency_hz;   /* 200-4000: buzzer tone frequency */
    __u16 duration_ms;    /* 1-1000: pulse length */
};

#define HAPTIC_IOC_MAGIC 'H'

/* Start a pulse; blocks until accepted by the driver, not until finished. */
#define HAPTIC_IOC_PULSE      _IOW(HAPTIC_IOC_MAGIC, 1, struct haptic_pulse)
/* Immediately silence the buzzer, cancelling any pulse in progress. */
#define HAPTIC_IOC_STOP       _IO(HAPTIC_IOC_MAGIC, 2)
/* Read back driver status: 0 = idle, 1 = pulse in progress. */
#define HAPTIC_IOC_GET_STATUS _IOR(HAPTIC_IOC_MAGIC, 3, __u8)

#endif /* HAPTIC_FEEDBACK_H */
