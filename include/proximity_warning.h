#ifndef PROXIMITY_WARNING_H
#define PROXIMITY_WARNING_H

/*
 * Wire format for the proximity_warning sysfs attribute, shared verbatim
 * between the kernel driver and user-space (Task 3, physics thread).
 *
 * Unlike /dev/haptic_feedback (a binary struct), this interface is a
 * plain text line so it stays trivially readable by any external tool
 * (cat, a shell script, a monitoring process) as required by the spec
 * ("any external system process can read this file").
 *
 * Line format, used identically for both write() (Task 3 -> driver) and
 * read() (driver -> any reader):
 *
 *     "<DIRECTION> <RISK>\n"
 *
 *   <DIRECTION> one of: NONE, NORTH, SOUTH, EAST, WEST
 *   <RISK>      integer risk level (see PROXIMITY_RISK_* below)
 *
 * Example lines: "NONE 0\n"  "EAST 1\n"
 *
 * Task 3 writes a new line every time the ball crosses the critical
 * distance threshold (exactly 1 LED cell from a wall) in either
 * direction: once when it becomes critical (e.g. "NORTH 1\n") and once
 * more when it is no longer critical (e.g. "NONE 0\n"). The driver simply
 * stores and reflects back the last line written; it does not interpret
 * direction or risk itself.
 */

#define PROXIMITY_WARNING_SYSFS_PATH "/sys/kernel/pendulum/proximity_warning"

/* Longest valid line, incl. worst case direction ("NORTH"), separator,
 * one risk digit, '\n' and a NUL terminator: "NORTH 1\n\0" = 9. Rounded
 * up for headroom if PROXIMITY_RISK_* ever needs 2 digits. */
#define PROXIMITY_WARNING_MAX_LEN 16

#define PROXIMITY_DIR_NONE  "NONE"
#define PROXIMITY_DIR_NORTH "NORTH"
#define PROXIMITY_DIR_SOUTH "SOUTH"
#define PROXIMITY_DIR_EAST  "EAST"
#define PROXIMITY_DIR_WEST  "WEST"

/* Risk levels. Only SAFE/WARNING are produced by the current single
 * critical-distance threshold; the numeric gap to 2/3 is reserved so a
 * finer-grained warning (e.g. distance-proportional) can be added later
 * without changing the line format or breaking existing readers. */
#define PROXIMITY_RISK_SAFE    0
#define PROXIMITY_RISK_WARNING 1

#endif /* PROXIMITY_WARNING_H */
