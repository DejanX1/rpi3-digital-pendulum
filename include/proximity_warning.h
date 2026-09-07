#ifndef PROXIMITY_WARNING_H
#define PROXIMITY_WARNING_H


#define PROXIMITY_WARNING_SYSFS_PATH "/sys/kernel/pendulum/proximity_warning"

#define PROXIMITY_WARNING_MAX_LEN 16

#define PROXIMITY_DIR_NONE  "NONE"
#define PROXIMITY_DIR_NORTH "NORTH"
#define PROXIMITY_DIR_SOUTH "SOUTH"
#define PROXIMITY_DIR_EAST  "EAST"
#define PROXIMITY_DIR_WEST  "WEST"

/* Risk levels. Only SAFE/WARNING are produced by the current single
 * critical-distance threshold. */
#define PROXIMITY_RISK_SAFE    0
#define PROXIMITY_RISK_WARNING 1

#endif
