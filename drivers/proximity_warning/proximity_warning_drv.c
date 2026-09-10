// SPDX-License-Identifier: GPL-2.0
/*
 * proximity_warning_drv.c - sysfs early-warning interface
 *
 * Exposes /sys/kernel/pendulum/proximity_warning via kobject/sysfs.
 * Allows user space to write and read threat direction/risk levels.
 *
 * Wire format (shared with user space via include/proximity_warning.h):
 *   "<DIRECTION> <RISK>\n"   e.g. "EAST 1\n", "NONE 0\n"
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/spinlock.h>
#include <linux/string.h>

#include "proximity_warning.h"

static struct kobject *pendulum_kobj;

/* Guards proximity_state: store() (Task 3, process context) vs. show()
 * (any reader, process context). No interrupt/timer context is involved
 * here (unlike the haptic driver's hrtimer), but the critical section is
 * still kept as short as a spinlock allows, consistent with how shared
 * state is protected elsewhere in this project. */
static DEFINE_SPINLOCK(proximity_lock);
static char proximity_state[PROXIMITY_WARNING_MAX_LEN] = PROXIMITY_DIR_NONE " 0\n";

static bool is_valid_direction(const char *dir)
{
	return strcmp(dir, PROXIMITY_DIR_NONE) == 0 ||
	       strcmp(dir, PROXIMITY_DIR_NORTH) == 0 ||
	       strcmp(dir, PROXIMITY_DIR_SOUTH) == 0 ||
	       strcmp(dir, PROXIMITY_DIR_EAST) == 0 ||
	       strcmp(dir, PROXIMITY_DIR_WEST) == 0;
}

static ssize_t proximity_warning_show(struct kobject *kobj,
				       struct kobj_attribute *attr, char *buf)
{
	ssize_t len;
	unsigned long flags;

	spin_lock_irqsave(&proximity_lock, flags);
	len = scnprintf(buf, PAGE_SIZE, "%s", proximity_state);
	spin_unlock_irqrestore(&proximity_lock, flags);

	return len;
}

static ssize_t proximity_warning_store(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t count)
{
	char direction[8];
	unsigned int risk;
	unsigned long flags;

	/* sysfs guarantees buf is NUL-terminated within PAGE_SIZE. */
	if (sscanf(buf, "%7s %u", direction, &risk) != 2)
		return -EINVAL;

	if (!is_valid_direction(direction))
		return -EINVAL;

	spin_lock_irqsave(&proximity_lock, flags);
	scnprintf(proximity_state, sizeof(proximity_state), "%s %u\n",
		  direction, risk);
	spin_unlock_irqrestore(&proximity_lock, flags);

	return count;
}

/* __ATTR(proximity_warning, ...) -> the sysfs file is named
 * "proximity_warning", matching PROXIMITY_WARNING_SYSFS_PATH. */
static struct kobj_attribute proximity_attr =
	__ATTR(proximity_warning, 0660, proximity_warning_show, proximity_warning_store);

static int __init proximity_warning_init(void)
{
	/* Folder: /sys/kernel/pendulum (kernel_kobj = /sys/kernel parent). */
	pendulum_kobj = kobject_create_and_add("pendulum", kernel_kobj);
	if (!pendulum_kobj) {
		pr_err("proximity_warning: cannot create /sys/kernel/pendulum\n");
		return -ENOMEM;
	}

	if (sysfs_create_file(pendulum_kobj, &proximity_attr.attr)) {
		pr_err("proximity_warning: cannot create proximity_warning file\n");
		kobject_put(pendulum_kobj);
		return -ENOMEM;
	}

	pr_info("proximity_warning: loaded, %s ready\n",
		PROXIMITY_WARNING_SYSFS_PATH);
	return 0;
}

static void __exit proximity_warning_exit(void)
{
	sysfs_remove_file(pendulum_kobj, &proximity_attr.attr);
	kobject_put(pendulum_kobj);
	pr_info("proximity_warning: unloaded\n");
}

module_init(proximity_warning_init);
module_exit(proximity_warning_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Sysfs early-warning interface: /sys/kernel/pendulum/proximity_warning");
