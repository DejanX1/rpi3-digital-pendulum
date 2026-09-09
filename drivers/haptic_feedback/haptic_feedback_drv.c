// SPDX-License-Identifier: GPL-2.0
/*
 * haptic_feedback_drv.c - direct-register GPIO driver for /dev/haptic_feedback
 *
 * Drives an external buzzer/vibration motor wired to one BCM2837 GPIO pin
 * on a Raspberry Pi 3B+. No gpiolib/pinctrl API is used: the project spec
 * requires the driver to map and toggle the SoC's GPIO registers directly
 *
 * The wire format (struct haptic_pulse, ioctl numbers) is the contract
 * agreed in include/haptic_feedback.h
 *
 * GPIO pin
 * This driver defaults to GPIO18 (header pin 12) and exposes it as a module
 * parameter so it can be changed at load time without a rebuild:
 *
 *   insmod haptic_feedback_drv.ko gpio_pin=17
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/spinlock.h>
#include <linux/math64.h>
#include <linux/errno.h>
#include <linux/moduleparam.h>

#include "haptic_feedback.h"

/* --- BCM2837 GPIO controller: physical address and register offsets --- */

#define BCM2837_PERI_BASE   0x3F000000UL
#define BCM2837_GPIO_BASE   (BCM2837_PERI_BASE + 0x200000UL)
#define BCM2837_GPIO_SIZE   PAGE_SIZE

#define GPFSEL0  0x00  /* function select, 6 registers x 10 pins x 3 bits */
#define GPSET0   0x1C  /* output set, pins 0-31 */
#define GPSET1   0x20  /* output set, pins 32-53 */
#define GPCLR0   0x28  /* output clear, pins 0-31 */
#define GPCLR1   0x2C  /* output clear, pins 32-53 */

#define GPIO_FSEL_OUTPUT 0x1
#define GPIO_MAX_PIN     53

static int gpio_pin = 18;
module_param(gpio_pin, int, 0444);
MODULE_PARM_DESC(gpio_pin, "BCM GPIO number driving the buzzer (default 18)");

struct haptic_dev {
	void __iomem *gpio_base;
	int gpio_pin;

	struct hrtimer timer;
	spinlock_t lock;        /* protects the fields below vs. the timer callback */
	bool pin_state;         /* logical level currently driven on gpio_pin */
	bool busy;              /* pulse in progress */
	ktime_t period_on;
	ktime_t period_off;
	ktime_t end_time;       /* ktime_get() deadline for the current pulse */
};

static struct haptic_dev haptic_dev;

/* --- direct register access, no gpiolib --- */

static void bcm2837_gpio_set_output(struct haptic_dev *hd)
{
	unsigned int reg = hd->gpio_pin / 10;
	unsigned int shift = (hd->gpio_pin % 10) * 3;
	void __iomem *addr = hd->gpio_base + GPFSEL0 + reg * 4;
	u32 val = readl(addr);

	val &= ~(0x7u << shift);
	val |= (GPIO_FSEL_OUTPUT << shift);
	writel(val, addr);
}

static inline void bcm2837_gpio_write(struct haptic_dev *hd, bool level)
{
	unsigned int bank = hd->gpio_pin / 32;
	unsigned int shift = hd->gpio_pin % 32;
	unsigned int off = level ? (bank ? GPSET1 : GPSET0)
				 : (bank ? GPCLR1 : GPCLR0);

	writel(1u << shift, hd->gpio_base + off);
}

/* --- hrtimer-driven square wave ---
 *
 * One pulse = a square wave at `frequency_hz`, `duration_ms` long, whose
 * duty cycle is `force_percent`. The timer fires once per half-period,
 * toggling the pin and re-arming itself for the other half; when the
 * pulse's total duration has elapsed it drives the pin low and stops.
 */

static enum hrtimer_restart haptic_timer_cb(struct hrtimer *timer)
{
	struct haptic_dev *hd = container_of(timer, struct haptic_dev, timer);
	unsigned long flags;
	bool next_state;
	ktime_t next_interval;

	spin_lock_irqsave(&hd->lock, flags);

	if (!hd->busy || ktime_after(ktime_get(), hd->end_time)) {
		hd->busy = false;
		hd->pin_state = false;
		spin_unlock_irqrestore(&hd->lock, flags);
		bcm2837_gpio_write(hd, false);
		return HRTIMER_NORESTART;
	}

	hd->pin_state = !hd->pin_state;
	next_state = hd->pin_state;
	next_interval = next_state ? hd->period_on : hd->period_off;

	spin_unlock_irqrestore(&hd->lock, flags);

	bcm2837_gpio_write(hd, next_state);
	hrtimer_forward_now(timer, next_interval);
	return HRTIMER_RESTART;
}

static int haptic_start_pulse(struct haptic_dev *hd, const struct haptic_pulse *p)
{
	unsigned long flags;
	u64 period_ns, on_ns;

	if (p->force_percent > 100 || p->frequency_hz < 200 || p->frequency_hz > 4000 ||
	    p->duration_ms < 1 || p->duration_ms > 1000)
		return -EINVAL;

	/* Cancel any pulse already in flight before starting the new one.
	 * Called with the lock NOT held: hrtimer_cancel() waits for a
	 * running callback to finish, and that callback also takes hd->lock,
	 * so holding it here would deadlock against a callback in flight on
	 * another CPU. */
	hrtimer_cancel(&hd->timer);

	period_ns = div_u64(NSEC_PER_SEC, p->frequency_hz);
	on_ns = div_u64(period_ns * p->force_percent, 100);

	spin_lock_irqsave(&hd->lock, flags);
	hd->period_on = ns_to_ktime(on_ns);
	hd->period_off = ns_to_ktime(period_ns - on_ns);
	hd->end_time = ktime_add_ms(ktime_get(), p->duration_ms);
	hd->pin_state = true;
	hd->busy = true;
	spin_unlock_irqrestore(&hd->lock, flags);

	bcm2837_gpio_write(hd, true);
	hrtimer_start(&hd->timer, hd->period_on, HRTIMER_MODE_REL);

	return 0;
}

static void haptic_stop(struct haptic_dev *hd)
{
	unsigned long flags;

	hrtimer_cancel(&hd->timer);
	spin_lock_irqsave(&hd->lock, flags);
	hd->busy = false;
	hd->pin_state = false;
	spin_unlock_irqrestore(&hd->lock, flags);
	bcm2837_gpio_write(hd, false);
}

static u8 haptic_get_status(struct haptic_dev *hd)
{
	unsigned long flags;
	u8 status;

	spin_lock_irqsave(&hd->lock, flags);
	status = hd->busy ? 1 : 0;
	spin_unlock_irqrestore(&hd->lock, flags);
	return status;
}

/* --- file_operations: write() and ioctl() both accept struct haptic_pulse,
 * per the agreed wire format in include/haptic_feedback.h --- */

static int haptic_open(struct inode *inode, struct file *file)
{
	pr_info("haptic_feedback: device opened\n");
	return 0;
}

static int haptic_release(struct inode *inode, struct file *file)
{
	pr_info("haptic_feedback: device closed\n");
	return 0;
}

static ssize_t haptic_write(struct file *file, const char __user *buf,
			     size_t count, loff_t *ppos)
{
	struct haptic_pulse p;
	int ret;

	if (count != sizeof(p))
		return -EINVAL;
	if (copy_from_user(&p, buf, sizeof(p)))
		return -EFAULT;

	ret = haptic_start_pulse(&haptic_dev, &p);
	if (ret)
		return ret;

	return count;
}

static long haptic_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct haptic_pulse p;
	u8 status;

	switch (cmd) {
	case HAPTIC_IOC_PULSE:
		if (copy_from_user(&p, (void __user *)arg, sizeof(p)))
			return -EFAULT;
		return haptic_start_pulse(&haptic_dev, &p);

	case HAPTIC_IOC_STOP:
		haptic_stop(&haptic_dev);
		return 0;

	case HAPTIC_IOC_GET_STATUS:
		status = haptic_get_status(&haptic_dev);
		if (copy_to_user((void __user *)arg, &status, sizeof(status)))
			return -EFAULT;
		return 0;

	default:
		return -ENOTTY;
	}
}

static struct file_operations haptic_fops = {
	.owner          = THIS_MODULE,
	.open           = haptic_open,
	.release        = haptic_release,
	.write          = haptic_write,
	.unlocked_ioctl = haptic_ioctl,
};

/* --- chardev plumbing: major/minor, cdev, class+device (-> /dev/haptic_feedback) --- */

static dev_t haptic_devnum;
static struct class *haptic_class;
static struct cdev haptic_cdev;

/* --- module init/exit --- */

static int __init haptic_init(void)
{
	if (gpio_pin < 0 || gpio_pin > GPIO_MAX_PIN) {
		pr_err("haptic_feedback: invalid gpio_pin=%d\n", gpio_pin);
		return -EINVAL;
	}
	haptic_dev.gpio_pin = gpio_pin;

	haptic_dev.gpio_base = ioremap(BCM2837_GPIO_BASE, BCM2837_GPIO_SIZE);
	if (!haptic_dev.gpio_base) {
		pr_err("haptic_feedback: failed to map GPIO registers at 0x%lx\n",
		       BCM2837_GPIO_BASE);
		return -ENOMEM;
	}

	spin_lock_init(&haptic_dev.lock);
	hrtimer_setup(&haptic_dev.timer, haptic_timer_cb, CLOCK_MONOTONIC,
		      HRTIMER_MODE_REL);

	bcm2837_gpio_set_output(&haptic_dev);
	bcm2837_gpio_write(&haptic_dev, false);

	/* Allocating Major number */
	if (alloc_chrdev_region(&haptic_devnum, 0, 1, "haptic_feedback") < 0) {
		pr_err("haptic_feedback: cannot allocate major number\n");
		goto r_unmap;
	}
	pr_info("haptic_feedback: major=%d minor=%d\n",
		MAJOR(haptic_devnum), MINOR(haptic_devnum));

	/* Creating cdev structure */
	cdev_init(&haptic_cdev, &haptic_fops);
	haptic_cdev.owner = THIS_MODULE;

	/* Adding character device to the system */
	if (cdev_add(&haptic_cdev, haptic_devnum, 1) < 0) {
		pr_err("haptic_feedback: cannot add the device to the system\n");
		goto r_chrdev;
	}

	/* Creating struct class */
	haptic_class = class_create("haptic_feedback_class");
	if (IS_ERR(haptic_class)) {
		pr_err("haptic_feedback: cannot create the struct class\n");
		goto r_cdev;
	}

	/* Creating device -> /dev/haptic_feedback */
	if (IS_ERR(device_create(haptic_class, NULL, haptic_devnum, NULL, "haptic_feedback"))) {
		pr_err("haptic_feedback: cannot create the device\n");
		goto r_class;
	}

	pr_info("haptic_feedback: loaded, driving GPIO%d -> /dev/haptic_feedback\n",
		haptic_dev.gpio_pin);
	return 0;

r_class:
	class_destroy(haptic_class);
r_cdev:
	cdev_del(&haptic_cdev);
r_chrdev:
	unregister_chrdev_region(haptic_devnum, 1);
r_unmap:
	iounmap(haptic_dev.gpio_base);
	return -1;
}

static void __exit haptic_exit(void)
{
	hrtimer_cancel(&haptic_dev.timer);
	bcm2837_gpio_write(&haptic_dev, false);

	device_destroy(haptic_class, haptic_devnum);
	class_destroy(haptic_class);
	cdev_del(&haptic_cdev);
	unregister_chrdev_region(haptic_devnum, 1);

	iounmap(haptic_dev.gpio_base);
	pr_info("haptic_feedback: unloaded\n");
}

module_init(haptic_init);
module_exit(haptic_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Direct-register GPIO buzzer driver for /dev/haptic_feedback");
