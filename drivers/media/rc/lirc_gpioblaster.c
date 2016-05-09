/*
 * lirc_gpioblaster.c
 *
 * lirc_gpioblaster  Modified version (receive removed) from the raspberry pi:
 *                   taken from https://github.com/DaveDavenport/lirc-gpio
 *
 * Copyright (C) 2014 Qball Cow <qball@gmpclient.org>,
 * Copyright (C) 2012 Aron Robert Szabo <aron@reon.hu>,
 *		      Michael Bishop <cleverca22@gmail.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/time.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/spinlock.h>
#include <linux/gpio.h>
#include <media/lirc.h>
#include <media/lirc_dev.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>

#define LIRC_DRIVER_NAME "lirc_gpioblaster"
#define RBUF_LEN 256
#define LIRC_TRANSMITTER_LATENCY 50

#ifndef MAX_UDELAY_MS
#define MAX_UDELAY_US 5000
#else
#define MAX_UDELAY_US (MAX_UDELAY_MS*1000)
#endif

#define dprintk(fmt, args...)					\
	do {							\
		if (debug)					\
			printk(KERN_DEBUG LIRC_DRIVER_NAME ": "	\
			       fmt, ## args);			\
	} while (0)

/* module parameters */

struct lirc_gpioblaster {
	int gpio_nr;
	bool active_low;
	bool softcarrier;
};

struct lirc_gpioblaster lrc_gpio;

/* enable debugging messages */
static bool debug;

/* forward declarations */
static long send_pulse(unsigned long length);
static void send_space(long length);
static void lirc_gpioblaster_exit(void);

static int lirc_gpioblaster_probe(struct platform_device *pdev);
static int lirc_gpioblaster_remove(struct platform_device *pdev);

static struct lirc_buffer rbuf;
static spinlock_t lock;

/* initialized/set in init_timing_params() */
static unsigned int freq = 38000;
static unsigned int duty_cycle = 50;
static unsigned long period;
static unsigned long pulse_width;
static unsigned long space_width;

static void safe_udelay(unsigned long usecs)
{
	while (usecs > MAX_UDELAY_US) {
		udelay(MAX_UDELAY_US);
		usecs -= MAX_UDELAY_US;
	}
	udelay(usecs);
}

static unsigned long read_current_us(void)
{
	struct timespec now;
	getnstimeofday(&now);

	return (now.tv_sec * 1000000) + (now.tv_nsec/1000);
}

static int init_timing_params(unsigned int new_duty_cycle,
	unsigned int new_freq)
{
	if (1000 * 1000000L / new_freq * new_duty_cycle / 100 <=
	    LIRC_TRANSMITTER_LATENCY)
		return -EINVAL;

	if (1000 * 1000000L / new_freq * (100 - new_duty_cycle) / 100 <=
	    LIRC_TRANSMITTER_LATENCY)
		return -EINVAL;

	duty_cycle = new_duty_cycle;
	freq = new_freq;
	period = 1000 * 1000000L / freq;
	pulse_width = period * duty_cycle / 100;
	space_width = period - pulse_width;
	dprintk("in init_timing_params, freq=%d pulse=%ld, "
		"space=%ld\n", freq, pulse_width, space_width);

	return 0;
}

static long send_pulse_softcarrier(unsigned long length)
{
	int flag;
	unsigned long actual, target;
	unsigned long actual_us, initial_us, target_us;

	length *= 1000;
	actual = 0; target = 0; flag = lrc_gpio.active_low;
	actual_us = read_current_us();

	while (actual < length) {
		if (flag) {
			gpio_set_value(lrc_gpio.gpio_nr, 0);
			target += space_width;
		}
		else {
			gpio_set_value(lrc_gpio.gpio_nr, 1);
			target += pulse_width;
		}
		initial_us = actual_us;
		target_us = actual_us + (target - actual) / 1000;
		/*
		 * Note - we've checked in ioctl that the pulse/space
		 * widths are big enough so that d is > 0
		 */
		if((int)(target_us - actual_us) > 0)
			udelay(target_us - actual_us);

		actual_us = read_current_us();
		actual += (actual_us - initial_us) * 1000;
		flag = !flag;
	}

	return (actual-length) / 1000;
}

static long send_pulse(unsigned long length)
{
	if (length <= 0)
		return 0;

	if (lrc_gpio.softcarrier) {
		return send_pulse_softcarrier(length);
	}
	else {
		gpio_set_value(lrc_gpio.gpio_nr, !lrc_gpio.active_low);
		safe_udelay(length);
		return 0;
	}
}

static void send_space(long length)
{
	gpio_set_value(lrc_gpio.gpio_nr, lrc_gpio.active_low);
	if (length <= 0)
		return;

	safe_udelay(length);
}

static int init_port(struct platform_device *pdev)
{
	int ret;
	int err = gpio_request(lrc_gpio.gpio_nr ,"gpio_remoteIRQ");

	if (!gpio_is_valid(lrc_gpio.gpio_nr)) {
		dev_err(&pdev->dev,
			"gpio_remote module: requested GPIO is not valid\n");
		return -1;
	}

	if(err) {
		dev_err(&pdev->dev,
			"gpio_remote module: failed to request GPIO %i\n",
							lrc_gpio.gpio_nr);
		goto exit_init_port;
	}

	err = gpio_direction_output(lrc_gpio.gpio_nr, lrc_gpio.active_low);

	if(err) {
		dev_err(&pdev->dev,
			"gpio_remote module: failed to set GPIO to ouput\n");
		ret = -ENODEV;
		goto exit_gpio_free_out_pin;
	}

	gpio_set_value(lrc_gpio.gpio_nr, lrc_gpio.active_low);

	return 0;

exit_gpio_free_out_pin:
	gpio_free(lrc_gpio.gpio_nr);
exit_init_port:
	return ret;
}

static int set_use_inc(void *data)
{
	return 0;
}

static void set_use_dec(void *data)
{
}

static ssize_t lirc_write(struct file *file, const char *buf,
	size_t n, loff_t *ppos)
{
	int i, count;
	unsigned long flags;
	long delta = 0;
	int *wbuf;

	count = n / sizeof(int);
	if (n % sizeof(int) || count % 2 == 0)
		return -EINVAL;

	wbuf = memdup_user(buf, n);
	if (IS_ERR(wbuf))
		return PTR_ERR(wbuf);

	spin_lock_irqsave(&lock, flags);

	for (i = 0; i < count; i++) {
		if (i%2)
			send_space(wbuf[i] - delta);
		else
			delta = send_pulse(wbuf[i]);
	}
	gpio_set_value(lrc_gpio.gpio_nr, lrc_gpio.active_low);
	spin_unlock_irqrestore(&lock, flags);
	kfree(wbuf);

	return n;
}

static long lirc_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
{
	int result;
	__u32 value;

	switch (cmd) {
	case LIRC_GET_SEND_MODE:
		return -ENOIOCTLCMD;
		break;
	case LIRC_SET_SEND_MODE:
		result = get_user(value, (__u32 *) arg);
		if (result)
			return result;
		/* only LIRC_MODE_PULSE supported */
		if (value != LIRC_MODE_PULSE)
			return -ENOSYS;
		break;
	case LIRC_GET_LENGTH:
		return -ENOSYS;
		break;
	case LIRC_SET_SEND_DUTY_CYCLE:
		dprintk("SET_SEND_DUTY_CYCLE\n");
		result = get_user(value, (__u32 *) arg);
		if (result)
			return result;
		if (value <= 0 || value > 100)
			return -EINVAL;
		return init_timing_params(value, freq);
		break;
	case LIRC_SET_SEND_CARRIER:
		dprintk("SET_SEND_CARRIER\n");
		result = get_user(value, (__u32 *) arg);
		if (result)
			return result;
		if (value > 500000 || value < 20000)
			return -EINVAL;
		return init_timing_params(duty_cycle, value);
		break;
	default:
		return lirc_dev_fop_ioctl(filep, cmd, arg);
	}
	return 0;
}

static const struct file_operations lirc_fops = {
	.owner		= THIS_MODULE,
	.write		= lirc_write,
	.unlocked_ioctl	= lirc_ioctl,
	.read		= lirc_dev_fop_read,
	.poll		= lirc_dev_fop_poll,
	.open		= lirc_dev_fop_open,
	.release	= lirc_dev_fop_close,
	.llseek		= no_llseek,
};

static struct lirc_driver driver = {
	.name		= LIRC_DRIVER_NAME,
	.minor		= -1,
	.code_length	= 1,
	.sample_rate	= 0,
	.data		= NULL,
	.add_to_buf	= NULL,
	.rbuf		= &rbuf,
	.set_use_inc	= set_use_inc,
	.set_use_dec	= set_use_dec,
	.fops		= &lirc_fops,
	.dev		= NULL,
	.owner		= THIS_MODULE,
};

static struct of_device_id lirc_gpioblaster_of_match[] = {
	{ .compatible = "lirc-gpioblaster", },
	{ },
};
MODULE_DEVICE_TABLE(of, lirc_gpioblaster_of_match);

static struct platform_driver lirc_gpioblaster_driver = {
	.probe  = lirc_gpioblaster_probe,
	.remove = lirc_gpioblaster_remove,
	.driver = {
		.name   = LIRC_DRIVER_NAME,
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(lirc_gpioblaster_of_match),
	},
};


static int lirc_gpioblaster_dt_init(struct device *dev)
{
	struct device_node *np = dev->of_node;
	enum of_gpio_flags flags;
	int gpio;

	gpio = of_get_gpio_flags(np, 0, &flags);
	if (gpio < 0) {
		if (gpio != -EPROBE_DEFER)
			dev_err(dev, "Failed to get gpio flags (%d)\n", gpio);
		return gpio;
	}

	lrc_gpio.gpio_nr = gpio;
	lrc_gpio.active_low = (flags & OF_GPIO_ACTIVE_LOW);
	lrc_gpio.softcarrier = of_property_read_bool(np, "softcarrier");

	return 0;
}

static void lirc_gpioblaster_exit(void)
{
	lirc_buffer_free(&rbuf);
}

static int lirc_gpioblaster_probe(struct platform_device *pdev)
{
	int result;

	/* need initialization before sending */
	result = lirc_buffer_init(&rbuf, sizeof(int), RBUF_LEN);
	if (result < 0)
		return -ENOMEM;

	if (pdev->dev.of_node)
		result = lirc_gpioblaster_dt_init(&pdev->dev);
	else
		goto exit_gpioblaster;

	result = init_port(pdev);
	if (result < 0)
		goto exit_gpioblaster;

	driver.features = LIRC_CAN_SET_SEND_DUTY_CYCLE |
			  LIRC_CAN_SET_SEND_CARRIER |
			  LIRC_CAN_SEND_PULSE;

	driver.dev = &pdev->dev;
	driver.minor = lirc_register_driver(&driver);

	if (driver.minor < 0) {
		dev_err(&pdev->dev, "device registration failed with %d\n",
								result);
		result = -EIO;
		goto exit_gpioblaster;
	}

	return 0;

exit_gpioblaster:
	lirc_gpioblaster_exit();

	return result;
}

static int lirc_gpioblaster_remove(struct platform_device *pdev)
{
	lirc_unregister_driver(driver.minor);

	gpio_free(lrc_gpio.gpio_nr);

	lirc_gpioblaster_exit();

	dev_info(&pdev->dev,"cleaned up module\n");

	return 0;
}

module_platform_driver(lirc_gpioblaster_driver);

MODULE_DESCRIPTION("Infra-red blaster driver for GPIO.");
MODULE_AUTHOR("Qball Cow <qball@gmpclient.org>");
MODULE_AUTHOR("Aron Robert Szabo <aron@reon.hu>");
MODULE_AUTHOR("Michael Bishop <cleverca22@gmail.com>");
MODULE_LICENSE("GPL");
