/*
 * drivers/input/touchscreen/ft5x0x_ts_v2.c
 *
 * FocalTech ft5x0x TouchScreen driver.
 *
 * Copyright (c) 2010  Focal tech Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * VERSION  DATE        AUTHOR     Note
 * 1.0      2010-01-05  WenFS      only support mulititouch  Wenfs 2010-10-01
 * 2.0      2011-09-05  Duxx       Add touch key, and project setting update,
 *                                 auto CLB command
 * 3.0      2016-07-07  H. Keller  Clean-up, remove inflexible touch keys and
 *                                 old platform data, configure finger count
 *                                 and resolution
 *
 */
#include <linux/module.h>
#include <linux/hrtimer.h>
#include <linux/slab.h>
#include <linux/device.h>

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

/* Device Tree */
#include <linux/of_gpio.h>

#define FT5X0X_DRIVER_NAME	"ft5x06_ts_v2"
#define FT5X0X_DRIVER_VERSION	"3.0"
#define FT5X0X_MAX_FINGERS	10
#define FT5X0X_I2C_ADDRESS	0x70		/* (unused, keep for info) */

#define SCREEN_MAX_X	800
#define SCREEN_MAX_Y	480
#define PRESS_MAX	255

enum ft5x0x_ts_regs {
	FT5X0X_REG_NOREGISTER		= 0xff,
	FT5X0X_REG_THGROUP		= 0x80,	/* Touch threshold,
						   related to sensitivity */
	FT5X0X_REG_THPEAK		= 0x81,
	FT5X0X_REG_THCAL		= 0x82,
	FT5X0X_REG_THWATER		= 0x83,
	FT5X0X_REG_THTEMP		= 0x84,
	FT5X0X_REG_THDIFF		= 0x85,
	FT5X0X_REG_CTRL			= 0x86,
	FT5X0X_REG_TIMEENTERMONITOR	= 0x87,
	FT5X0X_REG_PERIODACTIVE		= 0x88,	/* Report rate */
	FT5X0X_REG_PERIODMONITOR	= 0x89,
	FT5X0X_REG_HEIGHT_B		= 0x8a,
	FT5X0X_REG_MAX_FRAME		= 0x8b,
	FT5X0X_REG_DIST_MOVE		= 0x8c,
	FT5X0X_REG_DIST_POINT		= 0x8d,
	FT5X0X_REG_FEG_FRAME		= 0x8e,
	FT5X0X_REG_SINGLE_CLICK_OFFSET	= 0x8f,
	FT5X0X_REG_DOUBLE_CLICK_TIME_MIN= 0x90,
	FT5X0X_REG_SINGLE_CLICK_TIME	= 0x91,
	FT5X0X_REG_LEFT_RIGHT_OFFSET	= 0x92,
	FT5X0X_REG_UP_DOWN_OFFSET	= 0x93,
	FT5X0X_REG_DISTANCE_LEFT_RIGHT	= 0x94,
	FT5X0X_REG_DISTANCE_UP_DOWN	= 0x95,
	FT5X0X_REG_ZOOM_DIS_SQR		= 0x96,
	FT5X0X_REG_RADIAN_VALUE		= 0x97,
	FT5X0X_REG_MAX_X_HIGH		= 0x98,
	FT5X0X_REG_MAX_X_LOW		= 0x99,
	FT5X0X_REG_MAX_Y_HIGH		= 0x9a,
	FT5X0X_REG_MAX_Y_LOW		= 0x9b,
	FT5X0X_REG_K_X_HIGH		= 0x9c,
	FT5X0X_REG_K_X_LOW		= 0x9d,
	FT5X0X_REG_K_Y_HIGH		= 0x9e,
	FT5X0X_REG_K_Y_LOW		= 0x9f,
	FT5X0X_REG_AUTO_CLB_MODE	= 0xa0,
	FT5X0X_REG_LIB_VERSION_H	= 0xa1,
	FT5X0X_REG_LIB_VERSION_L	= 0xa2,
	FT5X0X_REG_CIPHER		= 0xa3,
	FT5X0X_REG_MODE			= 0xa4,
	FT5X0X_REG_PMODE		= 0xa5,	/* Power Consume Mode */
	FT5X0X_REG_FIRMID		= 0xa6,	/* Firmware version */
	FT5X0X_REG_STATE		= 0xa7,
	FT5X0X_REG_FOCALTECH_ID		= 0xa8,
	FT5X0X_REG_ERR			= 0xa9,
	FT5X0X_REG_CLB			= 0xaa,
};

/* FT5X0X_REG_PMODE */
#define PMODE_ACTIVE	0x00
#define PMODE_MONITOR	0x01
#define PMODE_STANDBY	0x02
#define PMODE_HIBERNATE	0x03


struct ft5x0x_ts_data {
	struct input_dev *input_dev;
	struct i2c_client *client;
	int fingers;			/* Number of supported fingers */
	int reset_pin;			/* GPIO for reset line */
	int wake_pin;			/* GPIO for wake-up line */
	int may_wakeup;			/* Touch may wake us up */
	struct mutex mutex;
	int threshold;
};

/* The defaults should result in a working touch, override with device tree */
static struct ft5x0x_ts_data tsdata_defaults = {
	.fingers = 2,
	.may_wakeup = 0,
	.wake_pin = -1,
	.reset_pin = -1,
	.threshold = -1,
};

/* Read I2C data from touch controller */
static int ft5x0x_i2c_rxdata(struct i2c_client *client, char *rxdata, int len)
{
	struct i2c_msg msgs[] = {
		{
			.addr	= client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= rxdata,
		},
		{
			.addr	= client->addr,
			.flags	= I2C_M_RD,
			.len	= len,
			.buf	= rxdata,
		},
	};

	return i2c_transfer(client->adapter, msgs, 2);
}

/* Write I2C data to touch controller */
static int ft5x0x_i2c_txdata(struct i2c_client *client, char *txdata, int len)
{
	struct i2c_msg msg[] = {
		{
			.addr	= client->addr,
			.flags	= 0,
			.len	= len,
			.buf	= txdata,
		},
	};

	return i2c_transfer(client->adapter, msg, 1);
}

/* Write register of ft5x0x */
static int ft5x0x_write_reg(struct i2c_client *client, u8 addr, u8 val)
{
	u8 buf[2];
	int ret;

	buf[0] = addr;
	buf[1] = val;
	ret = ft5x0x_i2c_txdata(client, buf, 2);
	if (ret < 0) {
		dev_err(&client->dev, "write 0x%x to reg 0x%x failed: %d\n",
			addr, val, ret);
		return ret;
	}

	return 0;
}

/* Read register of ft5x0x */
static int ft5x0x_read_reg(struct i2c_client *client, u8 addr, u8 *pdata)
{
	int ret;

	*pdata = addr;
	ret = ft5x0x_i2c_rxdata(client, pdata, 1);
	if (ret < 0) {
		dev_err(&client->dev, "read reg 0x%x failed: %d\n", addr, ret);
		return ret;
	}

	return 0;
}

struct ft5x0x_ts_attribute {
	struct device_attribute dattr;
	size_t field_offset;
	u8 limit_low;
	u8 limit_high;
	u8 addr_reg;
};

#define EDT_ATTR(_field, _mode, _addr_reg, _limit_low, _limit_high)	\
	struct ft5x0x_ts_attribute ft5x0x_ts_attr_##_field = {		\
		.dattr = __ATTR(_field, _mode,				\
				ft5x0x_ts_setting_show,			\
				ft5x0x_ts_setting_store),		\
		.field_offset = offsetof(struct ft5x0x_ts_data, _field),\
		.addr_reg = _addr_reg,					\
		.limit_low = _limit_low,				\
		.limit_high = _limit_high,				\
	}

static ssize_t ft5x0x_ts_setting_show(struct device *dev,
				       struct device_attribute *dattr,
				       char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ft5x0x_ts_data *tsdata = i2c_get_clientdata(client);
	struct ft5x0x_ts_attribute *attr =
			container_of(dattr, struct ft5x0x_ts_attribute, dattr);
	u8 *field = (u8 *)tsdata + attr->field_offset;
	u8 val;
	int ret = 0;
	u8 addr;

	mutex_lock(&tsdata->mutex);
	addr = attr->addr_reg;
	if (addr != FT5X0X_REG_NOREGISTER) {
		ret = ft5x0x_read_reg(client, addr, &val);
		if (ret < 0) {
			dev_err(&client->dev,
				"Failed to fetch attribute %s, error %d\n",
				dattr->attr.name, ret);
			goto out;
		}
	} else {
		val = *field;
	}

	if (val != *field) {
		dev_warn(&client->dev,
			 "%s: read (%d) and stored value (%d) differ\n",
			 dattr->attr.name, val, *field);
		*field = val;
	}

	ret = scnprintf(buf, PAGE_SIZE, "%d\n", val);
out:
	mutex_unlock(&tsdata->mutex);

	return ret;
}

static ssize_t ft5x0x_ts_setting_store(struct device *dev,
					struct device_attribute *dattr,
					const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ft5x0x_ts_data *tsdata = i2c_get_clientdata(client);
	struct ft5x0x_ts_attribute *attr =
			container_of(dattr, struct ft5x0x_ts_attribute, dattr);
	u8 *field = (u8 *)tsdata + attr->field_offset;
	unsigned int val;
	int ret;
	u8 addr;

	mutex_lock(&tsdata->mutex);
	ret = kstrtouint(buf, 0, &val);
	if (ret)
		goto out;

	if (val < attr->limit_low || val > attr->limit_high) {
		ret = -ERANGE;
		goto out;
	}

	addr = attr->addr_reg;

	if (addr != FT5X0X_REG_NOREGISTER) {
		ret = ft5x0x_write_reg(client, addr, val);
		if (ret < 0) {
			dev_err(&client->dev,
				"Failed to update attribute %s, error: %d\n",
				dattr->attr.name, ret);
			goto out;
		}
	}
	*field = val;
	ret = count;

out:
	mutex_unlock(&tsdata->mutex);
	return ret;
}

static EDT_ATTR(threshold, S_IWUSR | S_IRUGO, FT5X0X_REG_THGROUP, 20, 80);

static struct attribute *ft5x0x_ts_attrs[] = {
	&ft5x0x_ts_attr_threshold.dattr.attr,
	NULL
};

static const struct attribute_group ft5x0x_attr_group = {
	.attrs = ft5x0x_ts_attrs,
};

static irqreturn_t ft5x0x_ts_ist(int irq, void *dev_id)
{
	struct ft5x0x_ts_data *tsdata = dev_id;
	struct input_dev *input_dev = tsdata->input_dev;
	u8 buf[FT5X0X_MAX_FINGERS * 6 + 3];
	u8 *p;
	int i, ret;
	unsigned int touch_points;

	/* FIXME: We should use the mutex here to sync with sysfs accesses */
	buf[0] = 0;
	ret = ft5x0x_i2c_rxdata(tsdata->client, buf, tsdata->fingers * 6 + 3);
	if (ret < 0) {
		dev_err(&tsdata->client->dev, "Error reading touch data: %d\n",
			ret);

		return IRQ_HANDLED;
	}

	touch_points = buf[2] & 0xf;
	if (touch_points > tsdata->fingers)
		touch_points = tsdata->fingers;

	p = &buf[3];
	for (i = 0; i < tsdata->fingers; i++, p += 6) {
		unsigned int finger_id;
		unsigned int touch_event;
		bool down;

		finger_id = p[2] >> 4;
		if (finger_id >= tsdata->fingers)
			continue;

		touch_event = p[0] >> 6;
		down = ((touch_event == 0) || (touch_event == 2));

		input_mt_slot(input_dev, finger_id);
		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, down);
		if (!down)
			continue;

		input_report_abs(input_dev, ABS_MT_POSITION_X,
				 ((p[0] << 8) | p[1]) & 0xFFF);
		input_report_abs(input_dev, ABS_MT_POSITION_Y,
				 ((p[2] << 8) | p[3]) & 0xFFF);
		input_report_abs(input_dev, ABS_MT_WIDTH_MAJOR, 1);
	}
	input_mt_report_pointer_emulation(input_dev, true);
	input_sync(input_dev);

	if (!touch_points) {
		input_report_abs(tsdata->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		input_sync(tsdata->input_dev);
	}

	return IRQ_HANDLED;
}

static void ft5x0x_ts_init(struct ft5x0x_ts_data *tsdata)
{
	struct i2c_client *client = tsdata->client;
	unsigned char fw_id, rate, threshold;
	unsigned char chip_id, ctpm_id;

	if ((tsdata->threshold >= 20) && (tsdata->threshold <= 80))
		ft5x0x_write_reg(client, FT5X0X_REG_THGROUP, tsdata->threshold);

	/* Get some register information */
	ft5x0x_read_reg(client, FT5X0X_REG_FIRMID, &fw_id);
	ft5x0x_read_reg(client, FT5X0X_REG_PERIODACTIVE, &rate);
	ft5x0x_read_reg(client, FT5X0X_REG_THGROUP, &threshold);
	ft5x0x_read_reg(client, FT5X0X_REG_CIPHER, &chip_id);
	ft5x0x_read_reg(client, FT5X0X_REG_FOCALTECH_ID, &ctpm_id);

	dev_info(&client->dev, "Chip ID 0x%x, CTPM ID 0x%x, FW ID 0x%x\n",
		 chip_id, ctpm_id, fw_id);
	dev_info(&client->dev, "Report Rate 0x%x (%dHz), Threshold 0x%x (%d)\n",
		 rate, rate*10, threshold, threshold*4);
}

#ifdef CONFIG_OF
static int ft5x0x_i2c_ts_probe_dt(struct ft5x0x_ts_data *tsdata)
{
	u32 val;
	struct device_node *np = tsdata->client->dev.of_node;

	tsdata->may_wakeup = of_property_read_bool(np, "linux,wakeup");
	tsdata->reset_pin = of_get_named_gpio(np, "reset-gpios", 0);
	tsdata->wake_pin = of_get_named_gpio(np, "wake-gpios", 0);
	if ((of_property_read_u32(np, "fingers", &val) == 0) && (val > 0)) {
		if (val > FT5X0X_MAX_FINGERS)
			val = FT5X0X_MAX_FINGERS;
		tsdata->fingers = val;
	}
	if (of_property_read_u32(np, "threshold", &val) == 0)
		tsdata->threshold = val;

	return 0;
}
#endif


static int
ft5x0x_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ft5x0x_ts_data *tsdata;
	struct input_dev *input_dev;
	int err;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	dev_info(&client->dev, "Driver version " FT5X0X_DRIVER_VERSION "\n");

	input_dev = devm_input_allocate_device(&client->dev);
	if (!input_dev) {
		dev_err(&client->dev, "Failed to allocate input device\n");
		return -ENOMEM;
	}
	input_dev->name	= FT5X0X_DRIVER_NAME;

	tsdata = devm_kmalloc(&client->dev, sizeof(*tsdata), GFP_KERNEL);
	if (!tsdata)	{
		dev_err(&client->dev, "Failed to allocate driver data\n");
		return -ENOMEM;
	}
	*tsdata = tsdata_defaults;
	tsdata->input_dev = input_dev;
	tsdata->client = client;
	mutex_init(&tsdata->mutex);

#ifdef CONFIG_OF
	/* Get platform data from device tree */
	err = ft5x0x_i2c_ts_probe_dt(tsdata);
	if (err) {
		dev_err(&client->dev, "No valid device tree data\n");
		return err;
	}
#endif

	/* Toggle wake pin if given */
	if (gpio_is_valid(tsdata->wake_pin)) {
		devm_gpio_request_one(&client->dev, tsdata->wake_pin,
				      GPIOF_OUT_INIT_LOW, "ft5x0x");
		msleep(5);
		gpio_set_value(tsdata->wake_pin, 1);
	}

	/* Toggle reset line */
	if (gpio_is_valid(tsdata->reset_pin)) {
		devm_gpio_request_one(&client->dev, tsdata->reset_pin,
				      GPIOF_OUT_INIT_LOW, "ft5x0x");
		msleep(5);
		gpio_set_value(tsdata->reset_pin, 1);
		msleep(200);
	}

	msleep(150);  /* Make sure CTP already finish startup process */

	ft5x0x_ts_init(tsdata);

	set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);

	input_set_abs_params(input_dev, ABS_X, 0, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, SCREEN_MAX_Y, 0, 0);

	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
			     0, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
			     0, SCREEN_MAX_Y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, PRESS_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID,
			     0, tsdata->fingers - 1, 0, 0);
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(EV_ABS, input_dev->evbit);
	set_bit(BTN_TOUCH, input_dev->keybit);

	/* Parse touchscreen-size-x, touchscreen-size-y, etc */
	touchscreen_parse_of_params(input_dev);
	dev_info(&client->dev, "Setting resolution to %d x %d\n",
		 input_abs_get_max(input_dev, ABS_MT_POSITION_X),
		 input_abs_get_max(input_dev, ABS_MT_POSITION_Y));

	err = input_mt_init_slots(input_dev, tsdata->fingers, 0);
	if (err) {
		dev_err(&client->dev, "Unable to init MT slots.\n");
		return err;
	}

	i2c_set_clientdata(client, tsdata);
	err = devm_request_threaded_irq(&client->dev, client->irq, NULL,
					ft5x0x_ts_ist,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					client->name, tsdata);
	if (err) {
		dev_err(&client->dev, "Unable to request touchscreen IRQ\n");
		goto exit_unset_i2c_clientdata;
	}

	err = sysfs_create_group(&client->dev.kobj, &ft5x0x_attr_group);
	if (err) {
		dev_err(&client->dev, "Unable to create sysfs group\n");
		goto exit_unset_i2c_clientdata;
	}

	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev, "Failed to register input device\n");
		goto exit_remove_sysfs;
	}

	return 0;

exit_remove_sysfs:
	sysfs_remove_group(&client->dev.kobj, &ft5x0x_attr_group);

exit_unset_i2c_clientdata:
	i2c_set_clientdata(client, NULL);

	return err;
}

static int ft5x0x_ts_remove(struct i2c_client *client)
{
	sysfs_remove_group(&client->dev.kobj, &ft5x0x_attr_group);

	return 0;
}

static int __maybe_unused ft5x0x_ts_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ft5x0x_ts_data *tsdata = i2c_get_clientdata(client);

	if (tsdata->may_wakeup) {
		/* Touch panel may be used as wakeup event */
		if (device_may_wakeup(dev))
			enable_irq_wake(client->irq);
	} else {
		disable_irq(client->irq);

		/* Send touch panel to hibernation mode, but only if we can
		   wake it up later again, i.e. if wake_pin is valid. */
		if (gpio_is_valid(tsdata->wake_pin))
			ft5x0x_write_reg(client, FT5X0X_REG_PMODE,
					 PMODE_HIBERNATE);
	}

	return 0;
}

static int __maybe_unused ft5x0x_ts_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ft5x0x_ts_data *tsdata = i2c_get_clientdata(client);

	if (tsdata->may_wakeup) {
		if (device_may_wakeup(dev))
			disable_irq_wake(client->irq);
	} else {
		/* Wake up touch panel if it was in hibernation mode */
		if (gpio_is_valid(tsdata->wake_pin)) {
			gpio_set_value(tsdata->wake_pin, 0);
			msleep(100);
			gpio_set_value(tsdata->wake_pin, 1);
			msleep(100);
		}

		enable_irq(tsdata->client->irq);
	}

	return 0;
}
static SIMPLE_DEV_PM_OPS(ft5x0x_ts_pm_ops,
			 ft5x0x_ts_suspend, ft5x0x_ts_resume);

static const struct i2c_device_id ft5x0x_ts_id[] = {
	{ FT5X0X_DRIVER_NAME, 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, ft5x0x_ts_id);

#ifdef CONFIG_OF
static const struct of_device_id ft5x0x_of_match[] = {
	{ .compatible = "FocalTech,ft5206", },
	{ .compatible = "FocalTech,ft5306", },
	{ .compatible = "FocalTech,ft5406", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ft5x0x_of_match);
#endif

static struct i2c_driver ft5x0x_ts_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = FT5X0X_DRIVER_NAME,
		.of_match_table = of_match_ptr(ft5x0x_of_match),
		.pm = &ft5x0x_ts_pm_ops,
	},
	.id_table = ft5x0x_ts_id,
	.probe    = ft5x0x_ts_probe,
	.remove   = ft5x0x_ts_remove,
};

module_i2c_driver(ft5x0x_ts_driver);


MODULE_AUTHOR("<wenfs@Focaltech-systems.com>");
MODULE_DESCRIPTION("FocalTech ft5x0x TouchScreen driver");
MODULE_LICENSE("GPL");

