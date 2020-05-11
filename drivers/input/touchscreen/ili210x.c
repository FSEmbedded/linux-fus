#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/input/ili210x.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#define MAX_TOUCHES		2
#define DEFAULT_POLL_PERIOD	20

/* Touchscreen commands */
#define REG_TOUCHDATA		0x10
#define REG_TOUCH_REPORT	0x11
#define REG_PANEL_INFO		0x20
#define REG_FIRMWARE_VERSION	0x40
#define REG_CALIBRATE		0xcc
#define REG_PROTOCOL_VERSION	0x42

/* Status bits */
#define STATUS_ID_MASK		0x3F
#define STATUS_KEY_OR_SB	0x40
#define STATUS_TOUCH		0x80

struct finger {
	u8 x_low;
	u8 x_high;
	u8 y_low;
	u8 y_high;
} __packed;

struct touchdata_v1 {
	u8 status;
	struct finger finger[MAX_TOUCHES];
} __packed;

struct touchdata_v2 {
	u8 status;
	u8 x_high;
	u8 x_low;
	u8 y_high;
	u8 y_low;
} __packed;

struct keydata_v2 {
	u8 status;
	u8 key_id;
	u8 report_type;
	u8 reserved1;
	u8 reserved2;
} __packed;

struct sbdata_v2 {
	u8 status;
	u8 sb_id;
	u8 report_type;
	u8 sb_low;
	u8 sb_high;
} __packed;

union reportdata_v2 {
	struct touchdata_v2 touch;
	struct keydata_v2 key;
	struct sbdata_v2 sb;
} __packed;

struct panel_info_v1 {
	struct finger finger_max;
	u8 xchannel_num;
	u8 ychannel_num;
} __packed;

struct panel_info_v2 {
	struct panel_info_v1 v1;
	u8 max_touches;
	u8 extra_channel_num;
	u8 sb_low;			/* Number of keys if sb_high == 0xFF */
	u8 sb_high;
} __packed;

struct firmware_version {
	u8 id;
	u8 major;
	u8 minor;
} __packed;

struct protocol_version {
	u8 major;
	u8 minor;
} __packed;

struct ili210x {
	struct i2c_client *client;
	struct input_dev *input;
	bool (*get_pendown_state)(void);
	unsigned int poll_period;
	struct delayed_work dwork;
	unsigned int max_touches;
};

static int ili210x_read_reg(struct i2c_client *client, u8 reg, void *buf,
			    size_t len)
{
	struct i2c_msg msg[2] = {
		{
			.addr	= client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= &reg,
		},
		{
			.addr	= client->addr,
			.flags	= I2C_M_RD,
			.len	= len,
			.buf	= buf,
		}
	};

	if (i2c_transfer(client->adapter, msg, 2) != 2) {
		dev_err(&client->dev, "i2c transfer failed\n");
		return -EIO;
	}

	return 0;
}

/* --------------- Protocol V1.x ------------------------------------------- */

static void ili210x_report_events_v1(struct input_dev *input,
				     const struct touchdata_v1 *touchdata)
{
	int i;
	bool touch;
	unsigned int x, y;
	const struct finger *finger;

	for (i = 0; i < MAX_TOUCHES; i++) {
		input_mt_slot(input, i);

		finger = &touchdata->finger[i];

		touch = touchdata->status & (1 << i);
		input_mt_report_slot_state(input, MT_TOOL_FINGER, touch);
		if (touch) {
			x = finger->x_low | (finger->x_high << 8);
			y = finger->y_low | (finger->y_high << 8);

			input_report_abs(input, ABS_MT_POSITION_X, x);
			input_report_abs(input, ABS_MT_POSITION_Y, y);
		}
	}

	input_mt_report_pointer_emulation(input, false);
	input_sync(input);
}

static bool get_pendown_state(const struct ili210x *priv)
{
	bool state = false;

	if (priv->get_pendown_state)
		state = priv->get_pendown_state();

	return state;
}

static void ili210x_work_v1(struct work_struct *work)
{
	struct ili210x *priv = container_of(work, struct ili210x,
					    dwork.work);
	struct i2c_client *client = priv->client;
	struct touchdata_v1 touchdata;
	int error;

	error = ili210x_read_reg(client, REG_TOUCHDATA,
				 &touchdata, sizeof(touchdata));
	if (error) {
		dev_err(&client->dev,
			"Unable to get touchdata, err = %d\n", error);
		return;
	}

	ili210x_report_events_v1(priv->input, &touchdata);

	if ((touchdata.status & 0xf3) || get_pendown_state(priv))
		schedule_delayed_work(&priv->dwork,
				      msecs_to_jiffies(priv->poll_period));
}

/* --------------- Protocol V2.x ------------------------------------------- */

static void ili210x_touch_event_v2(const struct ili210x *priv,
				   const struct touchdata_v2 *touchdata)
{
	struct input_dev *input = priv->input;
	int touch = (touchdata->status & STATUS_TOUCH) != 0;

	input_mt_slot(input, touchdata->status & STATUS_ID_MASK);
	input_mt_report_slot_state(input, MT_TOOL_FINGER, touch);
	if (touch) {
		int x = (touchdata->x_high << 8) | touchdata->x_low;
		int y = (touchdata->y_high << 8) | touchdata->y_low;

		input_report_abs(input, ABS_MT_POSITION_X, x);
		input_report_abs(input, ABS_MT_POSITION_Y, y);
	}
}

static void ili210x_key_event_v2(const struct ili210x *priv,
				 const struct keydata_v2 *keydata)
{
	struct i2c_client *client = priv->client;

	dev_err(&client->dev, "### TODO: Key event %d\n", keydata->key_id);
}

static void ili210x_sb_event_v2(const struct ili210x *priv,
				const struct sbdata_v2 *sbdata)
{
	struct i2c_client *client = priv->client;

	dev_err(&client->dev, "### TODO: Scrollbar event %d, value %d\n",
		sbdata->sb_id, (sbdata->sb_high << 8) | sbdata->sb_low);
}

static void ili210x_work_v2(struct work_struct *work)
{
	struct ili210x *priv = container_of(work, struct ili210x,
					    dwork.work);
	struct i2c_client *client = priv->client;
	struct input_dev *input = priv->input;
	union reportdata_v2 touch_report;
	u8 touch_count;
	int error;

	error = ili210x_read_reg(client, REG_TOUCHDATA,
				 &touch_count, sizeof(touch_count));
	if (error) {
		dev_err(&client->dev,
			"Unable to get touch count, err = %d\n", error);
		return;
	}

	if (!touch_count)
		return;

	do {
		error = ili210x_read_reg(client, REG_TOUCH_REPORT,
				 &touch_report, sizeof(touch_report));
		if (error) {
			dev_err(&client->dev,
				"Unable to get report, err = %d\n", error);
			return;
		}
		if (!(touch_report.touch.status & STATUS_KEY_OR_SB))
			ili210x_touch_event_v2(priv, &touch_report.touch);
		else if (touch_report.key.report_type == 0)
			ili210x_key_event_v2(priv, &touch_report.key);
		else
			ili210x_sb_event_v2(priv, &touch_report.sb);
	} while (--touch_count);

	input_mt_report_pointer_emulation(input, false);
	input_sync(input);
}


/* --------------- Common Code --------------------------------------------- */

static irqreturn_t ili210x_irq(int irq, void *irq_data)
{
	struct ili210x *priv = irq_data;

	schedule_delayed_work(&priv->dwork, 0);

	return IRQ_HANDLED;
}

static ssize_t ili210x_calibrate(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ili210x *priv = i2c_get_clientdata(client);
	unsigned long calibrate;
	int rc;
	u8 cmd = REG_CALIBRATE;

	if (kstrtoul(buf, 10, &calibrate))
		return -EINVAL;

	if (calibrate > 1)
		return -EINVAL;

	if (calibrate) {
		rc = i2c_master_send(priv->client, &cmd, sizeof(cmd));
		if (rc != sizeof(cmd))
			return -EIO;
	}

	return count;
}
static DEVICE_ATTR(calibrate, 0644, NULL, ili210x_calibrate);

static struct attribute *ili210x_attributes[] = {
	&dev_attr_calibrate.attr,
	NULL,
};

static const struct attribute_group ili210x_attr_group = {
	.attrs = ili210x_attributes,
};

static struct ili210x_platform_data default_pdata = {
	.irq_flags = IRQF_TRIGGER_FALLING,
	.poll_period = DEFAULT_POLL_PERIOD,
	.get_pendown_state = NULL,
	.reset_gpio = -1,
};

#ifdef CONFIG_OF
static int ili210x_probe_dt(struct i2c_client *client,
			    struct ili210x_platform_data *pdata)
{
	struct device_node *np = client->dev.of_node;

	pdata->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);
	of_property_read_u32(np, "poll-period", &pdata->poll_period);

	return 0;
}
#endif

static int ili210x_i2c_probe(struct i2c_client *client,
				       const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct ili210x_platform_data *pdata = dev_get_platdata(dev);
	struct ili210x *priv;
	struct input_dev *input;
	struct panel_info_v2 panel;
	struct firmware_version firmware;
	struct protocol_version protocol;
	int xmax, ymax;
	int error;
	int len;

	dev_dbg(dev, "Probing for ILI210X I2C Touschreen driver");

	if (!pdata) {
		pdata = &default_pdata;
#ifdef CONFIG_OF
		/* Get platform data from device tree */
		ili210x_probe_dt(client, pdata);
#else
		dev_info(dev, "No platform data, using defaults\n");
#endif
	}

	if (client->irq <= 0) {
		dev_err(dev, "No IRQ!\n");
		return -EINVAL;
	}

	/*
	 * Issue reset to chip; the ILI2116 needs reset to be asserted for at
	 * least 50us and it needs 100ms delay after reset before I2C commands
	 * can be issued.
	 */
	if (gpio_is_valid(pdata->reset_gpio)) {
		devm_gpio_request_one(&client->dev, pdata->reset_gpio,
				      GPIOF_OUT_INIT_LOW, "ili210x");
		msleep(1);
		gpio_set_value(pdata->reset_gpio, 1);
		msleep(100);
	}

	/* Get firmware version */
	error = ili210x_read_reg(client, REG_FIRMWARE_VERSION,
				 &firmware, sizeof(firmware));
	if (error) {
		dev_err(dev, "Failed to get firmware version, err: %d\n",
			error);
		return error;
	}

	/* Get protocol version */
	error = ili210x_read_reg(client, REG_PROTOCOL_VERSION, &protocol,
				 sizeof(protocol));
	if (error) {
		dev_err(dev, "Failed to get protocol version, err: %d\n",
			error);
		return error;
	}

	/* Get panel info; in protocol version 2.x we have more info */
	if (protocol.major == 1)
		len = sizeof(struct panel_info_v1);
	else
		len = sizeof(struct panel_info_v2);
	error = ili210x_read_reg(client, REG_PANEL_INFO, &panel, len);
	if (error) {
		dev_err(dev, "Failed to get panel informations, err: %d\n",
			error);
		return error;
	}

	xmax = panel.v1.finger_max.x_low | (panel.v1.finger_max.x_high << 8);
	ymax = panel.v1.finger_max.y_low | (panel.v1.finger_max.y_high << 8);
	if (protocol.major == 1) {
		panel.max_touches = MAX_TOUCHES;
		panel.extra_channel_num = 0;
		panel.sb_low = 0;
		panel.sb_high = 0xFF;
	}

	/* Show info */
	dev_info(dev, "Firmware Version %d.%d.%d, Protocol V%d.%d\n", 
		 firmware.id, firmware.major, firmware.minor,
		 protocol.major, protocol.minor);
	dev_info(dev, "Resolution: %dx%d, Channels: %dx%d, Touch Points: %d\n",
		 xmax, ymax, panel.v1.xchannel_num, panel.v1.ychannel_num,
		 panel.max_touches);
	if (panel.extra_channel_num) {
		if (panel.sb_high == 0xFF) {
			dev_info(dev, "Extra Channels: %d, Max. Keys: %d\n",
				 panel.extra_channel_num, panel.sb_low);
		} else {
			dev_info(dev, "Extra Channels: %d, Scrollbar: 0-%d\n",
				 panel.extra_channel_num,
				 (panel.sb_high << 8) | panel.sb_low);
		}
	}

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	input = input_allocate_device();
	if (!priv || !input) {
		error = -ENOMEM;
		goto err_free_mem;
	}

	priv->client = client;
	priv->input = input;
	if (protocol.major == 1)
		INIT_DELAYED_WORK(&priv->dwork, ili210x_work_v1);
	else
		INIT_DELAYED_WORK(&priv->dwork, ili210x_work_v2);
	priv->get_pendown_state = pdata->get_pendown_state;
	priv->poll_period = pdata->poll_period ? : DEFAULT_POLL_PERIOD;
	priv->max_touches = panel.max_touches;

	/* Setup input device */
	input->name = "ILI210x Touchscreen";
	input->id.bustype = BUS_I2C;
	input->dev.parent = dev;

	__set_bit(EV_SYN, input->evbit);
	__set_bit(EV_KEY, input->evbit);
	__set_bit(EV_ABS, input->evbit);
	__set_bit(BTN_TOUCH, input->keybit);

	/* Single touch */
	input_set_abs_params(input, ABS_X, 0, xmax, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, ymax, 0, 0);

	/* Multi touch */
	input_mt_init_slots(input, priv->max_touches, 0);
	input_set_abs_params(input, ABS_MT_POSITION_X, 0, xmax, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, ymax, 0, 0);

	i2c_set_clientdata(client, priv);

	error = request_irq(client->irq, ili210x_irq, pdata->irq_flags,
			    client->name, priv);
	if (error) {
		dev_err(dev, "Unable to request touchscreen IRQ, err: %d\n",
			error);
		goto err_free_mem;
	}

	error = sysfs_create_group(&dev->kobj, &ili210x_attr_group);
	if (error) {
		dev_err(dev, "Unable to create sysfs attributes, err: %d\n",
			error);
		goto err_free_irq;
	}

	error = input_register_device(priv->input);
	if (error) {
		dev_err(dev, "Cannot regiser input device, err: %d\n", error);
		goto err_remove_sysfs;
	}

	device_init_wakeup(dev, 1);

	dev_dbg(dev,
		"ILI210x initialized (IRQ: %d), firmware version %d.%d.%d",
		client->irq, firmware.id, firmware.major, firmware.minor);

	return 0;

err_remove_sysfs:
	sysfs_remove_group(&dev->kobj, &ili210x_attr_group);
err_free_irq:
	free_irq(client->irq, priv);
err_free_mem:
	input_free_device(input);
	kfree(priv);
	return error;
}

static int ili210x_i2c_remove(struct i2c_client *client)
{
	struct ili210x *priv = i2c_get_clientdata(client);

	sysfs_remove_group(&client->dev.kobj, &ili210x_attr_group);
	free_irq(priv->client->irq, priv);
	cancel_delayed_work_sync(&priv->dwork);
	input_unregister_device(priv->input);
	kfree(priv);

	return 0;
}

static int __maybe_unused ili210x_i2c_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	if (device_may_wakeup(&client->dev))
		enable_irq_wake(client->irq);

	return 0;
}

static int __maybe_unused ili210x_i2c_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	if (device_may_wakeup(&client->dev))
		disable_irq_wake(client->irq);

	return 0;
}

static SIMPLE_DEV_PM_OPS(ili210x_i2c_pm,
			 ili210x_i2c_suspend, ili210x_i2c_resume);

static const struct i2c_device_id ili210x_i2c_id[] = {
	{ "ili210x", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ili210x_i2c_id);

#ifdef CONFIG_OF
static const struct of_device_id ili210x_of_match[] = {
	{ .compatible = "Ilitek,ili210x", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ili210x_of_match);
#endif

static struct i2c_driver ili210x_ts_driver = {
	.driver = {
		.name = "ili210x_i2c",
		.of_match_table = of_match_ptr(ili210x_of_match),
		.pm = &ili210x_i2c_pm,
	},
	.id_table = ili210x_i2c_id,
	.probe = ili210x_i2c_probe,
	.remove = ili210x_i2c_remove,
};

module_i2c_driver(ili210x_ts_driver);

MODULE_AUTHOR("Olivier Sobrie <olivier@sobrie.be>");
MODULE_DESCRIPTION("ILI210X I2C Touchscreen Driver");
MODULE_LICENSE("GPL");
