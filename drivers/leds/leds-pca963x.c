/*
 * Copyright 2011 bct electronic GmbH
 * Copyright 2013 Qtechnology/AS
 *
 * Author: Peter Meerwald <p.meerwald@bct-electronic.com>
 * Author: Ricardo Ribalda <ricardo.ribalda@gmail.com>
 *
 * Based on leds-pca955x.c
 *
 * This file is subject to the terms and conditions of version 2 of
 * the GNU General Public License.  See the file COPYING in the main
 * directory of this archive for more details.
 *
 * LED driver for the PCA9633 I2C LED driver (7-bit slave address 0x62)
 * LED driver for the PCA9634/5 I2C LED driver (7-bit slave address set by hw.)
 *
 * Note that hardware blinking violates the leds infrastructure driver
 * interface since the hardware only supports blinking all LEDs with the
 * same delay_on/delay_off rates.  That is, only the LEDs that are set to
 * blink will actually blink but all LEDs that are set to blink will blink
 * in identical fashion.  The delay_on/delay_off values of the last LED
 * that is set to blink will be used for all of the blinking LEDs.
 * Hardware blinking is disabled by default but can be enabled by setting
 * the 'blink_type' member in the platform_data struct to 'PCA963X_HW_BLINK'
 * or by adding the 'nxp,hw-blink' property to the DTS.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/leds.h>
#include <linux/pwm.h>
#include <linux/gpio.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/platform_data/leds-pca963x.h>

/* LED select registers determine the source that drives LED outputs */
#define PCA963X_LED_OFF		0x0	/* LED driver off */
#define PCA963X_LED_ON		0x1	/* LED driver on */
#define PCA963X_LED_PWM		0x2	/* Controlled through PWM */
#define PCA963X_LED_GRP_PWM	0x3	/* Controlled through PWM/GRPPWM */
#define PCA963X_LED_MASK	0x3	/* MASK LED */
#define PCA963X_LED_OUT		0x8	/* LEDOUT Register */

#define PCA963X_MODE2_DMBLNK	0x20	/* Enable blinking */

#define PCA963X_MODE1		0x00
#define PCA963X_MODE2		0x01
#define PCA963X_PWM_BASE	0x02

enum pca963x_type {
	pca9633,
	pca9634,
	pca9635,
};

struct pca963x_chipdef {
	u8			grppwm;
	u8			grpfreq;
	u8			ledout_base;
	int			n_leds;
};

static struct pca963x_chipdef pca963x_chipdefs[] = {
	[pca9633] = {
		.grppwm		= 0x6,
		.grpfreq	= 0x7,
		.ledout_base	= 0x8,
		.n_leds		= 4,
	},
	[pca9634] = {
		.grppwm		= 0xa,
		.grpfreq	= 0xb,
		.ledout_base	= 0xc,
		.n_leds		= 8,
	},
	[pca9635] = {
		.grppwm		= 0x12,
		.grpfreq	= 0x13,
		.ledout_base	= 0x14,
		.n_leds		= 16,
	},
};

/* Total blink period in milliseconds */
#define PCA963X_BLINK_PERIOD_MIN	42
#define PCA963X_BLINK_PERIOD_MAX	10667

static const struct i2c_device_id pca963x_id[] = {
	{ "pca9632", pca9633 },
	{ "pca9633", pca9633 },
	{ "pca9634", pca9634 },
	{ "pca9635", pca9635 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, pca963x_id);

struct pca963x_entry;

struct pca963x {
	struct pca963x_chipdef *chipdef;
	struct mutex mutex;
	struct i2c_client *client;
	struct pca963x_entry *entries;
	int n_led;
	enum pca963x_blink_type blink_type;
	enum pca963x_outdrv outdrv;
#ifdef CONFIG_GPIOLIB
	int n_gpio;
	struct gpio_chip gpio_chip;
#endif
#ifdef CONFIG_PWM
	int n_pwm;
	struct pwm_chip pwm_chip;
#endif
};

#define PCA963X_LED_STATE_BLINKING BIT(0)

struct pca963x_entry {
	struct pca963x *chip;
	struct led_classdev led_cdev;
	int state;
	int flags;			/* see leds-pca963x.h */
	int led_num;			/* 0 .. 15 potentially */
	int defval;
	char name[32];
	u8 gdc;
	u8 gfrq;
};

/* -------- Helper functions -------- */

/* Read a register via I2C */
static int pca963x_read_reg(struct pca963x *pca963x, u8 reg)
{
	int value;

	value = i2c_smbus_read_byte_data(pca963x->client, reg);
	if (value < 0) {
		dev_err(&pca963x->client->dev,
			"can not read from PCA963x register %d\n", reg);
	}

	return value;
}

/* Write a register via I2C */
static int pca963x_write_reg(struct pca963x *pca963x, u8 reg, u8 value)
{
	int err;

	err = i2c_smbus_write_byte_data(pca963x->client, reg, value);
	if (err < 0) {
		dev_err(&pca963x->client->dev,
			"can not write to PCA963x register %d\n", reg);
	}

	return err;
}

/* Read value, clear and/or set some bits, write back */
static int pca963x_clear_set_reg(struct pca963x *pca963x, u8 reg, u8 mask,
				 u8 value)
{
	u8 old;
	int err;

	err = pca963x_read_reg(pca963x, reg);
	if (err < 0)
		return err;

	old = (u8)err;
	value = (value & mask) | (old & ~mask);

	/* Spare the writeback if there are no changes */
	if (old != value) {
		err = pca963x_write_reg(pca963x, reg, value);
		if (err < 0)
			return err;
	}

	return 0;
}

/* Set the PWM value if necessary and return the LED mode */
static int pca963x_set_pwm(struct pca963x *pca963x, int value, int offset)
{
	int ret;
	struct pca963x_entry *entry = &pca963x->entries[offset];

	if (entry->flags & PCA963X_LED_FLAGS_ACTIVE_HIGH)
		value = 256 - value;

	if (value == 256)
		ret = PCA963X_LED_ON;
	else if (value == 0)
		ret = PCA963X_LED_OFF;
	else {
		ret = pca963x_write_reg(pca963x, PCA963X_PWM_BASE + offset,
					value);
		if (ret < 0)
			return ret;

		if (entry->state & PCA963X_LED_STATE_BLINKING)
			ret = PCA963X_LED_GRP_PWM;
		else
			ret = PCA963X_LED_PWM;
	}

	return ret;
}

/* Set the LED brightness (0..256) */
static int pca963x_set_brightness(struct pca963x *pca963x, int offset, int val)
{
	int ret;

	mutex_lock(&pca963x->mutex);

	ret = pca963x_set_pwm(pca963x, val, offset);
	if (ret >= 0) {
		u8 reg = pca963x->chipdef->ledout_base + offset/4;
		int shift = (offset % 4) * 2;

		ret = pca963x_clear_set_reg(pca963x, reg,
					    PCA963X_LED_MASK << shift,
					    ret << shift);
	}

	mutex_unlock(&pca963x->mutex);

	return ret;
}

/* Return current LED brightness (0=off..256=fully on) */
static int pca963x_get_brightness(struct pca963x *pca963x, int offset)
{
	u8 reg = pca963x->chipdef->ledout_base + offset/4;
	int shift = (offset % 4) * 2;
	int value;

	mutex_lock(&pca963x->mutex);

	value = pca963x_read_reg(pca963x, reg);
	if (value < 0)
		goto unlock;

	value = (value >> shift) & PCA963X_LED_MASK;
	switch (value) {
	case PCA963X_LED_ON:
		value = 256;
		break;
	case PCA963X_LED_OFF:
		value = 0;
		break;
	default:
		value = pca963x_read_reg(pca963x, PCA963X_PWM_BASE + offset);
		if (value < 0)
			goto unlock;
		break;
	}

	if (pca963x->entries[offset].flags & PCA963X_LED_FLAGS_ACTIVE_HIGH)
		value = 256 - value;

unlock:
	mutex_unlock(&pca963x->mutex);

	return value;
}

/* -------- LED interface -------- */

static void pca963x_blink(struct pca963x *pca963x, int offset)
{
	u8 ledout_addr = pca963x->chipdef->ledout_base + offset / 4;
	u8 ledout;
	u8 mode2 = i2c_smbus_read_byte_data(pca963x->client, PCA963X_MODE2);
	int shift = 2 * (offset % 4);
	u8 mask = 0x3 << shift;
	struct pca963x_entry *entry = &pca963x->entries[offset];

	i2c_smbus_write_byte_data(pca963x->client,
			pca963x->chipdef->grppwm, entry->gdc);

	i2c_smbus_write_byte_data(pca963x->client,
			pca963x->chipdef->grpfreq, entry->gfrq);

	if (!(mode2 & PCA963X_MODE2_DMBLNK))
		i2c_smbus_write_byte_data(pca963x->client, PCA963X_MODE2,
					  mode2 | PCA963X_MODE2_DMBLNK);

	mutex_lock(&pca963x->mutex);
	ledout = i2c_smbus_read_byte_data(pca963x->client, ledout_addr);
	if ((ledout & mask) != (PCA963X_LED_GRP_PWM << shift))
		i2c_smbus_write_byte_data(pca963x->client, ledout_addr,
			(ledout & ~mask) | (PCA963X_LED_GRP_PWM << shift));
	mutex_unlock(&pca963x->mutex);
}

static int pca963x_led_set(struct led_classdev *led_cdev,
			   enum led_brightness value)
{
	struct pca963x_entry *entry;
	struct pca963x *pca963x;

	if (value >= LED_FULL)
		value = 256;

	entry = container_of(led_cdev, struct pca963x_entry, led_cdev);
	pca963x = entry->chip;

	return pca963x_set_brightness(pca963x, entry->led_num, value);
}

enum led_brightness pca963x_led_get(struct led_classdev *led_cdev)
{
	struct pca963x_entry *entry;
	struct pca963x *pca963x;

	entry = container_of(led_cdev, struct pca963x_entry, led_cdev);
	pca963x = entry->chip;

	return pca963x_get_brightness(pca963x, entry->led_num);
}

static int pca963x_blink_set(struct led_classdev *led_cdev,
		unsigned long *delay_on, unsigned long *delay_off)
{
	struct pca963x_entry *entry;
	unsigned long time_on, time_off, period;
	u8 gdc, gfrq;

	entry = container_of(led_cdev, struct pca963x_entry, led_cdev);

	time_on = *delay_on;
	time_off = *delay_off;

	/* If both zero, pick reasonable defaults of 500ms each */
	if (!time_on && !time_off) {
		time_on = 500;
		time_off = 500;
	}

	period = time_on + time_off;

	/* If period not supported by hardware, default to someting sane. */
	if ((period < PCA963X_BLINK_PERIOD_MIN) ||
	    (period > PCA963X_BLINK_PERIOD_MAX)) {
		time_on = 500;
		time_off = 500;
		period = time_on + time_off;
	}

	/*
	 * From manual: duty cycle = (GDC / 256) ->
	 *	(time_on / period) = (GDC / 256) ->
	 *		GDC = ((time_on * 256) / period)
	 */
	gdc = (time_on * 256) / period;

	/*
	 * From manual: period = ((GFRQ + 1) / 24) in seconds.
	 * So, period (in ms) = (((GFRQ + 1) / 24) * 1000) ->
	 *		GFRQ = ((period * 24 / 1000) - 1)
	 */
	gfrq = (period * 24 / 1000) - 1;

	entry->gdc = gdc;
	entry->gfrq = gfrq;

	pca963x_blink(entry->chip, entry->led_num);

	*delay_on = time_on;
	*delay_off = time_off;

	return 0;
}

/* -------- PWM interface -------- */

#ifdef CONFIG_PWM
static int pca963x_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
			      int duty_ns, int period_ns)
{
	int value;
	struct pca963x *pca963x = container_of(chip, struct pca963x, pwm_chip);

	if (period_ns != 640000) {
		dev_warn(&pca963x->client->dev,
			 "Signal period must be fix at 640000 (1.5625 kHz)\n");
		return -EINVAL;
	}

#if 0 //###
	/*
	 * When PWM is off, do not change the hardware, just save the new
	 * period_ns and duty_ns (already done in PWM infrastructure).
	 */
	if (!pwm->state.enabled)
		return 0;
#endif //###

	value = ((duty_ns * 256 + period_ns/2) / period_ns);
	if (pwm->state.polarity == PWM_POLARITY_INVERSED)
		value = 256 - value;

	return pca963x_set_brightness(pca963x, pwm->hwpwm, value);
}

static int pca963x_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	/* Activate PWM with the current duty_ns value */
	return pca963x_pwm_config(chip, pwm, pwm->state.duty_cycle,
				  pwm->state.period);
}

static void pca963x_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	pca963x_pwm_config(chip, pwm, 0, pwm->state.period);
}

static int pca963x_pwm_set_polarity(struct pwm_chip *chip,
			struct pwm_device *pwm, enum pwm_polarity polarity)
{
	/*
	 * Setting pwm->state.polarity is all that needs to be done and this
	 * is already done in the PWM infrastructure. However we still need
	 * this empty function to indicate support for polarity inversion.
	 */

	return 0;
}

static int pca963x_pwm_request(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct pca963x *pca963x = container_of(chip, struct pca963x, pwm_chip);
	struct pca963x_entry *entry = &pca963x->entries[pwm->hwpwm];
	int value;

	if (!(entry->flags & PCA963X_LED_FLAGS_TYPE_PWM))
		return -ENODEV;

	/* fixed frequency signal 1.5625kHz */
	pwm->label = entry->name;
	pwm->state.period = 640000;

	value = pca963x_get_brightness(pca963x, pwm->hwpwm);
	if (value < 0)
		return value;

	if (pwm->state.polarity == PWM_POLARITY_INVERSED)
		value = 256 - value;

	pwm->state.duty_cycle = (value * pwm->state.period + 128) / 256;

	return 0;
}

static const struct pwm_ops pca963x_pwm_ops = {
	.config = pca963x_pwm_config,
	.set_polarity = pca963x_pwm_set_polarity,
	.enable = pca963x_pwm_enable,
	.disable = pca963x_pwm_disable,
	.request = pca963x_pwm_request,
	.owner = THIS_MODULE,
};
#endif

/* -------- GPIO interface -------- */

#ifdef CONFIG_GPIOLIB
static void pca963x_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct pca963x *pca963x;

	pca963x = container_of(chip, struct pca963x, gpio_chip);

	if (value > 0)
		value = 256;

	pca963x_set_brightness(pca963x, offset, value);
}

static int pca963x_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	int value;
	struct pca963x *pca963x;

	pca963x = container_of(chip, struct pca963x, gpio_chip);

	value = pca963x_get_brightness(pca963x, offset);
	if (value > 0)
		value = 1;

	return value;
}

static int pca963x_gpio_direction_output(struct gpio_chip *chip,
						unsigned offset, int value)
{
	pca963x_gpio_set(chip, offset, value);

	return 0;
}

static int pca963x_gpio_request(struct gpio_chip *chip, unsigned offset)
{
	struct pca963x *pca963x;

	pca963x = container_of(chip, struct pca963x, gpio_chip);
	if (!(pca963x->entries[offset].flags & PCA963X_LED_FLAGS_TYPE_GPIO))
		return -ENODEV;

	return 0;
}

const struct gpio_chip pca963x_gpio_ops = {
	.request	= pca963x_gpio_request,
	.get		= pca963x_gpio_get,
	.set		= pca963x_gpio_set,
	.direction_output = pca963x_gpio_direction_output,
	.owner		= THIS_MODULE,
};
#endif

/* -------- Initialization -------- */

/* Set default value for all LEDs with PCA963X_LED_FLAGS_KEEP_VALUE not set */
static int pca963x_set_default_values(struct pca963x *pca963x)
{
	u8 reg = pca963x->chipdef->ledout_base;
	u8 mask, val;
	int i, offset, ret, shift;
	struct pca963x_entry *entry;

	/* No need to aquire mutex, nobody knows us yet */
	for (offset = 0; offset < pca963x->chipdef->n_leds; offset += 4) {
		mask = 0;
		val = 0;
		for (i = offset; i < offset + 4; i++) {
			entry = &pca963x->entries[i];
			if (entry->flags & PCA963X_LED_FLAGS_KEEP_VALUE)
				continue;

			ret = pca963x_set_pwm(pca963x, entry->defval, i);
			if (ret < 0)
				return ret;

			shift = (i % 4) * 2;
			val |= ret << shift;
			mask |= PCA963X_LED_MASK << shift;
		}

		/* Write LED mode register if there are any changes */
		if (mask) {
			/* If whole register changes, spare reading it first */
			if (mask == 0xFF)
				ret = pca963x_write_reg(pca963x, reg, val);
			else
				ret = pca963x_clear_set_reg(pca963x, reg,
							    mask, val);
			if (ret < 0)
				return ret;
		}
		reg++;
	}

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
/* Parse device tree settings */
static int pca963x_dt_init(struct pca963x *pca963x)
{
	struct device_node *np = pca963x->client->dev.of_node, *child;
	u32 reg;
	int res;
	const void *type;
	const char *label;
	int count;
	int value;

	count = of_get_child_count(np);
	if (!count || count > pca963x->chipdef->n_leds) {
		dev_err(&pca963x->client->dev,
			"device tree node must have 1..%d children\n",
			pca963x->chipdef->n_leds);
		return -EINVAL;
	}

	for_each_child_of_node(np, child) {
		struct pca963x_entry *entry;

		res = of_property_read_u32(child, "reg", &reg);
		if ((res != 0) || (reg >= pca963x->chipdef->n_leds))
			continue;
		entry = &pca963x->entries[reg];

		/*
		 * The active state defines how any consumers see this pin. It
		 * is common practice to set the active state for the pin in
		 * such a way that any consumers can use it straight away
		 * without any further inversion. If a user needs to switch
		 * function at runtime, he can add an additional (!) inversion
		 * on GPIOs and PWMs via sysfs. For pins configured as LED, no
		 * additional inversion is available. If this is necessary,
		 * configure these pins as PWM and use pwm-leds instead.
		 */
		if (of_property_read_bool(child, "active-high"))
			entry->flags |= PCA963X_LED_FLAGS_ACTIVE_HIGH;
		else if (of_property_read_bool(child, "active-low"))
			entry->flags &= ~PCA963X_LED_FLAGS_ACTIVE_HIGH;

		value = 0;
		if (of_property_read_bool(child, "keep-value"))
			entry->flags |= PCA963X_LED_FLAGS_KEEP_VALUE;
		else if (of_property_read_u32(child, "value", &value) != 0) {
			if (of_property_read_bool(child, "default-on"))
				value = 256;
		}
		entry->defval = value;

		label = of_get_property(child, "label", NULL);
		if (label) {
			snprintf(entry->name, sizeof(entry->name),
				"pca963x:%s", label);
		} else {
			snprintf(entry->name, sizeof(entry->name),
				"pca963x:%d:%.2x:%d",
				pca963x->client->adapter->nr,
				pca963x->client->addr, reg);
		}

		type = of_get_property(child, "type", NULL);
 		if (!type || !strcmp(type, "LED") || !strcmp(type, "")) {
			pca963x->n_led++;
			entry->led_cdev.name = entry->name;
			entry->led_cdev.flags = 0;
			entry->led_cdev.default_trigger =
				of_get_property(child, "linux,default-trigger",
						NULL);
		}
#ifdef CONFIG_PWM
		else if (!strcmp(type, "PWM")) {
			pca963x->n_pwm++;
			entry->flags |= PCA963X_LED_FLAGS_TYPE_PWM;
		}
#endif
#ifdef CONFIG_GPIOLIB
		else if (!strcmp(type, "GPIO")) {
			pca963x->n_gpio++;
			entry->flags |= PCA963X_LED_FLAGS_TYPE_GPIO;
		}
#endif
		else {
			dev_err(&pca963x->client->dev, "unknown entry type\n");
			return -EINVAL;
		}
	}

	/* default to open-drain unless totem pole (push-pull) is specified */
	if (of_property_read_bool(np, "nxp,totem-pole"))
		pca963x->outdrv = PCA963X_TOTEM_POLE;
	else
		pca963x->outdrv = PCA963X_OPEN_DRAIN;

	/* default to software blinking unless hardware blinking is specified */
	if (of_property_read_bool(np, "nxp,hw-blink"))
		pca963x->blink_type = PCA963X_HW_BLINK;
	else
		pca963x->blink_type = PCA963X_SW_BLINK;

	return 0;
}

static const struct of_device_id of_pca963x_match[] = {
	{ .compatible = "nxp,pca9632", },
	{ .compatible = "nxp,pca9633", },
	{ .compatible = "nxp,pca9634", },
	{ .compatible = "nxp,pca9635", },
	{},
};
MODULE_DEVICE_TABLE(of, of_pca963x_match);
#else
static int pca963x_dt_init(struct pca963x *pca963x)
{
	return -ENODEV;
}
#endif

/* Copy info from platform data */
static int pca963x_platform_init(struct pca963x *pca963x,
				 struct pca963x_platform_data *pdata)
{
	int i;

	if ((pdata->leds.num_leds < 1)
	    || (pdata->leds.num_leds > pca963x->chipdef->n_leds)) {
		dev_err(&pca963x->client->dev,
			"board info must claim 1..%d LEDs\n",
			pca963x->chipdef->n_leds);
		return -EINVAL;
	}

	for (i = 0; i < pdata->leds.num_leds; i++) {
		struct pca963x_entry *entry = &pca963x->entries[i];
		struct led_info *pdata_led = &pdata->leds.leds[i];

		entry->flags = pdata_led->flags;
		if (entry->flags & PCA963X_LED_FLAGS_DEFAULT_ON)
			entry->defval = 256;
		else
			entry->defval = 0;

#ifdef CONFIG_PWM
		/* PWM takes precedence over GPIO if both flags are set */
		if (entry->flags & PCA963X_LED_FLAGS_TYPE_PWM) {
			entry->flags &= ~PCA963X_LED_FLAGS_TYPE_GPIO;
			pca963x->n_pwm++;
		}
#else
		entry->flags &= ~PCA963X_LED_FLAGS_TYPE_PWM;
#endif
#ifdef CONFIG_GPIOLIB
		if (entry->flags & PCA963X_LED_FLAGS_TYPE_GPIO)
			pca963x->n_gpio++;
#else
		entry->flags &= ~PCA963X_LED_FLAGS_TYPE_GPIO;
#endif
		if (!(entry->flags & PCA963X_LED_FLAGS_TYPE_MASK))
			pca963x->n_led++;
		if (pdata_led->name)
			snprintf(entry->name, sizeof(entry->name),
				"pca963x:%s", pdata_led->name);
		if (pdata_led->default_trigger)
			entry->led_cdev.default_trigger =
				pdata_led->default_trigger;
	}
	pca963x->outdrv = pdata->outdrv;
	pca963x->blink_type = pdata->blink_type;

	return 0;
}

/* Unregister all LEDS from count-1 to 0 (i.e. in reverse order) */
static void pca963x_free_leds(struct pca963x *pca963x, int count)
{
	while (count > 0) {
		struct pca963x_entry *entry = &pca963x->entries[--count];

		if (!(entry->flags & PCA963X_LED_FLAGS_TYPE_MASK)) {
			led_classdev_unregister(&entry->led_cdev);
		}
	}
}

static int pca963x_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct pca963x *pca963x;
	struct pca963x_platform_data *pdata;
	struct pca963x_entry *entry;
	int i, err;
	size_t size;

	pca963x = devm_kzalloc(&client->dev, sizeof(struct pca963x),
			       GFP_KERNEL);
	if (!pca963x)
		return -ENOMEM;

	pca963x->chipdef = &pca963x_chipdefs[id->driver_data];
	pca963x->client = client;
	mutex_init(&pca963x->mutex);

	size = sizeof(struct pca963x_entry) * pca963x->chipdef->n_leds;
	pca963x->entries = devm_kzalloc(&client->dev, size, GFP_KERNEL);
	if (!pca963x->entries)
		return -ENOMEM;

	pdata = dev_get_platdata(&client->dev);
	if (pdata)
		err = pca963x_platform_init(pca963x, pdata);
	else
		err = pca963x_dt_init(pca963x);
	if (err)
		return err;

	i2c_set_clientdata(client, pca963x);

	err = pca963x_set_default_values(pca963x);
	if (err < 0)
		return err;

	for (i = 0; i < pca963x->chipdef->n_leds; i++) {
		entry = &pca963x->entries[i];
		entry->led_num = i;
		entry->chip = pca963x;
		if (!(entry->flags & PCA963X_LED_FLAGS_TYPE_MASK)) {
			struct led_classdev *led_cdev = &entry->led_cdev;

			led_cdev->brightness_set_blocking = pca963x_led_set;
			led_cdev->brightness_get = pca963x_led_get;
			if (pca963x->blink_type == PCA963X_HW_BLINK)
				led_cdev->blink_set = pca963x_blink_set;
			err = led_classdev_register(&client->dev, led_cdev);
			if (err < 0) {
				dev_err(&client->dev,
					"could not register LED %d\n", i);
				goto exit;
			}
		}
	}
#ifdef CONFIG_PWM
	if (pca963x->n_pwm > 0) {
		pca963x->pwm_chip.of_pwm_n_cells = 3;
		pca963x->pwm_chip.of_xlate = of_pwm_xlate_with_flags;
		pca963x->pwm_chip.dev = &client->dev;
		pca963x->pwm_chip.ops = &pca963x_pwm_ops;
		pca963x->pwm_chip.npwm = pca963x->chipdef->n_leds;
		pca963x->pwm_chip.base = -1;
		err = pwmchip_add(&pca963x->pwm_chip);
		if (err < 0) {
			dev_err(&client->dev,"could not register PWM chip\n");
			goto exit;
		}
	}
#endif

#ifdef CONFIG_GPIOLIB
	if (pca963x->n_gpio > 0) {
		pca963x->gpio_chip = pca963x_gpio_ops;
		pca963x->gpio_chip.label = client->name;
		pca963x->gpio_chip.ngpio = pca963x->chipdef->n_leds;
		pca963x->gpio_chip.parent = &client->dev;
		pca963x->gpio_chip.base = -1;
		err = gpiochip_add(&pca963x->gpio_chip);
		if (err < 0) {
			dev_err(&client->dev, "could not register GPIO chip\n");
			goto err_gpio;
		}
	}
#endif

	/* Disable LED all-call address and set normal mode */
	pca963x_write_reg(pca963x, PCA963X_MODE1, 0x00);

	/* Configure output: open-drain or totem pole (push-pull) */
	if (pca963x->outdrv == PCA963X_OPEN_DRAIN)
		pca963x_write_reg(pca963x, PCA963X_MODE2, 0x01);
	else
		pca963x_write_reg(pca963x, PCA963X_MODE2, 0x05);

	return 0;

err_gpio:
	pwmchip_remove(&pca963x->pwm_chip);
exit:
	pca963x_free_leds(pca963x, i);

	return err;
}

static int pca963x_remove(struct i2c_client *client)
{
	struct pca963x *pca963x;

	pca963x = i2c_get_clientdata(client);

#ifdef CONFIG_GPIOLIB
	if (pca963x->n_gpio > 0)
		gpiochip_remove(&pca963x->gpio_chip);
#endif
#ifdef CONFIG_PWM
	if (pca963x->n_pwm > 0)
		pwmchip_remove(&pca963x->pwm_chip);
#endif
	if (pca963x->n_led > 0)
		pca963x_free_leds(pca963x, pca963x->chipdef->n_leds);

	return 0;
}

static struct i2c_driver pca963x_driver = {
	.driver = {
		.name	= "leds-pca963x",
		.of_match_table = of_match_ptr(of_pca963x_match),
	},
	.probe	= pca963x_probe,
	.remove	= pca963x_remove,
	.id_table = pca963x_id,
};

static int __init pca963x_init(void)
{
	return i2c_add_driver(&pca963x_driver);
}
subsys_initcall(pca963x_init);

static void __exit pca963x_exit(void)
{
	return i2c_del_driver(&pca963x_driver);
}

module_exit(pca963x_exit);

MODULE_AUTHOR("Peter Meerwald <p.meerwald@bct-electronic.com>");
MODULE_DESCRIPTION("PCA963X LED driver");
MODULE_LICENSE("GPL v2");
