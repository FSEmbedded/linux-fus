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
#include <linux/workqueue.h>
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

enum pca963x_type {
	pca9633,
	pca9634,
	pca9635,
};

struct pca963x_chipdef {
	u8			mode1;
	u8			mode2;
	u8			pwmout_base;
	u8			grppwm;
	u8			grpfreq;
	u8			ledout_base;
	int			n_leds;
};

static struct pca963x_chipdef pca963x_chipdefs[] = {
	[pca9633] = {
		.mode1		= 0x0,
		.mode2		= 0x1,
		.pwmout_base	= 0x2,
		.grppwm		= 0x6,
		.grpfreq	= 0x7,
		.ledout_base	= 0x8,
		.n_leds		= 4,
	},
	[pca9634] = {
		.mode1		= 0x0,
		.mode2		= 0x1,
		.pwmout_base	= 0x2,
		.grppwm		= 0xa,
		.grpfreq	= 0xb,
		.ledout_base	= 0xc,
		.n_leds		= 8,
	},
	[pca9635] = {
		.mode1		= 0x0,
		.mode2		= 0x1,
		.pwmout_base	= 0x2,
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

enum pca963x_cmd {
	BRIGHTNESS_SET,
	BLINK_SET,
};

struct pca963x_led;

enum pin_type {
	LED,
	PWM,
	GPIO,
};

struct pca963x {
	struct pca963x_chipdef *chipdef;
	struct mutex mutex;
	struct i2c_client *client;
	struct pca963x_led *leds;
	int n_pwm;
	int n_led;
	int n_gpio;
	struct gpio_chip gchip;
	struct pwm_chip chip;
	enum pca963x_blink_type blink_type;
	enum pca963x_outdrv outdrv;
};

struct pca963x_led {
	struct pca963x *chip;
	struct work_struct work;
	struct led_classdev led_cdev;
	enum pin_type type;
	enum led_brightness brightness;
	enum pca963x_cmd cmd;
	int flags;
	int led_num; /* 0 .. 7 potentially */
	char name[32];
	u8 gdc;
	u8 gfrq;
};

static void pca963x_brightness_work(struct pca963x_led *pca963x)
{
	u8 ledout_addr = pca963x->chip->chipdef->ledout_base
		+ (pca963x->led_num / 4);
	u8 ledout;
	int shift = 2 * (pca963x->led_num % 4);
	u8 mask = 0x3 << shift;
	enum led_brightness brightness;

	mutex_lock(&pca963x->chip->mutex);

	brightness = pca963x->brightness;
	if (pca963x->flags & PCA963X_FLAGS_ACTIVE_HIGH)
		brightness = LED_FULL - brightness;

	ledout = i2c_smbus_read_byte_data(pca963x->chip->client, ledout_addr);
	switch (brightness) {
	case LED_FULL:
		i2c_smbus_write_byte_data(pca963x->chip->client, ledout_addr,
			(ledout & ~mask) | (PCA963X_LED_ON << shift));
		break;
	case LED_OFF:
		i2c_smbus_write_byte_data(pca963x->chip->client, ledout_addr,
			ledout & ~mask);
		break;
	default:
		i2c_smbus_write_byte_data(pca963x->chip->client,
			pca963x->chip->chipdef->pwmout_base + pca963x->led_num,
			brightness);
		i2c_smbus_write_byte_data(pca963x->chip->client, ledout_addr,
			(ledout & ~mask) | (PCA963X_LED_PWM << shift));
		break;
	}
	mutex_unlock(&pca963x->chip->mutex);
}

static void pca963x_blink_work(struct pca963x_led *pca963x)
{
	u8 ledout_addr = pca963x->chip->chipdef->ledout_base +
		(pca963x->led_num / 4);
	u8 ledout;
	u8 mode2 = i2c_smbus_read_byte_data(pca963x->chip->client,
						pca963x->chip->chipdef->mode2);
	int shift = 2 * (pca963x->led_num % 4);
	u8 mask = 0x3 << shift;

	i2c_smbus_write_byte_data(pca963x->chip->client,
			pca963x->chip->chipdef->grppwm,	pca963x->gdc);

	i2c_smbus_write_byte_data(pca963x->chip->client,
			pca963x->chip->chipdef->grpfreq, pca963x->gfrq);

	if (!(mode2 & PCA963X_MODE2_DMBLNK))
		i2c_smbus_write_byte_data(pca963x->chip->client,
			pca963x->chip->chipdef->mode2,
			mode2 | PCA963X_MODE2_DMBLNK);

	mutex_lock(&pca963x->chip->mutex);
	ledout = i2c_smbus_read_byte_data(pca963x->chip->client, ledout_addr);
	if ((ledout & mask) != (PCA963X_LED_GRP_PWM << shift))
		i2c_smbus_write_byte_data(pca963x->chip->client, ledout_addr,
			(ledout & ~mask) | (PCA963X_LED_GRP_PWM << shift));
	mutex_unlock(&pca963x->chip->mutex);
}

static void pca963x_work(struct work_struct *work)
{
	struct pca963x_led *pca963x = container_of(work,
		struct pca963x_led, work);

	switch (pca963x->cmd) {
	case BRIGHTNESS_SET:
		pca963x_brightness_work(pca963x);
		break;
	case BLINK_SET:
		pca963x_blink_work(pca963x);
		break;
	}
}

static void pca963x_led_set(struct led_classdev *led_cdev,
	enum led_brightness value)
{
	struct pca963x_led *pca963x;

	pca963x = container_of(led_cdev, struct pca963x_led, led_cdev);

	pca963x->cmd = BRIGHTNESS_SET;
	pca963x->brightness = value;

	/*
	 * Must use workqueue for the actual I/O since I2C operations
	 * can sleep.
	 */
	schedule_work(&pca963x->work);
}

static int pca963x_blink_set(struct led_classdev *led_cdev,
		unsigned long *delay_on, unsigned long *delay_off)
{
	struct pca963x_led *pca963x;
	unsigned long time_on, time_off, period;
	u8 gdc, gfrq;

	pca963x = container_of(led_cdev, struct pca963x_led, led_cdev);

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

	pca963x->cmd = BLINK_SET;
	pca963x->gdc = gdc;
	pca963x->gfrq = gfrq;

	/*
	 * Must use workqueue for the actual I/O since I2C operations
	 * can sleep.
	 */
	schedule_work(&pca963x->work);

	*delay_on = time_on;
	*delay_off = time_off;

	return 0;
}

#if IS_ENABLED(CONFIG_OF)

static struct pca963x *
pca963x_dt_init(struct i2c_client *client, struct pca963x_chipdef *chip)
{
	struct device_node *np = client->dev.of_node, *child;
	struct pca963x *data;
	u32 reg;
	int res;
	const void *type;
	const char *label;
	reg = of_get_child_count(np);
	if (!reg || reg > chip->n_leds)
		return ERR_PTR(-ENODEV);

	data = devm_kzalloc(&client->dev,
			sizeof(struct pca963x), GFP_KERNEL);
	data->leds = devm_kzalloc(&client->dev,
			sizeof(struct pca963x_led) * chip->n_leds, GFP_KERNEL);
	data->n_pwm = 0;
	data->n_led = 0;
	data->n_gpio = 0;
	for_each_child_of_node(np, child) {
		res = of_property_read_u32(child, "reg", &reg);
		if(res != 0)
			continue;
		if (of_property_read_bool(child, "active-high"))
				data->leds[reg].flags |=
						PCA963X_FLAGS_ACTIVE_HIGH;
		if (of_property_read_bool(child, "default-on"))
				data->leds[reg].flags |=
						PCA963X_FLAGS_DEFAULT_ON;
		label = of_get_property(child, "label", NULL);
		if(label == NULL) {
			snprintf(data->leds[reg].name,
					sizeof(data->leds[reg].name),
				"pca963x:%d:%.2x:%d",
				client->adapter->nr,
				client->addr, reg);
		}
		else {
			snprintf(data->leds[reg].name,
					sizeof(data->leds[reg].name),
				"pca963x:%s",
				label);
		}
		type = of_get_property(child, "type", NULL);
 		if(!type || strcmp(type,"LED") == 0 || strcmp(type,"") == 0) {
			data->n_led++;
			data->leds[reg].type = LED;
			data->leds[reg].led_cdev.name = data->leds[reg].name;
			data->leds[reg].led_cdev.flags = data->leds[reg].flags;
			data->leds[reg].led_cdev.default_trigger =
			of_get_property(child, "linux,default-trigger", NULL);
			data->leds[reg].led_cdev.brightness_set =
							pca963x_led_set;
			if(of_property_read_bool(np, "nxp,hw-blink"))
				data->leds[reg].led_cdev.blink_set =
							pca963x_blink_set;
		}
#ifdef CONFIG_PWM
		else if(strcmp(type,"PWM") == 0) {
			data->n_pwm++;
			data->leds[reg].type = PWM;
		}
#endif
#ifdef CONFIG_GPIOLIB
		else if(strcmp(type, "GPIO") == 0) {
			data->n_gpio++;
			data->leds[reg].type = GPIO;
		}
#endif
		else {
			dev_err(&client->dev, "unknown type!\n");
			return NULL;
		}
	}

	/* default to open-drain unless totem pole (push-pull) is specified */
	if (of_property_read_bool(np, "nxp,totem-pole"))
		data->outdrv = PCA963X_TOTEM_POLE;
	else
		data->outdrv = PCA963X_OPEN_DRAIN;

	/* default to software blinking unless hardware blinking is specified */
	if (of_property_read_bool(np, "nxp,hw-blink"))
		data->blink_type = PCA963X_HW_BLINK;
	else
		data->blink_type = PCA963X_SW_BLINK;

	return data;
}

static const struct of_device_id of_pca963x_match[] = {
	{ .compatible = "nxp,pca9632", },
	{ .compatible = "nxp,pca9633", },
	{ .compatible = "nxp,pca9634", },
	{ .compatible = "nxp,pca9635", },
	{},
};
#else
static struct pca963x_platform_data *
pca963x_dt_init(struct i2c_client *client, struct pca963x_chipdef *chip)
{
	return ERR_PTR(-ENODEV);
}
#endif

static struct pca963x *
pca963x_platform_init(struct i2c_client *client,
	struct pca963x_platform_data *pdata, struct pca963x_chipdef *chip)
{
 	struct pca963x *pca963x_chip;
	int i;

	pca963x_chip = devm_kzalloc(&client->dev, sizeof(*pca963x_chip),
								GFP_KERNEL);
	if (!pca963x_chip)
		return NULL;

	for(i = 0; i < chip->n_leds; i++) {
		pca963x_chip->leds[i].flags = pdata->leds.leds[i].flags;
		pca963x_chip->leds[i].type = LED;
		if (pdata->leds.leds[i].name)
			snprintf(pca963x_chip->leds[i].name,
				sizeof(pca963x_chip->leds[i].name),
				"pca963x:%s", pdata->leds.leds[i].name);
		if (pdata->leds.leds[i].default_trigger)
			pca963x_chip->leds[i].led_cdev.default_trigger =
			pdata->leds.leds[i].default_trigger;
		pca963x_chip->leds[i].led_cdev.brightness_set=pca963x_led_set;
		if (pdata && pdata->blink_type == PCA963X_HW_BLINK)
			pca963x_chip->leds[i].led_cdev.blink_set =
							pca963x_blink_set;
	}
	pca963x_chip->outdrv = pdata->outdrv;
	return pca963x_chip;
}

#ifdef CONFIG_PWM
static int pca963x_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
			      int duty_ns, int period_ns)
{
	u8 calc;
	u8 val, mode, mask;
	struct pca963x *pca963x_chip;

	/*
	 * When PWM is off, do not change the hardware, just save the new
	 * period_ns and duty_ns (already done in PWM infrastructure).
	 */
	if (!pwm->state.enabled)
		return 0;

	pca963x_chip = container_of(chip, struct pca963x, chip);
	if(period_ns != 640000) {
		dev_warn(&pca963x_chip->client->dev,
			 "Signal period must be fix at 640000 (1.5625 kHz)\n");
		return 1;
	}

	calc = (u8)((duty_ns * 255 + 127) / pwm->state.period);
	if (pwm->state.polarity == PWM_POLARITY_NORMAL)
		calc = 255 - calc;

	if (calc == 255)
		mode = PCA963X_LED_ON;
	else if (calc == 0)
		mode = PCA963X_LED_OFF;
	else {
		u8 reg = pca963x_chip->chipdef->pwmout_base + pwm->hwpwm;

		i2c_smbus_write_byte_data(pca963x_chip->client, reg, calc);
		mode = PCA963X_LED_PWM;
	}

	/* Set new LED mode if different to previous mode */
	val = i2c_smbus_read_byte_data(pca963x_chip->client,
				       pca963x_chip->chipdef->ledout_base);
	mask = PCA963X_LED_MASK << (2 * pwm->hwpwm);
	mode <<= 2 * pwm->hwpwm;
	if ((val & mask) != mode) {
		val = (val & ~mask) | mode;
		i2c_smbus_write_byte_data(pca963x_chip->client,
				  pca963x_chip->chipdef->ledout_base, val);
	}

	return 0;
}

static int pca963x_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	/* Activate PWM with the current duty_ns value */
	return pca963x_pwm_config(chip, pwm, pwm->state.duty_cycle, pwm->state.period);
}

static void pca963x_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	/* Switch backlight off (duty_ns = 0) */
	pca963x_pwm_config(chip, pwm, 0, pwm->state.period);
}

static int pca963x_pwm_set_polarity(struct pwm_chip *chip,
			struct pwm_device *pwm, enum pwm_polarity polarity)
{
	pwm->state.polarity = polarity;

	return 0;
}

static void pca963x_pwm_free(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct pca963x *pca963x_chip;
	pca963x_chip = container_of(chip, struct pca963x, chip);

	i2c_smbus_write_byte_data(pca963x_chip->client,
				  pca963x_chip->chipdef->mode1, 0x00);
	pwmchip_remove(&pca963x_chip->chip);
	cancel_work_sync(&pca963x_chip->leds[pwm->hwpwm].work);
}

static int pca963x_pwm_request(struct pwm_chip *chip, struct pwm_device *pwm)
{
	int err = 0;
	struct pca963x *pca963x_chip;
	pca963x_chip = container_of(chip, struct pca963x, chip);
	if(pca963x_chip->leds[pwm->hwpwm].type != 1)
		return -ENODEV;

	/* fixed frequency signal 1.5625kHz */
	pwm->label = pca963x_chip->leds[pwm->hwpwm].name;
	pwm->flags  = pca963x_chip->leds[pwm->hwpwm].flags;
	pwm->state.period = 640000;

	if(pwm->flags & PCA963X_FLAGS_ACTIVE_HIGH)
		pwm->state.polarity = PWM_POLARITY_NORMAL;
	else
		pwm->state.polarity = PWM_POLARITY_INVERSED;

	if((pca963x_chip->leds[pwm->hwpwm].flags & PCA963X_FLAGS_DEFAULT_ON &&
	pwm->state.polarity == PWM_POLARITY_NORMAL) ||
	(!(pca963x_chip->leds[pwm->hwpwm].flags & PCA963X_FLAGS_DEFAULT_ON) &&
				pwm->state.polarity == PWM_POLARITY_INVERSED))
		pwm->state.duty_cycle = pwm->state.period;
	else
		pwm->state.duty_cycle = 0;

	return err;
}

static const struct pwm_ops pca963x_pwm_ops = {
	.free = pca963x_pwm_free,
	.config = pca963x_pwm_config,
	.set_polarity = pca963x_pwm_set_polarity,
	.enable = pca963x_pwm_enable,
	.disable = pca963x_pwm_disable,
	.request = pca963x_pwm_request,
	.owner = THIS_MODULE,
};
#endif


#ifdef CONFIG_GPIOLIB
static int pca963x_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	u8 mask;
	u8 val;
	struct pca963x *pca963x_chip;
	pca963x_chip = container_of(chip, struct pca963x, gchip);
	val = i2c_smbus_read_byte_data(pca963x_chip->client,
				       pca963x_chip->chipdef->ledout_base);

	mask = PCA963X_LED_ON << (2 * offset);

	return (mask & val);
}

static void pca963x_gpio_set(struct gpio_chip *chip, unsigned offset,
			     int value)
{
	u8 val;
	struct pca963x *pca963x_chip;
	pca963x_chip = container_of(chip, struct pca963x, gchip);

	val = i2c_smbus_read_byte_data(pca963x_chip->client,
				       pca963x_chip->chipdef->ledout_base);

	val &= ~(PCA963X_LED_MASK << (2 * offset));

	if(value)
		val |= PCA963X_LED_ON << (2 * offset);

 	i2c_smbus_write_byte_data(pca963x_chip->client,
				  pca963x_chip->chipdef->ledout_base, val);
}

static int pca963x_gpio_direction_output(struct gpio_chip *chip,
						unsigned offset, int value)
{
	pca963x_gpio_set(chip, offset, value);
	return 0;
}

#if 0 //### FIXME: Do we need this?
static int pca963x_gpio_request(struct gpio_chip *chip, unsigned offset)
{
	int value = 0;
	struct pca963x *pca963x_chip;
	pca963x_chip = container_of(chip, struct pca963x, gchip);

	if(pca963x_chip->leds[offset].type != GPIO)
		return -ENODEV;

	if(!(pca963x_chip->leds[offset].flags & PCA963X_FLAGS_ACTIVE_HIGH))
		gpiod_sysfs_set_active_low(gpio_to_desc(offset+chip->base),
					   PCA963X_FLAGS_ACTIVE_HIGH);

	if(pca963x_chip->leds[offset].flags & PCA963X_FLAGS_DEFAULT_ON)
		value = 1;

	pca963x_gpio_set(chip, offset, value);

	return 0;
}
#endif //###

const struct gpio_chip pca963x_gpio_ops = {
//###	.request	= pca963x_gpio_request, // ### FIXME: Do we need this?
	.get		= pca963x_gpio_get,
	.set		= pca963x_gpio_set,
	.direction_output = pca963x_gpio_direction_output,
	.owner		= THIS_MODULE,
};
#endif

static int pca963x_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct pca963x *pca963x_chip;
	struct pca963x_led *pca963x;
	struct pca963x_platform_data *pdata;
	struct pca963x_chipdef *chip;
	int i, err = 0, reg_failed = 0;
	unsigned int val = 0;

	chip = &pca963x_chipdefs[id->driver_data];
	pdata = dev_get_platdata(&client->dev);

	pca963x_chip = devm_kzalloc(&client->dev, sizeof(*pca963x_chip),
								GFP_KERNEL);
	if (!pca963x_chip)
		return -ENOMEM;
	/* initialize struct */
	if (!pdata)
		pca963x_chip = pca963x_dt_init(client, chip);
	else
		pca963x_chip = pca963x_platform_init(client, pdata, chip);

	if(!pca963x_chip)
		return -EINVAL;

	if (pdata && (pdata->leds.num_leds < 1 ||
				 pdata->leds.num_leds > chip->n_leds)) {
		dev_err(&client->dev, "board info must claim 1-%d LEDs",
								chip->n_leds);
		return -EINVAL;
	}

	pca963x = devm_kzalloc(&client->dev, chip->n_leds * sizeof(*pca963x),
								GFP_KERNEL);
	if (!pca963x)
		return -ENOMEM;

	i2c_set_clientdata(client, pca963x_chip);

	mutex_init(&pca963x_chip->mutex);
	pca963x_chip->chipdef = chip;
	pca963x_chip->client = client;

	/* Set LEDs to default*/
	for (i = 0; i < chip->n_leds; i++) {
		int flags = pca963x_chip->leds[i].flags;
			flags &= PCA963X_FLAGS_ACTIVE_HIGH
						| PCA963X_FLAGS_DEFAULT_ON;
			if ((flags == PCA963X_FLAGS_ACTIVE_HIGH)
				|| (flags == PCA963X_FLAGS_DEFAULT_ON)) {
				val |= PCA963X_LED_ON << (2 * i);
			}
	}
	i2c_smbus_write_byte_data(client, chip->ledout_base, (u8)val);
	if (chip->n_leds > 4)
		i2c_smbus_write_byte_data(client, chip->ledout_base + 1,
					  (u8)(val >> 8));

	for (i = 0; i < chip->n_leds; i++) {
		pca963x_chip->leds[i].led_num = i;
		pca963x_chip->leds[i].chip = pca963x_chip;
		if(pca963x_chip->leds[i].type == LED) {
			INIT_WORK(&pca963x_chip->leds[i].work, pca963x_work);
			err = led_classdev_register(&client->dev,
					&pca963x_chip->leds[i].led_cdev);
			if (err < 0)
				goto exit;
		}
	}
#ifdef CONFIG_PWM
	if(pca963x_chip->n_pwm>0) {
		INIT_WORK(&pca963x_chip->leds[i].work, pca963x_work);
		pca963x_chip->chip.of_pwm_n_cells = 3;
		pca963x_chip->chip.of_xlate = of_pwm_xlate_with_flags;
		pca963x_chip->chip.dev = &client->dev;
		pca963x_chip->chip.ops = &pca963x_pwm_ops;
		pca963x_chip->chip.npwm = 4;
		pca963x_chip->chip.base = -1;
		err = pwmchip_add(&pca963x_chip->chip);
		if(err < 0) {
			dev_warn(&client->dev,"could not register PWM-Chip\n");
			reg_failed++;
		}
	}
#endif

	if(pca963x_chip->n_gpio>0) {
		pca963x_chip->gchip = pca963x_gpio_ops;
		pca963x_chip->gchip.label = client->name;
		pca963x_chip->gchip.ngpio = 4;
		pca963x_chip->gchip.parent = &client->dev;
		pca963x_chip->gchip.base = -1;
		err = gpiochip_add(&pca963x_chip->gchip);
		if(err < 0) {
			dev_warn(&client->dev,
				 "could not register GPIO-Chip\n");
			reg_failed++;
		}
	}

	if(pca963x_chip->n_led == 0 && reg_failed == 2) {
		return err;
	}
	/* Disable LED all-call address and set normal mode */
	i2c_smbus_write_byte_data(client, chip->mode1, 0x00);
	if (pdata || pca963x_chip) {
		/* Configure output: open-drain or totem pole (push-pull) */
		if (pca963x_chip->outdrv == PCA963X_OPEN_DRAIN)
			i2c_smbus_write_byte_data(client, chip->mode2, 0x01);
		else
			i2c_smbus_write_byte_data(client, chip->mode2, 0x05);
	}

	return 0;

exit:
	while(i--) {
		led_classdev_unregister(&pca963x_chip->leds[i].led_cdev);
		cancel_work_sync(&pca963x_chip->leds[i].work);
	}

	return err;
}

static int pca963x_remove(struct i2c_client *client)
{
	struct pca963x *pca963x_chip;
	int i = 0;

	pca963x_chip = i2c_get_clientdata(client);

#ifdef CONFIG_GPIOLIB
	if(pca963x_chip->n_gpio > 0)
		gpiochip_remove(&pca963x_chip->gchip);
#endif
#ifdef CONFIG_PWM
	if(pca963x_chip->n_pwm > 0) {
		pwmchip_remove(&pca963x_chip->chip);
		cancel_work_sync(&pca963x_chip->leds[i].work);
	}
#endif
	for (i = 0; i < pca963x_chip->n_led; i++) {
		if(pca963x_chip->leds[i].type == LED) {
				led_classdev_unregister(
					&pca963x_chip->leds[i].led_cdev);
				cancel_work_sync(&pca963x_chip->leds[i].work);
		}
	}

	return 0;
}

static struct i2c_driver pca963x_driver = {
	.driver = {
		.name	= "leds-pca963x",
		.owner	= THIS_MODULE,
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
