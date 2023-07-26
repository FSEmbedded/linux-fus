/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * F&S Elektronik Systeme 2023
 *
 */

/* TODO: Interrupt handling */

#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/mfd/core.h>
#include <linux/mfd/tps65185.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/gpio.h>


static const unsigned short normal_i2c[] = {
	0x68, I2C_CLIENT_END
};

static struct mfd_cell tps65185_devs[] = {
	{ .name = "tps65185-pmic",  },
	{ .name = "tps65185-hwmon", },
};

int tps65185_reg_read(struct i2c_client *client, int reg, uint8_t *val)
{
	int ret;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) {
		dev_err(&client->dev, "Unable to read reg %d : %d\n",
			reg, ret);
		return ret;
	}

	*val = ret;

	return 0;
}
EXPORT_SYMBOL(tps65185_reg_read);

int tps65185_reg_write(struct i2c_client *client, int reg, uint8_t val)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret < 0)
		dev_err(&client->dev, "Unable to write to reg %d with %u : %d\n",
			reg, val, ret);

	return ret;
}
EXPORT_SYMBOL(tps65185_reg_write);

static int tps65185_i2c_detect(struct i2c_client *client,
			 struct i2c_board_info *info)
{
	int ret;
    uint8_t revid;
	const char * revid_str = 0;
	struct i2c_adapter *adapter = client->adapter;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -ENODEV;

	ret = tps65185_reg_read(client, TPS65185_REG_REVID, &revid);
	if (ret)
		return ret;

    switch(revid)
    {
        case TPS65185_1P0:
            revid_str = "TPS65185_1P0";
			ret = 0;
            break;
        case TPS65185_1P1:
            revid_str = "TPS65185_1P1";
			ret = 0;
            break;
        case TPS65185_1P2:
            revid_str = "TPS65185_1P2";
			ret = 0;
            break;
        case TPS651851_1P0:
            revid_str = "TPS651851_1P0";
			ret = 0;
            break;
        default:
            revid_str = "UNKNOWN";
            ret = -1;
    }

    dev_info(&client->dev,"detected PMIC is: %s\n", revid_str);
	return ret;
}

static int tps65185_i2c_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int ret;
	struct tps65185 *tps65185;
	struct device *dev = &client->dev;

	tps65185 = devm_kzalloc(dev, sizeof(*tps65185), GFP_KERNEL);
	if (!tps65185)
		return -ENOMEM;

	tps65185->dev = dev;
	tps65185->client = client;

	i2c_set_clientdata(client, tps65185);

	/* Init GPIO-PINS */
	/* WAKEUP */
	tps65185->gpio_pmic_wakeup = of_get_named_gpio(tps65185->dev->of_node,
					"gpio-pmic-wakeup", 0);
	if (!gpio_is_valid(tps65185->gpio_pmic_wakeup)) {
		dev_err(tps65185->dev, "no epdc pmic wakeup pin available\n");
		return tps65185->gpio_pmic_wakeup;
	}
	ret = devm_gpio_request_one(tps65185->dev, tps65185->gpio_pmic_wakeup,
				    GPIOF_OUT_INIT_LOW, "epdc-pmic-wake");
	if (ret < 0)
		return ret;

	/* POWERGOOD */
	tps65185->gpio_pmic_pwrgood = of_get_named_gpio(tps65185->dev->of_node,
					"gpio-pmic-pwrgood", 0);
	if (!gpio_is_valid(tps65185->gpio_pmic_pwrgood)) {
		dev_err(tps65185->dev, "no epdc pmic pwrgood pin available\n");
		return tps65185->gpio_pmic_pwrgood;
	}
	ret = devm_gpio_request_one(tps65185->dev, tps65185->gpio_pmic_pwrgood,
				     GPIOF_IN, "epdc-pwrstat");
	if(ret < 0)
		return ret;

	/* POWERUP */
	tps65185->gpio_pmic_pwrup = of_get_named_gpio(tps65185->dev->of_node,
					"gpio-pmic-pwrup", 0);
	if (!gpio_is_valid(tps65185->gpio_pmic_pwrup)) {
		dev_err(tps65185->dev, "no epdc pmic pwrup pin available\n");
		return tps65185->gpio_pmic_pwrup;
	}
	ret = devm_gpio_request_one(tps65185->dev, tps65185->gpio_pmic_pwrup,
				     GPIOF_OUT_INIT_LOW, "epdc-pwrup");
	if(ret < 0)
		return ret;

	/* call detect here, since the .detect function of
	 * i2c_driver won't be called if the driver's class
	 * and adapter's class do not match.
	 */
	gpio_set_value(tps65185->gpio_pmic_wakeup, 1);
	mdelay(5);
	ret = tps65185_i2c_detect(client, NULL);
	if (ret)
		return ret;

	tps65185->pdata = devm_kzalloc(dev, sizeof(*tps65185->pdata), GFP_KERNEL);
	if (!tps65185->pdata)
		return -ENOMEM;

	ret = devm_mfd_add_devices(dev, PLATFORM_DEVID_NONE, tps65185_devs,
				   ARRAY_SIZE(tps65185_devs), NULL, 0, NULL);
	if (ret) {
		dev_err(dev, "Failed to add pmic subdevices: %d\n", ret);
		return ret;
	}

	return 0;
}

static int tps65185_i2c_remove(struct i2c_client *i2c)
{
	struct tps65185 *tps65185 = dev_get_drvdata(&i2c->dev);
	/* set PMIC in SLEEP state */
	gpio_set_value(tps65185->gpio_pmic_wakeup, 0);
	return 0;
}

static const struct i2c_device_id tps65185_id[] = {
	{ "tps65185", 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, tps65185_id);

static const struct of_device_id tps65185_dt_ids[] = {
	{
		.compatible = "ti,tps65185",
	},
	{	/* sentinel */	}
};
MODULE_DEVICE_TABLE(of, tps65185_dt_ids);

static struct i2c_driver tps65185_driver = {
	.driver = {
		.name  = "tps65185",
		.owner = THIS_MODULE,
		.of_match_table = tps65185_dt_ids,
	},
	.probe  = tps65185_i2c_probe,
	.remove = tps65185_i2c_remove,
	.id_table = tps65185_id,
	.detect = tps65185_i2c_detect,
	.address_list = normal_i2c,
};

static int __init tps65185_init(void)
{
	return i2c_add_driver(&tps65185_driver);
}

static void __init tps65185_exit(void)
{
	i2c_del_driver(&tps65185_driver);
}

subsys_initcall(tps65185_init);
module_exit(tps65185_exit);

MODULE_DESCRIPTION("TPS65185 PMIC driver");
MODULE_AUTHOR("Claudio Senatore <senatore@fs-net.de>");
MODULE_LICENSE("GPL");
