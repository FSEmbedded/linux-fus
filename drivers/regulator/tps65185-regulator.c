/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * F&S Elektronik Systeme 2023
 *
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/mfd/tps65185.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/platform_device.h>

struct tps65185_data {
	int num_regulators;
	struct tps65185 *tps65185;
	struct regulator_dev **rdev;
};

/* array size is 0x40 */
const static int tps65185_vpos_vneg_voltages[] = {
	/* in uV */
	0,
	0,
	0,
	15000000,
	14750000,
	14500000,
	14250000,
	0,
};

/* trigger to power on VGL, VNEG, VGH, VPOS automatically */
static int tps65185_display_enable(struct regulator_dev *rdev)
{
	struct tps65185 *tps65185 = rdev_get_drvdata(rdev);

	gpio_set_value(tps65185->gpio_pmic_pwrup, 1);

	/* TODO: wait for power good
	 * max soft_start time is 50ms!
	 * typical 1ms
	 */
	mdelay(50);

	return 0;
}

static int tps65185_display_disable(struct regulator_dev *rdev)
{
	struct tps65185 *tps65185 = rdev_get_drvdata(rdev);

	gpio_set_value(tps65185->gpio_pmic_pwrup, 0);

	return 0;
}

static int tps65185_display_is_enabled(struct regulator_dev *rdev)
{
	struct tps65185 *tps65185 = rdev_get_drvdata(rdev);

	return !!gpio_get_value(tps65185->gpio_pmic_pwrgood);
}

static int tps65185_v3p3_enable(struct regulator_dev *rdev)
{
	int ret;
	struct tps65185 *tps65185 = rdev_get_drvdata(rdev);
	uint8_t value;

	ret = tps65185_reg_read(tps65185->client, TPS65185_REG_ENABLE, &value);
	if (ret)
		return ret;

	value |= ENABLE_REG_V3P3_EN;

	return tps65185_reg_write(tps65185->client, TPS65185_REG_ENABLE, value);
}

static int tps65185_v3p3_disable(struct regulator_dev *rdev)
{
	int ret;
	struct tps65185 *tps65185 = rdev_get_drvdata(rdev);
	uint8_t value;

	ret = tps65185_reg_read(tps65185->client, TPS65185_REG_ENABLE, &value);
	if (ret)
		return ret;

	value &= ~ENABLE_REG_V3P3_EN;

	return tps65185_reg_write(tps65185->client, TPS65185_REG_ENABLE, value);
}

static int tps65185_v3p3_is_enabled(struct regulator_dev *rdev)
{
	int ret;
	struct tps65185 *tps65185 = rdev_get_drvdata(rdev);
	uint8_t value;

	ret = tps65185_reg_read(tps65185->client, TPS65185_REG_ENABLE, &value);
	if (ret)
		return ret;

	return !!(value & ENABLE_REG_V3P3_EN);
}


static int tps65185_vcom_get_voltage_sel(struct regulator_dev *rdev)
{
	int ret;
	int vcom_setting;
	uint8_t tmp;
	struct tps65185 *tps65185 = rdev_get_drvdata(rdev);

	ret = tps65185_reg_read(tps65185->client, TPS65185_REG_VCOM1, &tmp);
	if (ret)
		return ret;

	vcom_setting = (int)tmp;

	ret = tps65185_reg_read(tps65185->client, TPS65185_REG_VCOM2, &tmp);
	if (ret)
		return ret;
	vcom_setting |= (int)(tmp & 0x1) << 8;
	if (!vcom_setting)
		return -EINVAL;

	return vcom_setting;
}

static int tps65185_vcom_set_voltage_sel(struct regulator_dev *rdev,
				       unsigned selector)
{
	int ret;
	struct tps65185 *tps65185 = rdev_get_drvdata(rdev);

	if (selector > 0x1FF)
		return -EINVAL;

	ret = tps65185_reg_write(tps65185->client, TPS65185_REG_VCOM2, selector >>8);
	if(ret)
		return ret;
	pr_info("%s: Selector=%d\n",__func__, selector);
	return tps65185_reg_write(tps65185->client, TPS65185_REG_VCOM1, selector & 0xFF);
}

/* VCOM = 0V + [(5,110V/ 511) * N]V, N = 0~511 */
static int tps65185_vcom_list_voltage(struct regulator_dev *rdev,
				    unsigned selector)
{
	int vol_uV;

	if (selector > 0x1FF)
		return -EINVAL;

	/* selector represents value in mV/10 */
	vol_uV = selector*10*1000;

	return vol_uV;
}

static int tps65185_vcom_is_enabled(struct regulator_dev *rdev)
{
	int ret;
	struct tps65185 *tps65185 = rdev_get_drvdata(rdev);
	uint8_t value;

	ret = tps65185_reg_read(tps65185->client, TPS65185_REG_ENABLE, &value);
	if (ret)
		return ret;

	return !!(value & ENABLE_REG_VCOM_EN);
};

static int tps65185_vcom_enable(struct regulator_dev *rdev)
{
	int ret;
	struct tps65185 *tps65185 = rdev_get_drvdata(rdev);
	uint8_t value;

	ret = tps65185_reg_read(tps65185->client, TPS65185_REG_ENABLE, &value);
	if (ret)
		return ret;

	value |= ENABLE_REG_VCOM_EN;

	return tps65185_reg_write(tps65185->client, TPS65185_REG_ENABLE, value);
}

static inline int tps65185_vcom_disable(struct regulator_dev *rdev)
{
	int ret;
	struct tps65185 *tps65185 = rdev_get_drvdata(rdev);
	uint8_t value;

	ret = tps65185_reg_read(tps65185->client, TPS65185_REG_ENABLE, &value);
	if (ret)
		return ret;

	value &= ~ENABLE_REG_VCOM_EN;

	return tps65185_reg_write(tps65185->client, TPS65185_REG_ENABLE, value);
	ret = tps65185_reg_read(tps65185->client, TPS65185_REG_ENABLE, &value);
}

static int tps65185_vpos_vneg_get_voltage_sel(struct regulator_dev *rdev)
{
	int ret;
	uint8_t vpos_vneg_setting;
	struct tps65185 *tps65185 = rdev_get_drvdata(rdev);

	ret = tps65185_reg_read(tps65185->client, TPS65185_REG_VADJ,
			      &vpos_vneg_setting);
	if (ret)
		return ret;

	return FIELD_GET(VPOS_VNEG_SETTING, vpos_vneg_setting);
}

static int tps65185_vpos_vneg_set_voltage_sel(struct regulator_dev *rdev,
					    unsigned selector)
{
	u8 vpos_vneg_setting;
	struct tps65185 *tps65185 = rdev_get_drvdata(rdev);

	if (unlikely(selector > VPOS_VNEG_SETTING))
		return -EINVAL;

	vpos_vneg_setting = FIELD_PREP(VPOS_VNEG_SETTING, selector);

	return tps65185_reg_write(tps65185->client,
				TPS65185_REG_VADJ, vpos_vneg_setting);
}

static struct regulator_ops tps65185_display_ops = {
	.enable			= tps65185_display_enable,
	.disable		= tps65185_display_disable,
	.is_enabled		= tps65185_display_is_enabled,
};

static struct regulator_ops tps65185_v3p3_ops = {
	.enable			= tps65185_v3p3_enable,
	.disable		= tps65185_v3p3_disable,
	.is_enabled		= tps65185_v3p3_is_enabled,
};

static struct regulator_ops tps65185_vcom_ops = {
	.enable				= tps65185_vcom_enable,
	.disable			= tps65185_vcom_disable,
	.is_enabled			= tps65185_vcom_is_enabled,
	.get_voltage_sel	= tps65185_vcom_get_voltage_sel,
	.set_voltage_sel	= tps65185_vcom_set_voltage_sel,
	.list_voltage		= tps65185_vcom_list_voltage,
};

static struct regulator_ops tps65185_vpos_ops = {
	.get_voltage_sel	= tps65185_vpos_vneg_get_voltage_sel,
	.set_voltage_sel	= tps65185_vpos_vneg_set_voltage_sel,
	.list_voltage		= regulator_list_voltage_table,
};

static struct regulator_ops tps65185_vneg_ops = {
	.get_voltage_sel	= tps65185_vpos_vneg_get_voltage_sel,
	.set_voltage_sel	= tps65185_vpos_vneg_set_voltage_sel,
	.list_voltage		= regulator_list_voltage_table,
};

static struct regulator_ops tps65185_vgh_ops = {
};

static struct regulator_ops tps65185_vgl_ops = {
};

static struct regulator_desc tps65185_reg[] = {
	{
		.name		= "DISPLAY",
		.id		= tps65185_DISPLAY,
		.ops		= &tps65185_display_ops,
		.type		= REGULATOR_VOLTAGE,
		.owner		= THIS_MODULE,
	},
	{
		.name		= "VPOS-LDO",
		.id		= tps65185_VPOS,
		.ops		= &tps65185_vpos_ops,
		.type		= REGULATOR_VOLTAGE,
		.n_voltages	= ARRAY_SIZE(tps65185_vpos_vneg_voltages),
		.volt_table	= tps65185_vpos_vneg_voltages,
		.owner		= THIS_MODULE,
	},
	{
		.name		= "VNEG-LDO",
		.id		= tps65185_VNEG,
		.ops		= &tps65185_vneg_ops,
		.type		= REGULATOR_VOLTAGE,
		.n_voltages	= ARRAY_SIZE(tps65185_vpos_vneg_voltages),
		.volt_table	= tps65185_vpos_vneg_voltages,
		.owner		= THIS_MODULE,
	},
	{
		.name		= "VGH-CHARGE-PUMP",
		.id		= tps65185_VGH,
		.ops		= &tps65185_vgh_ops,
		.type		= REGULATOR_VOLTAGE,
		.owner		= THIS_MODULE,
	},
	{
		.name		= "VGL-CHARGE-PUMP",
		.id		= tps65185_VGL,
		.ops		= &tps65185_vgl_ops,
		.type		= REGULATOR_VOLTAGE,
		.owner		= THIS_MODULE,
	},
	{
		.name		= "VCOM",
		.id		= tps65185_VCOM,
		.ops		= &tps65185_vcom_ops,
		.type		= REGULATOR_VOLTAGE,
		.n_voltages	= 512,
		.owner		= THIS_MODULE,
	},
	{
		.name		= "V3P3",
		.id		= tps65185_V3P3,
		.ops		= &tps65185_v3p3_ops,
		.type		= REGULATOR_VOLTAGE,
		.owner		= THIS_MODULE,
	},
};

static int tps65185_pmic_dt_parse_pdata(struct platform_device *pdev,
				      struct tps65185_platform_data *pdata)
{
	struct tps65185 *tps65185 = dev_get_drvdata(pdev->dev.parent);
	struct device_node *pmic_np, *regulators_np, *reg_np;
	struct tps65185_regulator_data *rdata;
	int i;

	pmic_np = of_node_get(tps65185->dev->of_node);
	if (!pmic_np) {
		dev_err(&pdev->dev, "could not find pmic sub-node\n");
		return -ENODEV;
	}

	regulators_np = of_find_node_by_name(pmic_np, "regulators");
	if (!regulators_np) {
		dev_err(&pdev->dev, "could not find regulators sub-node\n");
		of_node_put(pmic_np);
		return -EINVAL;
	}

	pdata->num_regulators = of_get_child_count(regulators_np);

	rdata = devm_kzalloc(&pdev->dev,
			     sizeof(*rdata) * pdata->num_regulators,
			     GFP_KERNEL);
	if (!rdata) {
		dev_err(&pdev->dev, "failed to allocate memory for regulator data\n");
		of_node_put(regulators_np);
		return -ENOMEM;
	}

	pdata->regulators = rdata;
	for_each_child_of_node(regulators_np, reg_np) {
		for (i = 0; i < ARRAY_SIZE(tps65185_reg); i++)
			if (!of_node_cmp(reg_np->name, tps65185_reg[i].name))
				break;

		if (i == ARRAY_SIZE(tps65185_reg)) {
			dev_warn(&pdev->dev, "unknown regulator %s\n", reg_np->name);
			continue;
		}

		rdata->id = i;
		rdata->initdata = of_get_regulator_init_data(&pdev->dev,
							     reg_np,
							     &tps65185_reg[i]);
		if (!rdata->initdata) {
			dev_err(&pdev->dev, "failed to get regulator init data\n");
			return -ENOMEM;
		}

		rdata->reg_node = reg_np;
		rdata++;
	}
	of_node_put(regulators_np);

	return 0;
}

static int tps65185_regulator_probe(struct platform_device *pdev)
{
	struct tps65185 *tps65185 = dev_get_drvdata(pdev->dev.parent);
	struct tps65185_platform_data *pdata = tps65185->pdata;
	struct tps65185_data *priv;
	struct regulator_dev **rdev;
	struct regulator_config config = { };
	int size, i, ret = 0;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	ret = tps65185_pmic_dt_parse_pdata(pdev, pdata);
	if (ret)
		return ret;

	size = sizeof(*rdev) * pdata->num_regulators;
	rdev = devm_kzalloc(&pdev->dev, size, GFP_KERNEL);
	if (!rdev)
		return -ENOMEM;

	priv->rdev = rdev;
	priv->num_regulators = pdata->num_regulators;
	platform_set_drvdata(pdev, priv);

	for (i = 0; i < pdata->num_regulators; i++) {
		int id = pdata->regulators[i].id;

		config.dev = &pdev->dev;
		config.init_data = pdata->regulators[i].initdata;
		config.driver_data = tps65185;
		config.of_node = pdata->regulators[i].reg_node;

		rdev[i] = devm_regulator_register(&pdev->dev, &tps65185_reg[id],
						  &config);
		if (IS_ERR(rdev[i])) {
			dev_err(&pdev->dev, "register regulator %s failed\n",
				tps65185_reg[id].name);
			return PTR_ERR(rdev[i]);
		}
	}

	return 0;
}

static int tps65185_regulator_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct platform_device_id tps65185_pmic_id[] = {
	{ "tps65185-pmic", 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(platform, tps65185_pmic_id);

static struct platform_driver tps65185_regulator_driver = {
	.probe  = tps65185_regulator_probe,
	.remove = tps65185_regulator_remove,
	.id_table = tps65185_pmic_id,
	.driver = {
		.name = "tps65185-pmic",
	},
};

static int __init tps65185_regulator_init(void)
{
	return platform_driver_register(&tps65185_regulator_driver);
}
subsys_initcall_sync(tps65185_regulator_init);

static void __exit tps65185_regulator_exit(void)
{
	platform_driver_unregister(&tps65185_regulator_driver);
}
module_exit(tps65185_regulator_exit);

MODULE_DESCRIPTION("tps65185 regulator driver");
MODULE_AUTHOR("Claudio Senatore <senatore@fs-net.de>");
MODULE_LICENSE("GPL");
