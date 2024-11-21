/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * F&S Elektronik Systeme 2023
 *
 */

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/mfd/tps65185.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

struct tps65185_hwmon {
	struct device *dev;
	struct device *hwmon_dev;
	struct tps65185 *tps65185;
};

static ssize_t tps65185_hwmon_temp_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static SENSOR_DEVICE_ATTR_RO(temp1_input, tps65185_hwmon_temp, 0);

static struct attribute *tps65185_hwmon_attrs[] = {
	&sensor_dev_attr_temp1_input.dev_attr.attr,
	NULL
};
ATTRIBUTE_GROUPS(tps65185_hwmon);

static ssize_t tps65185_hwmon_temp_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	int8_t tmst_value;
	struct tps65185_hwmon *hwmon = dev_get_drvdata(dev);

	ret = tps65185_reg_write(hwmon->tps65185->client,
			 TPS65185_REG_TMST1,
			 TMST1_READ_THERM);
	if(ret)
		return ret;

	ret = tps65185_reg_read(hwmon->tps65185->client,
			      TPS65185_REG_TMST1,
			      &tmst_value);
	if (ret)
		return ret;

	if(!(tmst_value & TMST1_CONV_END))
		return -EINVAL;

	ret = tps65185_reg_read(hwmon->tps65185->client,
			      TPS65185_REG_TMST_VALUE,
			      &tmst_value);
	if(ret)
		return ret;

	return sprintf(buf, "%d\n", (s32)((s8)tmst_value));
}

static int tps65185_hwmon_probe(struct platform_device *pdev)
{
	int ret;
	struct tps65185_hwmon *hwmon;

	hwmon = devm_kzalloc(&pdev->dev, sizeof(*hwmon), GFP_KERNEL);
	if (!hwmon)
		return -ENOMEM;
	hwmon->dev = &pdev->dev;

	hwmon->hwmon_dev = devm_hwmon_device_register_with_groups(hwmon->dev,
							"tps65185_hwmon", hwmon,
							tps65185_hwmon_groups);
	if (IS_ERR(hwmon->hwmon_dev)) {
		ret = PTR_ERR(hwmon->hwmon_dev);
		dev_err(hwmon->dev, "failed to register hwmon for tps65185 : %d\n", ret);

		return ret;
	}

	hwmon->tps65185 = dev_get_drvdata(pdev->dev.parent);
	WARN_ON(IS_ERR_OR_NULL(hwmon->tps65185));
	platform_set_drvdata(pdev, hwmon);

	return 0;
}

static int tps65185_hwmon_remove(struct platform_device *pdev)
{
	// Do Nothing
	return 0;
}

static const struct platform_device_id tps65185_hwmon_id[] = {
	{ "tps65185-hwmon", 0 },
	{ /* sentinel */    },
};

static struct platform_driver tps65185_hwmon_driver = {
	.probe	= tps65185_hwmon_probe,
	.remove	= tps65185_hwmon_remove,
	.id_table = tps65185_hwmon_id,
	.driver	= {
		.name = "tps65185_hwmon",
	},
};

module_platform_driver(tps65185_hwmon_driver);

MODULE_DESCRIPTION("tps65185 hardware monitor driver");
MODULE_AUTHOR("Claudio Senatore <senatore@fs-net.de>");
MODULE_LICENSE("GPL");
