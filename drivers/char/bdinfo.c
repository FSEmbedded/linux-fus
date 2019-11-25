/*
 * F&S board info driver
 *
 * Copyright (C) 2014 F&S Elektronik Systeme GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */
#include <linux/kobject.h>		/* kobject_create_and_add() */
#include <linux/string.h>		/* sprintf() */
#include <linux/sysfs.h>		/* sysfs_create_group() */
#include <linux/module.h>		/* module_init(), module_exit(), ... */
#include <linux/err.h>			/* ENOMEM, ENODEV, EBUSY, ... */
#include <linux/slab.h>			/* kzalloc() */
#include <linux/platform_device.h>	/* struct platform_device */
#include <linux/of.h>			/* __setup() */

struct bdinfo {
	const char **prop_name;
	const char **prop_val;
	unsigned int acount;
	unsigned int mode;
	struct kobj_attribute *kattr;
	struct attribute_group attr_group;
	struct kobject *bdinfo_kobj;
};

struct bdinfo bdi;

static char *login_tty;

static int __init login_tty_setup(char *arg)
{
	login_tty = arg;

	return 0;
}

__setup("login_tty=", login_tty_setup);

static ssize_t bdinfo_show(struct kobject *kobj,
				      struct kobj_attribute *attr, char *buf)
{
	int i;

	for (i = 0; i < bdi.acount; i++) {
		if (strcmp(bdi.prop_name[i],attr->attr.name) == 0) {
			break;
		}
	}

	return sprintf(buf, "%s\n", bdi.prop_val[i]);
}

static void bdinfo_free(void)
{
	if (bdi.attr_group.attrs) {
		kfree(bdi.attr_group.attrs);
		bdi.attr_group.attrs = NULL;
	}

	if (bdi.kattr) {
		kfree(bdi.kattr);
		bdi.kattr = NULL;
	}
}

static int __init bdinfo_alloc(struct platform_device *pdev)
{
	struct kobj_attribute *p;
	unsigned int i;

	bdi.kattr = kzalloc(bdi.acount * sizeof(struct kobj_attribute),
								GFP_KERNEL);
	if (!bdi.kattr) {
		dev_err(&pdev->dev, "cannot allocate sysfs attributes!\n");
		return -ENOMEM;
	}

	p = bdi.kattr;

	for (i = 0; i < bdi.acount; i++) {
		p->attr.name = bdi.prop_name[i];
		p->attr.mode = bdi.mode;
		p->show = bdinfo_show;
		p->store = NULL;
		p++;
	}

	memset(&bdi.attr_group, 0, sizeof(struct attribute_group));
	bdi.attr_group.attrs = kzalloc((bdi.acount+1) *
				sizeof(struct attribute *), GFP_KERNEL);
	if (!bdi.attr_group.attrs) {
		dev_err(&pdev->dev, "cannot allocate attrib pointers!\n");
		return -ENOMEM;
	}

	for (i = 0; i < bdi.acount; i++)
		bdi.attr_group.attrs[i] = &bdi.kattr[i].attr;

	bdi.attr_group.attrs[bdi.acount] = NULL;

	return 0;
}

static int bdinfo_remove(struct platform_device *pdev)
{
	/* ###TODO: get from pdev */
	if (bdi.bdinfo_kobj) {
		kobject_put(bdi.bdinfo_kobj);
		bdi.bdinfo_kobj = NULL;
	}

	bdinfo_free();

	return 0;
}

static int login_tty_init(struct platform_device *pdev, int i)
{
	char *buf;
	char *to = strchr(login_tty, ',');

	bdi.prop_name[i] = "login_tty";

	if (to) {
		ssize_t len = to - login_tty;

		buf = devm_kzalloc(&pdev->dev, len * sizeof(char), GFP_KERNEL);
		if(!buf)
			return -ENOMEM;

		memcpy(buf, login_tty, len);
		buf[len++] = 0;
		bdi.prop_val[i] = buf;
	} else
		bdi.prop_val[i] = login_tty;

	i++;

	bdi.prop_name[i] = "login_speed";

	if (!to || !*(++to))
		to = "0";

	bdi.prop_val[i] = to;

	return 0;
}

static int bdinfo_dt_init(struct platform_device *pdev)
{
	struct property *prop;
	const char *prefix = "bdinfo";
	size_t len = strlen(prefix);
	int i = 0;
	int ret = 0;

	bdi.acount = 0;
	bdi.mode = 0400;

	/* login_speed and login_tty = 2 */
	if (login_tty)
		bdi.acount = 2;

	for_each_property_of_node(pdev->dev.of_node, prop) {
		if ((!strncmp(prop->name, "name", 4) &&
					!strncmp(prop->value, prefix, len)) ||
					!strncmp(prop->name, "compatible", 10))
			continue;
		bdi.acount++;
	}

	bdi.prop_name = devm_kzalloc(&pdev->dev, bdi.acount *
						sizeof(char *), GFP_KERNEL);

	if (!bdi.prop_name)
		return -ENOMEM;

	bdi.prop_val = devm_kzalloc(&pdev->dev, bdi.acount *
						sizeof(char *), GFP_KERNEL);
	if (!bdi.prop_val)
		return -ENOMEM;

	for_each_property_of_node(pdev->dev.of_node, prop) {
		if ((!strncmp(prop->name, "name", 4) &&
					!strncmp(prop->value, prefix, len)) ||
					!strncmp(prop->name, "compatible", 10))
			continue;

		bdi.prop_val[i] = prop->value;
		bdi.prop_name[i] = prop->name;
		i++;
	}

	if (login_tty) {
		ret = login_tty_init(pdev, i);
	}

	return ret;
}

static int bdinfo_probe(struct platform_device *pdev)
{
	int ret = 0;

	ret = bdinfo_dt_init(pdev);

	if (ret)
		goto exit;

	ret = bdinfo_alloc(pdev);

	/* In case of error free everything that we allocated */
	if (ret)
		goto exit;

	bdi.bdinfo_kobj = kobject_create_and_add("bdinfo", NULL);

	if (!bdi.bdinfo_kobj) {
		dev_err(&pdev->dev, "cannot create kobject!\n");
		ret = -ENOMEM;
	} else
		ret = sysfs_create_group(bdi.bdinfo_kobj, &bdi.attr_group);

	if (ret)
		goto exit;

	return 0;

exit:
	bdinfo_remove(pdev);
	return ret;
}

static const struct of_device_id bdinfo_of_match[] = {
	{ .compatible = "bdinfo" },
	{},
};

static struct platform_driver bdinfo_driver = {
	.probe		= bdinfo_probe,
	.remove		= bdinfo_remove,
	.driver 	= {
		.name		= "bdinfo",
		.owner		= THIS_MODULE,
		.of_match_table = of_match_ptr(bdinfo_of_match),
	},
};

module_platform_driver(bdinfo_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Hartmut Keller <keller@fs-net.de>");
MODULE_DESCRIPTION("Board information");
