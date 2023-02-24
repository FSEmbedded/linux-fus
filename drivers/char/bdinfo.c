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
#include <linux/ctype.h>
#define GET_CELL(p)	(p += 4, *((const uint32_t *)(p-4)))
#define fdt32_to_cpu(x) be32_to_cpu(x)

struct bdinfo {
	const char **prop_name;
	const char **prop_val;
	int  *prop_length;
	unsigned int acount;
	unsigned int mode;
	struct kobj_attribute *kattr;
	struct attribute_group attr_group;
	struct kobject *bdinfo_kobj;
	struct bdinfo *bdinfo_child;
};

struct bdinfo bdi;

static char *login_tty;

static int __init login_tty_setup(char *arg)
{
	login_tty = arg;

	return 0;
}

__setup("login_tty=", login_tty_setup);

static bool util_is_printable_string(const void *data, int len)
{
	const char *s = data;
	const char *ss, *se;

	/* zero length is not */
	if (len == 0)
		return 0;

	/* must terminate with zero */
	if (s[len - 1] != '\0')
		return 0;

	se = s + len;

	while (s < se) {
		ss = s;
		while (s < se && *s && isprint((unsigned char)*s))
			s++;

		/* not zero, or not done yet */
		if (*s != '\0' || s == ss)
			return 0;

		s++;
	}

	return 1;
}

static ssize_t bdinfo_show(struct kobject *kobj,
				      struct kobj_attribute *attr, char *buf)
{
	int i,j;
	int num = 0;
	struct bdinfo *bdiTmp = &bdi;

	while (kobj != bdiTmp->bdinfo_kobj)
	{
		bdiTmp = bdiTmp->bdinfo_child;
		if (bdiTmp == NULL)
			return sprintf(buf, "%s\n", "Error, subnode not found");
	}

	for (i = 0; i < bdiTmp->acount; i++) {
		if (strcmp(bdiTmp->prop_name[i],attr->attr.name) == 0) {
			break;
		}
	}
	/* If value is a string */
	if (util_is_printable_string(bdiTmp->prop_val[i], bdiTmp->prop_length[i])) {
		return sprintf(buf, "%s\n", bdiTmp->prop_val[i]);
	}
	/* If value is a int32 */
	else if ((bdiTmp->prop_length[i] % 4) == 0) {
		for (j = 0; j < bdiTmp->prop_length[i]; j += 4)
			num += sprintf(buf + num,"0x%08x%s", fdt32_to_cpu(GET_CELL(bdiTmp->prop_val[i])),
			       j < (bdiTmp->prop_length[i] - 4) ? " " : "");
	}
	else {
		for (j = 0; j < bdiTmp->prop_length[i]; j++)
			num += sprintf(buf + num,"%02x%s", *bdiTmp->prop_val[i]++, j < bdiTmp->prop_length[i] - 1 ? " " : "");
	}

	return num;
}

static void bdinfo_free(void)
{
	struct bdinfo *bdiTmp = &bdi;
	struct bdinfo *bdiTmp_next = NULL;

	while(bdiTmp != NULL)
	{
		if (bdiTmp->attr_group.attrs) {
			kfree(bdiTmp->attr_group.attrs);
			bdiTmp->attr_group.attrs = NULL;
		}

		if (bdiTmp->kattr) {
			kfree(bdiTmp->kattr);
			bdiTmp->kattr = NULL;
		}

		bdiTmp_next = bdiTmp->bdinfo_child;
		bdiTmp->bdinfo_child = NULL;
		bdiTmp = bdiTmp_next;
	}
}

static int bdinfo_alloc(struct platform_device *pdev, struct bdinfo *pbdi)
{
	struct kobj_attribute *p;
	unsigned int i;

	pbdi->kattr = kzalloc(pbdi->acount * sizeof(struct kobj_attribute),
								GFP_KERNEL);
	if (!pbdi->kattr) {
		dev_err(&pdev->dev, "cannot allocate sysfs attributes!\n");
		return -ENOMEM;
	}

	p = pbdi->kattr;

	for (i = 0; i < pbdi->acount; i++) {
		sysfs_attr_init(&(p->attr));
		p->attr.name = pbdi->prop_name[i];
		p->attr.mode = pbdi->mode;
		p->show = bdinfo_show;
		p->store = NULL;
		p++;
	}

	memset(&pbdi->attr_group, 0, sizeof(struct attribute_group));
	pbdi->attr_group.attrs = kzalloc((pbdi->acount+1) *
				sizeof(struct attribute *), GFP_KERNEL);
	if (!pbdi->attr_group.attrs) {
		dev_err(&pdev->dev, "cannot allocate attrib pointers!\n");
		return -ENOMEM;
	}

	for (i = 0; i < pbdi->acount; i++)
		pbdi->attr_group.attrs[i] = &pbdi->kattr[i].attr;

	pbdi->attr_group.attrs[pbdi->acount] = NULL;

	return 0;
}

static int bdinfo_remove(struct platform_device *pdev)
{
	/* ###TODO: get from pdev */
	struct bdinfo *bdiTmp = &bdi;
	while(bdiTmp != NULL)
	{
		if (bdiTmp->bdinfo_kobj) {
			kobject_put(bdiTmp->bdinfo_kobj);
			bdiTmp->bdinfo_kobj = NULL;
		}
		bdiTmp = bdiTmp->bdinfo_child;
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
		bdi.prop_length[i] = len;
	} else
	{
		bdi.prop_val[i] = login_tty;
		bdi.prop_length[i] = strlen(login_tty);
	}
	i++;

	bdi.prop_name[i] = "login_speed";

	if (!to || !*(++to))
		to = "0";

	bdi.prop_val[i] = to;
	bdi.prop_length[i] = strlen(to) + 1;
	return 0;
}

static int dt_init(struct platform_device *pdev, struct bdinfo *pbdi, const char *prefix,
				   struct device_node *of_node)
{
	struct property *prop;
	size_t len = strlen(prefix);
	int i = 0;

	for_each_property_of_node(of_node, prop) {
		if ((!strncmp(prop->name, "name", 4) &&
					!strncmp(prop->value, prefix, len)) ||
					!strncmp(prop->name, "compatible", 10))
			continue;
		pbdi->acount++;
	}

	pbdi->prop_name = devm_kzalloc(&pdev->dev, pbdi->acount *
						sizeof(char *), GFP_KERNEL);

	if (!pbdi->prop_name)
		return -ENOMEM;

	pbdi->prop_val = devm_kzalloc(&pdev->dev, pbdi->acount *
						sizeof(char *), GFP_KERNEL);
	if (!pbdi->prop_val)
		return -ENOMEM;

	pbdi->prop_length = devm_kzalloc(&pdev->dev, pbdi->acount *
						sizeof(int), GFP_KERNEL);
	if (!pbdi->prop_val)
		return -ENOMEM;

	for_each_property_of_node(of_node, prop) {
		if ((!strncmp(prop->name, "name", 4) &&
					!strncmp(prop->value, prefix, len)) ||
					!strncmp(prop->name, "compatible", 10))
			continue;

		pbdi->prop_val[i] = prop->value;
		pbdi->prop_name[i] = prop->name;
		pbdi->prop_length[i] = prop->length;
		i++;
	}
	return i;
}

static int bdinfo_init(struct platform_device *pdev)
{
	const char *prefix = "bdinfo";
	int ret = 0;
	int i = 0;

	bdi.mode = 0400;
	bdi.acount = 0;

	/* fs_linux_version = 1 */
	bdi.acount += 1;
	/* login_speed and login_tty = 2 */
	if (login_tty)
		bdi.acount += 2;

	i = dt_init(pdev, &bdi, prefix, pdev->dev.of_node);

	bdi.prop_name[i] = "fs_linux_version";
	bdi.prop_val[i] = fs_linux_version;
	bdi.prop_length[i] = strlen(fs_linux_version)+1;
	i++;

	if (login_tty) {
		ret = login_tty_init(pdev, i);
	}

	if (ret)
		return ret;

	ret = bdinfo_alloc(pdev, &bdi);

	/* In case of error free everything that we allocated */
	if (ret)
		return ret;

	bdi.bdinfo_kobj = kobject_create_and_add("bdinfo", NULL);

	if (!bdi.bdinfo_kobj) {
		dev_err(&pdev->dev, "cannot create kobject!\n");
		ret = -ENOMEM;
	} else
		ret = sysfs_create_group(bdi.bdinfo_kobj, &bdi.attr_group);

	bdi.bdinfo_child = NULL;

	return ret;
}

static int bdinfo_subnode_init(struct platform_device *pdev)
{
	int ret = 0;
	struct device_node *child;
	struct bdinfo *bdinfo_tmp = &bdi;

	for_each_child_of_node(pdev->dev.of_node, child)
	{
		bdinfo_tmp->bdinfo_child = devm_kzalloc(&pdev->dev,sizeof(struct bdinfo), GFP_KERNEL);

		if(!bdinfo_tmp->bdinfo_child)
			return -ENOMEM;

		bdinfo_tmp = bdinfo_tmp->bdinfo_child;
		bdinfo_tmp->acount = 0;
		bdinfo_tmp->mode = 0400;

		dt_init(pdev, bdinfo_tmp, child->name, child);
		ret = bdinfo_alloc(pdev, bdinfo_tmp);

		if (ret)
			return ret;

		bdinfo_tmp->bdinfo_kobj = kobject_create_and_add(child->name, bdi.bdinfo_kobj);

		if (!bdi.bdinfo_kobj) {
			dev_err(&pdev->dev, "cannot create kobject!\n");
			ret = -ENOMEM;
		} else
			ret = sysfs_create_group(bdinfo_tmp->bdinfo_kobj, &bdinfo_tmp->attr_group);
	}

	bdinfo_tmp->bdinfo_child = NULL;
	return ret;
}

static int bdinfo_probe(struct platform_device *pdev)
{
	int ret = 0;

	ret = bdinfo_init(pdev);
	if (ret)
		goto exit;

	bdinfo_subnode_init(pdev);
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
