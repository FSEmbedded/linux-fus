/*
 * F&S auxiliary core driver
 *
 * Copyright (C) 2017 F&S Elektronik Systeme GmbH
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
#include <linux/kobject.h>			/* kobject_create_and_add() */
#include <linux/string.h>			/* sprintf() */
#include <linux/sysfs.h>			/* sysfs_create_group() */
#include <linux/module.h>			/* module calls */
#include <linux/slab.h>				/* kzalloc() */
#include <linux/platform_device.h>		/* struct platform_device */
#include <linux/of.h>				/* __setup() */
#include <asm/io.h>				/* io remap */
#include <linux/fs.h>				/* vfs_*, filp_*, PATH_MAX */
#include <asm/uaccess.h>			/* get_fs, set_fs */
#include <linux/clk.h>				/* devm_clk_get, ... */
#include <linux/clk-provider.h>			/* __clk_is_enabled */
#include <linux/cpu.h>
#include "../../arch/arm/mach-imx/mx6.h"	/* MX6Q_X_BASE_ADDR */


#define CCGR3			0x74
#define ASSERT_RESET		0x10
#define ENABLE_M4_CORE		0x00400000
#define M4_BOOTROM_BASE_ADDR	0x007f8000


struct auxiliary_core {
	struct kobject *auxiliary_core_kobj;
	struct attribute_group attr_group;
	struct platform_device *pdev;
	struct clk *clk;
	unsigned int acount;
	unsigned int mem_addr;
	char *path;
} ac;


static ssize_t bootaux_show(struct kobject *kobj,
				      struct kobj_attribute *attr, char *buf)
{
	/* FIXME: we are mapping a register were mutual exclusion of the
	 *        register not be guaranteed
	 */
	void __iomem *src_base_addr = ioremap(MX6Q_SRC_BASE_ADDR, SZ_4);
	u32 val = 0;

	val = readl_relaxed(src_base_addr);
	iounmap(src_base_addr);

	return sprintf(buf, "Auxiliary clock %sabled, auxiliary core %s\n",
		       __clk_is_enabled(ac.clk) ? "en" : "dis", val &
					ASSERT_RESET ? "stopped" : "running");
}

static ssize_t mem_show(struct kobject *kobj,
				      struct kobj_attribute *attr, char *buf)
{
	if (ac.path == NULL)
		return sprintf(buf, "mem undefined\n");
	else
		return sprintf(buf, "%s\n", ac.path);
}

static ssize_t mem_addr_show(struct kobject *kobj,
				      struct kobj_attribute *attr, char *buf)
{

	return sprintf(buf, "0x%x\n", ac.mem_addr);
}

static ssize_t bootaux_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	void __iomem *src_base_addr = NULL;
	void __iomem *tcm_base_addr = NULL;
	void __iomem *mem_base_addr = NULL;
	unsigned int addr = 0;
	int ret = 0, len = 0;
	char *value;
	u32 val = 0;

	/* if \n is at the end of the string then replace it through \0 */
	len  = strnlen(buf, MAX_INPUT);
	if (len && buf[len - 1] != '\n')
		len++;
	value = kmalloc(len, GFP_KERNEL);
	memcpy(value, buf, len - 1);
	value[len - 1] = '\0';

	/* FIXME: we are mapping a register were mutual exclusion of the
	 *        register not be guaranteed
	 */
	src_base_addr = ioremap(MX6Q_SRC_BASE_ADDR, SZ_4);

	if (!strcmp(value, "pause")) {
		clk_disable_unprepare(ac.clk);
	} else if (!strcmp(value, "stop")) {
		clk_disable_unprepare(ac.clk);
		/* assert reset */
		val = readl_relaxed(src_base_addr);
		val |= ASSERT_RESET;
		writel_relaxed(val, src_base_addr);
		/* disable M4 core */
		val = readl_relaxed(src_base_addr);
		val &= ~ENABLE_M4_CORE;
		writel_relaxed(val, src_base_addr);
	} else if (!strcmp(value, "reset")) {
		/* assert reset */
		val = readl_relaxed(src_base_addr);
		val |= ASSERT_RESET;
		writel_relaxed(val, src_base_addr);
	} else if (!strcmp(value, "start")) {
		clk_prepare_enable(ac.clk);
	} else {
		if (!strcmp(value, "run")) {
			addr = ac.mem_addr;
		} else {
			ret = kstrtouint(buf, 0, &addr);
			if (ret) {
				iounmap(src_base_addr);
				return ret;
			}
		}
		/* read assert sw reset */
		val = readl_relaxed(src_base_addr);
		if (val & ASSERT_RESET) {
			dev_info(&ac.pdev->dev, "Starting auxiliary core at "
							"0x%08x ... \n", addr);
			if (!addr) {
				dev_err(&ac.pdev->dev, "failed wrong "
								"address \n");
				iounmap(src_base_addr);
				return -EFAULT;
			}

			if(!__clk_is_enabled(ac.clk))
				clk_prepare_enable(ac.clk);

			if (addr != M4_BOOTROM_BASE_ADDR) {
				/* FIXME: we are mapping a register were mutual
				 * exclusion of the register not be guaranteed
				 */
				tcm_base_addr = ioremap(M4_BOOTROM_BASE_ADDR,
									SZ_8);
				mem_base_addr = ioremap(addr, SZ_8);
				memcpy(tcm_base_addr, mem_base_addr, 8);
			}

			/* Enable M4 */
			val = readl_relaxed(src_base_addr);
			val |= ENABLE_M4_CORE;
			writel_relaxed(val, src_base_addr);

			/* Clear SW Reset */
			val = readl_relaxed(src_base_addr);
			val &= ~ASSERT_RESET;
			writel_relaxed(val, src_base_addr);
		}
		else {
			dev_info(&ac.pdev->dev, "Auxiliary core is already "
								"up \n");
		}
	}
	iounmap(src_base_addr);
	if (tcm_base_addr != NULL)
		iounmap(tcm_base_addr);
	if (mem_base_addr != NULL)
		iounmap(mem_base_addr);

	return count;
}

static ssize_t mem_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct file *filep = NULL;
	struct kstat stat;
	void __iomem *mem_base_addr = NULL;
	char *raw_fmts =  NULL;
	mm_segment_t fs;
	int logstrs_size = 0;
	int error = 0;
	int len = 0;

	if(!__clk_is_enabled(ac.clk))
		dev_err(&ac.pdev->dev, "auxiliary core clock is not enabled! \n");

	len  = strnlen(buf, PATH_MAX);
	if (len && buf[len - 1] != '\n')
		len++;
	if (ac.path != NULL)
		kfree(ac.path);

	ac.path = kmalloc(len, GFP_KERNEL);
	memcpy(ac.path, buf, len - 1);
	ac.path[len - 1] = '\0';

	fs = get_fs();
	set_fs(KERNEL_DS);

	filep = filp_open(ac.path, O_RDONLY, 0);
	if (IS_ERR(filep)) {
		dev_err(&ac.pdev->dev, "Failed to open the file %s \n",
								ac.path);
		goto fail;
	}

	error = vfs_getattr(&filep->f_path, &stat);
	if (error) {
		dev_err(&ac.pdev->dev, "Failed to stat file %s \n",  ac.path);
		goto fail;
	}

	logstrs_size = (int) stat.size;

	raw_fmts = kmalloc(logstrs_size, GFP_KERNEL);
	if (raw_fmts == NULL) {
		dev_err(&ac.pdev->dev, "Failed to allocate memory \n");
		goto fail;
	}
	/* FIXME: we are mapping a register were mutual exclusion of
	 *        the register not be guaranteed
	 */
	mem_base_addr = ioremap(ac.mem_addr, logstrs_size);
	if (vfs_read(filep, mem_base_addr, logstrs_size,
				&filep->f_pos) != logstrs_size) {
		dev_err(&ac.pdev->dev, "Failed to read file %s \n",
							ac.path);
		iounmap(mem_base_addr);
		goto fail;
	}
	iounmap(mem_base_addr);


fail:
	if (raw_fmts) {
		kfree(raw_fmts);
		raw_fmts = NULL;
	}
	if (!IS_ERR(filep))
		filp_close(filep, NULL);
	set_fs(fs);

	return count;
}

static ssize_t mem_addr_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned int addr;
	int error;

	error = kstrtouint(buf, 0, &addr);
	if (error)
		return -EINVAL;

	ac.mem_addr = addr;

	return count;
}

struct kobj_attribute kattr [] = {
	{
		/* set/show file */
		.attr = {.name = "mem", .mode = 0600},
		.show = mem_show,
		.store = mem_store,
	},
	{
		/* set/show auxiliary core */
		.attr = {.name = "bootaux", .mode = 0600},
		.show = bootaux_show,
		.store = bootaux_store,

	},
	{
		/* set auxiliary core */
		.attr = {.name = "mem_addr", .mode = 0600},
		.show = mem_addr_show,
		.store = mem_addr_store,
	},
};

static int __init auxiliary_core_alloc(struct platform_device *pdev)
{
	unsigned int i;

	memset(&ac.attr_group, 0, sizeof(struct attribute_group));
	ac.attr_group.attrs = kzalloc((ac.acount+1) *
				sizeof(struct attribute *), GFP_KERNEL);
	if (!ac.attr_group.attrs) {
		dev_err(&pdev->dev, "cannot allocate attrib pointers! \n");
		return -ENOMEM;
	}

	for (i = 0; i < ac.acount; i++)
		ac.attr_group.attrs[i] = &kattr[i].attr;

	ac.attr_group.attrs[ac.acount] = NULL;

	return 0;
}

static int auxiliary_core_init(struct platform_device *pdev)
{
	ac.mem_addr = M4_BOOTROM_BASE_ADDR;
	ac.path = NULL;
	ac.pdev = pdev;
	/* set number of variables to create */
	ac.acount = sizeof(kattr)/sizeof(struct kobj_attribute);
	ac.clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(ac.clk)) {
		ac.clk = NULL;
		return -EINVAL;
	}
	clk_prepare_enable(ac.clk);

	return 0;
}

static void auxiliary_core_free(void)
{
	if (ac.attr_group.attrs) {
		kfree(ac.attr_group.attrs);
		ac.attr_group.attrs = NULL;
	}
}

static int auxiliary_core_remove(struct platform_device *pdev)
{
	/* ###TODO: get from pdev */
	if (ac.auxiliary_core_kobj) {
		kobject_put(ac.auxiliary_core_kobj);
		ac.auxiliary_core_kobj = NULL;
	}

	auxiliary_core_free();

	return 0;
}

static int auxiliary_core_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device *cpudev = get_cpu_device(nr_cpu_ids - 1);

	/* initialize */
	ret = auxiliary_core_init(pdev);
	if (ret)
		goto exit;

	ret = auxiliary_core_alloc(pdev);
	/* In case of error free everything that we allocated */
	if (ret)
		goto exit;

	ac.auxiliary_core_kobj = kobject_create_and_add("aux_core",
							cpudev->kobj.parent);
	if (!ac.auxiliary_core_kobj) {
		dev_err(&pdev->dev, "cannot create kobject!\n");
		ret = -ENOMEM;
	} else {
		ret = sysfs_create_group(ac.auxiliary_core_kobj,
							&ac.attr_group);
	}
	if (ret)
		goto exit;

	return 0;

exit:
	auxiliary_core_remove(pdev);
	return ret;
}

static const struct of_device_id auxiliary_core_of_match[] = {
	{ .compatible = "auxiliary-core" },
	{},
};

static struct platform_driver auxiliary_core_driver = {
	.probe		= auxiliary_core_probe,
	.remove		= auxiliary_core_remove,
	.driver 	= {
		.name		= "auxiliary_core",
		.owner		= THIS_MODULE,
		.of_match_table = of_match_ptr(auxiliary_core_of_match),
	},
};

module_platform_driver(auxiliary_core_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Patrick Jakob <jakob@fs-net.de>");
MODULE_DESCRIPTION("use auxiliary core");
