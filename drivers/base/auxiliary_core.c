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

#define TCM_START 		0x007f8000
#define TCM_SIZE		0x7FFF
#define OCRAM_START		0x00910000
#define OCRAM_SIZE		0xEFFF

struct allowed_mem {
	uint32_t start;
	uint32_t size;
	uint32_t end;
};

static struct auxiliary_core {
	struct kobject *auxiliary_core_kobj;
	struct attribute_group attr_group;
	struct platform_device *pdev;
	struct clk *clk;
	struct allowed_mem allowed_tcm;
	struct allowed_mem allowed_ocram;
	struct allowed_mem allowed_dram;
	unsigned int acount;
	unsigned int mem_addr;
	char *path;
} ac;

/* For the bootaux command we implemented a state machine to switch between
 * the different modes. Below you find the bit coding for the state machine.
 * The different states will be set in the arch_auxiliary_core_set function and
 * to get the current state you can use the function arch_auxiliary_core_get.
 * If we get a undefined state we will immediately set the state to aux_off.
 */
/******************************************************************************
****************|              bit coding             | state |****************
-------------------------------------------------------------------------------
****************| assert_reset | m4_clock | m4_enable | state |****************
===============================================================================
****************|       1      |     0    |     0     |    off    |************
****************|       1      |     1    |     1     |  stopped  |************
****************|       0      |     1    |     1     |  running  |************
****************|       0      |     0    |     1     |  paused   |************
****************|       0      |     0    |     0     | undefined |************
****************|       0      |     1    |     0     | undefined |************
****************|       1      |     0    |     1     | undefined |************
****************|       1      |     1    |     0     | undefined |************
*******************************************************************************
**|   transitions    |   state   |            transitions             |********
*******************************************************************************
                      -----------
                      |   OFF   |
                      -----------
          |          |           ^           ^                   ^
          |    Start |           |  off      |                   |
          |          v           |           |                   |
          |           -----------            |                   |
    run/  |           | Stopped |            | off               |
    addr  |           -----------            |                   |
          |          |           ^           |           ^       |
          | run/addr |           | stop      |           |       | off
          v          v           |                       |       |
                      -----------                        |       |
                      | Running |                        | stop  |
                      -----------                        |       |
                     |           ^                       |       |
               pause |           | continue              |       |
                     v           | run/addr (restart)    |       |
                      -----------
                      | Paused  |
                      -----------
******************************************************************************/
enum aux_state {
	aux_off,
	aux_stopped,
	aux_running,
	aux_paused,
	aux_undefined,
};

int arch_auxiliary_core_set_reset_address(u32 boot_private_data)
{
	void __iomem *tcm_base_addr = NULL;
	void __iomem *mem_base_addr = NULL;

	if (!boot_private_data)
		return 1;

	if (boot_private_data != M4_BOOTROM_BASE_ADDR) {
		/* FIXME: we are mapping a register were mutual
		* exclusion of the register not be guaranteed
		*/
		tcm_base_addr = ioremap(M4_BOOTROM_BASE_ADDR, SZ_8);
		mem_base_addr = ioremap(boot_private_data, SZ_8);
		memcpy(tcm_base_addr, mem_base_addr, 8);

		iounmap(tcm_base_addr);
		iounmap(mem_base_addr);
	}

	return 0;
}

void arch_auxiliary_core_set(u32 core_id, enum aux_state old_state, enum aux_state new_state)
{
	/* FIXME: we are mapping a register were mutual exclusion of the
	 *        register not be guaranteed
	 */
	void __iomem *src_base_addr = ioremap(MX6Q_SRC_BASE_ADDR, SZ_4);
	u32 val = 0;

	if ((new_state == aux_off || new_state == aux_stopped) && old_state != aux_off && old_state != aux_stopped) {
		/* Assert SW reset, i.e. stop M4 if running */
		val = readl_relaxed(src_base_addr);
		val |= ASSERT_RESET;
		writel_relaxed(val, src_base_addr);
	}

	if (new_state == aux_off && old_state != aux_off) {
		/* Disable M4 */
		val = readl_relaxed(src_base_addr);
		val &= ~ENABLE_M4_CORE;
		writel_relaxed(val, src_base_addr);
	}

	if ((new_state == aux_off || new_state == aux_paused) && old_state != aux_off && old_state != aux_paused) {
		/* Disable M4 clock */
		if (__clk_is_enabled(ac.clk))
			clk_disable_unprepare(ac.clk);
	}

	if ((new_state == aux_stopped || new_state == aux_running) && old_state != aux_stopped && old_state != aux_running) {
		/* Enable M4 clock */
		clk_prepare_enable(ac.clk);
	}

	if (!(new_state == aux_off) && !(old_state != aux_off)) {
		/* Enable M4 */
		val = readl_relaxed(src_base_addr);
		val |= ENABLE_M4_CORE;
		writel_relaxed(val, src_base_addr);
	}

	if ((new_state == aux_running || new_state == aux_paused) && old_state != aux_running && old_state != aux_paused) {
		/* Assert SW reset, i.e. stop M4 if running */
		val = readl_relaxed(src_base_addr);
		val &= ~ASSERT_RESET;
		writel_relaxed(val, src_base_addr);
	}

	iounmap(src_base_addr);
}

enum aux_state arch_auxiliary_core_get(u32 core_id)
{
	/* FIXME: we are mapping a register were mutual exclusion of the
	 *        register not be guaranteed
	 */
	void __iomem *src_base_addr = ioremap(MX6Q_SRC_BASE_ADDR, SZ_4);
	u32 val = 0;
	int flags = 0;

	val = readl_relaxed(src_base_addr);
	if (val & ASSERT_RESET)
		flags |= 0x4;
	if (val & ENABLE_M4_CORE)
		flags |= 0x1;
	iounmap(src_base_addr);

	if(__clk_is_enabled(ac.clk))
		flags |= 0x2;

	switch (flags)
	{
		case 0x4:
			return aux_off;
		case 0x7:
			return aux_stopped;
		case 0x3:
			return aux_running;
		case 0x1:
			return aux_paused;
	}

	return aux_undefined;
}

static ssize_t bootaux_show(struct kobject *kobj,
				      struct kobj_attribute *attr, char *buf)
{
	const char *state_name[] = {"off", "stopped", "running", "paused"};
	enum aux_state old_state;
	enum aux_state new_state;

	old_state = arch_auxiliary_core_get(0);
	if (old_state == aux_undefined) {
		new_state = aux_off;
		arch_auxiliary_core_set(0, old_state, new_state);
		new_state = arch_auxiliary_core_get(0);
	}
	else
		new_state = old_state;

	/* Print auxiliary core state */
	return sprintf(buf, "auxiliary core %s\n", state_name[new_state]);
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
	const char *state_name[] = {"off", "stopped", "running", "paused"};
	unsigned int addr = 0;
	int ret = 0, len = 0;
	char *value;
	enum aux_state old_state;
	enum aux_state new_state;

	/* if \n is at the end of the string then replace it through \0 */
	len  = strnlen(buf, MAX_INPUT);
	if (len && buf[len - 1] != '\n')
		len++;
	value = kmalloc(len, GFP_KERNEL);
	memcpy(value, buf, len - 1);
	value[len - 1] = '\0';

	old_state = arch_auxiliary_core_get(0);
	if (old_state == aux_undefined) {
		new_state = aux_off;
		arch_auxiliary_core_set(0, old_state, new_state);
		old_state = new_state;
	}

	if (!strcmp(value, "start") && (old_state == aux_off))
		new_state = aux_stopped;
	else if (!strcmp(value, "stop"))
		new_state = aux_stopped;
	else if (!strcmp(value, "pause") && (old_state == aux_running))
		new_state = aux_paused;
	else if (!strcmp(value, "continue") && (old_state == aux_paused))
		new_state = aux_running;
	else if (!strcmp(value, "off"))
		new_state = aux_off;
	else if (!strcmp(value, "run") || (buf[0] >= '0' && buf[0] <= '9'))
	{
		if(!strcmp(value, "run"))
			addr = ac.mem_addr;
		else {
			ret = kstrtouint(buf, 0, &addr);
			if (ret) {
				dev_err(&ac.pdev->dev, "Bad address\n");
				return -EFAULT;
			}
		}
		new_state = aux_stopped;
		arch_auxiliary_core_set(0, old_state, new_state);

		ret = arch_auxiliary_core_set_reset_address(addr);
		if (ret) {
			dev_err(&ac.pdev->dev, "Bad address\n");
			return -EFAULT;
		}
		old_state = new_state;
		new_state = aux_running;
	}
	else {
		dev_err(&ac.pdev->dev, "Command %s unknown or not allowed if"
		            " auxiliary core %s!\n", buf, state_name[old_state]);
		return -EFAULT;
	}

	arch_auxiliary_core_set(0, old_state, new_state);
	kfree(value);
	return count;
}

static int inRange(unsigned low, unsigned high, unsigned x)
{
	return (low <= x && x <= high);
}

void print_valid_adresses(void){
}

static struct allowed_mem* check_mem_addr(unsigned int addr) {

	/* In tcm only 0x7f8000 is allowed as start address*/
	if (addr == ac.allowed_tcm.start)
		return &ac.allowed_tcm;
	if (inRange(ac.allowed_ocram.start, ac.allowed_ocram.end, addr))
		return &ac.allowed_ocram;
	if (inRange(ac.allowed_dram.start, ac.allowed_dram.end, addr))
		return &ac.allowed_dram;
	else {
		dev_err(&ac.pdev->dev,"Invalid address! Valid adresses are:\r\n");
		dev_err(&ac.pdev->dev,"TCM:   0x%x \r\n",ac.allowed_tcm.start);
		dev_err(&ac.pdev->dev,"OCRAM: 0x%x to 0x%x \r\n",ac.allowed_ocram.start, ac.allowed_ocram.end);
		dev_err(&ac.pdev->dev,"DRAM:  0x%x to 0x%x \r\n",ac.allowed_dram.start, ac.allowed_dram.end);
		return NULL;
	}
}

static int check_file_size(unsigned int file_size)
{

	struct allowed_mem* mem_area;

	mem_area = check_mem_addr(ac.mem_addr);

	if ( ac.mem_addr + file_size > mem_area->end)
		return -1;
	else
		return 0;

}

static ssize_t mem_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct file *filep = NULL;
	struct kstat stat;
	void __iomem *mem_base_addr = NULL;
	char *raw_fmts =  NULL;
	mm_segment_t fs = 0;
	int logstrs_size = 0;
	int error = 0;
	int len = 0;

	if(!__clk_is_enabled(ac.clk)) {
		dev_err(&ac.pdev->dev, "auxiliary core clock is not enabled! \n");
		goto fail;
	}

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

	error = vfs_getattr(&filep->f_path, &stat, STATX_BASIC_STATS, AT_STATX_SYNC_AS_STAT);
	if (error) {
		dev_err(&ac.pdev->dev, "Failed to stat file %s \n",  ac.path);
		goto fail;
	}

	logstrs_size = (int) stat.size;

	if (check_file_size(logstrs_size)) {
		dev_err(&ac.pdev->dev, "File size to big for memory! \n");
		goto fail;
	}
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
	if (raw_fmts)
		kfree(raw_fmts);

	if (!IS_ERR(filep) && filep)
		filp_close(filep, NULL);

	if (fs)
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
	if (!check_mem_addr(addr)) {
		return -EINVAL;
	}
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
	/* Set valid memory regions for M4 execution.
	   Dram is set in probe_dts */
	ac.allowed_tcm.start = TCM_START;
	ac.allowed_tcm.size = TCM_SIZE;
	ac.allowed_tcm.end = ac.allowed_tcm.start + ac.allowed_tcm.size;

	ac.allowed_ocram.start = OCRAM_START;
	ac.allowed_ocram.size = OCRAM_SIZE;
	ac.allowed_ocram.end = ac.allowed_ocram.start + ac.allowed_ocram.size;

	/* set number of variables to create */
	ac.acount = sizeof(kattr)/sizeof(struct kobj_attribute);
	ac.clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(ac.clk)) {
		ac.clk = NULL;
		return -EINVAL;
	}

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

static int auxiliary_core_probe_dts(struct platform_device *pdev)
{
	uint32_t dt_value;
	struct device_node *dt_node;

	dt_node = of_find_node_by_path("/reserved-memory");
	dt_node = of_get_child_by_name (dt_node,"by-uboot");
	if (!dt_node){
		dev_warn(&pdev->dev, "Could not find reserved memory node.\n");
		dev_warn(&pdev->dev, "Cannot load RPMSG or DRAM examples!\n");
		ac.allowed_dram.start=0;
		ac.allowed_dram.end=0;
		ac.allowed_dram.size=0;
		return 0;
	}

	of_property_read_u32_index(dt_node, "reg",0, &dt_value);
	ac.allowed_dram.start = dt_value;
	of_property_read_u32_index(dt_node, "reg",1, &dt_value);
	ac.allowed_dram.size = dt_value;
	ac.allowed_dram.end = ac.allowed_dram.start + ac.allowed_dram.size;
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

	ret = auxiliary_core_probe_dts(pdev);
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
