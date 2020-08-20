/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2011-2014 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2019 NXP
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdesc.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/console.h>
#include <linux/io.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/fsl_devices.h>
#include <linux/slab.h>
#include <linux/of.h>

#include <linux/mipi_csi2.h>

#include "mxc_mipi_csi2.h"

static struct mipi_csi2_info *gmipi_csi2;

void _mipi_csi2_lock(struct mipi_csi2_info *info)
{
	if (!in_irq() && !in_softirq())
		mutex_lock(&info->mutex_lock);
}

void _mipi_csi2_unlock(struct mipi_csi2_info *info)
{
	if (!in_irq() && !in_softirq())
		mutex_unlock(&info->mutex_lock);
}

static inline void mipi_csi2_write(struct mipi_csi2_info *info,
		unsigned value, unsigned offset)
{
	writel(value, info->mipi_csi2_base + offset);
}

static inline unsigned int mipi_csi2_read(struct mipi_csi2_info *info,
		unsigned offset)
{
	return readl(info->mipi_csi2_base + offset);
}

/*!
 * This function is called to enable the mipi csi2 interface.
 *
 * @param	info		mipi csi2 hander
 * @return      Returns setted value
 */
bool mipi_csi2_enable(struct mipi_csi2_info *info)
{
	bool status;

	_mipi_csi2_lock(info);

	if (!info->mipi_en) {
		info->mipi_en = true;
		clk_prepare_enable(info->cfg_clk);
		clk_prepare_enable(info->dphy_clk);
	} else
		mipi_dbg("mipi csi2 already enabled!\n");

	status = info->mipi_en;

	_mipi_csi2_unlock(info);

	return status;
}
EXPORT_SYMBOL(mipi_csi2_enable);

/*!
 * This function is called to disable the mipi csi2 interface.
 *
 * @param	info		mipi csi2 hander
 * @return      Returns setted value
 */
bool mipi_csi2_disable(struct mipi_csi2_info *info)
{
	bool status;

	_mipi_csi2_lock(info);

	if (info->mipi_en) {
		info->mipi_en = false;
		clk_disable_unprepare(info->dphy_clk);
		clk_disable_unprepare(info->cfg_clk);
	} else
		mipi_dbg("mipi csi2 already disabled!\n");

	status = info->mipi_en;

	_mipi_csi2_unlock(info);

	return status;
}
EXPORT_SYMBOL(mipi_csi2_disable);

/*!
 * This function is called to get mipi csi2 disable/enable status.
 *
 * @param	info		mipi csi2 hander
 * @return      Returns mipi csi2 status
 */
bool mipi_csi2_get_status(struct mipi_csi2_info *info)
{
	bool status;

	_mipi_csi2_lock(info);
	status = info->mipi_en;
	_mipi_csi2_unlock(info);

	return status;
}
EXPORT_SYMBOL(mipi_csi2_get_status);

/*!
 * This function is called to set mipi lanes.
 *
 * @param	info		mipi csi2 hander
 * @return      Returns setted value
 */
unsigned int mipi_csi2_set_lanes(struct mipi_csi2_info *info)
{
	unsigned int lanes;

	_mipi_csi2_lock(info);
	mipi_csi2_write(info, info->lanes - 1, MIPI_CSI2_N_LANES);
	lanes = mipi_csi2_read(info, MIPI_CSI2_N_LANES);
	_mipi_csi2_unlock(info);

	return lanes;
}
EXPORT_SYMBOL(mipi_csi2_set_lanes);

/*!
 * This function is called to set mipi data type.
 *
 * @param	info		mipi csi2 hander
 * @return      Returns setted value
 */
unsigned int mipi_csi2_set_datatype(struct mipi_csi2_info *info,
					unsigned int v_channel, unsigned int datatype)
{
	unsigned int dtype;

	if (v_channel >= MAX_VIRTUAL_CHAN_NUM)
		return 0;

	_mipi_csi2_lock(info);
	info->mipi_chan[v_channel].datatype = datatype;
	dtype = info->mipi_chan[v_channel].datatype;
	_mipi_csi2_unlock(info);

	return dtype;
}
EXPORT_SYMBOL(mipi_csi2_set_datatype);

/*!
 * This function is called to get mipi data type.
 *
 * @param	info		mipi csi2 hander
 * @return      Returns mipi data type
 */
unsigned int mipi_csi2_get_datatype(struct mipi_csi2_info *info, 
					unsigned int v_channel)
{
	unsigned int dtype;

	if (v_channel >= MAX_VIRTUAL_CHAN_NUM)
		return 0;

	_mipi_csi2_lock(info);
	dtype = info->mipi_chan[v_channel].datatype;
	_mipi_csi2_unlock(info);

	return dtype;
}
EXPORT_SYMBOL(mipi_csi2_get_datatype);

/*!
 * This function is called to get mipi csi2 dphy status.
 *
 * @param	info		mipi csi2 hander
 * @return      Returns dphy status
 */
unsigned int mipi_csi2_dphy_status(struct mipi_csi2_info *info)
{
	unsigned int status;

	_mipi_csi2_lock(info);
	status = mipi_csi2_read(info, MIPI_CSI2_PHY_STATE);
	_mipi_csi2_unlock(info);

	return status;
}
EXPORT_SYMBOL(mipi_csi2_dphy_status);

/*!
 * This function is called to get mipi csi2 error1 status.
 *
 * @param	info		mipi csi2 hander
 * @return      Returns error1 value
 */
unsigned int mipi_csi2_get_error1(struct mipi_csi2_info *info)
{
	unsigned int err1;

	_mipi_csi2_lock(info);
	err1 = mipi_csi2_read(info, MIPI_CSI2_ERR1);
	_mipi_csi2_unlock(info);

	return err1;
}
EXPORT_SYMBOL(mipi_csi2_get_error1);

/*!
 * This function is called to get mipi csi2 error1 status.
 *
 * @param	info		mipi csi2 hander
 * @return      Returns error1 value
 */
unsigned int mipi_csi2_get_error2(struct mipi_csi2_info *info)
{
	unsigned int err2;

	_mipi_csi2_lock(info);
	err2 = mipi_csi2_read(info, MIPI_CSI2_ERR2);
	_mipi_csi2_unlock(info);

	return err2;
}
EXPORT_SYMBOL(mipi_csi2_get_error2);

/*!
 * This function is called to enable mipi to ipu pixel clock.
 *
 * @param	info		mipi csi2 hander
 * @return      Returns 0 on success or negative error code on fail
 */
int mipi_csi2_pixelclk_enable(struct mipi_csi2_info *info)
{
	return clk_prepare_enable(info->pixel_clk);
}
EXPORT_SYMBOL(mipi_csi2_pixelclk_enable);

/*!
 * This function is called to disable mipi to ipu pixel clock.
 *
 * @param	info		mipi csi2 hander
 * @return      Returns 0 on success or negative error code on fail
 */
void mipi_csi2_pixelclk_disable(struct mipi_csi2_info *info)
{
	clk_disable_unprepare(info->pixel_clk);
}
EXPORT_SYMBOL(mipi_csi2_pixelclk_disable);

/*!
 * This function is called to power on mipi csi2.
 *
 * @param	info		mipi csi2 hander
 *           	mipi_lane_bps	mipi csi2 data rate for each data lane, Mbps.
 *                    It should be MIPI clock * 2 for DDR mode mipi clock.
 * @return      Returns 0 on success or negative error code on fail
 */
int mipi_csi2_reset(struct mipi_csi2_info *info, int mipi_lane_bps)
{
	int value;
	
	dev_info(&info->pdev->dev, "mipi_csi2_reset: mipi_lane_bps = %d Mbps\n", mipi_lane_bps);
	_mipi_csi2_lock(info);

	mipi_csi2_write(info, 0x0, MIPI_CSI2_PHY_SHUTDOWNZ);
	mipi_csi2_write(info, 0x0, MIPI_CSI2_DPHY_RSTZ);
	mipi_csi2_write(info, 0x0, MIPI_CSI2_CSI2_RESETN);

	mipi_csi2_write(info, 0x00000001, MIPI_CSI2_PHY_TST_CTRL0);
	mipi_csi2_write(info, 0x00000000, MIPI_CSI2_PHY_TST_CTRL1);
	mipi_csi2_write(info, 0x00000000, MIPI_CSI2_PHY_TST_CTRL0);
	mipi_csi2_write(info, 0x00000002, MIPI_CSI2_PHY_TST_CTRL0);
	mipi_csi2_write(info, 0x00010044, MIPI_CSI2_PHY_TST_CTRL1);
	mipi_csi2_write(info, 0x00000000, MIPI_CSI2_PHY_TST_CTRL0);
	if (mipi_lane_bps < 90)
		value = 0x00;
	else if (mipi_lane_bps < 100)
		value = 0x20;
	else if (mipi_lane_bps < 110)
		value = 0x40;
	else if (mipi_lane_bps < 125)
		value = 0x02;
	else if (mipi_lane_bps < 140)
		value = 0x22;
	else if (mipi_lane_bps < 150)
		value = 0x42;
	else if (mipi_lane_bps < 160)
		value = 0x04;
	else if (mipi_lane_bps < 180)
		value = 0x24;
	else if (mipi_lane_bps < 200)
		value = 0x44;
	else if (mipi_lane_bps < 210)
		value = 0x06;
	else if (mipi_lane_bps < 240)
		value = 0x26;
	else if (mipi_lane_bps < 250)
		value = 0x46;
	else if (mipi_lane_bps < 270)
		value = 0x08;
	else if (mipi_lane_bps < 300)
		value = 0x28;
	else if (mipi_lane_bps < 330)
		value = 0x48;
	else if (mipi_lane_bps < 360)
		value = 0x2a;
	else if (mipi_lane_bps < 400)
		value = 0x4a;
	else if (mipi_lane_bps < 450)
		value = 0x0c;
	else if (mipi_lane_bps < 500)
		value = 0x2c;
	else if (mipi_lane_bps < 550)
		value = 0x0e;
	else if (mipi_lane_bps < 600)
		value = 0x2e;
	else if (mipi_lane_bps < 650)
		value = 0x10;
	else if (mipi_lane_bps < 700)
		value = 0x30;
	else if (mipi_lane_bps < 750)
		value = 0x12;
	else if (mipi_lane_bps < 800)
		value = 0x32;
	else if (mipi_lane_bps < 850)
		value = 0x14;
	else if (mipi_lane_bps < 900)
		value = 0x34;
	else if (mipi_lane_bps < 950)
		value = 0x54;
	else
		value = 0x74;
	dev_info(&info->pdev->dev, "mipi_csi2_reset: value = 0x%x.\n", value);

	mipi_csi2_write(info, value, MIPI_CSI2_PHY_TST_CTRL1);
	mipi_csi2_write(info, 0x00000002, MIPI_CSI2_PHY_TST_CTRL0);
	mipi_csi2_write(info, 0x00000000, MIPI_CSI2_PHY_TST_CTRL0);

	mipi_csi2_write(info, 0xffffffff, MIPI_CSI2_PHY_SHUTDOWNZ);
	mipi_csi2_write(info, 0xffffffff, MIPI_CSI2_DPHY_RSTZ);
	mipi_csi2_write(info, 0xffffffff, MIPI_CSI2_CSI2_RESETN);

	_mipi_csi2_unlock(info);

	return 0;
}
EXPORT_SYMBOL(mipi_csi2_reset);

/*!
 * This function is called to get mipi csi2 info.
 *
 * @return      Returns mipi csi2 info struct pointor
 */
struct mipi_csi2_info *mipi_csi2_get_info(void)
{
	return gmipi_csi2;
}
EXPORT_SYMBOL(mipi_csi2_get_info);

/*!
 * This function is called to get mipi csi2 bind ipu num.
 *
 * @return      Returns mipi csi2 bind ipu num
 */
int mipi_csi2_get_bind_ipu(struct mipi_csi2_info *info, unsigned int v_channel)
{
	int ipu_id;

	_mipi_csi2_lock(info);
	ipu_id = info->mipi_chan[v_channel].ipu_id;
	_mipi_csi2_unlock(info);

	return ipu_id;
}
EXPORT_SYMBOL(mipi_csi2_get_bind_ipu);

/*!
 * This function is called to get mipi csi2 bind csi num.
 *
 * @return      Returns mipi csi2 bind csi num
 */
unsigned int mipi_csi2_get_bind_csi(struct mipi_csi2_info *info, unsigned int v_channel)
{
	unsigned int csi_id;

	_mipi_csi2_lock(info);
	csi_id = info->mipi_chan[v_channel].csi_id;
	_mipi_csi2_unlock(info);

	return csi_id;
}
EXPORT_SYMBOL(mipi_csi2_get_bind_csi);

/**
 * This function is called by the driver framework to initialize the MIPI CSI2
 * device.
 *
 * @param	pdev	The device structure for the MIPI CSI2 passed in by the
 *			driver framework.
 *
 * @return      Returns 0 on success or negative error code on error
 */
static int mipi_csi2_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = pdev->dev.of_node, *child;
	struct resource *res;
	u32 mipi_csi2_dphy_ver;
	int ret, v_channel;

	gmipi_csi2 = kmalloc(sizeof(struct mipi_csi2_info), GFP_KERNEL);
	if (!gmipi_csi2) {
		ret = -ENOMEM;
		goto alloc_failed;
	}

	ret = of_property_read_u32(np, "lanes", &(gmipi_csi2->lanes));
	if (ret) {
		dev_err(&pdev->dev, "lanes missing or invalid\n");
		goto err;
	}
	if (gmipi_csi2->lanes > 4) {
		dev_err(&pdev->dev, "invalid lanes for mipi csi2!\n");
		ret = -EINVAL;
		goto err;
	}

	for_each_child_of_node(np, child) {
		struct mipi_csi2_chan *chan;

		if (!of_device_is_available(child))
			continue;

		ret = of_property_read_u32(child, "v_channel", &v_channel);
		if (ret) {
			dev_err(&pdev->dev, "v_channel missing\n");
			goto err;
		}

		if (v_channel > 3) {
			dev_err(&pdev->dev, "v_channel invalid\n");
			ret = -EINVAL;
			goto err;
		}

		chan = &(gmipi_csi2->mipi_chan[v_channel]);

		ret = of_property_read_u32(child, "ipu_id", &(gmipi_csi2->mipi_chan[v_channel].ipu_id));
		if (ret) {
			dev_err(&pdev->dev, "ipu_id missing or invalid\n");
			goto err;
		}

		ret = of_property_read_u32(child, "csi_id", &(gmipi_csi2->mipi_chan[v_channel].csi_id));
		if (ret) {
			dev_err(&pdev->dev, "csi_id missing or invalid\n");
			goto err;
		}

		if ((gmipi_csi2->mipi_chan[v_channel].ipu_id < 0) || 
			(gmipi_csi2->mipi_chan[v_channel].ipu_id > 1) ||
			(gmipi_csi2->mipi_chan[v_channel].csi_id > 1)) {
			dev_err(&pdev->dev, "invalid param for mipi csi2!\n");
			ret = -EINVAL;
			goto err;
		}
	}

	/* initialize mutex */
	mutex_init(&gmipi_csi2->mutex_lock);

	/* get mipi csi2 informaiton */
	gmipi_csi2->pdev = pdev;
	gmipi_csi2->mipi_en = false;

	gmipi_csi2->cfg_clk = devm_clk_get(dev, "cfg_clk");
	if (IS_ERR(gmipi_csi2->cfg_clk)) {
		dev_err(&pdev->dev, "failed to get cfg_clk\n");
		ret = PTR_ERR(gmipi_csi2->cfg_clk);
		goto err;
	}

	/* get mipi dphy clk */
	gmipi_csi2->dphy_clk = devm_clk_get(dev, "dphy_clk");
	if (IS_ERR(gmipi_csi2->dphy_clk)) {
		dev_err(&pdev->dev, "failed to get dphy pll_ref_clk\n");
		ret = PTR_ERR(gmipi_csi2->dphy_clk);
		goto err;
	}

	/* get mipi to ipu pixel clk */
	gmipi_csi2->pixel_clk = devm_clk_get(dev, "pixel_clk");
	if (IS_ERR(gmipi_csi2->pixel_clk)) {
		dev_err(&pdev->dev, "failed to get mipi pixel clk\n");
		ret = PTR_ERR(gmipi_csi2->pixel_clk);
		goto err;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		ret = -ENODEV;
		goto err;
	}

	/* mipi register mapping */
	gmipi_csi2->mipi_csi2_base = ioremap(res->start, PAGE_SIZE);
	if (!gmipi_csi2->mipi_csi2_base) {
		ret = -ENOMEM;
		goto err;
	}

	/* mipi dphy clk enable for register access */
	clk_prepare_enable(gmipi_csi2->dphy_clk);
	/* get mipi csi2 dphy version */
	mipi_csi2_dphy_ver = mipi_csi2_read(gmipi_csi2, MIPI_CSI2_VERSION);

	clk_disable_unprepare(gmipi_csi2->dphy_clk);

	platform_set_drvdata(pdev, gmipi_csi2);

	dev_info(&pdev->dev, "i.MX MIPI CSI2 driver probed\n");
	dev_info(&pdev->dev, "i.MX MIPI CSI2 dphy version is 0x%x\n",
						mipi_csi2_dphy_ver);

	return 0;

err:
	kfree(gmipi_csi2);
alloc_failed:
	dev_err(&pdev->dev, "i.MX MIPI CSI2 driver probed -  error\n");
	return ret;
}

static int mipi_csi2_remove(struct platform_device *pdev)
{
	/* unmapping mipi register */
	iounmap(gmipi_csi2->mipi_csi2_base);

	kfree(gmipi_csi2);

	dev_set_drvdata(&pdev->dev, NULL);

	return 0;
}

static const struct of_device_id imx_mipi_csi2_dt_ids[] = {
	{ .compatible = "fsl,imx6q-mipi-csi2", },
	{ /* sentinel */ }
};

static struct platform_driver mipi_csi2_driver = {
	.driver = {
		   .name = "mxc_mipi_csi2",
		   .of_match_table = imx_mipi_csi2_dt_ids,
	},
	.probe = mipi_csi2_probe,
	.remove = mipi_csi2_remove,
};

static int __init mipi_csi2_init(void)
{
	int err;

	err = platform_driver_register(&mipi_csi2_driver);
	if (err) {
		pr_err("mipi_csi2_driver register failed\n");
		return -ENODEV;
	}

	pr_info("MIPI CSI2 driver module loaded\n");

	return 0;
}

static void __exit mipi_csi2_cleanup(void)
{
	platform_driver_unregister(&mipi_csi2_driver);
}

subsys_initcall(mipi_csi2_init);
module_exit(mipi_csi2_cleanup);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("i.MX MIPI CSI2 driver");
MODULE_LICENSE("GPL");
