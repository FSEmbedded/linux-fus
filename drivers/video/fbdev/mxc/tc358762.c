/*
 * Copyright 2019 F&S Elektronik Systeme GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/pm_runtime.h>
#include <linux/fb.h>
#include <linux/mxcfb.h>
#include <video/of_display_timing.h>
#include <video/videomode.h>

#include "mipi_dsi.h"

/* register addresses */
#define TC358762_REG_SYSPMCTRL			0x47c
#define TC358762_REG_CHIP_ID			0x4a0
#define TC358762_REG_LANE_ENABLE		0x210
#define TC358762_REG_D0S_CLRSIPOCOUNT		0x164
#define TC358762_REG_D1S_CLRSIPOCOUNT		0x168
#define TC358762_REG_LPTXTIMECNT		0x114
#define TC358762_REG_SPICTRL_SPIRCMR		0x450
#define TC358762_REG_SPITCR			0x454
#define TC358762_REG_LCDCTRL_PORT		0x420
#define TC358762_REG_HSR_HBPR			0x424
#define TC358762_REG_HDISPR_HFPR		0x428
#define TC358762_REG_VSR_VBPR			0x42c
#define TC358762_REG_VDISPR_VFPR		0x430
#define TC358762_REG_VFUEN			0x434
#define TC358762_REG_SYSCTRL			0x464
#define TC358762_REG_STARTPPI			0x104
#define TC358762_REG_STARTDSI			0x204
#define TC358762_REG_SYSPLL1			0x468
#define TC358762_REG_SYSPLL3			0x470
/* SYSPMCTRL */
#define TC358762_SLEEP				0x80
#define TC358762_WAKEUP				0x0
/* CHIP ID */
#define TC358762_CHIP_ID			0x62
/* LANEENABLE */
#define TC358762_CLOCK_LANE_ENABLE		0x1
#define TC358762_L0_ENABLE			0x2
#define TC358762_L1_ENABLE			0x4
/* LCDCTRL */
#define TC358762_RGB666				0x00
#define TC358762_RGB565				0x20
#define TC358762_RGB888				0x50
#define TC358762_DPI_EN				0x100
#define TC358762_VTGEN_ON			0x2
#define TC358762_DCLK_POL_INV			0x100000
#define TC358762_VSYNC_POL_HIGH			0x80000
#define TC358762_DE_POL_HIGH			0x40000
#define TC358762_HSYNC_POL_HIGH			0x20000
/* SYSCTRL */
#define TC358762_DATA_IO_STRENGTH		0x1
#define TC358762_STROBE_IO_STRENGTH		0x1
#define TC358762_STROBE_IO_STRENGTH_OFFSET	0x2
#define TC358762_PCLK_DIV_OFFSET		0x8

/* SYSPLL */
#define TC358762_PLL_INTE_OFFSET		26
#define TC358762_PLL_REFDIV_OFFSET		21
#define TC358762_PLL_OUTDIV_OFFSET		16

struct tc358762_reg_info {
	unsigned int ref_clk;
	unsigned int sclk;
	unsigned int pxlform;
	unsigned int vmode_flags;
	unsigned int inte;
	unsigned int ref_div;
	unsigned int out_div;
	unsigned int pclk_div;
	unsigned int tlpx;
	unsigned int ths_prepare;
	unsigned int ths_zero;
	unsigned int mipi_lane;
};

struct tc358762_info {
	struct i2c_client *client;
	struct fb_videomode *fb_vmode;
	struct device_node *remote;
	struct tc358762_reg_info reg_info;
};

static int tc358762_read_reg(struct i2c_client *client,
			       u16 reg, u16 len, u32 *val)
{
	struct i2c_msg xfer[2];
	u8 buf[2];
	int ret;

	buf[0] = (reg >> 8) & 0xff;
	buf[1] = reg & 0xff;

	/* Write register */
	xfer[0].addr = client->addr;
	xfer[0].flags = 0;
	xfer[0].len = 2;
	xfer[0].buf = buf;

	/* Read data */
	xfer[1].addr = client->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = len;
	xfer[1].buf = (u8 *)val;

	ret = i2c_transfer(client->adapter, xfer, 2);
	if (ret == 2) {
		ret = 0;
	} else {
		if (ret >= 0)
			ret = -EIO;
		dev_err(&client->dev, "%s: i2c transfer failed (%d)\n",
			__func__, ret);
	}

	return ret;
}

static int tc358762_write_reg(struct i2c_client *client,
			       u16 reg, u32 val)
{
	struct i2c_msg xfer;
	u8 buf[6];
	int ret;

	memset(buf, 0x0, sizeof(buf));
	buf[0] = (reg >> 8) & 0xff;
	buf[1] = reg & 0xff;

	memcpy(&buf[2], &val, 4);
	/* Write register */
	xfer.addr = client->addr;
	xfer.flags = 0;
	xfer.len = sizeof(buf);
	xfer.buf = buf;

	ret = i2c_transfer(client->adapter, &xfer, 1);
	if (ret == 1) {
		ret = 0;
	} else {
		if (ret >= 0)
			ret = -EIO;
		dev_err(&client->dev, "%s: i2c transfer failed (%d)\n",
			__func__, ret);
	}

	return ret;
}

static int tc358762_wakeup(struct i2c_client *client)
{
	int ret = 0;

	ret = tc358762_write_reg(client, TC358762_REG_SYSPMCTRL,
				 TC358762_WAKEUP);
	if (ret) {
		dev_err(&client->dev, "Failed to write i2c messages (%d)\n",
			ret);
		return ret;
	}
	/* needs at least 500µs after wakeup, to write/read register */
	udelay(500);

	return ret;
}

static int tc358762_sleep(struct i2c_client *client)
{
	int ret = 0;

	ret = tc358762_write_reg(client, TC358762_REG_SYSPMCTRL,
				 TC358762_SLEEP);
	if (ret) {
		dev_err(&client->dev, "Failed to write i2c messages (%d)\n",
			ret);
		return ret;
	}
	/* needs at least 500µs after set sleep mode, to write/read register */
	udelay(500);

	return ret;
}

static int tc358762_detect(struct i2c_client *client,
			  struct i2c_board_info *info)
{
	int ret;
	u8 chip_rev = 0;
	u32 val = 0;
	struct i2c_adapter *adapter = client->adapter;

	/* check i2c functionality */
	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -ENODEV;

	/* wake up chip */
	ret = tc358762_wakeup(client);
	if (ret)
		return ret;

	/* i2c check id */
	ret = tc358762_read_reg(client, TC358762_REG_CHIP_ID, 4, &val);
	if (ret) {
		dev_err(&client->dev, "Failed to read i2c chip id (%d)\n",
			ret);
		goto err;
	}

	/* set to sleep mode */
	ret = tc358762_sleep(client);
	if (ret)
		return ret;

	chip_rev = (val >> 8);
	if (chip_rev != TC358762_CHIP_ID) {
		dev_info(&adapter->dev, "Unsupported chip id: 0x%2x\n",
			 chip_rev);
		ret = -ENODEV;
		goto err;
	}

	return 0;

err:
	tc358762_sleep(client);
	return ret;
}

//### TODO
#if 0
static void calc_divider_mipi_clksrc(struct tc358762_info *info)
{
	int dsi_clk;
	int sclk = (dsi_clk + 2) / 4;
	int pix_clk = PICOS2KHZ(info->fb_vmode->pixclock);
	int pclk_div = 0;
	int dif = INT_MAX;

	for (pclk_div = 1; pclk_div <= 3; pclk_div++)
	{
		int pix_clk_new = 0;
		int new_dif = 0;

		pix_clk_new = (sclk + (pclk_div / 2)) / pclk_div;

		if(pix_clk_new > pix_clk)
		{
			new_dif = pix_clk_new - pix_clk;
		}
		else
		{
			new_dif = pix_clk - pix_clk_new;
		}
		if(new_dif <= dif)
		{
			dif = new_dif;
			if (pclk_div == 3)
				info->reg_info.pclk_div = 4;
			else
				info->reg_info.pclk_div = pclk_div;
		}
	}
}
#endif

static void calc_divider_ext_clksrc(struct tc358762_info *info)
{
	int inte = 0, ref_div = 0, out_div = 0, pclk_div = 0;
	int ref_clk = info->reg_info.ref_clk, ico_clk = 0;
	int pix_clk = PICOS2KHZ(info->fb_vmode->pixclock);
	int dif = INT_MAX;

	for(inte=200000/ref_clk; inte <= 63; inte++)
	{
		for (ref_div = (inte*ref_clk + 399000)/400000; ref_div < inte *
			ref_clk/200000; ref_div++)
		{
			ico_clk = inte * ref_clk / ref_div;
			for (pclk_div = 1; pclk_div <= 3; pclk_div++)
			{
				int n = pclk_div * pix_clk;
				int pix_clk_new = 0;
				int divider = 0;
				int new_dif = 0;
				out_div = (ico_clk + n/2)/n;
				divider = ref_div * out_div * pclk_div;
				pix_clk_new = (ref_clk * inte +
					(divider / 2)) / divider;

				if(pix_clk_new > pix_clk)
				{
					new_dif = pix_clk_new - pix_clk;
				}
				else
				{
					new_dif = pix_clk - pix_clk_new;
				}
				if(new_dif <= dif)
				{
					dif = new_dif;
					info->reg_info.inte = inte;
					info->reg_info.ref_div = ref_div;
					info->reg_info.out_div = out_div;
					info->reg_info.sclk =
						pix_clk_new * pclk_div;
					if (pclk_div == 3)
						info->reg_info.pclk_div = 4;
					else
						info->reg_info.pclk_div =
							pclk_div;
				}
			}
		}
	}
}

static uint32_t get_disp_polarity(uint32_t flags)
{
	uint32_t val = 0;

	if (flags & DISPLAY_FLAGS_VSYNC_HIGH)
		val |= TC358762_VSYNC_POL_HIGH;
	if (flags & DISPLAY_FLAGS_HSYNC_HIGH)
		val |= TC358762_HSYNC_POL_HIGH;
	if (flags & DISPLAY_FLAGS_DE_HIGH)
		val |= TC358762_DE_POL_HIGH;
	if (flags & DISPLAY_FLAGS_PIXDATA_NEGEDGE)
		val |= TC358762_DCLK_POL_INV;

	return val;
}

static uint32_t pxfmt_to_reg(unsigned int pxlform)
{
	switch (pxlform) {
	case 24:
		return TC358762_RGB888;
	case 16:
		return TC358762_RGB565;
	default:
		return TC358762_RGB666;
	}
}

static int tc358762_setup(struct tc358762_info *info)
{
	struct i2c_client *client = info->client;
	int ret = 0, lanes = 0;
	u32 val = 0;

	/* wakeup chip */
	ret = tc358762_wakeup(client);
	if (ret)
		return ret;

	if (info->reg_info.mipi_lane == 2)
	{
		val = TC358762_L0_ENABLE | TC358762_L1_ENABLE;
	}
	else if (lanes == 1)
	{
		val = TC358762_L0_ENABLE;
	}
	else
	{
		dev_err(&client->dev, "failed to setup TC358762, invalid lanes"
			"%d\n", lanes);
		ret = -EINVAL;
		goto err;
	}
	/* set lanes */
	val |= TC358762_CLOCK_LANE_ENABLE;
	ret = tc358762_write_reg(client, TC358762_REG_LANE_ENABLE, val);
		if (ret)
			goto err;

	/* calc_divider */
	if (info->reg_info.ref_clk) {
		calc_divider_ext_clksrc(info);
	}
	else {
		//### TODO
		//calc_divider_mipi_clksrc(info);
		ret = -EINVAL;
		goto err;
	}

	/* counter for data lanes */
	val = (((2 * info->reg_info.ths_prepare + info->reg_info.ths_zero) *
		info->reg_info.sclk + 2000000 -1) / 2000000) - 5;
	ret = tc358762_write_reg(client, TC358762_REG_D0S_CLRSIPOCOUNT, val);
	if (ret)
		goto err;
	if(info->reg_info.mipi_lane == 2)
	{
		ret = tc358762_write_reg(client,
					 TC358762_REG_D1S_CLRSIPOCOUNT, val);
		if (ret)
			goto err;
	}
	/* timing generation counter */
	val = (info->reg_info.tlpx * info->reg_info.sclk + 500000) / 1000000;
	ret = tc358762_write_reg(client, TC358762_REG_LPTXTIMECNT, val);
	if (ret)
		goto err;
	/* set SPI register, default values from excel sheet */
	ret = tc358762_write_reg(client, TC358762_REG_SPICTRL_SPIRCMR, 0x62);
	if (ret)
		goto err;
	ret = tc358762_write_reg(client, TC358762_REG_SPITCR, 0x122);
	if (ret)
		goto err;
	/* LCD features */
	val = get_disp_polarity(info->reg_info.vmode_flags);
	val |= pxfmt_to_reg(info->reg_info.pxlform);
	val |= TC358762_VTGEN_ON;
	val |= TC358762_DPI_EN;
	ret = tc358762_write_reg(client, TC358762_REG_LCDCTRL_PORT, val);
	if (ret)
		goto err;
	/* hsync_len[0:15], hback_porch [16:31] */
	val = info->fb_vmode->hsync_len;
	val |= (info->fb_vmode->left_margin << 16);
	ret = tc358762_write_reg(client, TC358762_REG_HSR_HBPR, val);
	if (ret)
		goto err;
	/* hactive_len[0:15], hfront_porch [16:31] */
	val = info->fb_vmode->xres;
	val |= (info->fb_vmode->right_margin << 16);
	ret = tc358762_write_reg(client, TC358762_REG_HDISPR_HFPR, val);
	if (ret)
		goto err;
	/* vsync_len[0:15], vback_porch [16:31] */
	val = info->fb_vmode->vsync_len;
	val |= (info->fb_vmode->upper_margin << 16);
	ret = tc358762_write_reg(client, TC358762_REG_VSR_VBPR, val);
	if (ret)
		goto err;
	/* vactive_len[0:15], vfront_porch [16:31] */
	val = info->fb_vmode->yres;
	val |= (info->fb_vmode->lower_margin << 16);
	ret = tc358762_write_reg(client, TC358762_REG_VDISPR_VFPR, val);
	if (ret)
		goto err;
	/* upload settings */
	ret = tc358762_write_reg(client, TC358762_REG_VFUEN, 0x1);
	if (ret)
		goto err;
	/* SYSCTRL */
	val = TC358762_DATA_IO_STRENGTH;
	val |= (TC358762_STROBE_IO_STRENGTH <<
		TC358762_STROBE_IO_STRENGTH_OFFSET);
	val |= (info->reg_info.pclk_div << TC358762_PCLK_DIV_OFFSET);
	ret = tc358762_write_reg(client, TC358762_REG_SYSCTRL, val);
	if (ret)
		goto err;
	/* START PPI */
	ret = tc358762_write_reg(client, TC358762_REG_STARTPPI, 0x1);
	if (ret)
		goto err;
	/* START DSI */
	ret = tc358762_write_reg(client, TC358762_REG_STARTDSI, 0x1);
	if (ret)
		goto err;
	/* SYSPLL1 - PRESCL always 0 */
	ret = tc358762_write_reg(client, TC358762_REG_SYSPLL1, 0x4);
	if (ret)
		goto err;

	if (info->reg_info.ref_clk)
	{
		/* SYSPLL3 */
		val = (info->reg_info.inte << TC358762_PLL_INTE_OFFSET);
		val |= (info->reg_info.out_div << TC358762_PLL_OUTDIV_OFFSET);
		val |= (info->reg_info.ref_div << TC358762_PLL_REFDIV_OFFSET);
		ret = tc358762_write_reg(client, TC358762_REG_SYSPLL3, val);
		if (ret)
			goto err;
	}

	/* save settings */
	ret = tc358762_sleep(client);
	if (ret)
		return ret;
	ret = tc358762_wakeup(client);
	if (ret)
		return ret;

	dev_info(&client->dev, "setup complete!\n");
	return ret;

err:
	tc358762_sleep(client);
	return ret;
}

static int tc358762_init_disp_dt(struct tc358762_info *info)
{
	struct device *dev = &info->client->dev;
	struct device_node *np = dev->of_node;
	struct device_node *display_np;
	struct videomode vm;
	struct fb_videomode fb_vm;
	struct display_timings *timings = NULL;
	int ret = 0;

	ret = of_property_read_u32(np, "data-lanes-num",
				   &info->reg_info.mipi_lane);
	if (ret < 0)
	{
		ret = of_property_read_u32(info->remote, "data-lanes-num",
					   &info->reg_info.mipi_lane);
		if (ret < 0)
			return -EINVAL;
	}
	ret = of_property_read_u32(np, "ref-clk", &info->reg_info.ref_clk);
	if (ret < 0)
		info->reg_info.ref_clk = 0;
	ret = of_property_read_u32(np, "tlpx", &info->reg_info.tlpx);
	if (ret < 0)
		return -EINVAL;
	ret = of_property_read_u32(np, "ths-prepare",
				&info->reg_info.ths_prepare);
	if (ret < 0)
		return -EINVAL;
	ret = of_property_read_u32(np, "ths-zero", &info->reg_info.ths_zero);
	if (ret < 0)
		return -EINVAL;

	display_np = of_parse_phandle(np, "display", 0);
	if (!display_np) {
		dev_err(dev, "failed to find display phandle\n");
		return -ENOENT;
	}

	ret = of_property_read_u32(display_np, "bus-width",
				&info->reg_info.pxlform);
	if (ret < 0) {
		dev_err(dev, "failed to get property bus-width\n");
		goto put_display_node;
	}

	timings = of_get_display_timings(display_np);
	if (!timings) {
		dev_err(dev, "failed to get display timings\n");
		ret = -ENOENT;
		goto put_display_node;
	}

	ret = videomode_from_timings(timings, &vm, 0);
	if (ret < 0)
		goto put_display_node;
	ret = fb_videomode_from_videomode(&vm, &fb_vm);
	if (ret < 0)
		goto put_display_node;

	if (!(vm.flags & DISPLAY_FLAGS_DE_HIGH))
		fb_vm.sync |= FB_SYNC_OE_LOW_ACT;
	if (vm.flags & DISPLAY_FLAGS_PIXDATA_NEGEDGE)
		fb_vm.sync |= FB_SYNC_CLK_LAT_FALL;

	info->fb_vmode = devm_kzalloc(dev,
					sizeof(struct fb_videomode),
					GFP_KERNEL);
	memcpy(info->fb_vmode, &fb_vm, sizeof(struct fb_videomode));

	info->reg_info.vmode_flags = vm.flags;

put_display_node:
	of_node_put(display_np);

	return ret;
}

static int tc358762_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int ret = 0;
	struct tc358762_info *info;
	struct device *dev = &client->dev;
	struct device_node *endpoint = NULL;

	info = devm_kzalloc(dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->client = client;
	i2c_set_clientdata(client, info);

	/* get dsi source endpoint */
	endpoint = of_graph_get_next_endpoint(dev->of_node, NULL);
	if(endpoint)
	{
		info->remote = of_graph_get_remote_port_parent(endpoint);
		if (!info->remote)
			dev_info(dev, "DSI remote endpoint not exist\n");
	}
	else {
		dev_info(dev, "DSI source endpoint not exist\n");
	}

	ret = tc358762_init_disp_dt(info);
	if (ret < 0)
		return ret;

	if ((ret = tc358762_detect(client, NULL)))
		return ret;

	if ((ret = tc358762_setup(info)))
		return ret;

#ifndef CONFIG_FB_IMX64
	pm_runtime_enable(dev);
#endif

	return 0;
}

static int tc358762_remove(struct i2c_client *client)
{
	struct tc358762_info *info;

	tc358762_sleep(client);

	info = i2c_get_clientdata(client);
	i2c_set_clientdata(client, NULL);

#ifndef CONFIG_FB_IMX64
	pm_runtime_disable(&client->dev);
#endif

	return 0;
}

static const struct i2c_device_id tc358762_id[] = {
	{"tc358762", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, tc358762_id);

static const struct of_device_id tc358762_dt_ids[] = {
	{ .compatible = "toshiba,tc358762", .data = NULL, },
	{ /* sentinel */ }
};

static struct i2c_driver tc358762_i2c_driver = {
	.driver = {
		.name = "tc358762",
		.owner = THIS_MODULE,
		.of_match_table = tc358762_dt_ids,
	},
	.probe  = tc358762_probe,
	.remove = tc358762_remove,
	.id_table = tc358762_id,
	.detect = tc358762_detect,
};

static __init int tc358762_init(void)
{
	u8 err = 0;

	err = i2c_add_driver(&tc358762_i2c_driver);
	if (err != 0)
		pr_err("%s: i2c driver register failed, error = %d\n",
			__func__, err);

	pr_debug("%s (ret=%d)\n", __func__, err);
	return err;
}

static void __exit tc358762_exit(void)
{
	i2c_del_driver(&tc358762_i2c_driver);
}

module_init(tc358762_init);
module_exit(tc358762_exit);

MODULE_AUTHOR("F&S Elektronik Systeme");
MODULE_DESCRIPTION("TC358762 MIPI to RGB converter driver");
MODULE_LICENSE("GPL");
