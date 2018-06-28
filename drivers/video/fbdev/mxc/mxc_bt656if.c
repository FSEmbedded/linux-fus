/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/init.h>
#include <linux/ipu.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mxcfb.h>
#include <linux/of_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>

#include "mxc_dispdrv.h"

struct mxc_bt656_platform_data {
	u32 default_ifmt;
	u32 ipu_id;
	u32 disp_id;
};

struct mxc_bt656if_data {
	struct platform_device *pdev;
	struct mxc_dispdrv_handle *disp_bt656if;
};

#define DISPDRV_BT656	"bt656"

static struct fb_videomode bt656if_modedb[] = {
	{
	/* NTSC Interlaced output */
	"BT656-4-NTSC", 60, 720, 487, 37037,
	16, 2,
	17, 3,
	276, 1,
	FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	FB_VMODE_INTERLACED,
	FB_MODE_IS_DETAILED,},
	{
	/* NTSC Interlaced output */
	"BT656-NTSC", 60, 720, 480, 37037,
	19, 3,
	20, 3,
	276, 1,
	FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	FB_VMODE_INTERLACED,
	FB_MODE_IS_DETAILED,},
	{
	/* PAL Interlaced output */
	"BT656-PAL", 50, 720, 576, 37037,
	22, 2,
	23, 2,
	288, 1,
	FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	FB_VMODE_INTERLACED,
	FB_MODE_IS_DETAILED,},
	{
	/* NTSC Progressive output */
	"BT656-480P", 60, 720, 480, 18518,
	36, 9,
	0, 0,
	276, 1,
	FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	FB_VMODE_NONINTERLACED,
	FB_MODE_IS_DETAILED,},
	{
	/* PAL Progressive output */
	"BT656-576P", 50, 720, 576, 18518,
	44, 5,
	0, 0,
	288, 1,
	FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	FB_VMODE_NONINTERLACED,
	FB_MODE_IS_DETAILED,},
};
static int bt656if_modedb_sz = ARRAY_SIZE(bt656if_modedb);

static struct fb_videomode bt1120if_modedb[] = {
	{
	/* NTSC Interlaced output */
	"BT1120-NTSC", 60, 720, 480, 74074,
	19, 3,
	20, 3,
	142, 1,
	FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	FB_VMODE_INTERLACED,
	FB_MODE_IS_DETAILED,},
	{
	/* PAL Interlaced output */
	"BT1120-PAL", 50, 720, 576, 74074,
	22, 2,
	23, 2,
	148, 1,
	FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	FB_VMODE_INTERLACED,
	FB_MODE_IS_DETAILED,},
	{
	/* 1080I60 Interlaced output */
	"BT1120-1080I60", 60, 1920, 1080, 13468,
	20, 3,
	20, 2,
	280, 1,
	FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	FB_VMODE_INTERLACED,
	FB_MODE_IS_DETAILED,},
	{
	/* 1080I50 Interlaced output */
	"BT1120-1080I50", 50, 1920, 1080, 13468,
	20, 3,
	20, 2,
	720, 1,
	FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	FB_VMODE_INTERLACED,
	FB_MODE_IS_DETAILED,},
	{
	/* NTSC Progressive output */
	"BT1120-480P", 60, 720, 480, 37037,
	36, 9,
	0, 0,
	142, 1,
	FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	FB_VMODE_NONINTERLACED,
	FB_MODE_IS_DETAILED,},
	{
	/* PAL Progressive output */
	"BT1120-576P", 50, 720, 576, 37037,
	44, 5,
	0, 0,
	148, 1,
	FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	FB_VMODE_NONINTERLACED,
	FB_MODE_IS_DETAILED,},
	{
	/* 1080P60 Progressive output */
	"BT1120-1080P60", 60, 1920, 1080, 6734,
	41, 4,
	0, 0,
	280, 1,
	FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	FB_VMODE_NONINTERLACED,
	FB_MODE_IS_DETAILED,},
	{
	/* 1080P50 Progressive output */
	"BT1120-1080P50", 50, 1920, 1080, 6734,
	41, 4,
	0, 0,
	720, 1,
	FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	FB_VMODE_NONINTERLACED,
	FB_MODE_IS_DETAILED,},
};
static int bt1120if_modedb_sz = ARRAY_SIZE(bt1120if_modedb);

static int bt656if_init(struct mxc_dispdrv_handle *disp,
	struct mxc_dispdrv_setting *setting)
{
	int ret, i;
	struct mxc_bt656if_data *bt656if = mxc_dispdrv_getdata(disp);
	struct device *dev = &bt656if->pdev->dev;
	struct mxc_bt656_platform_data *plat_data = dev->platform_data;
	struct fb_videomode *modedb = bt656if_modedb;
	int modedb_sz = bt656if_modedb_sz;

	/* use platform defined ipu/di */
	ret = ipu_di_to_crtc(dev, plat_data->ipu_id,
			     plat_data->disp_id, &setting->crtc);
	if (ret < 0)
		return ret;

	if (plat_data->default_ifmt == IPU_PIX_FMT_BT1120) {
		modedb = bt1120if_modedb;
		modedb_sz = bt1120if_modedb_sz;
	}

	ret = fb_find_mode(&setting->fbi->var, setting->fbi, setting->dft_mode_str,
				modedb, modedb_sz, NULL, setting->default_bpp);
	if (!ret) {
		fb_videomode_to_var(&setting->fbi->var, &modedb[0]);
		setting->if_fmt = plat_data->default_ifmt;
	}

	INIT_LIST_HEAD(&setting->fbi->modelist);
	for (i = 0; i < modedb_sz; i++) {
		fb_add_videomode(&modedb[i],
				&setting->fbi->modelist);
	}

	return ret;
}

void bt656if_deinit(struct mxc_dispdrv_handle *disp)
{
	/*TODO*/
}

static struct mxc_dispdrv_driver bt656if_drv = {
	.name 	= DISPDRV_BT656,
	.init 	= bt656if_init,
	.deinit	= bt656if_deinit,
};

static int bt656_get_of_property(struct platform_device *pdev,
				struct mxc_bt656_platform_data *plat_data)
{
	struct device_node *np = pdev->dev.of_node;
	int err;
	u32 ipu_id, disp_id;
	const char *default_ifmt;

	err = of_property_read_string(np, "default_ifmt", &default_ifmt);
	if (err) {
		dev_dbg(&pdev->dev, "get of property default_ifmt fail\n");
		return err;
	}
	err = of_property_read_u32(np, "ipu_id", &ipu_id);
	if (err) {
		dev_dbg(&pdev->dev, "get of property ipu_id fail\n");
		return err;
	}
	err = of_property_read_u32(np, "disp_id", &disp_id);
	if (err) {
		dev_dbg(&pdev->dev, "get of property disp_id fail\n");
		return err;
	}

	plat_data->ipu_id = ipu_id;
	plat_data->disp_id = disp_id;
	if (!strncmp(default_ifmt, "BT656", 5))
		plat_data->default_ifmt = IPU_PIX_FMT_BT656;
	else if (!strncmp(default_ifmt, "BT1120", 6))
		plat_data->default_ifmt = IPU_PIX_FMT_BT1120;
	else {
		dev_err(&pdev->dev, "err default_ifmt!\n");
		return -ENOENT;
	}

	return err;
}

static int mxc_bt656if_probe(struct platform_device *pdev)
{
	int ret;
	struct pinctrl *pinctrl;
	struct mxc_bt656if_data *bt656if;
	struct mxc_bt656_platform_data *plat_data;

	dev_dbg(&pdev->dev, "%s enter\n", __func__);
	bt656if = devm_kzalloc(&pdev->dev, sizeof(struct mxc_bt656if_data),
				GFP_KERNEL);
	if (!bt656if)
		return -ENOMEM;
	plat_data = devm_kzalloc(&pdev->dev,
				sizeof(struct mxc_bt656_platform_data),
				GFP_KERNEL);
	if (!plat_data)
		return -ENOMEM;
	pdev->dev.platform_data = plat_data;

	ret = bt656_get_of_property(pdev, plat_data);
	if (ret < 0) {
		dev_err(&pdev->dev, "get bt656 of property fail\n");
		return ret;
	}

	pinctrl = devm_pinctrl_get_select_default(&pdev->dev);
	if (IS_ERR(pinctrl)) {
		dev_err(&pdev->dev, "can't get/select pinctrl\n");
		return PTR_ERR(pinctrl);
	}

	bt656if->pdev = pdev;
	bt656if->disp_bt656if = mxc_dispdrv_register(&bt656if_drv);
	mxc_dispdrv_setdata(bt656if->disp_bt656if, bt656if);

	dev_set_drvdata(&pdev->dev, bt656if);
	dev_dbg(&pdev->dev, "%s exit\n", __func__);

	return ret;
}

static int mxc_bt656if_remove(struct platform_device *pdev)
{
	struct mxc_bt656if_data *bt656if = dev_get_drvdata(&pdev->dev);

	mxc_dispdrv_puthandle(bt656if->disp_bt656if);
	mxc_dispdrv_unregister(bt656if->disp_bt656if);
	kfree(bt656if);
	return 0;
}

static const struct of_device_id imx_bt656_dt_ids[] = {
	{ .compatible = "fsl,bt656"},
	{ /* sentinel */ }
};
static struct platform_driver mxc_bt656if_driver = {
	.driver = {
		.name = "mxc_bt656if",
		.of_match_table	= imx_bt656_dt_ids,
	},
	.probe = mxc_bt656if_probe,
	.remove = mxc_bt656if_remove,
};

static int __init mxc_bt656if_init(void)
{
	return platform_driver_register(&mxc_bt656if_driver);
}

static void __exit mxc_bt656if_exit(void)
{
	platform_driver_unregister(&mxc_bt656if_driver);
}

module_init(mxc_bt656if_init);
module_exit(mxc_bt656if_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("i.MX ipuv3 BT656 and BT1120 extern port driver");
MODULE_LICENSE("GPL");
