// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for panels based on NewVision NV3051D-T controller
 *
 * Copyright (C) Purism SPC 2019
 */

#include <drm/drmP.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/media-bus-format.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>

#include <video/display_timing.h>
#include <video/mipi_display.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_print.h>

#define DRV_NAME "panel-newvision-nv3051d"

/* Manufacturer specific Commands send via DSI */
#define NV3051D_CMD_ENEXTC			0xFF
#define NV3051D_CMD_ALL_PIXEL_OFF 	0x22
#define NV3051D_CMD_ALL_PIXEL_ON 	0x23

struct nv3051d {
	struct device *dev;
	struct drm_panel panel;
	struct gpio_desc *reset_gpio;
	struct regulator *vci;
	struct regulator *iovcc;
	bool prepared;

	struct dentry *debugfs;
	const struct nv3051d_panel_desc *desc;
};

struct nv3051d_panel_desc {
	const struct drm_display_mode *mode;
	unsigned int lanes;
	unsigned long mode_flags;
	enum mipi_dsi_pixel_format format;
	int (*init_sequence)(struct nv3051d *ctx);
};

static const u32 nv3051d_bus_formats[] = {
	MEDIA_BUS_FMT_RGB888_1X24,
};

static inline struct nv3051d *panel_to_nv3051d(struct drm_panel *panel)
{
	return container_of(panel, struct nv3051d, panel);
}

#define dsi_generic_write_seq(dsi, seq...) do {				\
		static const u8 d[] = { seq };				\
		int ret;						\
		ret = mipi_dsi_generic_write(dsi, d, ARRAY_SIZE(d));	\
		if (ret < 0)						\
			return ret;					\
	} while (0)


static int ee0350et_init_sequence(struct nv3051d *ctx)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	dsi_generic_write_seq(dsi, 0xFF,0x30);
	dsi_generic_write_seq(dsi, 0xFF,0x52);
	dsi_generic_write_seq(dsi, 0xFF,0x01);
	dsi_generic_write_seq(dsi, 0xE3,0x00);

	dsi_generic_write_seq(dsi, 0x40,0x0a);

	dsi_generic_write_seq(dsi, 0x03,0x40);
	dsi_generic_write_seq(dsi, 0x04,0x00);
	dsi_generic_write_seq(dsi, 0x05,0x03);

	dsi_generic_write_seq(dsi, 0x20,0x90);//2LANE

	dsi_generic_write_seq(dsi, 0x24,0x0c);
	dsi_generic_write_seq(dsi, 0x25,0x06);
	dsi_generic_write_seq(dsi, 0x26,0x14);
	dsi_generic_write_seq(dsi, 0x27,0x14);

	dsi_generic_write_seq(dsi, 0x28,0x57);
	dsi_generic_write_seq(dsi, 0x29,0x01);
	dsi_generic_write_seq(dsi, 0x2A,0xdf);

	dsi_generic_write_seq(dsi, 0x38,0x9C);
	dsi_generic_write_seq(dsi, 0x39,0xA7);
	dsi_generic_write_seq(dsi, 0x3A,0x53);

	dsi_generic_write_seq(dsi, 0x44,0x00);
	dsi_generic_write_seq(dsi, 0x49,0x3C);
	dsi_generic_write_seq(dsi, 0x59,0xfe);
	dsi_generic_write_seq(dsi, 0x5c,0x00);

	dsi_generic_write_seq(dsi, 0x91,0x57);
	dsi_generic_write_seq(dsi, 0x92,0x57);
	dsi_generic_write_seq(dsi, 0xA0,0x55);
	dsi_generic_write_seq(dsi, 0xA1,0x50);

	dsi_generic_write_seq(dsi, 0xA4,0x9C);
	dsi_generic_write_seq(dsi, 0xA7,0x02);
	dsi_generic_write_seq(dsi, 0xA8,0x01);
	dsi_generic_write_seq(dsi, 0xA9,0x01);
	dsi_generic_write_seq(dsi, 0xAA,0xFC);
	dsi_generic_write_seq(dsi, 0xAB,0x28);
	dsi_generic_write_seq(dsi, 0xAC,0x06);
	dsi_generic_write_seq(dsi, 0xAD,0x06);
	dsi_generic_write_seq(dsi, 0xAE,0x06);
	dsi_generic_write_seq(dsi, 0xAF,0x03);
	dsi_generic_write_seq(dsi, 0xB0,0x08);
	dsi_generic_write_seq(dsi, 0xB1,0x26);
	dsi_generic_write_seq(dsi, 0xB2,0x28);
	dsi_generic_write_seq(dsi, 0xB3,0x28);
	dsi_generic_write_seq(dsi, 0xB4,0x33);
	dsi_generic_write_seq(dsi, 0xB5,0x08);
	dsi_generic_write_seq(dsi, 0xB6,0x26);
	dsi_generic_write_seq(dsi, 0xB7,0x08);
	dsi_generic_write_seq(dsi, 0xB8,0x26);

	dsi_generic_write_seq(dsi, 0xFF,0x30);
	dsi_generic_write_seq(dsi, 0xFF,0x52);
	dsi_generic_write_seq(dsi, 0xFF,0x02);
	dsi_generic_write_seq(dsi, 0xB0,0x0B);
	dsi_generic_write_seq(dsi, 0xB1,0x16);
	dsi_generic_write_seq(dsi, 0xB2,0x17);
	dsi_generic_write_seq(dsi, 0xB3,0x2C);
	dsi_generic_write_seq(dsi, 0xB4,0x32);
	dsi_generic_write_seq(dsi, 0xB5,0x3B);
	dsi_generic_write_seq(dsi, 0xB6,0x29);
	dsi_generic_write_seq(dsi, 0xB7,0x40);
	dsi_generic_write_seq(dsi, 0xB8,0x0d);
	dsi_generic_write_seq(dsi, 0xB9,0x05);
	dsi_generic_write_seq(dsi, 0xBA,0x12);
	dsi_generic_write_seq(dsi, 0xBB,0x10);
	dsi_generic_write_seq(dsi, 0xBC,0x12);
	dsi_generic_write_seq(dsi, 0xBD,0x15);
	dsi_generic_write_seq(dsi, 0xBE,0x19);
	dsi_generic_write_seq(dsi, 0xBF,0x0E);
	dsi_generic_write_seq(dsi, 0xC0,0x16);
	dsi_generic_write_seq(dsi, 0xC1,0x0A);
	dsi_generic_write_seq(dsi, 0xD0,0x0C);
	dsi_generic_write_seq(dsi, 0xD1,0x17);
	dsi_generic_write_seq(dsi, 0xD2,0x14);
	dsi_generic_write_seq(dsi, 0xD3,0x2E);
	dsi_generic_write_seq(dsi, 0xD4,0x32);
	dsi_generic_write_seq(dsi, 0xD5,0x3C);
	dsi_generic_write_seq(dsi, 0xD6,0x22);
	dsi_generic_write_seq(dsi, 0xD7,0x3D);
	dsi_generic_write_seq(dsi, 0xD8,0x0D);
	dsi_generic_write_seq(dsi, 0xD9,0x07);
	dsi_generic_write_seq(dsi, 0xDA,0x13);
	dsi_generic_write_seq(dsi, 0xDB,0x13);
	dsi_generic_write_seq(dsi, 0xDC,0x11);
	dsi_generic_write_seq(dsi, 0xDD,0x15);
	dsi_generic_write_seq(dsi, 0xDE,0x19);
	dsi_generic_write_seq(dsi, 0xDF,0x10);
	dsi_generic_write_seq(dsi, 0xE0,0x17);
	dsi_generic_write_seq(dsi, 0xE1,0x0A);

	dsi_generic_write_seq(dsi, 0xFF,0x30);
	dsi_generic_write_seq(dsi, 0xFF,0x52);
	dsi_generic_write_seq(dsi, 0xFF,0x03);

	dsi_generic_write_seq(dsi, 0x00,0x2A);
	dsi_generic_write_seq(dsi, 0x01,0x2A);
	dsi_generic_write_seq(dsi, 0x02,0x2A);
	dsi_generic_write_seq(dsi, 0x03,0x2A);
	dsi_generic_write_seq(dsi, 0x04,0x61);
	dsi_generic_write_seq(dsi, 0x05,0x80);
	dsi_generic_write_seq(dsi, 0x06,0xc7);
	dsi_generic_write_seq(dsi, 0x07,0x01);

	dsi_generic_write_seq(dsi, 0x08,0x82);
	dsi_generic_write_seq(dsi, 0x09,0x83);

	dsi_generic_write_seq(dsi, 0x30,0x2A);
	dsi_generic_write_seq(dsi, 0x31,0x2A);
	dsi_generic_write_seq(dsi, 0x32,0x2A);
	dsi_generic_write_seq(dsi, 0x33,0x2A);
	dsi_generic_write_seq(dsi, 0x34,0x61);
	dsi_generic_write_seq(dsi, 0x35,0xc5);
	dsi_generic_write_seq(dsi, 0x36,0x80);
	dsi_generic_write_seq(dsi, 0x37,0x23);

	dsi_generic_write_seq(dsi, 0x40,0x82);
	dsi_generic_write_seq(dsi, 0x41,0x83);
	dsi_generic_write_seq(dsi, 0x42,0x80);
	dsi_generic_write_seq(dsi, 0x43,0x81);

	dsi_generic_write_seq(dsi, 0x44,0x11);
	dsi_generic_write_seq(dsi, 0x45,0xe6);
	dsi_generic_write_seq(dsi, 0x46,0xe5);
	dsi_generic_write_seq(dsi, 0x47,0x11);
	dsi_generic_write_seq(dsi, 0x48,0xe8);
	dsi_generic_write_seq(dsi, 0x49,0xe7);

	dsi_generic_write_seq(dsi, 0x50,0x02);
	dsi_generic_write_seq(dsi, 0x51,0x01);
	dsi_generic_write_seq(dsi, 0x52,0x04);
	dsi_generic_write_seq(dsi, 0x53,0x03);

	dsi_generic_write_seq(dsi, 0x54,0x11);
	dsi_generic_write_seq(dsi, 0x55,0xea);
	dsi_generic_write_seq(dsi, 0x56,0xe9);
	dsi_generic_write_seq(dsi, 0x57,0x11);
	dsi_generic_write_seq(dsi, 0x58,0xec);
	dsi_generic_write_seq(dsi, 0x59,0xeb);

	dsi_generic_write_seq(dsi, 0x7e,0x02);
	dsi_generic_write_seq(dsi, 0x7f,0x80);
	dsi_generic_write_seq(dsi, 0xe0,0x5a);

	dsi_generic_write_seq(dsi, 0xB1,0x00);
	dsi_generic_write_seq(dsi, 0xB4,0x0e);
	dsi_generic_write_seq(dsi, 0xB5,0x0f);
	dsi_generic_write_seq(dsi, 0xB6,0x04);
	dsi_generic_write_seq(dsi, 0xB7,0x07);
	dsi_generic_write_seq(dsi, 0xB8,0x06);
	dsi_generic_write_seq(dsi, 0xB9,0x05);
	dsi_generic_write_seq(dsi, 0xBA,0x0f);
	dsi_generic_write_seq(dsi, 0xC7,0x00);
	dsi_generic_write_seq(dsi, 0xCA,0x0e);
	dsi_generic_write_seq(dsi, 0xCB,0x0f);
	dsi_generic_write_seq(dsi, 0xCC,0x04);
	dsi_generic_write_seq(dsi, 0xCD,0x07);
	dsi_generic_write_seq(dsi, 0xCE,0x06);
	dsi_generic_write_seq(dsi, 0xCF,0x05);
	dsi_generic_write_seq(dsi, 0xD0,0x0f);

	dsi_generic_write_seq(dsi, 0x81,0x0f);
	dsi_generic_write_seq(dsi, 0x84,0x0e);
	dsi_generic_write_seq(dsi, 0x85,0x0f);
	dsi_generic_write_seq(dsi, 0x86,0x07);
	dsi_generic_write_seq(dsi, 0x87,0x04);
	dsi_generic_write_seq(dsi, 0x88,0x05);
	dsi_generic_write_seq(dsi, 0x89,0x06);
	dsi_generic_write_seq(dsi, 0x8A,0x00);
	dsi_generic_write_seq(dsi, 0x97,0x0f);
	dsi_generic_write_seq(dsi, 0x9A,0x0e);
	dsi_generic_write_seq(dsi, 0x9B,0x0f);
	dsi_generic_write_seq(dsi, 0x9C,0x07);
	dsi_generic_write_seq(dsi, 0x9D,0x04);
	dsi_generic_write_seq(dsi, 0x9E,0x05);
	dsi_generic_write_seq(dsi, 0x9F,0x06);
	dsi_generic_write_seq(dsi, 0xA0,0x00);

	dsi_generic_write_seq(dsi, 0xFF,0x30);
	dsi_generic_write_seq(dsi, 0xFF,0x52);
	dsi_generic_write_seq(dsi, 0xFF,0x02);
	dsi_generic_write_seq(dsi, 0x01,0x01);
	dsi_generic_write_seq(dsi, 0x02,0xDA);
	dsi_generic_write_seq(dsi, 0x03,0xBA);
	dsi_generic_write_seq(dsi, 0x04,0xA8);
	dsi_generic_write_seq(dsi, 0x05,0x9A);
	dsi_generic_write_seq(dsi, 0x06,0x70);
	dsi_generic_write_seq(dsi, 0x07,0xFF);
	dsi_generic_write_seq(dsi, 0x08,0x91);
	dsi_generic_write_seq(dsi, 0x09,0x90);
	dsi_generic_write_seq(dsi, 0x0A,0xFF);
	dsi_generic_write_seq(dsi, 0x0B,0x8F);
	dsi_generic_write_seq(dsi, 0x0C,0x60);
	dsi_generic_write_seq(dsi, 0x0D,0x58);
	dsi_generic_write_seq(dsi, 0x0E,0x48);
	dsi_generic_write_seq(dsi, 0x0F,0x38);
	dsi_generic_write_seq(dsi, 0x10,0x2B);

	dsi_generic_write_seq(dsi, 0xFF,0x30);
	dsi_generic_write_seq(dsi, 0xFF,0x52);
	dsi_generic_write_seq(dsi, 0xFF,0x00);
	dsi_generic_write_seq(dsi, 0x36,0x02);

	dsi_generic_write_seq(dsi, 0x11,0x00);
	msleep( 200 );

	dsi_generic_write_seq(dsi, 0x29,0x00);
	msleep(10);


	return 0;
}

static const struct drm_display_mode ee0350et_mode = {
	.hdisplay    = 640,
	.hsync_start = 640 + 20,
	.hsync_end   = 640 + 20 + 2,
	.htotal	     = 640 + 20 + 2 + 20,
	.vdisplay    = 480,
	.vsync_start = 480 + 4,
	.vsync_end   = 480 + 4 + 2,
	.vtotal	     = 480 + 4 + 2 + 12,
	//.clock	     = 20378,
	.clock	     = 20000,
	.flags	     = DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC,
	.width_mm    = 70,
	.height_mm   = 52,
};

static const struct nv3051d_panel_desc ee0350et_desc = {
	.mode = &ee0350et_mode,
	.lanes = 2,
	.mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST,
	.format = MIPI_DSI_FMT_RGB888,
	.init_sequence = ee0350et_init_sequence,
};

static int nv3051d_enable(struct drm_panel *panel)
{
	struct nv3051d *ctx = panel_to_nv3051d(panel);
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	int ret;

	ret = ctx->desc->init_sequence(ctx);
	if (ret < 0) {
		DRM_DEV_ERROR(ctx->dev, "Panel init sequence failed: %d\n",
			      ret);
		return ret;
	}

	msleep(20);

	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret < 0) {
		DRM_DEV_ERROR(ctx->dev, "Failed to exit sleep mode: %d\n", ret);
		return ret;
	}

	/* Panel is operational 200 msec after exit sleep mode */
	msleep(200);

	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret)
		return ret;

	/* Panel is operational 10 msec after display on */
	msleep(10);

	DRM_DEV_DEBUG_DRIVER(ctx->dev, "Panel init sequence done\n");

	return 0;
}

static int nv3051d_disable(struct drm_panel *panel)
{
	struct nv3051d *ctx = panel_to_nv3051d(panel);
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	int ret;

	ret = mipi_dsi_dcs_set_display_off(dsi);
	if (ret < 0)
		DRM_DEV_ERROR(ctx->dev,
			      "Failed to turn off the display: %d\n", ret);

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret < 0)
		DRM_DEV_ERROR(ctx->dev,
			      "Failed to enter sleep mode: %d\n", ret);

	return 0;
}

static int nv3051d_unprepare(struct drm_panel *panel)
{
	struct nv3051d *ctx = panel_to_nv3051d(panel);

	if (!ctx->prepared)
		return 0;

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	regulator_disable(ctx->iovcc);
	regulator_disable(ctx->vci);
	ctx->prepared = false;

	return 0;
}

static int nv3051d_prepare(struct drm_panel *panel)
{
	struct nv3051d *ctx = panel_to_nv3051d(panel);
	int ret;

	if (ctx->prepared)
		return 0;

	DRM_DEV_DEBUG_DRIVER(ctx->dev, "Resetting the panel\n");
	ret = regulator_enable(ctx->vci);
	if (ret < 0) {
		DRM_DEV_ERROR(ctx->dev,
			      "Failed to enable vci supply: %d\n", ret);
		return ret;
	}
	ret = regulator_enable(ctx->iovcc);
	if (ret < 0) {
		DRM_DEV_ERROR(ctx->dev,
			      "Failed to enable iovcc supply: %d\n", ret);
		goto disable_vci;
	}

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(20, 40);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	msleep(20);

	ctx->prepared = true;

	return 0;

disable_vci:
	regulator_disable(ctx->vci);
	return ret;
}

static int nv3051d_get_modes(struct drm_panel *panel,
			     struct drm_connector *connector)
{
	struct nv3051d *ctx = panel_to_nv3051d(panel);
	struct drm_display_mode *mode;
	int ret;

	mode = drm_mode_duplicate(connector->dev, &ee0350et_mode);
	if (!mode) {
		DRM_DEV_ERROR(ctx->dev, "Failed to add mode %ux%u@%u\n",
			      ctx->desc->mode->hdisplay, ctx->desc->mode->vdisplay,
			      drm_mode_vrefresh(ctx->desc->mode));
		return -ENOMEM;
	}

	drm_mode_set_name(mode);

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;

	ret = drm_display_info_set_bus_formats(&connector->display_info,
			nv3051d_bus_formats, ARRAY_SIZE(nv3051d_bus_formats));
	if (ret)
		return ret;

	drm_mode_probed_add(connector, mode);

	return 1;
}

static const struct drm_panel_funcs nv3051d_drm_funcs = {
	.disable   = nv3051d_disable,
	.unprepare = nv3051d_unprepare,
	.prepare   = nv3051d_prepare,
	.enable	   = nv3051d_enable,
	.get_modes = nv3051d_get_modes,
};

static int allpixelson_set(void *data, u64 val)
{
	struct nv3051d *ctx = data;
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);

	DRM_DEV_DEBUG_DRIVER(ctx->dev, "Setting all pixels on\n");
	/* Switch to Page 0 */
	dsi_generic_write_seq(dsi, NV3051D_CMD_ENEXTC, 0x30);
	dsi_generic_write_seq(dsi, NV3051D_CMD_ENEXTC, 0x52);
	dsi_generic_write_seq(dsi, NV3051D_CMD_ENEXTC, 0x00);

	dsi_generic_write_seq(dsi, NV3051D_CMD_ALL_PIXEL_ON);
	msleep(val * 1000);
	/* Reset the panel to get video back */
	drm_panel_disable(&ctx->panel);
	drm_panel_unprepare(&ctx->panel);
	drm_panel_prepare(&ctx->panel);
	drm_panel_enable(&ctx->panel);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(allpixelson_fops, NULL,
			allpixelson_set, "%llu\n");

static void nv3051d_debugfs_init(struct nv3051d *ctx)
{
	ctx->debugfs = debugfs_create_dir(DRV_NAME, NULL);

	debugfs_create_file("allpixelson", 0600, ctx->debugfs, ctx,
			    &allpixelson_fops);
}

static void nv3051d_debugfs_remove(struct nv3051d *ctx)
{
	debugfs_remove_recursive(ctx->debugfs);
	ctx->debugfs = NULL;
}

static int nv3051d_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct nv3051d *ctx;
	int ret;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->reset_gpio)) {
		DRM_DEV_ERROR(dev, "cannot get reset gpio\n");
		return PTR_ERR(ctx->reset_gpio);
	}

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;
	ctx->desc = of_device_get_match_data(dev);

	dsi->mode_flags = ctx->desc->mode_flags;
	dsi->format = ctx->desc->format;
	dsi->lanes = ctx->desc->lanes;

	ctx->vci = devm_regulator_get(dev, "vci");
	if (IS_ERR(ctx->vci)) {
		ret = PTR_ERR(ctx->vci);
		if (ret != -EPROBE_DEFER)
			DRM_DEV_ERROR(dev,
				      "Failed to request vci regulator: %d\n",
				      ret);
		return ret;
	}
	ctx->iovcc = devm_regulator_get(dev, "iovcc");
	if (IS_ERR(ctx->iovcc)) {
		ret = PTR_ERR(ctx->iovcc);
		if (ret != -EPROBE_DEFER)
			DRM_DEV_ERROR(dev,
				      "Failed to request iovcc regulator: %d\n",
				      ret);
		return ret;
	}

	drm_panel_init(&ctx->panel, dev, &nv3051d_drm_funcs, DRM_MODE_CONNECTOR_DSI);
	ctx->panel.dev = &dsi->dev;
	ctx->panel.funcs = &nv3051d_drm_funcs;

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		DRM_DEV_ERROR(dev,
			      "mipi_dsi_attach failed (%d). Is host ready?\n",
			      ret);
		drm_panel_remove(&ctx->panel);
		return ret;
	}

	DRM_DEV_INFO(dev, "%ux%u@%u %ubpp dsi %udl - ready\n",
		     ctx->desc->mode->hdisplay, ctx->desc->mode->vdisplay,
		     drm_mode_vrefresh(ctx->desc->mode),
		     mipi_dsi_pixel_format_to_bpp(dsi->format), dsi->lanes);

	nv3051d_debugfs_init(ctx);
	return 0;
}

static void nv3051d_shutdown(struct mipi_dsi_device *dsi)
{
	struct nv3051d *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = drm_panel_unprepare(&ctx->panel);
	if (ret < 0)
		DRM_DEV_ERROR(&dsi->dev, "Failed to unprepare panel: %d\n",
			      ret);

	ret = drm_panel_disable(&ctx->panel);
	if (ret < 0)
		DRM_DEV_ERROR(&dsi->dev, "Failed to disable panel: %d\n",
			      ret);
}

static int nv3051d_remove(struct mipi_dsi_device *dsi)
{
	struct nv3051d *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	nv3051d_shutdown(dsi);

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		DRM_DEV_ERROR(&dsi->dev, "Failed to detach from DSI host: %d\n",
			      ret);

	drm_panel_remove(&ctx->panel);

	nv3051d_debugfs_remove(ctx);

	return 0;
}

static const struct of_device_id nv3051d_of_match[] = {
	{ .compatible = "eagleeyetech,ee0350et-2", .data = &ee0350et_desc },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, nv3051d_of_match);

static struct mipi_dsi_driver nv3051d_driver = {
	.probe	= nv3051d_probe,
	.remove = nv3051d_remove,
	.shutdown = nv3051d_shutdown,
	.driver = {
		.name = DRV_NAME,
		.of_match_table = nv3051d_of_match,
	},
};
module_mipi_dsi_driver(nv3051d_driver);

MODULE_AUTHOR("Patrick Jakob <jakob@fs-net.de>");
MODULE_DESCRIPTION("DRM driver for NewVision NV3051D-T based MIPI DSI panels");
MODULE_LICENSE("GPL v2");
