// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018 Samsung Electronics Co., Ltd
 *
 */

#include <linux/clk.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <drm/drmP.h>
#include <linux/gpio/consumer.h>
#include <linux/of_graph.h>
#include <linux/regulator/consumer.h>
#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

#define FLD_MASK(start, end)    (((1 << ((start) - (end) + 1)) - 1) << (end))
#define FLD_VAL(val, start, end) (((val) << (end)) & FLD_MASK(start, end))

/* PPI layer registers */
#define PPI_STARTPPI		0x0104 /* START control bit */
#define PPI_LPTXTIMECNT		0x0114 /* LPTX timing signal */
#define PPI_LANEENABLE		0x0134 /* Enables each lane */
#define PPI_TX_RX_TA		0x013C /* BTA timing parameters */
#define PPI_D0S_CLRSIPOCOUNT	0x0164 /* Assertion timer for Lane 0 */
#define PPI_D1S_CLRSIPOCOUNT	0x0168 /* Assertion timer for Lane 1 */
#define PPI_D2S_CLRSIPOCOUNT	0x016C /* Assertion timer for Lane 2 */
#define PPI_D3S_CLRSIPOCOUNT	0x0170 /* Assertion timer for Lane 3 */
#define PPI_START_FUNCTION	1

/* DSI layer registers */
#define DSI_STARTDSI		0x0204 /* START control bit of DSI-TX */
#define DSI_LANEENABLE		0x0210 /* Enables each lane */
#define DSI_RX_START		1

/* Video path registers */
#define VP_CTRL			0x0450 /* Video Path Control */
#define VP_CTRL_MSF(v)		FLD_VAL(v, 0, 0) /* Magic square in RGB666 */
#define VP_CTRL_VTGEN(v)	FLD_VAL(v, 4, 4) /* Use chip clock for timing */
#define VP_CTRL_EVTMODE(v)	FLD_VAL(v, 5, 5) /* Event mode */
#define VP_CTRL_RGB888(v)	FLD_VAL(v, 8, 8) /* RGB888 mode */
#define VP_CTRL_VSDELAY(v)	FLD_VAL(v, 31, 20) /* VSYNC delay */
#define VP_CTRL_HSPOL		BIT(17) /* Polarity of HSYNC signal */
#define VP_CTRL_DEPOL		BIT(18) /* Polarity of DE signal */
#define VP_CTRL_VSPOL		BIT(19) /* Polarity of VSYNC signal */
#define VP_HTIM1		0x0454 /* Horizontal Timing Control 1 */
#define VP_HTIM1_HBP(v)		FLD_VAL(v, 24, 16)
#define VP_HTIM1_HSYNC(v)	FLD_VAL(v, 8, 0)
#define VP_HTIM2		0x0458 /* Horizontal Timing Control 2 */
#define VP_HTIM2_HFP(v)		FLD_VAL(v, 24, 16)
#define VP_HTIM2_HACT(v)	FLD_VAL(v, 10, 0)
#define VP_VTIM1		0x045C /* Vertical Timing Control 1 */
#define VP_VTIM1_VBP(v)		FLD_VAL(v, 23, 16)
#define VP_VTIM1_VSYNC(v)	FLD_VAL(v, 7, 0)
#define VP_VTIM2		0x0460 /* Vertical Timing Control 2 */
#define VP_VTIM2_VFP(v)		FLD_VAL(v, 23, 16)
#define VP_VTIM2_VACT(v)	FLD_VAL(v, 10, 0)
#define VP_VFUEN		0x0464 /* Video Frame Timing Update Enable */

/* LVDS registers */
#define LV_MX0003		0x0480 /* Mux input bit 0 to 3 */
#define LV_MX0407		0x0484 /* Mux input bit 4 to 7 */
#define LV_MX0811		0x0488 /* Mux input bit 8 to 11 */
#define LV_MX1215		0x048C /* Mux input bit 12 to 15 */
#define LV_MX1619		0x0490 /* Mux input bit 16 to 19 */
#define LV_MX2023		0x0494 /* Mux input bit 20 to 23 */
#define LV_MX2427		0x0498 /* Mux input bit 24 to 27 */
#define LV_MX(b0, b1, b2, b3)	(FLD_VAL(b0, 4, 0) | FLD_VAL(b1, 12, 8) | \
				FLD_VAL(b2, 20, 16) | FLD_VAL(b3, 28, 24))

/* Input bit numbers used in mux registers */
enum {
	LVI_R0,
	LVI_R1,
	LVI_R2,
	LVI_R3,
	LVI_R4,
	LVI_R5,
	LVI_R6,
	LVI_R7,
	LVI_G0,
	LVI_G1,
	LVI_G2,
	LVI_G3,
	LVI_G4,
	LVI_G5,
	LVI_G6,
	LVI_G7,
	LVI_B0,
	LVI_B1,
	LVI_B2,
	LVI_B3,
	LVI_B4,
	LVI_B5,
	LVI_B6,
	LVI_B7,
	LVI_HS,
	LVI_VS,
	LVI_DE,
	LVI_L0
};

#define LV_CFG			0x049C /* LVDS Configuration */
#define LV_PHY0			0x04A0 /* LVDS PHY 0 */
#define LV_PHY0_RST(v)		FLD_VAL(v, 22, 22) /* PHY reset */
#define LV_PHY0_IS(v)		FLD_VAL(v, 15, 14)
#define LV_PHY0_ND(v)		FLD_VAL(v, 4, 0) /* Frequency range select */
#define LV_PHY0_PRBS_ON(v)	FLD_VAL(v, 20, 16) /* Clock/Data Flag pins */

/* System registers */
#define SYS_RST			0x0504 /* System Reset */
#define SYS_ID			0x0580 /* System ID */

#define SYS_RST_I2CS		BIT(0) /* Reset I2C-Slave controller */
#define SYS_RST_I2CM		BIT(1) /* Reset I2C-Master controller */
#define SYS_RST_LCD		BIT(2) /* Reset LCD controller */
#define SYS_RST_BM		BIT(3) /* Reset Bus Management controller */
#define SYS_RST_DSIRX		BIT(4) /* Reset DSI-RX and App controller */
#define SYS_RST_REG		BIT(5) /* Reset Register module */

#define LPX_PERIOD		2
#define TTA_SURE		3
#define TTA_GET			0x20000

/* Lane enable PPI and DSI register bits */
#define LANEENABLE_CLEN		BIT(0)
#define LANEENABLE_L0EN		BIT(1)
#define LANEENABLE_L1EN		BIT(2)
#define LANEENABLE_L2EN		BIT(3)
#define LANEENABLE_L3EN		BIT(4)

/* LVCFG fields */
#define LV_CFG_LVEN		BIT(0)
#define LV_CFG_LVDLINK		BIT(1)
#define LV_CFG_CLKPOL1		BIT(2)
#define LV_CFG_CLKPOL2		BIT(3)

static const char * const tc358764_supplies[] = {
	"vddc", "vddio", "vddlvds"
};

static const u32 tc358764_bus_formats[] = {
	MEDIA_BUS_FMT_RGB888_1X24,
	MEDIA_BUS_FMT_RGB666_1X18,
	MEDIA_BUS_FMT_RGB565_1X16,
};

struct tc358764 {
	struct device *dev;
	struct regulator_bulk_data supplies[ARRAY_SIZE(tc358764_supplies)];
	struct gpio_desc *gpio_reset;
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	//struct backlight_device *backlight;

	int error;

	bool prepared;
	bool enabled;

	struct videomode vm;
	u32 width_mm;
	u32 height_mm;
        u32 mode_flags;

	struct clk *extclk;
	u32 clk_rate;
};

#if 0
static void tc358764_read(struct tc358764 *ctx, u16 addr, u32 *val)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	const struct mipi_dsi_host_ops *ops = dsi->host->ops;
	struct mipi_dsi_msg msg = {
		.type = MIPI_DSI_GENERIC_READ_REQUEST_2_PARAM,
		.channel = dsi->channel,
		.flags = MIPI_DSI_MSG_USE_LPM,
		.tx_buf = &addr,
		.tx_len = 2,
		.rx_buf = val,
		.rx_len = 4
	};
	ssize_t ret;

	if (!ops || !ops->transfer)
		return;

	if (ctx->error)
		return;

	addr = cpu_to_le16(addr);

	ret = ops->transfer(dsi->host, &msg);
	if (ret >= 0)
		*val = le32_to_cpu(*val);
}
#endif

int tc358764_write(struct tc358764 *ctx, u16 addr, u32 val)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	const struct mipi_dsi_host_ops *ops = dsi->host->ops;
	u8 data[6];
	int ret;
	struct mipi_dsi_msg msg = {
		.type = MIPI_DSI_GENERIC_LONG_WRITE,
		.channel = dsi->channel,
		.flags = MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_REQ_ACK,
		.tx_buf = data,
		.tx_len = 6
	};

	if (!ops || !ops->transfer)
		return -EINVAL;

	data[0] = addr;
	data[1] = addr >> 8;
	data[2] = val;
	data[3] = val >> 8;
	data[4] = val >> 16;
	data[5] = val >> 24;

	ret = ops->transfer(dsi->host, &msg);

	return ret;
}

static inline struct tc358764 *panel_to_tc358764(struct drm_panel *panel)
{
	return container_of(panel, struct tc358764, panel);
}

static void tc358764_reset(struct tc358764 *ctx)
{
	msleep(20);
	gpiod_set_value(ctx->gpio_reset, 1);
	msleep(20);
	gpiod_set_value(ctx->gpio_reset, 0);
	msleep(40);
}

static int tc358764_configure_regulators(struct tc358764 *ctx)
{
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(ctx->supplies); ++i)
		ctx->supplies[i].supply = tc358764_supplies[i];

	ret = devm_regulator_bulk_get(ctx->dev, ARRAY_SIZE(ctx->supplies),
				      ctx->supplies);
	if (ret < 0)
		dev_err(ctx->dev, "failed to get regulators: %d\n", ret);

	return ret;
}

static int tc358764_panel_prepare(struct drm_panel *panel)
{
	struct tc358764 *ctx = panel_to_tc358764(panel);
	int ret;

	if (ctx->prepared)
		return 0;

	ret = regulator_bulk_enable(ARRAY_SIZE(ctx->supplies), ctx->supplies);
	if (ret < 0)
		dev_err(ctx->dev, "error enabling regulators (%d)\n", ret);

	tc358764_reset(ctx);

	ctx->prepared = true;

	return 0;
}

static int tc358764_panel_unprepare(struct drm_panel *panel)
{
	struct tc358764 *ctx = panel_to_tc358764(panel);
	struct device *dev = ctx->dev;
	int ret;

	if (!ctx->prepared)
		return 0;

	if (ctx->enabled) {
		DRM_DEV_ERROR(dev, "Panel still enabled!\n");
		return -EPERM;
	}

	ret = regulator_bulk_disable(ARRAY_SIZE(ctx->supplies), ctx->supplies);
	if (ret < 0)
		dev_err(ctx->dev, "error disabling regulators (%d)\n", ret);

	ctx->prepared = false;

	return 0;
}

static int tc358764_panel_enable(struct drm_panel *panel)
{
	struct tc358764 *ctx = panel_to_tc358764(panel);
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct device *dev = &dsi->dev;

	if (ctx->enabled)
		return 0;

	if (!ctx->prepared) {
		DRM_DEV_ERROR(dev, "Panel not prepared!\n");
		return -EPERM;
	}

	ctx->enabled = true;

	return 0;

}

static int tc358764_panel_disable(struct drm_panel *panel)
{
	struct tc358764 *ctx = panel_to_tc358764(panel);
#if 0
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct device *dev = &dsi->dev;
	int ret;
#endif

	if (!ctx->enabled)
		return 0;

#if 0

	ret = mipi_dsi_dcs_set_display_off(dsi);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to set display OFF (%d)\n", ret);
		return ret;
    }

	usleep_range(60000, 70000);

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to enter sleep mode (%d)\n", ret);
		return ret;
	}

	usleep_range(60000, 70000);

	rad->backlight->props.power = FB_BLANK_POWERDOWN;
	backlight_update_status(rad->backlight);
#endif

	ctx->enabled = false;

	return 0;
}

static int tc358764_get_modes(struct drm_panel *panel)
{
	struct tc358764 *ctx = panel_to_tc358764(panel);
	struct device *dev = ctx->dev;
	struct drm_connector *connector = panel->connector;
	struct drm_display_mode *mode;
	u32 *bus_flags = &connector->display_info.bus_flags;
	int ret;

	mode = drm_mode_create(connector->dev);
	if (!mode) {
		DRM_DEV_ERROR(dev, "Failed to create display mode!\n");
		return 0;
	}

	drm_display_mode_from_videomode(&ctx->vm, mode);
	mode->width_mm = ctx->width_mm;
	mode->height_mm = ctx->height_mm;
	connector->display_info.width_mm = ctx->width_mm;
	connector->display_info.height_mm = ctx->height_mm;
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;

	if (ctx->vm.flags & DISPLAY_FLAGS_DE_HIGH)
		*bus_flags |= DRM_BUS_FLAG_DE_HIGH;
	if (ctx->vm.flags & DISPLAY_FLAGS_DE_LOW)
		*bus_flags |= DRM_BUS_FLAG_DE_LOW;
	if (ctx->vm.flags & DISPLAY_FLAGS_PIXDATA_NEGEDGE)
		*bus_flags |= DRM_BUS_FLAG_PIXDATA_NEGEDGE;
	if (ctx->vm.flags & DISPLAY_FLAGS_PIXDATA_POSEDGE)
		*bus_flags |= DRM_BUS_FLAG_PIXDATA_POSEDGE;

	ret = drm_display_info_set_bus_formats(&connector->display_info,
			tc358764_bus_formats, ARRAY_SIZE(tc358764_bus_formats));
	if (ret)
		return ret;

	drm_mode_probed_add(panel->connector, mode);

	return 1;
}

static const struct drm_panel_funcs tc358764_panel_funcs = {
	.prepare = tc358764_panel_prepare,
	.unprepare = tc358764_panel_unprepare,
	.enable = tc358764_panel_enable,
	.disable = tc358764_panel_disable,
	.get_modes = tc358764_get_modes,
};


/*
 * The clock might range from 66MHz (30Hz refresh rate)
 * to 132MHz (60Hz refresh rate)
 */

static const struct display_timing default_timing = {

	.pixelclock = { 28000000, 29500000, 32000000 },
	.hactive = { 800, 800, 800 },
	.hfront_porch = { 61, 91, 141 },
	.hback_porch = { 60, 90, 140 },
	.hsync_len = { 12, 12, 12 },
	.vactive = { 480, 480, 480 },
	.vfront_porch = { 4, 9, 30 },
	.vback_porch = { 4, 8, 28 },
	.vsync_len = { 2, 2, 2 },
	.flags =  DISPLAY_FLAGS_HSYNC_HIGH |
		 	  DISPLAY_FLAGS_VSYNC_HIGH |
			  DISPLAY_FLAGS_DE_HIGH,
};

static int tc358764_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct tc358764 *ctx;
	struct device_node *np = dev->of_node;
	struct device_node *timings;
	int ret;

	ctx = devm_kzalloc(dev, sizeof(struct tc358764), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dsi = dsi;
	ctx->dev = dev;

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;

	/*
	* 'display-timings' is optional, so verify if the node is present
	* before calling of_get_videomode so we won't get console error
	* messages
	*/
	timings = of_get_child_by_name(np, "display-timings");
	if (timings) {
		of_node_put(timings);
		ret = of_get_videomode(np, &ctx->vm, 0);
	} else {
		videomode_from_timing(&default_timing, &ctx->vm);
	}

	of_property_read_u32(np, "panel-width-mm", &ctx->width_mm);
	of_property_read_u32(np, "panel-height-mm", &ctx->height_mm);

	if(of_property_read_u32(np, "flags", &ctx->mode_flags)) {
	  /* Default mode flags for mipi dsi phy:
	   * Use sync event burst mode...
	   */
	  dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST
		| MIPI_DSI_MODE_VIDEO_AUTO_VERT | MIPI_DSI_MODE_LPM;

	}else{
	  dsi->mode_flags = (MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_LPM) | ctx->mode_flags;
	}
	    

	/* Set initial values for the sensor struct. */
	ctx->extclk = devm_clk_get(dev, "ext-clk");
	if (IS_ERR(ctx->extclk)) {
		/* assuming clock enabled by default */
		ctx->extclk = NULL;
		dev_err(dev, "extclk clock-frequency missing or invalid\n");
		return PTR_ERR(ctx->extclk);
	}



	ret = of_property_read_u32(dev->of_node, "ext-clk-rate",
					&(ctx->clk_rate));
	if (ret) {
		ctx->clk_rate = 33500000;
	}

	/* Set extclk before clk on */
	ret = clk_set_rate(ctx->extclk, ctx->clk_rate);
	if (ret < 0) {
		dev_info(dev, "set rate %d failed. Ret: %u\n",ctx->clk_rate, ret);
	}
	else
	{
		ret = clk_prepare_enable(ctx->extclk);
		if (ret < 0) {
			dev_err(dev, "%s: enable ext clk fail\n", __func__);
			return -EINVAL;
		}
	}

	ctx->gpio_reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->gpio_reset)) {
		dev_err(dev, "no reset GPIO pin provided\n");
		return PTR_ERR(ctx->gpio_reset);
	}

	ret = tc358764_configure_regulators(ctx);
	if (ret < 0)
		return ret;

	drm_panel_init(&ctx->panel);
	ctx->panel.funcs = &tc358764_panel_funcs;
	ctx->panel.dev = dev;

	ret = drm_panel_add(&ctx->panel);

	if (ret < 0)
		return ret;

	dev_set_drvdata(dev, &ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		drm_panel_remove(&ctx->panel);
		dev_err(dev, "failed to attach dsi err:%d\n", ret);
	}

	return ret;
}

static int tc358764_remove(struct mipi_dsi_device *dsi)
{
	struct tc358764 *ctx = mipi_dsi_get_drvdata(dsi);
	struct device *dev = &dsi->dev;
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		DRM_DEV_ERROR(dev, "Failed to detach from host (%d)\n",
			ret);

	drm_panel_detach(&ctx->panel);

	if (ctx->panel.dev)
		drm_panel_remove(&ctx->panel);

	return 0;
}

static void tc358764_panel_shutdown(struct mipi_dsi_device *dsi)
{
	struct tc358764 *ctx = mipi_dsi_get_drvdata(dsi);

	tc358764_panel_disable(&ctx->panel);
	tc358764_panel_unprepare(&ctx->panel);
}

static const struct of_device_id tc358764_of_match[] = {
	{ .compatible = "toshiba,tc358764" },
	{ }
};
MODULE_DEVICE_TABLE(of, tc358764_of_match);

static struct mipi_dsi_driver tc358764_driver = {
	.probe = tc358764_probe,
	.remove = tc358764_remove,
	.shutdown = tc358764_panel_shutdown,
	.driver = {
		.name = "tc358764",
		.owner = THIS_MODULE,
		.of_match_table = tc358764_of_match,
	},
};
module_mipi_dsi_driver(tc358764_driver);

MODULE_DESCRIPTION("MIPI-DSI based Driver for TC358764 DSI/LVDS Bridge");
MODULE_LICENSE("GPL v2");
