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
#include <linux/gpio/consumer.h>
#include <linux/of_graph.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
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

static const char * const tc358775_supplies[] = {
	"vddc", "vddio", "vddlvds", "vlcd"
};

static const u32 tc358775_bus_formats[] = {
	MEDIA_BUS_FMT_RGB888_1X24,
	MEDIA_BUS_FMT_RGB666_1X18,
	MEDIA_BUS_FMT_RGB565_1X16,
};

struct tc_def_reg_config {
	u32 address;
	u32 data;
};

const static struct tc_def_reg_config tc_defconfig [] = {
	/* dsi basic parameters in lp mode */
	{0x013c, 0x00010002},
	{0x0114, 0x00000001},
	{0x0164, 0x00000002},
	{0x0168, 0x00000002},
	{0x016C, 0x00000002},
	{0x0170, 0x00000002},
	{0x0134, 0x0000001F},
	{0x0210, 0x0000001F},
	{0x0104, 0x00000001},
	{0x0204, 0x00000001},
	{0x0450, 0x03F00120},
	{0x0454, 0x002E0001},
	{0x0458, 0x00D20320},
	{0x045C, 0x00170002},
	{0x0460, 0x001601E0},
	{0x0464, 0x00000001},

	{0x04A0, 0x0044802D}, /* after 100us delay*/
	{0x04A0, 0x0004802D},
	{0x0504, 0x00000004},
/* color mapping settings	*/
	{0x0480, 0x03020100},
	{0x0484, 0x08050704},
	{0x0488, 0x0F0E0A09},
	{0x048C, 0x100D0C0B},
	{0x0490, 0x12111716},
	{0x0494, 0x1B151413},
	{0x0498, 0x061A1918},
 /* LVDS enable */
	{0x049C, 0x00000031},
};

struct tc358775 {
	struct device *dev;
	struct regulator_bulk_data supplies[ARRAY_SIZE(tc358775_supplies)];
	struct gpio_desc *gpio_reset;
	struct gpio_desc *gpio_stby;

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
    struct tc_def_reg_config *def_config;
	size_t defconfig_size;

};

static int tc358775_clear_error(struct tc358775 *ctx)
{
	int ret = ctx->error;

	ctx->error = 0;
	return ret;
}

static void tc358775_read(struct tc358775 *ctx, u16 addr, u32 *val)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;

	if (ctx->error)
		return;

	cpu_to_le16s(&addr);
	ret = mipi_dsi_generic_read(dsi, &addr, sizeof(addr), val, sizeof(*val));
	if (ret >= 0)
		le32_to_cpus(val);

}

static void tc358775_write(struct tc358775 *ctx, u16 addr, u32 val)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;
	u8 data[6];

	if (ctx->error)
		return;

	data[0] = addr;
	data[1] = addr >> 8;
	data[2] = val;
	data[3] = val >> 8;
	data[4] = val >> 16;
	data[5] = val >> 24;

	ret = mipi_dsi_generic_write(dsi, data, sizeof(data));
	if (ret < 0)
		ctx->error = ret;
}

static int tc358775_init(struct tc358775 *ctx)
{
	u32 v = 0;
	int i;
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	tc358775_read(ctx, SYS_ID, &v);
	if (ctx->error)
		return tc358775_clear_error(ctx);
	dev_info(ctx->dev, "ID: %#x\n", v);

	/* write config */
	for(i = 0; i < ctx->defconfig_size; i++)
	{
		/* write default config */
		tc358775_write(ctx, ctx->def_config[i].address, ctx->def_config[i].data);


		if( (0x04A0 == ctx->def_config[i].address) && (ctx->def_config[i].data & 0x00400000))
		{
			udelay(100);
		}
	}

	return tc358775_clear_error(ctx);
}

static inline struct tc358775 *panel_to_tc358775(struct drm_panel *panel)
{
	return container_of(panel, struct tc358775, panel);
}

static void tc358775_reset(struct tc358775 *ctx)
{
	gpiod_set_value_cansleep(ctx->gpio_reset, 0);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(ctx->gpio_reset, 1);
	usleep_range(1000, 2000);
}

static int tc358775_configure_regulators(struct tc358775 *ctx)
{
	int i, ret;
	for (i = 0; i < ARRAY_SIZE(ctx->supplies); ++i)
		ctx->supplies[i].supply = tc358775_supplies[i];

	ret = devm_regulator_bulk_get(ctx->dev, ARRAY_SIZE(ctx->supplies),
				      ctx->supplies);
	if (ret < 0)
		dev_err(ctx->dev, "failed to get regulators: %d\n", ret);

	return ret;
}

static int tc358775_panel_prepare(struct drm_panel *panel)
{
	struct tc358775 *ctx = panel_to_tc358775(panel);
	int ret;

	if (ctx->prepared)
		return 0;

	ret = regulator_bulk_enable(ARRAY_SIZE(ctx->supplies), ctx->supplies);
	if (ret < 0)
		dev_err(ctx->dev, "error enabling regulators (%d)\n", ret);

	if (ctx->extclk != NULL) {
	/* Set extclk before clk on */
	ret = clk_set_rate(ctx->extclk, ctx->clk_rate);
		if (ret < 0) {
			dev_info(ctx->dev, "set rate %d failed. Ret: %u\n",ctx->clk_rate, ret);
		}
		else
		{
			ret = clk_prepare_enable(ctx->extclk);
			if (ret < 0) {
				dev_err(ctx->dev, "%s: enable ext clk fail\n", __func__);
				return -EINVAL;
			}
		}
	}
	udelay(500);
	if (ctx->gpio_stby){
		gpiod_set_value_cansleep(ctx->gpio_stby, 1);
		udelay(50);
	}
	tc358775_reset(ctx);

	ctx->prepared = true;

	return 0;
}

static int tc358775_panel_unprepare(struct drm_panel *panel)
{
	struct tc358775 *ctx = panel_to_tc358775(panel);
	struct device *dev = ctx->dev;
	int ret;

	if (!ctx->prepared)
		return 0;

	if (ctx->enabled) {
		dev_err(dev, "Panel still enabled!\n");
		return -EPERM;
	}
	gpiod_set_value_cansleep(ctx->gpio_stby, 0);

	if (ctx->extclk != NULL) {
		clk_disable_unprepare(ctx->extclk);
	}
	ret = regulator_bulk_disable(ARRAY_SIZE(ctx->supplies), ctx->supplies);
	if (ret < 0)
		dev_err(ctx->dev, "error disabling regulators (%d)\n", ret);

	ctx->prepared = false;

	return 0;
}

static int tc358775_panel_enable(struct drm_panel *panel)
{
	struct tc358775 *ctx = panel_to_tc358775(panel);
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct device *dev = &dsi->dev;

	if (ctx->enabled)
		return 0;

	if (!ctx->prepared) {
		dev_err(dev, "Panel not prepared!\n");
		return -EPERM;
	}
	tc358775_init(ctx);
	ctx->enabled = true;

	return 0;

}

static int tc358775_panel_disable(struct drm_panel *panel)
{
	struct tc358775 *ctx = panel_to_tc358775(panel);
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

static int tc358775_get_modes(struct drm_panel *panel,
				struct drm_connector *connector)
{
	struct tc358775 *ctx = panel_to_tc358775(panel);
	struct device *dev = ctx->dev;
	struct drm_display_mode *mode;
	u32 *bus_flags = &connector->display_info.bus_flags;
	int ret;

	mode = drm_mode_create(connector->dev);
	if (!mode) {
		dev_err(dev, "Failed to create display mode!\n");
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
	if (ctx->vm.flags & DRM_BUS_FLAG_PIXDATA_DRIVE_NEGEDGE)
		*bus_flags |= DRM_BUS_FLAG_PIXDATA_DRIVE_NEGEDGE;
	if (ctx->vm.flags & DRM_BUS_FLAG_PIXDATA_DRIVE_POSEDGE )
		*bus_flags |= DRM_BUS_FLAG_PIXDATA_DRIVE_POSEDGE ;

	ret = drm_display_info_set_bus_formats(&connector->display_info,
			tc358775_bus_formats, ARRAY_SIZE(tc358775_bus_formats));
	if (ret)
		return ret;

	drm_mode_probed_add(connector, mode);

	return 1;
}

static const struct drm_panel_funcs tc358775_panel_funcs = {
	.prepare = tc358775_panel_prepare,
	.unprepare = tc358775_panel_unprepare,
	.enable = tc358775_panel_enable,
	.disable = tc358775_panel_disable,
	.get_modes = tc358775_get_modes,
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

static int tc_bridge_read_data_from_dt(struct tc358775 *tc)
{
	struct property *prop;
	int addresses[100];
	int values[100];
	int err = 0, i;
	int addrs_len = 0, vals_len = 0;
	struct device_node *np = tc->dev->of_node;
	struct device *dev = tc->dev;

	/* gets the I2C register address to write */
	prop = of_find_property(np, "reg,address", NULL);
	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;

	addrs_len = prop->length / sizeof(int);

    tc->def_config = devm_kzalloc(dev, sizeof(*tc->def_config)*addrs_len, GFP_KERNEL);

	err = of_property_read_u32_array(np, "reg,address", addresses, addrs_len);
	if (err && (err != -EINVAL)) {
		dev_err(dev, "Unable to read 'reg,address'\n");
		goto DT_READ_FAIL;
	}

	/* gets the I2C register data to write */
	prop = of_find_property(np, "reg,value", NULL);
	if (!prop)
	{
		err = -EINVAL;
		goto DT_READ_FAIL;
	}

	if (!prop->value)
	{
		err = -ENODATA;
		goto DT_READ_FAIL;
	}

	vals_len = prop->length / sizeof(u32);
	if (addrs_len != vals_len) {
		dev_err(dev, "invalid 'reg,value' length should be same as address.\n");
		goto DT_READ_FAIL;
	}

	err = of_property_read_u32_array(np, "reg,value", values, vals_len);
	if (err && (err != -EINVAL)) {
		dev_err(dev, "Unable to read 'reg,value'\n");
		goto DT_READ_FAIL;
	}

	for(i=0;i<addrs_len; i++)
	{
		tc->def_config[i].address = addresses[i];
		tc->def_config[i].data = values[i];
	}
	tc->defconfig_size = addrs_len;

	return err;

DT_READ_FAIL:
    devm_kfree(dev, tc->def_config);
	tc->def_config = 0;
    return err;
}

static int tc358775_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct tc358775 *ctx;
	struct device_node *np = dev->of_node;
	struct device_node *timings;
	int ret;

	ctx = devm_kzalloc(dev, sizeof(struct tc358775), GFP_KERNEL);
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
	if (!IS_ERR(ctx->extclk)) {
		dev_info(dev, "Using extclk\n");

		ret = of_property_read_u32(dev->of_node, "ext-clk-rate",
						&(ctx->clk_rate));
		if (ret) {
			ctx->clk_rate = 33500000;
		}
	}
	else {
		ctx->extclk = NULL;
		dev_info(dev, "Using MIPI-DSI clk\n");
	}

	ctx->gpio_stby = devm_gpiod_get_optional(dev, "stby", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->gpio_stby)) {
		dev_err(dev, "no gpio_stby GPIO pin provided\n");
		return PTR_ERR(ctx->gpio_reset);
	}

	ctx->gpio_reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->gpio_reset)) {
		dev_err(dev, "no reset GPIO pin provided\n");
		return PTR_ERR(ctx->gpio_reset);
	}

	ret = tc358775_configure_regulators(ctx);
	if (ret < 0)
		return ret;

	/* use default config if reg values aren't defined in dt */
	if(tc_bridge_read_data_from_dt(ctx))
	{
		dev_info(ctx->dev, " Use default config. \n");
		ctx->def_config = (struct tc_def_reg_config*) tc_defconfig;
		ctx->defconfig_size = 27;
	}

	drm_panel_init(&ctx->panel, dev, &tc358775_panel_funcs,
		       DRM_MODE_CONNECTOR_DSI);

	drm_panel_add(&ctx->panel);

	dev_set_drvdata(dev, &ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		drm_panel_remove(&ctx->panel);
		dev_err(dev, "failed to attach dsi err:%d\n", ret);
	}

	return ret;
}

static int tc358775_remove(struct mipi_dsi_device *dsi)
{
	struct tc358775 *ctx = mipi_dsi_get_drvdata(dsi);
	struct device *dev = &dsi->dev;
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(dev, "Failed to detach from host (%d)\n",
			ret);

	if (ctx->panel.dev)
		drm_panel_remove(&ctx->panel);

	return 0;
}

static void tc358775_panel_shutdown(struct mipi_dsi_device *dsi)
{
	struct tc358775 *ctx = mipi_dsi_get_drvdata(dsi);

	tc358775_panel_disable(&ctx->panel);
	tc358775_panel_unprepare(&ctx->panel);
}

static const struct of_device_id tc358775_of_match[] = {
	{ .compatible = "toshiba,fus-tc358775" },
	{ }
};
MODULE_DEVICE_TABLE(of, tc358775_of_match);

static struct mipi_dsi_driver tc358775_driver = {
	.probe = tc358775_probe,
	.remove = tc358775_remove,
	.shutdown = tc358775_panel_shutdown,
	.driver = {
		.name = "panel-tc358775",
		.owner = THIS_MODULE,
		.of_match_table = tc358775_of_match,
	},
};
module_mipi_dsi_driver(tc358775_driver);

MODULE_DESCRIPTION("MIPI-DSI based Driver for TC358775 DSI/LVDS Bridge");
MODULE_LICENSE("GPL v2");
