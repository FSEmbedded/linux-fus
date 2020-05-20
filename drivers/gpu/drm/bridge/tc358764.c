/*
 * tc358764 DSI to LVDS bridge driver
 *
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
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_dp_helper.h>
#include <drm/drm_edid.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>

/* Registers */

/* Display Parallel Interface */
#define DPIPXLFMT		0x0440
#define VS_POL_ACTIVE_LOW		(1 << 10)
#define HS_POL_ACTIVE_LOW		(1 << 9)
#define DE_POL_ACTIVE_HIGH		(0 << 8)
#define SUB_CFG_TYPE_CONFIG1		(0 << 2) /* LSB aligned */
#define SUB_CFG_TYPE_CONFIG2		(1 << 2) /* Loosely Packed */
#define SUB_CFG_TYPE_CONFIG3		(2 << 2) /* LSB aligned 8-bit */
#define DPI_BPP_RGB888			(0 << 0)
#define DPI_BPP_RGB666			(1 << 0)
#define DPI_BPP_RGB565			(2 << 0)

/* Video Path */
#define VPCTRL0			0x0450
#define OPXLFMT_RGB666			(0 << 8)
#define OPXLFMT_RGB888			(1 << 8)
#define FRMSYNC_DISABLED		(0 << 4) /* Video Timing Gen Disabled */
#define FRMSYNC_ENABLED			(1 << 4) /* Video Timing Gen Enabled */
#define MSF_DISABLED			(0 << 0) /* Magic Square FRC disabled */
#define MSF_ENABLED			(1 << 0) /* Magic Square FRC enabled */
#define HTIM01			0x0454
#define HTIM02			0x0458
#define VTIM01			0x045c
#define VTIM02			0x0460
#define VFUEN0			0x0464
#define VFUEN				BIT(0)   /* Video Frame Timing Upload */

/* System */
#define TC_IDREG		0x0580
#define SYSCTRL			0x0510
#define DP0_AUDSRC_NO_INPUT		(0 << 3)
#define DP0_AUDSRC_I2S_RX		(1 << 3)
#define DP0_VIDSRC_NO_INPUT		(0 << 0)
#define DP0_VIDSRC_DSI_RX		(1 << 0)
#define DP0_VIDSRC_DPI_RX		(2 << 0)
#define DP0_VIDSRC_COLOR_BAR		(3 << 0)

/* Control */
#define DP0CTL			0x0600
#define VID_MN_GEN			BIT(6)   /* Auto-generate M/N values */
#define EF_EN				BIT(5)   /* Enable Enhanced Framing */
#define VID_EN				BIT(1)   /* Video transmission enable */
#define DP_EN				BIT(0)   /* Enable DPTX function */

/* Clocks */
#define DP0_VIDMNGEN0		0x0610
#define DP0_VIDMNGEN1		0x0614
#define DP0_VMNGENSTATUS	0x0618

/* Main Channel */
#define DP0_SECSAMPLE		0x0640
#define DP0_VIDSYNCDELAY	0x0644
#define DP0_TOTALVAL		0x0648
#define DP0_STARTVAL		0x064c
#define DP0_ACTIVEVAL		0x0650
#define DP0_SYNCVAL		0x0654
#define DP0_MISC		0x0658
#define TU_SIZE_RECOMMENDED		(63) /* LSCLK cycles per TU */
#define BPC_6				(0 << 5)
#define BPC_8				(1 << 5)

/* AUX channel */
#define DP0_AUXCFG0		0x0660
#define DP0_AUXCFG1		0x0664
#define AUX_RX_FILTER_EN		BIT(16)

#define DP0_AUXADDR		0x0668
#define DP0_AUXWDATA(i)		(0x066c + (i) * 4)
#define DP0_AUXRDATA(i)		(0x067c + (i) * 4)
#define DP0_AUXSTATUS		0x068c
#define AUX_STATUS_MASK			0xf0
#define AUX_STATUS_SHIFT		4
#define AUX_TIMEOUT			BIT(1)
#define AUX_BUSY			BIT(0)
#define DP0_AUXI2CADR		0x0698

/* Link Training */
#define DP0_SRCCTRL		0x06a0
#define DP0_SRCCTRL_SCRMBLDIS		BIT(13)
#define DP0_SRCCTRL_EN810B		BIT(12)
#define DP0_SRCCTRL_NOTP		(0 << 8)
#define DP0_SRCCTRL_TP1			(1 << 8)
#define DP0_SRCCTRL_TP2			(2 << 8)
#define DP0_SRCCTRL_LANESKEW		BIT(7)
#define DP0_SRCCTRL_SSCG		BIT(3)
#define DP0_SRCCTRL_LANES_1		(0 << 2)
#define DP0_SRCCTRL_LANES_2		(1 << 2)
#define DP0_SRCCTRL_BW27		(1 << 1)
#define DP0_SRCCTRL_BW162		(0 << 1)
#define DP0_SRCCTRL_AUTOCORRECT		BIT(0)
#define DP0_LTSTAT		0x06d0
#define LT_LOOPDONE			BIT(13)
#define LT_STATUS_MASK			(0x1f << 8)
#define LT_CHANNEL1_EQ_BITS		(DP_CHANNEL_EQ_BITS << 4)
#define LT_INTERLANE_ALIGN_DONE		BIT(3)
#define LT_CHANNEL0_EQ_BITS		(DP_CHANNEL_EQ_BITS)
#define DP0_SNKLTCHGREQ		0x06d4
#define DP0_LTLOOPCTRL		0x06d8
#define DP0_SNKLTCTRL		0x06e4

/* PHY */
#define DP_PHY_CTRL		0x0800
#define DP_PHY_RST			BIT(28)  /* DP PHY Global Soft Reset */
#define BGREN				BIT(25)  /* AUX PHY BGR Enable */
#define PWR_SW_EN			BIT(24)  /* PHY Power Switch Enable */
#define PHY_M1_RST			BIT(12)  /* Reset PHY1 Main Channel */
#define PHY_RDY				BIT(16)  /* PHY Main Channels Ready */
#define PHY_M0_RST			BIT(8)   /* Reset PHY0 Main Channel */
#define PHY_A0_EN			BIT(1)   /* PHY Aux Channel0 Enable */
#define PHY_M0_EN			BIT(0)   /* PHY Main Channel0 Enable */

/* PLL */
#define DP0_PLLCTRL		0x0900
#define DP1_PLLCTRL		0x0904	/* not defined in DS */
#define PXL_PLLCTRL		0x0908
#define PLLUPDATE			BIT(2)
#define PLLBYP				BIT(1)
#define PLLEN				BIT(0)
#define PXL_PLLPARAM		0x0914
#define IN_SEL_REFCLK			(0 << 14)
#define SYS_PLLPARAM		0x0918
#define REF_FREQ_38M4			(0 << 8) /* 38.4 MHz */
#define REF_FREQ_19M2			(1 << 8) /* 19.2 MHz */
#define REF_FREQ_26M			(2 << 8) /* 26 MHz */
#define REF_FREQ_13M			(3 << 8) /* 13 MHz */
#define SYSCLK_SEL_LSCLK		(0 << 4)
#define LSCLK_DIV_1			(0 << 0)
#define LSCLK_DIV_2			(1 << 0)

/* Test & Debug */
#define TSTCTL			0x0a00
#define PLL_DBG			0x0a04

static bool tc_test_pattern;
module_param_named(test, tc_test_pattern, bool, 0644);

struct tc_def_reg_config {
	u32 address;
	u32 data;
};

static const u32 tc358764_bus_formats[] = {
	MEDIA_BUS_FMT_RGB888_1X24,
	MEDIA_BUS_FMT_RGB666_1X18,
	MEDIA_BUS_FMT_RGB565_1X16,
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

struct tc_edp_link {
	struct drm_dp_link	base;
	u8			assr;
	int			scrambler_dis;
	int			spread;
	int			coding8b10b;
	u8			swing;
	u8			preemp;
};

struct tc_data {
	struct device		*dev;
	struct regmap		*regmap;

	struct drm_bridge	bridge;
	struct drm_connector	connector;
	struct drm_panel	*panel;

	/* link settings */
	struct tc_edp_link	link;

	/* display edid */
	struct edid		*edid;
	/* current mode */
	struct drm_display_mode	*mode;

	u32			rev;
	u8			assr;

	struct gpio_desc	*sd_gpio;
	struct gpio_desc	*reset_gpio;
	struct clk		*refclk;
	struct clk		*extclk;
	struct clk		*clk_cfg;

    struct tc_def_reg_config *def_config;
	size_t defconfig_size;

};

static inline struct tc_data *bridge_to_tc(struct drm_bridge *b)
{
	return container_of(b, struct tc_data, bridge);
}

static inline struct tc_data *connector_to_tc(struct drm_connector *c)
{
	return container_of(c, struct tc_data, connector);
}

/* Simple macros to avoid repeated error checks */
#define tc_write(reg, var)					\
	do {							\
		ret = regmap_write(tc->regmap, reg, var);	\
		if (ret)					\
			goto err;				\
	} while (0)
#define tc_read(reg, var)					\
	do {							\
		ret = regmap_read(tc->regmap, reg, var);	\
		if (ret)					\
			goto err;				\
	} while (0)

static inline int tc_poll_timeout(struct regmap *map, unsigned int addr,
				  unsigned int cond_mask,
				  unsigned int cond_value,
				  unsigned long sleep_us, u64 timeout_us)
{
	ktime_t timeout = ktime_add_us(ktime_get(), timeout_us);
	unsigned int val;
	int ret;

	for (;;) {
		ret = regmap_read(map, addr, &val);
		if (ret)
			break;
		if ((val & cond_mask) == cond_value)
			break;
		if (timeout_us && ktime_compare(ktime_get(), timeout) > 0) {
			ret = regmap_read(map, addr, &val);
			break;
		}
		if (sleep_us)
			usleep_range((sleep_us >> 2) + 1, sleep_us);
	}
	return ret ?: (((val & cond_mask) == cond_value) ? 0 : -ETIMEDOUT);
}

static const char * const training_pattern1_errors[] = {
	"No errors",
	"Max voltage reached error",
	"Loop counter expired error",
	"res", "res", "res"
};

static const char * const training_pattern2_errors[] = {
	"No errors",
	"Clock recovery failed error",
	"Loop counter expired error",
	"res", "res", "res"
};

static u32 tc_srcctrl(struct tc_data *tc)
{
	/*
	 * No training pattern, skew lane 1 data by two LSCLK cycles with
	 * respect to lane 0 data, AutoCorrect Mode = 0
	 */
	u32 reg = DP0_SRCCTRL_NOTP | DP0_SRCCTRL_LANESKEW;

	if (tc->link.scrambler_dis)
		reg |= DP0_SRCCTRL_SCRMBLDIS;	/* Scrambler Disabled */
	if (tc->link.coding8b10b)
		/* Enable 8/10B Encoder (TxData[19:16] not used) */
		reg |= DP0_SRCCTRL_EN810B;
	if (tc->link.spread)
		reg |= DP0_SRCCTRL_SSCG;	/* Spread Spectrum Enable */
	if (tc->link.base.num_lanes == 2)
		reg |= DP0_SRCCTRL_LANES_2;	/* Two Main Channel Lanes */
	if (tc->link.base.rate != 162000)
		reg |= DP0_SRCCTRL_BW27;	/* 2.7 Gbps link */
	return reg;
}

static void tc_wait_pll_lock(struct tc_data *tc)
{
	/* Wait for PLL to lock: up to 2.09 ms, depending on refclk */
	usleep_range(3000, 6000);
}

static int tc_pxl_pll_en(struct tc_data *tc, u32 refclk, u32 pixelclock)
{
	int ret;
	int i_pre, best_pre = 1;
	int i_post, best_post = 1;
	int div, best_div = 1;
	int mul, best_mul = 1;
	int delta, best_delta;
	int ext_div[] = {1, 2, 3, 5, 7};
	int best_pixelclock = 0;
	int vco_hi = 0;

	dev_dbg(tc->dev, "PLL: requested %d pixelclock, ref %d\n", pixelclock,
		refclk);
	best_delta = pixelclock;
	/* Loop over all possible ext_divs, skipping invalid configurations */
	for (i_pre = 0; i_pre < ARRAY_SIZE(ext_div); i_pre++) {
		/*
		 * refclk / ext_pre_div should be in the 1 to 200 MHz range.
		 * We don't allow any refclk > 200 MHz, only check lower bounds.
		 */
		if (refclk / ext_div[i_pre] < 1000000)
			continue;
		for (i_post = 0; i_post < ARRAY_SIZE(ext_div); i_post++) {
			for (div = 1; div <= 16; div++) {
				u32 clk;
				u64 tmp;

				tmp = pixelclock * ext_div[i_pre] *
				      ext_div[i_post] * div;
				do_div(tmp, refclk);
				mul = tmp;

				/* Check limits */
				if ((mul < 1) || (mul > 128))
					continue;

				clk = (refclk / ext_div[i_pre] / div) * mul;
				/*
				 * refclk * mul / (ext_pre_div * pre_div)
				 * should be in the 150 to 650 MHz range
				 */
				if ((clk > 650000000) || (clk < 150000000))
					continue;

				clk = clk / ext_div[i_post];
				delta = clk - pixelclock;

				if (abs(delta) < abs(best_delta)) {
					best_pre = i_pre;
					best_post = i_post;
					best_div = div;
					best_mul = mul;
					best_delta = delta;
					best_pixelclock = clk;
				}
			}
		}
	}
	if (best_pixelclock == 0) {
		dev_err(tc->dev, "Failed to calc clock for %d pixelclock\n",
			pixelclock);
		return -EINVAL;
	}

	dev_dbg(tc->dev, "PLL: got %d, delta %d\n", best_pixelclock,
		best_delta);
	dev_dbg(tc->dev, "PLL: %d / %d / %d * %d / %d\n", refclk,
		ext_div[best_pre], best_div, best_mul, ext_div[best_post]);

	/* if VCO >= 300 MHz */
	if (refclk / ext_div[best_pre] / best_div * best_mul >= 300000000)
		vco_hi = 1;
	/* see DS */
	if (best_div == 16)
		best_div = 0;
	if (best_mul == 128)
		best_mul = 0;

	/* Power up PLL and switch to bypass */
	tc_write(PXL_PLLCTRL, PLLBYP | PLLEN);

	tc_write(PXL_PLLPARAM,
		 (vco_hi << 24) |		/* For PLL VCO >= 300 MHz = 1 */
		 (ext_div[best_pre] << 20) |	/* External Pre-divider */
		 (ext_div[best_post] << 16) |	/* External Post-divider */
		 IN_SEL_REFCLK |		/* Use RefClk as PLL input */
		 (best_div << 8) |		/* Divider for PLL RefClk */
		 (best_mul << 0));		/* Multiplier for PLL */

	/* Force PLL parameter update and disable bypass */
	tc_write(PXL_PLLCTRL, PLLUPDATE | PLLEN);

	tc_wait_pll_lock(tc);

	return 0;
err:
	return ret;
}

static int tc_pxl_pll_dis(struct tc_data *tc)
{
	/* Enable PLL bypass, power down PLL */
	return regmap_write(tc->regmap, PXL_PLLCTRL, PLLBYP);
}

static int tc_stream_clock_calc(struct tc_data *tc)
{
	int ret;
	/*
	 * If the Stream clock and Link Symbol clock are
	 * asynchronous with each other, the value of M changes over
	 * time. This way of generating link clock and stream
	 * clock is called Asynchronous Clock mode. The value M
	 * must change while the value N stays constant. The
	 * value of N in this Asynchronous Clock mode must be set
	 * to 2^15 or 32,768.
	 *
	 * LSCLK = 1/10 of high speed link clock
	 *
	 * f_STRMCLK = M/N * f_LSCLK
	 * M/N = f_STRMCLK / f_LSCLK
	 *
	 */
	tc_write(DP0_VIDMNGEN1, 32768);

	return 0;
err:
	return ret;
}

static int tc_set_video_mode(struct tc_data *tc, struct drm_display_mode *mode)
{
	int ret;
	int vid_sync_dly;
	int max_tu_symbol;

	int left_margin = mode->htotal - mode->hsync_end;
	int right_margin = mode->hsync_start - mode->hdisplay;
	int hsync_len = mode->hsync_end - mode->hsync_start;
	int upper_margin = mode->vtotal - mode->vsync_end;
	int lower_margin = mode->vsync_start - mode->vdisplay;
	int vsync_len = mode->vsync_end - mode->vsync_start;

	/*
	 * Recommended maximum number of symbols transferred in a transfer unit:
	 * DIV_ROUND_UP((input active video bandwidth in bytes) * tu_size,
	 *              (output active video bandwidth in bytes))
	 * Must be less than tu_size.
	 */
	max_tu_symbol = TU_SIZE_RECOMMENDED - 1;

	dev_dbg(tc->dev, "set mode %dx%d\n",
		mode->hdisplay, mode->vdisplay);
	dev_dbg(tc->dev, "H margin %d,%d sync %d\n",
		left_margin, right_margin, hsync_len);
	dev_dbg(tc->dev, "V margin %d,%d sync %d\n",
		upper_margin, lower_margin, vsync_len);
	dev_dbg(tc->dev, "total: %dx%d\n", mode->htotal, mode->vtotal);


	/*
	 * LCD Ctl Frame Size
	 * datasheet is not clear of vsdelay in case of DPI
	 * assume we do not need any delay when DPI is a source of
	 * sync signals
	 */
	tc_write(VPCTRL0, (0 << 20) /* VSDELAY */ |
		 OPXLFMT_RGB888 | FRMSYNC_DISABLED | MSF_DISABLED);
	tc_write(HTIM01, (ALIGN(left_margin, 2) << 16) | /* H back porch */
			 (ALIGN(hsync_len, 2) << 0));	 /* Hsync */
	tc_write(HTIM02, (ALIGN(right_margin, 2) << 16) |  /* H front porch */
			 (ALIGN(mode->hdisplay, 2) << 0)); /* width */
	tc_write(VTIM01, (upper_margin << 16) |		/* V back porch */
			 (vsync_len << 0));		/* Vsync */
	tc_write(VTIM02, (lower_margin << 16) |		/* V front porch */
			 (mode->vdisplay << 0));	/* height */
	tc_write(VFUEN0, VFUEN);		/* update settings */

	/* Test pattern settings */
	tc_write(TSTCTL,
		 (120 << 24) |	/* Red Color component value */
		 (20 << 16) |	/* Green Color component value */
		 (99 << 8) |	/* Blue Color component value */
		 (1 << 4) |	/* Enable I2C Filter */
		 (2 << 0) |	/* Color bar Mode */
		 0);

	/* DP Main Stream Attributes */
	vid_sync_dly = hsync_len + left_margin + mode->hdisplay;
	tc_write(DP0_VIDSYNCDELAY,
		 (max_tu_symbol << 16) |	/* thresh_dly */
		 (vid_sync_dly << 0));

	tc_write(DP0_TOTALVAL, (mode->vtotal << 16) | (mode->htotal));

	tc_write(DP0_STARTVAL,
		 ((upper_margin + vsync_len) << 16) |
		 ((left_margin + hsync_len) << 0));

	tc_write(DP0_ACTIVEVAL, (mode->vdisplay << 16) | (mode->hdisplay));

	tc_write(DP0_SYNCVAL, (vsync_len << 16) | (hsync_len << 0));

	tc_write(DPIPXLFMT, VS_POL_ACTIVE_LOW | HS_POL_ACTIVE_LOW |
		 DE_POL_ACTIVE_HIGH | SUB_CFG_TYPE_CONFIG1 | DPI_BPP_RGB888);

	tc_write(DP0_MISC, (max_tu_symbol << 23) | (TU_SIZE_RECOMMENDED << 16) |
			   BPC_8);

	return 0;
err:
	return ret;
}

static int tc_link_training(struct tc_data *tc, int pattern)
{
	const char * const *errors;
	u32 srcctrl = tc_srcctrl(tc) | DP0_SRCCTRL_SCRMBLDIS |
		      DP0_SRCCTRL_AUTOCORRECT;
	int timeout;
	int retry;
	u32 value;
	int ret;

	if (pattern == DP_TRAINING_PATTERN_1) {
		srcctrl |= DP0_SRCCTRL_TP1;
		errors = training_pattern1_errors;
	} else {
		srcctrl |= DP0_SRCCTRL_TP2;
		errors = training_pattern2_errors;
	}

	/* Set DPCD 0x102 for Training Part 1 or 2 */
	tc_write(DP0_SNKLTCTRL, DP_LINK_SCRAMBLING_DISABLE | pattern);

	tc_write(DP0_LTLOOPCTRL,
		 (0x0f << 28) |	/* Defer Iteration Count */
		 (0x0f << 24) |	/* Loop Iteration Count */
		 (0x0d << 0));	/* Loop Timer Delay */

	retry = 5;
	do {
		/* Set DP0 Training Pattern */
		tc_write(DP0_SRCCTRL, srcctrl);

		/* Enable DP0 to start Link Training */
		tc_write(DP0CTL, DP_EN);

		/* wait */
		timeout = 1000;
		do {
			tc_read(DP0_LTSTAT, &value);
			udelay(1);
		} while ((!(value & LT_LOOPDONE)) && (--timeout));
		if (timeout == 0) {
			dev_err(tc->dev, "Link training timeout!\n");
		} else {
			int pattern = (value >> 11) & 0x3;
			int error = (value >> 8) & 0x7;

			dev_dbg(tc->dev,
				"Link training phase %d done after %d uS: %s\n",
				pattern, 1000 - timeout, errors[error]);
			if (pattern == DP_TRAINING_PATTERN_1 && error == 0)
				break;
			if (pattern == DP_TRAINING_PATTERN_2) {
				value &= LT_CHANNEL1_EQ_BITS |
					 LT_INTERLANE_ALIGN_DONE |
					 LT_CHANNEL0_EQ_BITS;
				/* in case of two lanes */
				if ((tc->link.base.num_lanes == 2) &&
				    (value == (LT_CHANNEL1_EQ_BITS |
					       LT_INTERLANE_ALIGN_DONE |
					       LT_CHANNEL0_EQ_BITS)))
					break;
				/* in case of one line */
				if ((tc->link.base.num_lanes == 1) &&
				    (value == (LT_INTERLANE_ALIGN_DONE |
					       LT_CHANNEL0_EQ_BITS)))
					break;
			}
		}
		/* restart */
		tc_write(DP0CTL, 0);
		usleep_range(10, 20);
	} while (--retry);
	if (retry == 0) {
		dev_err(tc->dev, "Failed to finish training phase %d\n",
			pattern);
	}

	return 0;
err:
	return ret;
}

static int tc_main_link_setup(struct tc_data *tc)
{
	struct device *dev = tc->dev;
	unsigned int rate;
	u32 dp_phy_ctrl;
	int timeout;
	u32 value;
	int ret;

	/* display mode should be set at this point */
	if (!tc->mode)
		return -EINVAL;

	/* from excel file - DP0_SrcCtrl */
	tc_write(DP0_SRCCTRL, DP0_SRCCTRL_SCRMBLDIS | DP0_SRCCTRL_EN810B |
		 DP0_SRCCTRL_LANESKEW | DP0_SRCCTRL_LANES_2 |
		 DP0_SRCCTRL_BW27 | DP0_SRCCTRL_AUTOCORRECT);
	/* from excel file - DP1_SrcCtrl */
	tc_write(0x07a0, 0x00003083);

	rate = clk_get_rate(tc->refclk);
	switch (rate) {
	case 38400000:
		value = REF_FREQ_38M4;
		break;
	case 26000000:
		value = REF_FREQ_26M;
		break;
	case 19200000:
		value = REF_FREQ_19M2;
		break;
	case 13000000:
		value = REF_FREQ_13M;
		break;
	default:
		return -EINVAL;
	}
	value |= SYSCLK_SEL_LSCLK | LSCLK_DIV_2;
	tc_write(SYS_PLLPARAM, value);
	/* Setup Main Link */
	dp_phy_ctrl = BGREN | PWR_SW_EN | BIT(2) | PHY_A0_EN |  PHY_M0_EN;
	tc_write(DP_PHY_CTRL, dp_phy_ctrl);
	msleep(100);

	/* PLL setup */
	tc_write(DP0_PLLCTRL, PLLUPDATE | PLLEN);
	tc_wait_pll_lock(tc);

	tc_write(DP1_PLLCTRL, PLLUPDATE | PLLEN);
	tc_wait_pll_lock(tc);

	/* PXL PLL setup */
	if (tc_test_pattern) {
		ret = tc_pxl_pll_en(tc, clk_get_rate(tc->refclk),
				    1000 * tc->mode->clock);
		if (ret)
			goto err;
	}

	/* Reset/Enable Main Links */
	dp_phy_ctrl |= DP_PHY_RST | PHY_M1_RST | PHY_M0_RST;
	tc_write(DP_PHY_CTRL, dp_phy_ctrl);
	usleep_range(100, 200);
	dp_phy_ctrl &= ~(DP_PHY_RST | PHY_M1_RST | PHY_M0_RST);
	tc_write(DP_PHY_CTRL, dp_phy_ctrl);

	timeout = 1000;
	do {
		tc_read(DP_PHY_CTRL, &value);
		udelay(1);
	} while ((!(value & PHY_RDY)) && (--timeout));

	if (timeout == 0) {
		dev_err(dev, "timeout waiting for phy become ready");
		return -ETIMEDOUT;
	}

	/* Set misc: 8 bits per color */
	ret = regmap_update_bits(tc->regmap, DP0_MISC, BPC_8, BPC_8);
	if (ret)
		goto err;

	ret = tc_link_training(tc, DP_TRAINING_PATTERN_1);
	if (ret)
		goto err;

	ret = tc_link_training(tc, DP_TRAINING_PATTERN_2);
	if (ret)
		goto err;

	/* Clear Training Pattern, set AutoCorrect Mode = 1 */
	tc_write(DP0_SRCCTRL, tc_srcctrl(tc) | DP0_SRCCTRL_AUTOCORRECT);

	ret = tc_set_video_mode(tc, tc->mode);
	if (ret)
		goto err;

	/* Set M/N */
	ret = tc_stream_clock_calc(tc);
	if (ret)
		goto err;

	return 0;
err:
	return ret;
}

static int tc_main_link_stream(struct tc_data *tc, int state)
{
	int ret;
	u32 value;

	dev_dbg(tc->dev, "stream: %d\n", state);

	if (state) {
		value = VID_MN_GEN | DP_EN;
		if (tc->link.base.capabilities & DP_LINK_CAP_ENHANCED_FRAMING)
			value |= EF_EN;
		tc_write(DP0CTL, value);
		/*
		 * VID_EN assertion should be delayed by at least N * LSCLK
		 * cycles from the time VID_MN_GEN is enabled in order to
		 * generate stable values for VID_M. LSCLK is 270 MHz or
		 * 162 MHz, VID_N is set to 32768 in  tc_stream_clock_calc(),
		 * so a delay of at least 203 us should suffice.
		 */
		usleep_range(500, 1000);
		value |= VID_EN;
		tc_write(DP0CTL, value);
		/* Set input interface */
		value = DP0_AUDSRC_NO_INPUT;
		if (tc_test_pattern)
			value |= DP0_VIDSRC_COLOR_BAR;
		else
			value |= DP0_VIDSRC_DPI_RX;
		tc_write(SYSCTRL, value);
	} else {
		tc_write(DP0CTL, 0);
	}

	return 0;
err:
	return ret;
}

static void tc_bridge_pre_enable(struct drm_bridge *bridge)
{
	struct tc_data *tc = bridge_to_tc(bridge);

	drm_panel_prepare(tc->panel);
}

static void tc_bridge_enable(struct drm_bridge *bridge)
{
	struct tc_data *tc = bridge_to_tc(bridge);
	int ret;

	ret = tc_main_link_setup(tc);
	if (ret < 0) {
		dev_err(tc->dev, "main link setup error: %d\n", ret);
		return;
	}

	ret = tc_main_link_stream(tc, 1);
	if (ret < 0) {
		dev_err(tc->dev, "main link stream start error: %d\n", ret);
		return;
	}

	drm_panel_enable(tc->panel);
}

static void tc_bridge_disable(struct drm_bridge *bridge)
{
	struct tc_data *tc = bridge_to_tc(bridge);
	int ret;

	drm_panel_disable(tc->panel);

	ret = tc_main_link_stream(tc, 0);
	if (ret < 0)
		dev_err(tc->dev, "main link stream stop error: %d\n", ret);
}

static void tc_bridge_post_disable(struct drm_bridge *bridge)
{
	struct tc_data *tc = bridge_to_tc(bridge);

	drm_panel_unprepare(tc->panel);
}

static bool tc_bridge_mode_fixup(struct drm_bridge *bridge,
				 const struct drm_display_mode *mode,
				 struct drm_display_mode *adj)
{
	/* Fixup sync polarities, both hsync and vsync are active low */
	adj->flags = mode->flags;
	adj->flags |= (DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC);
	adj->flags &= ~(DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC);

	return true;
}

static int tc_connector_mode_valid(struct drm_connector *connector,
				   struct drm_display_mode *mode)
{

	/* DPI interface clock limitation: upto 154 MHz */
	if (mode->clock > 154000)
		return MODE_CLOCK_HIGH;

	return MODE_OK;
}

static void tc_bridge_mode_set(struct drm_bridge *bridge,
			       struct drm_display_mode *mode,
			       struct drm_display_mode *adj)
{
	struct tc_data *tc = bridge_to_tc(bridge);

	tc->mode = mode;
}

static int tc_connector_get_modes(struct drm_connector *connector)
{
	struct tc_data *tc = connector_to_tc(connector);
	struct edid *edid = 0;
	unsigned int count;

	drm_display_info_set_bus_formats(&connector->display_info,
			tc358764_bus_formats, ARRAY_SIZE(tc358764_bus_formats));

	if (tc->panel && tc->panel->funcs && tc->panel->funcs->get_modes) {
		count = tc->panel->funcs->get_modes(tc->panel);
		if (count > 0)
			return count;
	}

	kfree(tc->edid);
	tc->edid = edid;
	if (!edid)
		return 0;

	drm_connector_update_edid_property(connector, edid);
	count = drm_add_edid_modes(connector, edid);

	return count;
}

static void tc_connector_set_polling(struct tc_data *tc,
				     struct drm_connector *connector)
{
	/* TODO: add support for HPD */
	connector->polled = DRM_CONNECTOR_POLL_CONNECT |
			    DRM_CONNECTOR_POLL_DISCONNECT;
}

static struct drm_encoder *
tc_connector_best_encoder(struct drm_connector *connector)
{
	struct tc_data *tc = connector_to_tc(connector);

	return tc->bridge.encoder;
}

static const struct drm_connector_helper_funcs tc_connector_helper_funcs = {
	.get_modes = tc_connector_get_modes,
	.mode_valid = tc_connector_mode_valid,
	.best_encoder = tc_connector_best_encoder,
};

static const struct drm_connector_funcs tc_connector_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static int tc_bridge_attach(struct drm_bridge *bridge)
{
	u32 bus_format = MEDIA_BUS_FMT_RGB888_1X24;
	struct tc_data *tc = bridge_to_tc(bridge);
	struct drm_device *drm = bridge->dev;
	int ret;

	/* Create eDP connector */
	drm_connector_helper_add(&tc->connector, &tc_connector_helper_funcs);
	ret = drm_connector_init(drm, &tc->connector, &tc_connector_funcs,
				 DRM_MODE_CONNECTOR_eDP);
	if (ret)
		return ret;

	if (tc->panel)
		drm_panel_attach(tc->panel, &tc->connector);

	drm_display_info_set_bus_formats(&tc->connector.display_info,
					 &bus_format, 1);
	drm_connector_attach_encoder(&tc->connector, tc->bridge.encoder);

	return 0;
}

static int tc_bridge_read_data_from_dt(struct tc_data *tc)
{
	struct property *prop;
	int addresses[100];
	int values[100];
	int err = 0, i;
	int addrs_len = 0, vals_len = 0;
	struct device_node *np = tc->bridge.of_node;
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

static const struct drm_bridge_funcs tc_bridge_funcs = {
	.attach = tc_bridge_attach,
	.mode_set = tc_bridge_mode_set,
	.pre_enable = tc_bridge_pre_enable,
	.enable = tc_bridge_enable,
	.disable = tc_bridge_disable,
	.post_disable = tc_bridge_post_disable,
	.mode_fixup = tc_bridge_mode_fixup,
};

static const struct regmap_range tc_volatile_ranges[] = {
	regmap_reg_range(DP0_AUXWDATA(0), DP0_AUXSTATUS),
	regmap_reg_range(DP0_LTSTAT, DP0_SNKLTCHGREQ),
	regmap_reg_range(DP_PHY_CTRL, DP_PHY_CTRL),
	regmap_reg_range(DP0_PLLCTRL, PXL_PLLCTRL),
	regmap_reg_range(VFUEN0, VFUEN0),
};

static const struct regmap_access_table tc_volatile_table = {
	.yes_ranges = tc_volatile_ranges,
	.n_yes_ranges = ARRAY_SIZE(tc_volatile_ranges),
};

static const struct regmap_config tc_regmap_config = {
	.name = "tc358764",
	.reg_bits = 16,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = PLL_DBG,
	.cache_type = REGCACHE_RBTREE,
	.volatile_table = &tc_volatile_table,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
};

static int tc_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct tc_data *tc;
	int ret, i;

	tc = devm_kzalloc(dev, sizeof(*tc), GFP_KERNEL);
	if (!tc)
		return -ENOMEM;

	tc->dev = dev;

	/* port@2 is the output port */
	ret = drm_of_find_panel_or_bridge(dev->of_node, 2, 0, &tc->panel, NULL);
	if (ret && ret != -ENODEV)
		return ret;


	/* Shut down GPIO is optional */
	tc->sd_gpio = devm_gpiod_get_optional(dev, "shutdown", GPIOD_OUT_HIGH);
	if (IS_ERR(tc->sd_gpio))
		return PTR_ERR(tc->sd_gpio);

	if (tc->sd_gpio) {
		gpiod_set_value_cansleep(tc->sd_gpio, 0);
		usleep_range(5000, 10000);
	}

	tc->refclk = devm_clk_get(dev, "ref");
	if (IS_ERR(tc->refclk)) {
		ret = PTR_ERR(tc->refclk);
		dev_err(dev, "Failed to get refclk: %d\n", ret);
		return ret;
	}

	/* Reset GPIO is optional */
	tc->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(tc->reset_gpio))
		return PTR_ERR(tc->reset_gpio);

	if (tc->reset_gpio) {
		gpiod_set_value_cansleep(tc->reset_gpio, 0);
		msleep(40);
	}

	tc->regmap = devm_regmap_init_i2c(client, &tc_regmap_config);
	if (IS_ERR(tc->regmap)) {
		ret = PTR_ERR(tc->regmap);
		dev_err(dev, "Failed to initialize regmap: %d\n", ret);
		return ret;
	}

	ret = regmap_read(tc->regmap, TC_IDREG, &tc->rev);
	if (ret) {
		dev_err(tc->dev, "can not read device ID: %d\n", ret);
		return ret;
	}

	if ((tc->rev != 0x6500)) {
		dev_err(tc->dev, "invalid device ID: 0x%08x\n", tc->rev);
		return -EINVAL;
	}

	dev_info(tc->dev, " Rev: 0x%08x \n", tc->rev);

	tc_connector_set_polling(tc, &tc->connector);

	tc->bridge.funcs = &tc_bridge_funcs;
	tc->bridge.of_node = dev->of_node;
	drm_bridge_add(&tc->bridge);

	i2c_set_clientdata(client, tc);

	/* use default config if reg values aren't defined in dt */
	if(tc_bridge_read_data_from_dt(tc))
	{
		dev_info(tc->dev, " Use default config. \n");
		tc->def_config = (struct tc_def_reg_config*) tc_defconfig;
		tc->defconfig_size = 27;
	}

	/* write config */
	for(i = 0; i < tc->defconfig_size; i++)
	{
		/* write default config */
		ret = regmap_write(tc->regmap, tc->def_config[i].address, tc->def_config[i].data);
		if (ret) {
			dev_err(tc->dev, "can not write entry %d ret: %d\n", i, ret);
			return ret;
		}

		if( (0x04A0 == tc->def_config[i].address) && (tc->def_config[i].data & 0x00400000))
		{
			udelay(1000);
		}
	}

	return 0;
}

static int tc_remove(struct i2c_client *client)
{
	struct tc_data *tc = i2c_get_clientdata(client);

	drm_bridge_remove(&tc->bridge);

	tc_pxl_pll_dis(tc);

	return 0;
}

static const struct i2c_device_id tc358764_i2c_ids[] = {
	{ "tc358764", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tc358764_i2c_ids);

static const struct of_device_id tc358764_of_ids[] = {
	{ .compatible = "toshiba,tc358764", },
	{ }
};
MODULE_DEVICE_TABLE(of, tc358764_of_ids);

static struct i2c_driver tc358764_driver = {
	.driver = {
		.name = "tc358764",
		.of_match_table = tc358764_of_ids,
	},
	.id_table = tc358764_i2c_ids,
	.probe = tc_probe,
	.remove	= tc_remove,
};
module_i2c_driver(tc358764_driver);

MODULE_DESCRIPTION("tc358764 DSI to LVDS bridge driver");
MODULE_LICENSE("GPL v2");
