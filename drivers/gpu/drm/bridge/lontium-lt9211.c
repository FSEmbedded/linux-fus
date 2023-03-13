// SPDX-License-Identifier: GPL-2.0
/*
 * Author: Philipp Gerbach , <gerbach@fs-net.de>
 *
 * Based on example code by Lontium.
 *
 * This is a driver for the lontium-lt9211 as MIPI2RGB converter.
 */

#include <linux/gpio/consumer.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_of.h>
#include <drm/drm_mipi_dsi.h>
#include <video/display_timing.h>

#include "lontium-lt9211.h"

/* TODO: */
//#define ENABLE_IRQ

static inline struct lt9211 *bridge_to_lt9211(struct drm_bridge *b)
{
	return container_of(b, struct lt9211, bridge);
}

static int LT9211_SystemInt(struct lt9211 *lt9211)
{
	int ret;
	const struct reg_sequence seq[] = {
		/* Switch to 0x82xx */
		{ 0xff, 0x82 },
		/* Set reg 0x01 */
		{ 0x01, 0x18 },
		/* Switch to 0x86xx */
		{ 0xff, 0x86 },
		/* Set reg 0x06 */
		{ 0x06, 0x61 },
		/* fm for sys_clk */
		{ 0x07, 0xa8 },
		/* Switch to 0x87xx */
		{ 0xff, 0x87 },
		/* TXPLL vco cur_sel hardware change wait timer setting. */
		{ 0x14, 0x08 },
		/* TXPLL vco cur_sel hardware change wait timer setting. */
		{ 0x15, 0x00 },
		/* The step setting for TXPLL vco cur_sel */
		{ 0x18, 0x0f },
		/* ?? */
		{ 0x22, 0x08 },
		/* ?? */
		{ 0x23, 0x00 },
		/* ?? */
		{ 0x26, 0x0f },
	};

	ret = regmap_multi_reg_write(lt9211->regmap, seq, ARRAY_SIZE(seq));

	return ret;
}

/* helper function to access bus_formats */
static struct drm_connector *get_connector(struct drm_encoder *encoder)
{
	struct drm_device *dev = encoder->dev;
	struct drm_connector *connector;

	list_for_each_entry(connector, &dev->mode_config.connector_list, head)
		if (connector->encoder == encoder)
			return connector;

	return NULL;
}

static void get_bus_format(struct lt9211 *lt9211)
{
	struct drm_bridge *bridge = &lt9211->bridge;
	struct drm_connector *connector = get_connector(bridge->encoder);

	usleep_range(200, 300);

	switch (connector->display_info.bus_formats[0]) {
	case MEDIA_BUS_FMT_RGB888_1X7X4_SPWG:
	case MEDIA_BUS_FMT_RGB888_1X7X4_JEIDA:
	case MEDIA_BUS_FMT_RGB888_1X24:
	case MEDIA_BUS_FMT_RGB888_3X8:
		/* RGB888 */
		lt9211->bpc = 8;
		break;
	case MEDIA_BUS_FMT_RGB666_1X7X3_SPWG:
	case MEDIA_BUS_FMT_RGB666_1X18:
		/* RGB666 */
		lt9211->bpc = 6;
		break;
	default:
		lt9211->bpc = 0;
	};
}

static int LT9211_Pattern(struct lt9211 *lt9211)
{
	u32 pclk_khz;
	u8 dessc_pll_post_div = 0;
	u32 pcr_m = 0, pcr_k = 0;
	u16 hbp = lt9211->mode.htotal - lt9211->mode.hsync_end;
	u16 vbp = lt9211->mode.vtotal - lt9211->mode.vsync_end;
	u16 hs = lt9211->mode.hsync_end - lt9211->mode.hsync_start;
	u16 vs = lt9211->mode.vsync_end - lt9211->mode.vsync_start;
	int ret = 0;

	struct reg_sequence seq[] = {
		{ 0xff, 0xf9 },
		{ 0x3e, 0x80 },
		{ 0xff, 0x85 },
		/* Chip active RX source select: Chip Video pattern gen. */
		{ 0x88, 0xc0 },
		 /* Display black wire in white background.
		  *(R,G,B) DP_DATA, 00, 00;
		  */
		{ 0xa1, 0x53 },
		/* Pattern data value set. */
		{ 0xa2, 0xf0 },
		{ 0xa3, (u8)((hs + hbp) / 256) },
		{ 0xa4, (u8)((hs + hbp) % 256) },
		{ 0xa5, (u8)((vs + vbp) % 256) },
		{ 0xa6, (u8)(lt9211->mode.hdisplay / 256) },
		{ 0xa7, (u8)(lt9211->mode.hdisplay % 256) },
		{ 0xa8, (u8)(lt9211->mode.vdisplay / 256) },
		{ 0xa9, (u8)(lt9211->mode.vdisplay % 256) },
		{ 0xaa, (u8)(lt9211->mode.htotal / 256) },
		{ 0xab, (u8)(lt9211->mode.htotal % 256) },
		{ 0xac, (u8)(lt9211->mode.vtotal / 256) },
		{ 0xad, (u8)(lt9211->mode.vtotal % 256) },
		{ 0xae, (u8)(hs / 256) },
		{ 0xaf, (u8)(hs % 256) },
		{ 0xb0, (u8)(vs % 256) },
		{ 0xff, 0x82 },
		{ 0x2d, 0x48 },
		{ 0x35, 0x83 },
		{ 0xff, 0xd0 },
		{ 0x2d, 0x7f },
		{ 0x31, 0x00 },
		{ 0x26, 0x80 | ((u8) pcr_m) },
		{ 0x27, (u8)((pcr_k >> 16) & 0xff) },
		{ 0x28, (u8)((pcr_k >> 8) & 0xff) },
		{ 0x29, (u8)(pcr_k & 0xff) },
	};

	/* Due to the reason that polarity invert function is not
	 * available in pattern mode, we try to simulate it, by duplicating
	 * hsync in case of negative HSYNC/VSYNC (hs *=2).
	 * So the display should get the correct cycle.
	 */
	if ((lt9211->mode.flags & DRM_MODE_FLAG_NHSYNC) &&
		(lt9211->mode.flags & DRM_MODE_FLAG_NVSYNC)) {
		hs *= 2;
		seq[6].def = (u8)((hs + hbp) / 256);
		seq[7].def = (u8)((hs + hbp) % 256);
		seq[17].def = (u8)(hs / 256);
		seq[18].def = (u8)(hs % 256);
	}

	pclk_khz = lt9211->mode.clock;

	if (pclk_khz < PCLK_KHZ_44000) {
		seq[22].def = 0x83;
		dessc_pll_post_div = 16;
	}
	else if (pclk_khz < PCLK_KHZ_88000) {
		seq[22].def = 0x82;
		dessc_pll_post_div = 8;
	}
	else if (pclk_khz < PCLK_KHZ_176000) {
		seq[22].def = 0x81;
		dessc_pll_post_div = 4;
	}
	else if (pclk_khz < PCLK_KHZ_352000) {
		seq[22].def = 0x80;
		dessc_pll_post_div = 0;
	}
	else {
		dev_err(lt9211->dev, "PCLK to high!\n");
		return -EINVAL;
	}

	pcr_m = (pclk_khz * dessc_pll_post_div) / 25;
	pcr_k = pcr_m % 1000;
	pcr_m = pcr_m / 1000;

	pcr_k <<= 14;

	seq[26].def = 0x80 | ((u8) pcr_m);
	seq[27].def = (u8)((pcr_k >> 16) & 0xff);
	seq[28].def = (u8)((pcr_k >> 8) & 0xff);
	seq[29].def = (u8)(pcr_k & 0xff);

	ret = regmap_multi_reg_write(lt9211->regmap, seq, ARRAY_SIZE(seq));

	return ret;
}

static int LT9211_MipiRxPhy(struct lt9211 *lt9211)
{
	int ret;
	u8 val = (lt9211->num_dsi_lanes & 0x4) ? 0x0 : lt9211->num_dsi_lanes;
	const struct reg_sequence seq[] = {
		/* lt9211_System_Init */
		{ 0xff, 0xd0 },
		/* 0: 4 Lane / 1: 1 Lane / 2 : 2 Lane / 3: 3 Lane */
		{ 0x00, val },
		/* Mipi rx phy */
		{ 0xff, 0x82 },
		/* port A mipi rx enable */
		{ 0x02, 0x44 },
		/* port A/B input 8205/0a bit6_4:EQ current setting */
		{ 0x05, 0x36 },
		/* port A CK lane swap  0x32--0x36 for WYZN Glassbit2-
		 * Port A mipi/lvds rx s2p input clk select: 1 =
		 * From outer path.
		 */
		{ 0x0d, 0x26 },
		/* bit6_4:Port B Mipi/lvds rx abs refer current  0x26 0x76 */
		{ 0x17, 0x0c },
		{ 0x1d, 0x0c },
		/* eq control for LIEXIN  horizon line display issue
		 * 0xf7->0x80
		 */
		{ 0x0a, 0x81 },
		/* eq control  0x77->0x00 */
		{ 0x0b, 0x00 },
		/* port a clk enable */
		{ 0x07, 0x9f },
		/* port a lprx enable */
		{ 0x08, 0xfc },
		/* port diff swap */
		/* port a diff swap */
		{ 0x09, 0x01 },
		/* port b diff swap */
		{ 0x11, 0x01 },
		/* port lane swap */
		{ 0xff, 0x86 },
		/* port a lane swap - 1b: no swap */
		{ 0x33, 0x1b },
		/* port b lane swap - 1b: no swap */
		{ 0x34, 0x1b },
	};

	ret = regmap_multi_reg_write(lt9211->regmap, seq, ARRAY_SIZE(seq));

	return ret;
}

static int LT9211_MipiRxDigital(struct lt9211 *lt9211)
{
	int ret;
	const struct reg_sequence seq[] = {
		{ 0xff, 0x86 },
		/* mipirx HL swap */
		{ 0x30, 0x85 },
		{ 0xff, 0xd8 },
		/* mipirx HL swap bit 7-0:portAinput */
		{ 0x16, 0x00 },
		{ 0xff, 0xd0 },
		/* rpta mode enable, ensure da_mlrx_lptx_en=0 */
		{ 0x43, 0x12 },
		/* mipi rx controller - settle */
		{ 0x02, MIPI_SETTLE_VALUE },
	};

	ret = regmap_multi_reg_write(lt9211->regmap, seq, ARRAY_SIZE(seq));

	return ret;
}

static int LT9211_SetVideoTiming(struct lt9211 *lt9211)
{
	u8 vsync = (lt9211->mode.vsync_end - lt9211->mode.vsync_start);
	u16 vfront_porch = lt9211->mode.vsync_start - lt9211->mode.vdisplay;
	u8 hsync = (lt9211->mode.hsync_end - lt9211->mode.hsync_start);
	u16 hfront_porch = lt9211->mode.hsync_start - lt9211->mode.hdisplay;
	int ret;
	const struct reg_sequence seq[] = {
		{ 0xff, 0xd0 },
		//vtotal[15:8]
		{ 0x0d, (u8)(lt9211->mode.vtotal >> 8) },
		//vtotal[7:0]
		{ 0x0e, (u8)(lt9211->mode.vtotal) },
		//vactive[15:8]
		{ 0x0f, (u8)(lt9211->mode.vdisplay >> 8) },
		//vactive[7:0]
		{ 0x10, (u8)(lt9211->mode.vdisplay) },
		//vs[7:0]
		{ 0x15, vsync },
		//vfp[15:8]
		{ 0x17, (u8)(vfront_porch >> 8) },
		//vfp[7:0]
		{ 0x18, (u8)(vfront_porch) },
		//htotal[15:8]
		{ 0x11, (u8)(lt9211->mode.htotal >> 8) },
		//htotal[7:0]
		{ 0x12, (u8)(lt9211->mode.htotal) },
		//hactive[15:8]
		{ 0x13, (u8)(lt9211->mode.hdisplay >> 8) },
		//hactive[7:0]
		{ 0x14, (u8)(lt9211->mode.hdisplay) },
		//hs[7:0]
		{ 0x16, hsync },
		//hfp[15:8]
		{ 0x19, (u8)(hfront_porch >> 8) },
		//hfp[7:0]
		{ 0x1a, (u8)(hfront_porch) },
	};

	msleep(100);

	ret = regmap_multi_reg_write(lt9211->regmap, seq, ARRAY_SIZE(seq));

	return ret;
}

static int LT9211_TimingSet(struct lt9211 *lt9211)
{
	u16 hact;
	u16 vact;
	u8 fmt;
	u8 pa_lpn = 0;
	int ret = 0;
	unsigned int val1, val2;

	msleep(500); //500-->100

	ret = regmap_write(lt9211->regmap, 0xff, 0xd0);
	if (ret < 0)
		return ret;
	ret = regmap_read(lt9211->regmap, 0x82, &val1);
	if (ret < 0)
		return ret;
	ret = regmap_read(lt9211->regmap, 0x83, &val2);
	if (ret < 0)
		return ret;

	hact = (val1 << 8) + val2;
	hact = hact / 3;

	ret = regmap_read(lt9211->regmap, 0x84, &val1);
	if (ret < 0)
		return ret;

	fmt = (val1 & 0x0f);

	ret = regmap_read(lt9211->regmap, 0x85, &val1);
	if (ret < 0)
		return ret;
	ret = regmap_read(lt9211->regmap, 0x86, &val2);
	if (ret < 0)
		return ret;

	vact = (val1 << 8) + val2;

	ret = regmap_read(lt9211->regmap, 0x9c, &val1);
	if (ret < 0)
		return ret;

	pa_lpn = val1;

	dev_dbg(lt9211->dev, "\r\nhact = %i", hact);
	dev_dbg(lt9211->dev, "\r\nvact = %i", vact);
	dev_dbg(lt9211->dev, "\r\nfmt = %x", fmt);
	dev_dbg(lt9211->dev, "\r\npa_lpn = %x", pa_lpn);

	msleep(100);

	if ((hact != lt9211->mode.hdisplay)
			|| (vact != lt9211->mode.vdisplay))
		dev_info(lt9211->dev, "Could not detect resolution\n");

	ret = LT9211_SetVideoTiming(lt9211);
	if (ret < 0)
		return ret;

	return ret;
}

#if 0
static int LT9211_MipiRxPll(struct lt9211 *lt9211)
{
	u32 pclk_khz = lt9211->mode.clock;
	/* dessc pll */
	regmap_write(lt9211->regmap, 0xff, 0x82);
	regmap_write(lt9211->regmap, 0x2d, 0x48);

	if ((pclk_khz) < PCLK_KHZ_44000) {
		regmap_write(lt9211->regmap, 0x35, PIXCLK_22M_44M);/*0x83*/
	}

	else if (pclk_khz < PCLK_KHZ_88000) {
		regmap_write(lt9211->regmap, 0x35, PIXCLK_44M_88M); /*0x83*/
	}

	else if (pclk_khz < PCLK_KHZ_176000) {
		regmap_write(lt9211->regmap, 0x35, PIXCLK_88M_176M); /*0x81*/
	}

	else if (pclk_khz < PCLK_KHZ_352000) {
		/*0x80*/
		regmap_write(lt9211->regmap, 0x35, PIXCLK_LARGER_THAN_176M);
	}

	return 0;
}
#endif

static int LT9211_DesscPll_mipi(struct lt9211 *lt9211)
{
	u32 pclk;
	u8 pll_lock_flag;
	u8 i;
	u8 pll_post_div;
	u8 pcr_m = 0;
	unsigned int val;
	int ret = 0;
	struct reg_sequence seq[] = {
		{ 0xff, 0x82 },
		{ 0x2d, 0x48 },
		{ 0x35, 0x81 },
		{ 0xff, 0xd0 },
		/* M_up_limit */
		{ 0x2d, 0x40 },
		/* M_low_limit */
		{ 0x31, 0x10 },
		{ 0x26, (pcr_m | 0x80) },
		{ 0xff, 0x81 },
		{ 0x20, 0xef },
		{ 0x20, 0xff },
	};
	const struct reg_sequence seq_2[] = {
		{ 0xff, 0x81 },
		/* pll lock logic reset */
		{ 0x11, 0xfb },
		{ 0x11, 0xff },
		{ 0xff, 0x87 },
	};
	const struct reg_sequence seq_3[] = {
		{ 0xff, 0x81 },
		/* dessc pll sw rst */
		{ 0x20, 0xef },
		{ 0x20, 0xff },
	};

	pclk = lt9211->mode.clock;

	dev_dbg(lt9211->dev, "LT9211_DesscPll: set rx pll = %d\n", pclk);

	if (pclk > 80000) {
		seq[2].def = 0x81;
		pll_post_div = 0x01;
	} else if (pclk > 20000) {
		seq[2].def = 0x82;
		pll_post_div = 0x02;
	} else {
		seq[2].def = 0x83;
		pll_post_div = 0x04;
	}

	pcr_m = (u8)((pclk * 4 * pll_post_div) / 25000);
	dev_dbg(lt9211->dev, "LT9211_DesscPll: set rx pll pcr_m = 0x%x",
									pcr_m);

	seq[6].def = (pcr_m | 0x80);

	ret = regmap_multi_reg_write(lt9211->regmap, seq, ARRAY_SIZE(seq));
	if (ret < 0)
		return ret;

	/* pll lock status */
	for (i = 0; i < 6; i++) {
		ret = regmap_multi_reg_write(lt9211->regmap, seq_2,
							ARRAY_SIZE(seq_2));
		if (ret < 0)
			return ret;

		ret = regmap_read(lt9211->regmap, 0x04, &val);
		if (ret < 0)
			return ret;

		pll_lock_flag = val;
		if (pll_lock_flag & 0x01) {
			dev_dbg(lt9211->dev,
					"LT9211_DesscPll: dessc pll locked");
			break;
		} else {
			/* dessc pll sw rst */
			ret = regmap_multi_reg_write(lt9211->regmap, seq_3,
							ARRAY_SIZE(seq_3));
				if (ret < 0)
					return ret;
			dev_dbg(lt9211->dev,
			"LT9211_DesscPll: dessc pll unlocked,sw reset");
		}
	}

	return ret;
}

static int LT9211_MipiPcr(struct lt9211 *lt9211)
{
	u8 loopx;
	u8 pcr_m;
	unsigned int val;
	int ret = 0;
	struct reg_sequence seq[] = {
		{ 0xff, 0xd0 },
		/* fifo position */
		{ 0x0c, 0x60 },
		/* fifo position */
		{ 0x1c, 0x60 },
		/* pcr mode( de hs vs) */
		{ 0x24, 0x70 },
		/* M up limit */
		{ 0x2d, 0x30 },
		/* M down limit */
		{ 0x31, 0x08 },
		/* stage1 hs mode */
		/* line limit */
		{ 0x25, 0xf0 },
		/* step in limit */
		{ 0x2a, 0x30 },
		/* hs_step */
		{ 0x21, 0x4f },
		{ 0x22, 0x00 },
		/* stage2 hs mode */
		/* RGD_DIFF_SND[7:4],RGD_DIFF_FST[3:0] */
		{ 0x1e, 0x01 },
		/* hs_step */
		{ 0x23, 0x80 },
		/* stage2 de mode */
		/* de adjust pre line */
		{ 0x0a, 0x02 },
		/* de_threshold 1 */
		{ 0x38, 0x02 },
		/* de_threshold 2 */
		{ 0x39, 0x04 },
		/* de_threshold 3 */
		{ 0x3a, 0x08 },
		/* de_threshold 4 */
		{ 0x3b, 0x10 },
		/* de_step 1 */
		{ 0x3f, 0x04 },
		/* de_step 2 */
		{ 0x40, 0x08 },
		/* de_step 3 */
		{ 0x41, 0x10 },
		/* de_step 4 */
		{ 0x42, 0x20 },
		/* stable out */
		{ 0x2b, 0xa0 },
	};

	ret = regmap_multi_reg_write(lt9211->regmap, seq, ARRAY_SIZE(seq));
	if (ret < 0)
		return ret;

	//msleep(100);

	/* enable HW pcr_m */
	ret = regmap_write(lt9211->regmap, 0xff, 0xd0);
	if (ret < 0)
		return ret;
	ret = regmap_read(lt9211->regmap, 0x26, &val);
	if (ret < 0)
		return ret;

	pcr_m = val;
	pcr_m &= 0x7f;

	if ((regmap_write(lt9211->regmap, 0x26, pcr_m) < 0) ||
	    (regmap_write(lt9211->regmap, 0x27, 0x0f) < 0) ||
		(regmap_write(lt9211->regmap, 0xff, 0x81) < 0) ||
		(regmap_write(lt9211->regmap, 0x20, 0xbf) < 0) ||
		(regmap_write(lt9211->regmap, 0x20, 0xff) < 0))
		return -ENXIO;

	msleep(5);

	if ((regmap_write(lt9211->regmap, 0x0B, 0x6F) < 0) ||
		(regmap_write(lt9211->regmap, 0x0B, 0xFF) < 0))
		return -ENXIO;

	msleep(800);

	/* Check pcr_stable 10 */
	for (loopx = 0; loopx < 10; loopx++)
	{
		msleep(200);
		ret = regmap_write(lt9211->regmap, 0xff, 0xd0);
		if (ret < 0)
			return ret;
		regmap_read(lt9211->regmap, 0x87, &val);
		if (val & 0x08) {
			dev_dbg(lt9211->dev, "LT9211 pcr stable\n");
			break;
		} else {
			dev_dbg(lt9211->dev, "LT9211 pcr unstable %x !!!!\n",
									val);
		}
	}

	ret = regmap_write(lt9211->regmap, 0xff, 0xd0);
	if (ret < 0)
		return ret;

	ret = regmap_read(lt9211->regmap, 0x94, &val);
	if (ret < 0)
		return ret;

	dev_dbg(lt9211->dev, "LT9211 pcr_stable_M=%x\n", (val & 0x7F));

	return ret;
}

static int LT9211_RXCSC(struct lt9211 *lt9211)
{
	u8 val1 = 0, val2 = 0;
	int ret = 0;

	if (LT9211_Output_Mode == OUTPUT_RGB888) {
		if (Video_Input_Mode == INPUT_RGB888) {
			val1 = 0x00;
			val2 = 0x00;
		} else if ( Video_Input_Mode == INPUT_YCbCr444) {
			val1 = 0x0f;
			val2 = 0x00;
		} else if ( Video_Input_Mode == INPUT_YCbCr422_16BIT) {
			val1 = 0x00;
			val2 = 0x03;
		}
	} else if ((LT9211_Output_Mode == OUTPUT_BT656_8BIT)
			|| (LT9211_Output_Mode == OUTPUT_BT1120_16BIT)
			|| (LT9211_Output_Mode == OUTPUT_YCbCr422_16BIT)) {
		if (Video_Input_Mode == INPUT_RGB888) {
			val1 = 0x0f;
			val2 = 0x30;
		} else if (Video_Input_Mode == INPUT_YCbCr444) {
			val1 = 0x00;
			val2 = 0x30;
		} else if (Video_Input_Mode == INPUT_YCbCr422_16BIT) {
			val1 = 0x00;
			val2 = 0x00;
		}
	} else if (LT9211_Output_Mode == OUTPUT_YCbCr444) {
		if ( Video_Input_Mode == INPUT_RGB888) {
			val1 = 0x0f;
			val2 = 0x00;
		} else if (Video_Input_Mode == INPUT_YCbCr444) {
			val1 = 0x00;
			val2 = 0x00;
		} else if (Video_Input_Mode == INPUT_YCbCr422_16BIT) {
			val1 = 0x00;
			val2 = 0x03;
		}
	}

	if ((regmap_write(lt9211->regmap, 0xff, 0xf9) < 0) ||
	    (regmap_write(lt9211->regmap, 0x86, val1) < 0) ||
		(regmap_write(lt9211->regmap, 0x87, val2) < 0))
		return ret;

	return ret;
}

static int LT9211_Txpll(struct lt9211 *lt9211)
{
	u8 loopx;
	unsigned int val;
	int ret = 0;
	const struct reg_sequence seq[] = {
		{ 0xff, 0x82 },
		/* Txpll full rate mode enable */
		{ 0x36, 0x01 },
		/* Txpll pre divider value setting: Div4,
		 * Txpll lock detector clock enable,
		 * Second order passive LPF mode PLL cp current
		 */
		{ 0x37, 0x2a },
		/* Txpll post divider value setting:DIV2,
		 * Txpll clk test selection: Fb_clk
		 */
		{ 0x38, 0x06 },
		{ 0x39, 0x30 },
		{ 0x3a, 0x8e },
		{ 0xff, 0x87 },
		{ 0x37, 0x14 },
		{ 0x13, 0x00 },
		{ 0x13, 0x80 },
	};

	ret = regmap_multi_reg_write(lt9211->regmap, seq, ARRAY_SIZE(seq));
	if (ret < 0)
		return ret;

	msleep(100);

	/* Check Tx PLL cal */
	for (loopx = 0; loopx < 10; loopx++) {
		ret = regmap_read(lt9211->regmap, 0x1f, &val);
		if (ret < 0)
			return ret;
		if (val & 0x80) {
			ret = regmap_read(lt9211->regmap, 0x20, &val);
			if (ret < 0)
				return ret;
			if (val & 0x80) {
				dev_dbg(lt9211->dev, "LT9211 tx pll lock\n");
			} else {
				dev_dbg(lt9211->dev,
						"LT9211 tx pll unlocked\n");
			}
			dev_dbg(lt9211->dev, "LT9211 tx pll call done");
			break;
		} else {
			dev_dbg(lt9211->dev,
					"LT9211 tx pll unlocked %x\n", val);
		}
	}

	return ret;
}

static int LT9211_TxPhy(struct lt9211 *lt9211)
{
	int ret;
	struct reg_sequence seq[] = {
		/* Switch to 0x82xx */
		{ 0xff, 0x82 },
		/* Setup TTL ouput */
		{ 0x62, 0x01 },
		/* Setup TTL ouput */
		{ 0x63, 0xff },
		/* Setup TTL output drive strength */
		{ 0x6b, 0xff },
	};

	/* if chip revision is 0x1801e4, then register 0x63 must be 0x00,
	 * otherwise 0xff. This information was given by lontium itself
	 * by an e-mail.
	 */
	if (lt9211->chip_rev  == 0x1801e4)
		seq[2].def = 0x0;

	/* PCLK polarity: 0x21 = polarity swap, 0x01 = normal polarity */
	if (lt9211->pclk_invert)
		seq[1].def = 0x21;

	ret = regmap_multi_reg_write(lt9211->regmap, seq, ARRAY_SIZE(seq));

	return ret;
}

/* The polarity can only be inverted for RGB output. The polarity can not
 * be inverted in pattern mode, only in normal mode.
 */
static int LT9211_InvertRGB_HSVSPoarity(struct lt9211 *lt9211)
{
	int ret = 0;
	const struct reg_sequence seq[] = {
		{ 0xff, 0xd0 },
		/* to invert HS VS Poarity to Negative in REG 0xD020 bit7-6
		 * to 1, Not bit5-DE: 0x08 -> positiv edge
		 */
		{ 0x20, 0xC8 },
	};

	dev_info(lt9211->dev, "LT9211 invert HS VS Poarity\n");

	ret = regmap_multi_reg_write(lt9211->regmap, seq, ARRAY_SIZE(seq));

	return ret;
}

static int LT9211_TxDigital(struct lt9211 *lt9211)
{
	/* OUTPUT_RGB888 */
	int ret = 0;
	u8 val = lt9211->rx_source << 6;
	struct reg_sequence seq[] = {
		/* Switch to 0x85xx */
		{ 0xff, 0x85 },
		/* Chip active RX source select: MIPI rx; MIPI and LVDS tx
		 * shared logic select, 1 for MIPI tx, 0 for LVDS tx.
		*/
		/* ### TODO: should be detected by reading encoder instead
		 * of dt entry
		 */
		{ 0x88, 0x50 },
		/* BT related. */
		{ 0x60, 0x00 },
		/* Output data[23:0] is RGB. */
		/* ### TODO: Decive Tree conf */
		{ 0x6d, 0x00 },
		/* Output 24 bit active data. */
		/* ### TODO: dynamic e.g. via Device / simple panel */
		{ 0x6e, 0x00 },
		/* Switch to 0x81xx */
		{ 0xff, 0x81 },
		/* enable pix clock / bt clock */
		{ 0x36, 0xc0 },
	};

	if (lt9211->rx_source == 0 || lt9211->rx_source == 1)
		val |= lt9211->rx_source << 4;

	/* Setup correct input source according to device-tree input */
	seq[1].def = val;

	get_bus_format(lt9211);
	/* Overwrite bus format if available from DT */
	if (lt9211->bus_fmt)
		lt9211->bpc = lt9211->bus_fmt;

	if(lt9211->bpc == 0) {
		dev_err(lt9211->dev, "bus format not supported!\n");
		return -EINVAL;
	}
	val = lt9211->rgb_output_mode << 5;
	/* set 0x856d 24B_mode_sel register */
	seq[3].def = val;
	ret = regmap_multi_reg_write(lt9211->regmap, seq, ARRAY_SIZE(seq));

	return ret;
}

#ifdef DEBUG
static int LT9211_ClockCheckDebug(struct lt9211 *lt9211)
{
	u32 val1, val2, val3;
	u32 fm_value;
	int ret = 0;

	if ((regmap_write(lt9211->regmap, 0xff, 0x86) < 0) ||
	    (regmap_write(lt9211->regmap, 0x00, 0x0a) < 0))
		return ret;

	msleep(300);
	fm_value = 0;

	if ((regmap_read(lt9211->regmap, 0x08, &val1) < 0) ||
	    (regmap_read(lt9211->regmap, 0x09, &val2) < 0) ||
	    (regmap_read(lt9211->regmap, 0x0a, &val3) < 0))
		return ret;

	val1 = val1 & (0x0f);
	fm_value = (val1 << 8);
	fm_value = fm_value + val2;
	fm_value = (fm_value << 8);
	fm_value = fm_value + val3;

	dev_dbg(lt9211->dev, "dessc pixel clock: %i\n", fm_value);

	return ret;
}

static int LT9211_VideoCheckDebug(struct lt9211 *lt9211)
{
	unsigned int sync_polarity, vs, hs, vbp, vfp, hbp, hfp, vtotal, htotal,
			vact, hact;
	unsigned int val1, val2;
	int ret = 0;

	if(regmap_write(lt9211->regmap, 0xff, 0x86) < 0)
		return ret;

	if ((regmap_read(lt9211->regmap, 0x70, &sync_polarity) < 0) ||
	    (regmap_read(lt9211->regmap, 0x71, &vs) < 0) ||
	    (regmap_read(lt9211->regmap, 0x72, &val1) < 0) ||
	    (regmap_read(lt9211->regmap, 0x73, &val2) < 0) ||
        (regmap_read(lt9211->regmap, 0x74, &vbp) < 0) ||
	    (regmap_read(lt9211->regmap, 0x75, &vfp) < 0))
		return ret;

	hs = (val1 << 8) + val2;

	ret = regmap_read(lt9211->regmap, 0x76, &val1);
	if (ret < 0)
		return ret;
	ret = regmap_read(lt9211->regmap, 0x77, &val2);
	if (ret < 0)
		return ret;

	hbp = (val1 << 8) + val2;

	ret = regmap_read(lt9211->regmap, 0x78, &val1);
	if (ret < 0)
		return ret;
	ret = regmap_read(lt9211->regmap, 0x79, &val2);
	if (ret < 0)
		return ret;

	hfp = (val1 << 8) + val2;

	ret = regmap_read(lt9211->regmap, 0x7A, &val1);
	if (ret < 0)
		return ret;
	ret = regmap_read(lt9211->regmap, 0x7B, &val2);
	if (ret < 0)
		return ret;

	vtotal = (val1 << 8) + val2;

	ret = regmap_read(lt9211->regmap, 0x7C, &val1);
	if (ret < 0)
		return ret;
	ret = regmap_read(lt9211->regmap, 0x7D, &val2);
	if (ret < 0)
		return ret;

	htotal = (val1 << 8) + val2;

	ret = regmap_read(lt9211->regmap, 0x7E, &val1);
	if (ret < 0)
		return ret;
	ret = regmap_read(lt9211->regmap, 0x7F, &val2);
	if (ret < 0)
		return ret;

	vact = (val1 << 8) + val2;

	ret = regmap_read(lt9211->regmap, 0x80, &val1);
	if (ret < 0)
		return ret;
	ret = regmap_read(lt9211->regmap, 0x81, &val2);
	if (ret < 0)
		return ret;

	hact = (val1 << 8) + val2;

	dev_dbg(lt9211->dev, "sync_polarity = %x\n", sync_polarity);
	dev_dbg(lt9211->dev, "hfp %i, hs %i, hbp %i, hact %i, htotal %i\n",
			hfp, hs, hbp, hact, htotal);
	dev_dbg(lt9211->dev, "vfp %i, vs %i, vbp %i, vact %i, vtotal %i\n",
			vfp, vs, vbp, vact, vtotal);

	return ret;
}
#endif

#if 0
void LT9211_BT_Set(struct lt9211 *lt9211) {

	u16 tmp_data;
	if((LT9211_OutPutModde == OUTPUT_BT1120_16BIT) ||
		(LT9211_OutPutModde == OUTPUT_BT656_8BIT))
	{
		tmp_data = hs+hbp;
		regmap_write(lt9211->regmap,0xff,0x85);
		regmap_write(lt9211->regmap,0x61,(u8)(tmp_data>>8));
		regmap_write(lt9211->regmap,0x62,(u8)tmp_data);
		regmap_write(lt9211->regmap,0x63,(u8)(hact>>8));
		regmap_write(lt9211->regmap,0x64,(u8)hact);
		regmap_write(lt9211->regmap,0x65,(u8)(htotal>>8));
		regmap_write(lt9211->regmap,0x66,(u8)htotal);
		tmp_data = vs+vbp;
		regmap_write(lt9211->regmap,0x67,(u8)tmp_data);
		regmap_write(lt9211->regmap,0x68,0x00);
		regmap_write(lt9211->regmap,0x69,(u8)(vact>>8));
		regmap_write(lt9211->regmap,0x6a,(u8)vact);
		regmap_write(lt9211->regmap,0x6b,(u8)(vtotal>>8));
		regmap_write(lt9211->regmap,0x6c,(u8)vtotal);
	}
}
#endif

static int lt9211_debug_pattern(struct lt9211 *lt9211)
{
	int ret;

	dev_info(lt9211->dev, "lt9211_debug_pattern\n");

	ret = LT9211_SystemInt(lt9211);
	if (ret) {
		dev_err(lt9211->dev, "LT9211_SystemInt failed!\n");
		return ret;
	}

	msleep(100);

	ret = LT9211_TxDigital(lt9211);
	if (ret) {
		dev_err(lt9211->dev, "LT9211_TxDigital failed!\n");
		return ret;
	}

	ret = LT9211_TxPhy(lt9211);
	if (ret) {
		dev_err(lt9211->dev, "LT9211_TxPhy failed!\n");
		return ret;
	}

	ret = LT9211_Pattern(lt9211);
	if (ret) {
		dev_err(lt9211->dev, "LT9211_Pattern failed!\n");
		return ret;
	}

	msleep(10);

	ret = LT9211_Txpll(lt9211);
	if (ret) {
		dev_err(lt9211->dev, "LT9211_Txpll failed!\n");
		return ret;
	}

	msleep(10);

#ifdef DEBUG
	ret = LT9211_ClockCheckDebug(lt9211);
	if (ret) {
		dev_err(lt9211->dev, "LT9211_ClockCheckDebug failed!\n");
		return ret;
	}
#endif

	return ret;
}

static int lt9211_init(struct lt9211 *lt9211)
{
	int ret;

	dev_dbg(lt9211->dev, "LT9211_mipi to TTL\n");
	ret = LT9211_SystemInt(lt9211);
	if (ret) {
		dev_err(lt9211->dev, "LT9211_SystemInt failed!\n");
		return ret;
	}

	msleep(100);

	ret = LT9211_MipiRxPhy(lt9211);
	if (ret) {
		dev_err(lt9211->dev, "LT9211_MipiRxPhy failed!\n");
		return ret;
	}

	ret = LT9211_MipiRxDigital(lt9211);
	if (ret) {
		dev_err(lt9211->dev, "LT9211_MipiRxDigital failed!\n");
		return ret;
	}

	ret = LT9211_TimingSet(lt9211);
	if (ret) {
		dev_err(lt9211->dev, "LT9211_TimingSet failed!\n");
		return ret;
	}

	/* not used for TTL output */
	//LT9211_MipiRxPll(lt9211);

	ret = LT9211_DesscPll_mipi(lt9211);
	if (ret) {
		dev_err(lt9211->dev, "LT9211_DesscPll_mipi failed!\n");
		return ret;
	}

	ret = LT9211_MipiPcr(lt9211);
	if (ret) {
		dev_err(lt9211->dev, "LT9211_MipiPcr failed!\n");
		return ret;
	}

	ret = LT9211_TxDigital(lt9211);
	if (ret) {
		dev_err(lt9211->dev, "LT9211_TxDigital failed!\n");
		return ret;
	}

	ret = LT9211_TxPhy(lt9211);
	if (ret) {
		dev_err(lt9211->dev, "LT9211_TxPhy failed!\n");
		return ret;
	}

	/* only for TTL output */
	if ((lt9211->mode.flags & DRM_MODE_FLAG_NHSYNC) &&
		(lt9211->mode.flags & DRM_MODE_FLAG_NVSYNC)) {
		ret = LT9211_InvertRGB_HSVSPoarity(lt9211);
		if (ret) {
			dev_err(lt9211->dev,
				"LT9211_InvertRGB_HSVSPoarity failed!\n");
			return ret;
		}
	}

	msleep(10);

	ret = LT9211_Txpll(lt9211);
	if (ret) {
		dev_err(lt9211->dev, "LT9211_Txpll failed!\n");
		return ret;
	}

	ret = LT9211_RXCSC(lt9211);
	if (ret) {
		dev_err(lt9211->dev, "LT9211_RXCSC failed!\n");
		return ret;
	}

#ifdef DEBUG
	ret = LT9211_ClockCheckDebug(lt9211);
	if (ret) {
		dev_err(lt9211->dev, "LT9211_ClockCheckDebug failed!\n");
		return ret;
	}

	ret = LT9211_VideoCheckDebug(lt9211);
	if (ret) {
		dev_err(lt9211->dev, "LT9211_VideoCheckDebug failed!\n");
		return ret;
	}
#endif

	/* only necessary for BT output */
	//LT9211_BT_Set(lt9211);

	return ret;
}

#ifdef ENABLE_IRQ
static irqreturn_t lt9211_irq_thread_handler(int irq, void *dev_id)
{
	struct lt9211 *lt9211 = dev_id;
	/* TODO: */
	dev_info(lt9211->dev, "irq occured\n");

	return IRQ_HANDLED;
}
#endif

static int lt9211_regulator_init(struct lt9211 *lt9211)
{
	int ret;

	lt9211->supplies[0].supply = "vdd";
	lt9211->supplies[1].supply = "vcc";

	ret = devm_regulator_bulk_get(lt9211->dev, 2, lt9211->supplies);
	if (ret < 0)
		return ret;

	return regulator_set_load(lt9211->supplies[0].consumer, 300000);
}

static int lt9211_regulator_enable(struct lt9211 *lt9211) {

	int ret;

	ret = regulator_enable(lt9211->supplies[0].consumer);
	if (ret < 0)
		return ret;

	usleep_range(1000, 10000);

	ret = regulator_enable(lt9211->supplies[1].consumer);
	if (ret < 0) {
		regulator_disable(lt9211->supplies[0].consumer);
		return ret;
	}

	return ret;
}

static int lt9211_regulator_disable(struct lt9211 *lt9211) {

	int ret;

	ret = regulator_disable(lt9211->supplies[0].consumer);
	if (ret < 0)
		return ret;

	usleep_range(1000, 10000);

	ret = regulator_disable(lt9211->supplies[1].consumer);
	if (ret < 0) {
		return ret;
	}

	return ret;
}

static int lt9211_gpio_init(struct lt9211 *lt9211)
{
	lt9211->reset_gpio = devm_gpiod_get(lt9211->dev, "reset",
							GPIOD_OUT_HIGH);
	if (IS_ERR(lt9211->reset_gpio)) {
		dev_err(lt9211->dev, "failed to acquire reset gpio\n");
		return PTR_ERR(lt9211->reset_gpio);
	}

	return 0;
}

static int lt9211_read_device_rev(struct lt9211 *lt9211)
{
	uint32_t rev = 0;
	uint32_t rev_tmp = 0;
	int ret = 0;

	regmap_write(lt9211->regmap, 0xff, 0x81);
	ret = regmap_read(lt9211->regmap, 0x00, &rev_tmp);
	rev |= rev_tmp << 16;
	ret = regmap_read(lt9211->regmap, 0x01, &rev_tmp);
	rev |= rev_tmp << 8;
	ret = regmap_read(lt9211->regmap, 0x02, &rev_tmp);
	rev |= rev_tmp;

	if (ret) {
		dev_err(lt9211->dev, "failed to read revision: %d\n", ret);
	} else {
		lt9211->chip_rev = rev;
		dev_info(lt9211->dev, "lt9211 revision: 0x%x\n", rev);
	}

	return ret;
}

static void lt9211_bridge_pre_enable(struct drm_bridge *bridge)
{
	struct lt9211 *lt9211 = bridge_to_lt9211(bridge);
	int ret;

	ret = lt9211_regulator_enable(lt9211);
	if (ret)
		dev_err(lt9211->dev, "regulator enable failed, %d\n", ret);

	if (lt9211->reset_gpio)
	{
		gpiod_set_value_cansleep(lt9211->reset_gpio, 1);
		msleep(20);

		gpiod_set_value_cansleep(lt9211->reset_gpio, 0);
		msleep(20);

		gpiod_set_value_cansleep(lt9211->reset_gpio, 1);
		msleep(100);
	}
}

static void lt9211_bridge_post_disable(struct drm_bridge *bridge)
{
	struct lt9211 *lt9211 = bridge_to_lt9211(bridge);
	int ret;

	if (lt9211->reset_gpio) {
		gpiod_set_value_cansleep(lt9211->reset_gpio, 0);
		msleep(100);
	}

	ret = lt9211_regulator_disable(lt9211);
	if (ret)
		dev_err(lt9211->dev, "regulator disable failed, %d\n", ret);
}

static void lt9211_bridge_enable(struct drm_bridge *bridge)
{
	struct lt9211 *lt9211 = bridge_to_lt9211(bridge);
	struct drm_display_mode *mode;
	int ret;

	/* take the mode from simple panel instead of the adjusted mode. The
	 * adjusted mode will be adjusted by the encoder if some parameters
	 * like flags are doesn´t match to the input side. The converter chip
	 * doesn´t care about that and can setup it´s own flags therefore we
	 * take parameters directly from the display which is connected to it.
	 */
	mode = &bridge->encoder->crtc->state->mode;
	drm_mode_copy(&lt9211->mode, mode);

	if (!lt9211->debug_pattern) {
		ret = lt9211_init(lt9211);
		if (ret)
			dev_err(lt9211->dev, "lt9211_init failed\n");
	} else {
		ret = lt9211_debug_pattern(lt9211);
		if (ret)
			dev_err(lt9211->dev, "lt9211_debug_pattern failed\n");
	}
}

static enum drm_mode_status
lt9211_mode_valid(struct drm_bridge *bridge,
	      const struct drm_display_info *info,
	      const struct drm_display_mode *mode)
{
/*
	 * Maximum pixel clock speed < 352MHz
	 */
	if (mode->clock >= PCLK_KHZ_352000)
		return MODE_CLOCK_HIGH;

	return MODE_OK;
}

static int lt9211_bridge_attach(struct drm_bridge *bridge,
				enum drm_bridge_attach_flags flags)
{
	struct lt9211 *lt9211 = bridge_to_lt9211(bridge);
	struct mipi_dsi_host *host;
	struct mipi_dsi_device *dsi;
	int ret;

	const struct mipi_dsi_device_info info = {
		.type = "LT9211",
		.channel = 0,
		.node = NULL,
	};

	host = of_find_mipi_dsi_host_by_node(lt9211->host_node);
	if (!host) {
		dev_err(lt9211->dev, "failed to find dsi host\n");
		return -EPROBE_DEFER;
	}

	dsi = mipi_dsi_device_register_full(host, &info);
	if (IS_ERR(dsi)) {
		dev_err(lt9211->dev, "failed to create dsi device\n");
		ret = PTR_ERR(dsi);
		goto err_dsi_device;
	}

	dsi->lanes = lt9211->num_dsi_lanes;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST;

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		dev_err(lt9211->dev, "failed to attach dsi to host\n");
		goto err_dsi_attach;
	}

	/* Attach the panel-bridge to the dsi bridge */
	return drm_bridge_attach(bridge->encoder, lt9211->panel_bridge,
				 &lt9211->bridge, flags);

err_dsi_attach:
	mipi_dsi_device_unregister(dsi);
err_dsi_device:
	return ret;
}

static const struct drm_bridge_funcs lt9211_bridge_funcs = {
	.attach = lt9211_bridge_attach,
	.pre_enable = lt9211_bridge_pre_enable,
	.enable = lt9211_bridge_enable,
	.mode_valid = lt9211_mode_valid,
	.post_disable = lt9211_bridge_post_disable,
};

static int lt9211_parse_dt(struct device_node *np, struct lt9211 *lt9211)
{
	struct device_node *endpoint;
	struct device_node *parent;
	struct property *prop;
	int len = 0, ret = 0;

	/*
	 * To get the data-lanes of dsi, we need to access the dsi0_out of
	 * port1 of dsi0 endpoint from bridge port0 of d2l_in
	 */
	endpoint = of_graph_get_endpoint_by_regs(np, LT9211_DSI_IN, -1);
	if (endpoint)
	{
		/* dsi0_out node */
		parent = of_graph_get_remote_port_parent(endpoint);
		of_node_put(endpoint);
		if (parent)
		{
			/* dsi0 port 1 */
			endpoint = of_graph_get_endpoint_by_regs(parent, 1,
									-1);
			of_node_put(parent);
			if (endpoint)
			{
				prop = of_find_property(endpoint,
							"data-lanes", &len);
				of_node_put(endpoint);
				if (!prop)
				{
					dev_err(lt9211->dev,
						"failed to find data lane\n");
					return -EPROBE_DEFER;
				}
			}
		}
	}

	lt9211->num_dsi_lanes = len / sizeof(u32);

	if (lt9211->num_dsi_lanes < 1 || lt9211->num_dsi_lanes > 4)
		return -EINVAL;

	lt9211->host_node = of_graph_get_remote_node(np, 0, 0);
	if (!lt9211->host_node)
		return -ENODEV;

	of_node_put(lt9211->host_node);

	lt9211->debug_pattern = of_property_read_bool(np, "lt,debug-pattern");

	ret = of_property_read_u8(np, "rx-source", &lt9211->rx_source);
	if (ret) {
		dev_err(lt9211->dev, "no input source set!\n");
		return ret;
	}

	if (lt9211->rx_source > 3 || lt9211->rx_source < 0) {
		dev_err(lt9211->dev,
		"input source not available (0x%x)!\n", lt9211->rx_source);
		return -EINVAL;
	}

	ret = of_property_read_u8(np, "bus-fmt", &lt9211->bus_fmt);
	if (!ret) {
		if (lt9211->bus_fmt != 6 && lt9211->bus_fmt != 8) {
				lt9211->bus_fmt = 0;
				dev_dbg(lt9211->dev, "Bus format not legal!\n");
		}
	} else {
		lt9211->bus_fmt = 0;
		dev_dbg(lt9211->dev, "No bus format set!\n");
	}

	lt9211->pclk_invert = of_property_read_bool(np, "pclk-invert");

	ret = of_property_read_u8(np, "rgb-output-mode", &lt9211->rgb_output_mode);
	if (!ret) {
		if(lt9211->rgb_output_mode > 7)
			lt9211->rgb_output_mode = 4;
	} else {
		/* default output mode */
		lt9211->rgb_output_mode = 4;
	}

	return ret;
}

static int lt9211_probe(struct i2c_client *client,
		const struct i2c_device_id *id) {

	struct lt9211 *lt9211;
	struct device *dev = &client->dev;
	struct drm_panel *panel;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(dev, "device doesn't support I2C\n");
		return -ENODEV;
	}

	lt9211 = devm_kzalloc(dev, sizeof(*lt9211), GFP_KERNEL);
	if (!lt9211)
		return -ENOMEM;

	ret = drm_of_find_panel_or_bridge(dev->of_node, LT9211_RGB_OUT0, 0,
								&panel, NULL);
	if (ret < 0)
		return ret;
	if (!panel)
		return -ENODEV;

	lt9211->panel_bridge = devm_drm_panel_bridge_add(dev, panel);
	if (IS_ERR(lt9211->panel_bridge))
		return PTR_ERR(lt9211->panel_bridge);

	lt9211->dev = dev;
	lt9211->client = client;

	ret = lt9211_parse_dt(dev->of_node, lt9211);
	if (ret) {
		dev_err(dev, "failed to parse device tree\n");
		return ret;
	}

	lt9211->regmap = devm_regmap_init_i2c(client, &lt9211_regmap_config);
	if (IS_ERR(lt9211->regmap)) {
		dev_err(lt9211->dev, "regmap i2c init failed\n");
		return PTR_ERR(lt9211->regmap);
	}

	ret = lt9211_gpio_init(lt9211);
	if (ret < 0)
		goto err_of_put;

	ret = lt9211_regulator_init(lt9211);
	if (ret < 0)
		goto err_of_put;

#ifdef ENABLE_IRQ
	ret = devm_request_threaded_irq(dev, client->irq, NULL,
					lt9211_irq_thread_handler,
					IRQF_ONESHOT, "lt9211", lt9211);
	if (ret) {
		dev_err(dev, "failed to request irq\n");
		goto err_disable_regulators;
	}
#endif
	i2c_set_clientdata(client, lt9211);

	ret = lt9211_read_device_rev(lt9211);
	if (ret) {
		dev_err(dev, "failed to read chip rev\n");
		goto err_disable_regulators;
	}

	lt9211->bridge.funcs = &lt9211_bridge_funcs;
	lt9211->bridge.of_node = dev->of_node;

	drm_bridge_add(&lt9211->bridge);

	return ret;

err_disable_regulators:
	regulator_bulk_disable(ARRAY_SIZE(lt9211->supplies),
			lt9211->supplies);
err_of_put:
	return ret;
}

static int lt9211_remove(struct i2c_client *client) {

	struct lt9211 *lt9211 = i2c_get_clientdata(client);

	disable_irq(client->irq);

	drm_bridge_remove(&lt9211->bridge);

	regulator_bulk_disable(ARRAY_SIZE(lt9211->supplies), lt9211->supplies);

	return 0;
}

static struct i2c_device_id lt9211_id[] = {
	{ "lontium,lt9211", 0 },
	{ }
};

static const struct of_device_id lt9211_match_table[] = {
	{ .compatible = "lontium,lt9211" },
	{ }
};
MODULE_DEVICE_TABLE( of, lt9211_match_table);

static struct i2c_driver lt9211_driver = {
	.driver = {
		.name = "lt9211",
		.of_match_table = lt9211_match_table,
	},
	.probe = lt9211_probe,
	.remove = lt9211_remove,
	.id_table = lt9211_id,
};
module_i2c_driver( lt9211_driver);

MODULE_AUTHOR("Philipp Gerbach <gerbach@fs-net.de>");
MODULE_DESCRIPTION("LONTIUM LT9211 MIPI2RGB Converter Driver");
MODULE_LICENSE("GPL v2");
