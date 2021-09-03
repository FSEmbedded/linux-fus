// SPDX-License-Identifier: GPL-2.0
/*
 * Author: Philipp Gerbach , <gerbach@fs-net.de>
 *
 * Based on example code by Lontium.
 *
 * This is a driver for the lontium-lt9211 as MIPI2RGB converter.
 */

#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>

#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/of_platform.h>

#include <video/mipi_display.h>
#include <video/display_timing.h>

#define MIPI_SETTLE_VALUE 0x05 //0x05  0x0a
#define lt9211_PAGE_CONTROL	0xff

typedef enum LT9211_OUTPUTMODE_ENUM {
	OUTPUT_RGB888 = 0,
	OUTPUT_BT656_8BIT = 1,
	OUTPUT_BT1120_16BIT = 2,
	OUTPUT_LVDS_2_PORT = 3,
	OUTPUT_LVDS_1_PORT = 4,
	OUTPUT_YCbCr444 = 5,
	OUTPUT_YCbCr422_16BIT = 6
} Video_Output_Mode_TypeDef;

#define LT9211_OutPutModde  OUTPUT_RGB888

typedef enum VIDEO_INPUTMODE_ENUM {
	INPUT_RGB888 = 1, INPUT_YCbCr444 = 2, INPUT_YCbCr422_16BIT = 3
} Video_Input_Mode_TypeDef;

#define Video_Input_Mode  INPUT_RGB888

/* [1:0]PIXCK_DIVSEL
 * 00 176M~352M
 * 01 88M~176M
 * 10 44M~88M
 * 11 22M~44M
 */
typedef enum _REG8235_PIXCK_DIVSEL   ////dessc pll to generate pixel clk
{
	PIXCLK_LARGER_THAN_176M = 0x80,
	PIXCLK_88M_176M = 0x81,
	PIXCLK_44M_88M = 0x82,
	PIXCLK_22M_44M = 0x83
} REG8235_PIXCK_DIVSEL_TypeDef;

#define  PCLK_KHZ_44000    44000
#define  PCLK_KHZ_88000    88000
#define  PCLK_KHZ_176000   176000
#define  PCLK_KHZ_352000   352000

struct video_timing {
	u16 hfp;
	u16 hs;
	u16 hbp;
	u16 hact;
	u16 htotal;
	u16 vfp;
	u16 vs;
	u16 vbp;
	u16 vact;
	u16 vtotal;
	u32 pclk_khz;
};

struct lt9211 {
	struct device *dev;
	struct drm_bridge bridge;
	struct drm_connector connector;

	struct regmap *regmap;
	struct device_node *host_node;
	/* for no mipi panle */
	struct platform_device panel_ofdev;

	struct device_node *dsi0_node;
	struct device_node *dsi1_node;
	struct mipi_dsi_device *dsi0;
	struct mipi_dsi_device *dsi1;
	struct platform_device *audio_pdev;
	struct drm_panel *panel;

	struct gpio_desc *reset_gpio;
	struct gpio_desc *enable_gpio;

	bool power_on;
	bool sleep;
	bool debug_mode;
	struct regulator_bulk_data supplies[2];

	struct i2c_client *client;

	struct drm_display_mode *curr_mode;
	enum drm_connector_status status;

	u32 vic;
	u32 num_mipi_lanes;

	struct video_timing video_timing;
};

static const struct regmap_range_cfg lt9211_ranges[] = { { .name =
		"register_range", .range_min = 0, .range_max = 0xffff, .selector_reg =
		lt9211_PAGE_CONTROL, .selector_mask = 0xff, .selector_shift = 0,
		.window_start = 0, .window_len = 0x100, },
};

static const struct regmap_config lt9211_regmap_config = { .reg_bits = 8,
		.val_bits = 8, .max_register = 0xffff,
//	.ranges = lt9211_ranges,
//	.num_ranges = ARRAY_SIZE(lt9211_ranges),
};

static void LT9211_SystemInt(struct lt9211 *lt9211) {

	int ret;

	const struct reg_sequence seq[] = {

	{ 0x14, 0x08 }, /* TXPLL vco cur_sel hardware change wait timer setting. */
	{ 0x15, 0x00 }, /* TXPLL vco cur_sel hardware change wait timer setting. */
	{ 0x18, 0x0f }, /* The step setting for TXPLL vco cur_sel */
	{ 0x22, 0x08 }, /* ?? */
	{ 0x23, 0x00 }, /* ?? */
	{ 0x26, 0x0f }, /* ?? */
	};

	regmap_write(lt9211->regmap, 0xff, 0x82);
	regmap_write(lt9211->regmap, 0x01, 0x18);

	regmap_write(lt9211->regmap, 0xff, 0x86);
	regmap_write(lt9211->regmap, 0x06, 0x61);
	regmap_write(lt9211->regmap, 0x07, 0xa8);

	regmap_write(lt9211->regmap, 0xff, 0x87);

	ret = regmap_multi_reg_write(lt9211->regmap, seq, ARRAY_SIZE(seq));
}

static void LT9211_TxDigital(struct lt9211 *lt9211) {

	/* OUTPUT_RGB888 */
	int ret;

	const struct reg_sequence seq[] = {
	/* lt9211_System_Init */
	{ 0x88, 0x50 }, /* Chip active RX source select: MIPI rx; MIPI and LVDS tx shared logic select, 1 for MIPI tx, 0 for LVDS tx.*/
	{ 0x60, 0x00 }, /* BT related. */
	{ 0x6d, 0x00 }, /* Output data[23:0] is RGB. *//*TODO: Decive Tree conf */
	{ 0x6e, 0x00 }, /* Output 24 bit active data. */

	};

	regmap_write(lt9211->regmap, 0xff, 0x85);
	ret = regmap_multi_reg_write(lt9211->regmap, seq, ARRAY_SIZE(seq));
	regmap_write(lt9211->regmap, 0xff, 0x81);
	regmap_write(lt9211->regmap, 0x36, 0xc0);

	dev_dbg(lt9211->dev, "LT9211 set to OUTPUT_RGB888\n");

}

static void LT9211_TxPhy(struct lt9211 *lt9211) {

	int ret;

	const struct reg_sequence seq[] = {
	/* lt9211_System_Init */
	{ 0x62, 0x21 }, /* TTL output enable*//*TODO: Decive Tree conf */
	{ 0x6b, 0xff }, /* TTL D23~0 output drive strength set:13mA; TTL DCK output drive strength set :23mA */
	};

	regmap_write(lt9211->regmap, 0xff, 0x82);
	ret = regmap_multi_reg_write(lt9211->regmap, seq, ARRAY_SIZE(seq));
}

void LT9211_Pattern(struct lt9211 *lt9211, struct video_timing *video_format) {

	u32 pclk_khz;
	u8 dessc_pll_post_div = 0;
	u32 pcr_m, pcr_k;

	int ret, val;
	const struct reg_sequence seq[] = {
	/* lt9211_System_Init */
	{ 0x88, 0xc0 }, /* Chip active RX source select: Chip Video pattern gen. */
	{ 0xa1, 0x53 }, /* Display black wire in white background. (R,G,B) DP_DATA, 00, 00; */
	{ 0xa2, 0xf0 }, /* Pattern data value set. */
	{ 0xa3, (u8)((video_format->hs + video_format->hbp) / 256) }, { 0xa4, (u8)(
			(video_format->hs + video_format->hbp) % 256) }, { 0xa5, (u8)(
			(video_format->vs + video_format->vbp) % 256) }, { 0xa6, (u8)(
			video_format->hact / 256) },
			{ 0xa7, (u8)(video_format->hact % 256) }, { 0xa8, (u8)(
					video_format->vact / 256) }, { 0xa9, (u8)(
					video_format->vact % 256) }, { 0xaa, (u8)(
					video_format->htotal / 256) }, { 0xab, (u8)(
					video_format->htotal % 256) }, { 0xac, (u8)(
					video_format->vtotal / 256) }, { 0xad, (u8)(
					video_format->vtotal % 256) }, { 0xae, (u8)(
					video_format->hs / 256) }, { 0xaf, (u8)(
					video_format->hs % 256) }, { 0xb0, (u8)(
					video_format->vs % 256) }, };

	pclk_khz = video_format->pclk_khz;

	regmap_write(lt9211->regmap, 0xff, 0xf9);
	regmap_write(lt9211->regmap, 0x3e, 0x80);
	regmap_write(lt9211->regmap, 0xff, 0x85);

	ret = regmap_multi_reg_write(lt9211->regmap, seq, ARRAY_SIZE(seq));
	regmap_write(lt9211->regmap, 0xff, 0x82);
	regmap_write(lt9211->regmap, 0x2d, 0x48);

	if (pclk_khz < 44000) {
		regmap_write(lt9211->regmap, 0x35, 0x83);
		dessc_pll_post_div = 16;
	}

	else if (pclk_khz < 88000) {
		regmap_write(lt9211->regmap, 0x35, 0x82);
		dessc_pll_post_div = 8;
	}

	else if (pclk_khz < 176000) {
		regmap_write(lt9211->regmap, 0x35, 0x81);
		dessc_pll_post_div = 4;
	}

	else if (pclk_khz < 352000) {
		regmap_write(lt9211->regmap, 0x35, 0x80);
		dessc_pll_post_div = 0;
	}

	pcr_m = (pclk_khz * dessc_pll_post_div) / 25;
	pcr_k = pcr_m % 1000;
	pcr_m = pcr_m / 1000;

	pcr_k <<= 14;
	regmap_write(lt9211->regmap, 0xff, 0xd0);
	regmap_write(lt9211->regmap, 0x2d, 0x7f);
	regmap_write(lt9211->regmap, 0x31, 0x00);

	regmap_write(lt9211->regmap, 0x26, 0x80 | ((u8) pcr_m));
	regmap_write(lt9211->regmap, 0x27, (u8)((pcr_k >> 16) & 0xff)); //K
	regmap_write(lt9211->regmap, 0x28, (u8)((pcr_k >> 8) & 0xff)); //K
	regmap_write(lt9211->regmap, 0x29, (u8)(pcr_k & 0xff)); //K

	regmap_read(lt9211->regmap, 0x26, &val);
	dev_dbg(lt9211->dev, "0xd026: %x\n", val);
	regmap_read(lt9211->regmap, 0x27, &val);
	dev_dbg(lt9211->dev, "0xd027: %x\n", val);
	regmap_read(lt9211->regmap, 0x28, &val);
	dev_dbg(lt9211->dev, "0xd028: %x\n", val);
	regmap_read(lt9211->regmap, 0x29, &val);
	dev_dbg(lt9211->dev, "0xd029: %x\n", val);
}

void LT9211_Txpll(struct lt9211 *lt9211) {

	u8 loopx;
	unsigned int val;
	int ret;

	const struct reg_sequence seq[] = { { 0x36, 0x01 }, // Txpll full rate mode enable
			{ 0x37, 0x2a }, // Txpll pre divider value setting: Div4 , Txpll lock detector clock enable, Second order passive LPF mode PLL cp current
			{ 0x38, 0x06 }, // Txpll post divider value setting:DIV2, Txpll clk test selection: Fb_clk
			{ 0x39, 0x30 }, { 0x3a, 0x8e }, };

	regmap_write(lt9211->regmap, 0xff, 0x82);
	ret = regmap_multi_reg_write(lt9211->regmap, seq, ARRAY_SIZE(seq));
	regmap_write(lt9211->regmap, 0xff, 0x87);
	regmap_write(lt9211->regmap, 0x37, 0x14);
	regmap_write(lt9211->regmap, 0x13, 0x00);
	regmap_write(lt9211->regmap, 0x13, 0x80);
	msleep(100);

	for (loopx = 0; loopx < 10; loopx++) { //Check Tx PLL cal
		regmap_read(lt9211->regmap, 0x1f, &val);
		if (val & 0x80) {
			regmap_read(lt9211->regmap, 0x20, &val);
			if (val & 0x80) {
				dev_dbg(lt9211->dev, "LT9211 tx pll lock\n");
			} else {
				dev_dbg(lt9211->dev, "LT9211 tx pll unlocked\n");
			}
			dev_dbg(lt9211->dev, "LT9211 tx pll cal done");
			break;
		} else {
			dev_dbg(lt9211->dev, "LT9211 tx pll unlocked %x\n", val);
		}
	}
}

void LT9211_MipiRxPhy(struct lt9211 *lt9211) {

	regmap_write(lt9211->regmap, 0xff, 0xd0);
	regmap_write(lt9211->regmap, 0x00, lt9211->num_mipi_lanes);
	/* Mipi rx phy */
	regmap_write(lt9211->regmap, 0xff, 0x82);
	regmap_write(lt9211->regmap, 0x02, 0x44); //port A mipi rx enable
			/*port A/B input 8205/0a bit6_4:EQ current setting*/
	regmap_write(lt9211->regmap, 0x05, 0x36); //port A CK lane swap  0x32--0x36 for WYZN Glassbit2- Port A mipi/lvds rx s2p input clk select: 1 = From outer path.
	regmap_write(lt9211->regmap, 0x0d, 0x26); //bit6_4:Port B Mipi/lvds rx abs refer current  0x26 0x76
	regmap_write(lt9211->regmap, 0x17, 0x0c);
	regmap_write(lt9211->regmap, 0x1d, 0x0c);

	regmap_write(lt9211->regmap, 0x0a, 0x81); //eq control for LIEXIN  horizon line display issue 0xf7->0x80
	regmap_write(lt9211->regmap, 0x0b, 0x00); //eq control  0x77->0x00

	/*port a*/
	regmap_write(lt9211->regmap, 0x07, 0x9f); //port clk enable  （只开Portb时,porta的lane0 clk要打开）
	regmap_write(lt9211->regmap, 0x08, 0xfc); //port lprx enable

	/*port diff swap*/
	regmap_write(lt9211->regmap, 0x09, 0x01); //port a diff swap
	regmap_write(lt9211->regmap, 0x11, 0x01); //port b diff swap

	/*port lane swap*/
	regmap_write(lt9211->regmap, 0xff, 0x86);
	regmap_write(lt9211->regmap, 0x33, 0x1b); //port a lane swap	1b:no swap
	regmap_write(lt9211->regmap, 0x34, 0x1b); //port b lane swap 1b:no swap

}

void LT9211_MipiRxDigital(struct lt9211 *lt9211) {

	regmap_write(lt9211->regmap, 0xff, 0x86);
	regmap_write(lt9211->regmap, 0x30, 0x85); //mipirx HL swap

	regmap_write(lt9211->regmap, 0xff, 0xD8);
	regmap_write(lt9211->regmap, 0x16, 0x00); //mipirx HL swap	  bit7- 0:portAinput

	regmap_write(lt9211->regmap, 0xff, 0xd0);
	regmap_write(lt9211->regmap, 0x43, 0x12); //rpta mode enable,ensure da_mlrx_lptx_en=0
	regmap_write(lt9211->regmap, 0x02, MIPI_SETTLE_VALUE); //mipi rx controller	//settle
}

void LT9211_SetVideoTiming(struct lt9211 *lt9211,
		struct video_timing *video_format) {

	msleep(100);
	regmap_write(lt9211->regmap, 0xff, 0xd0);
	regmap_write(lt9211->regmap, 0x0d, (u8)(video_format->vtotal >> 8)); //vtotal[15:8]
	regmap_write(lt9211->regmap, 0x0e, (u8)(video_format->vtotal)); //vtotal[7:0]
	regmap_write(lt9211->regmap, 0x0f, (u8)(video_format->vact >> 8)); //vactive[15:8]
	regmap_write(lt9211->regmap, 0x10, (u8)(video_format->vact)); //vactive[7:0]
	regmap_write(lt9211->regmap, 0x15, (u8)(video_format->vs)); //vs[7:0]
	regmap_write(lt9211->regmap, 0x17, (u8)(video_format->vfp >> 8)); //vfp[15:8]
	regmap_write(lt9211->regmap, 0x18, (u8)(video_format->vfp)); //vfp[7:0]

	regmap_write(lt9211->regmap, 0x11, (u8)(video_format->htotal >> 8)); //htotal[15:8]
	regmap_write(lt9211->regmap, 0x12, (u8)(video_format->htotal)); //htotal[7:0]
	regmap_write(lt9211->regmap, 0x13, (u8)(video_format->hact >> 8)); //hactive[15:8]
	regmap_write(lt9211->regmap, 0x14, (u8)(video_format->hact)); //hactive[7:0]
	regmap_write(lt9211->regmap, 0x16, (u8)(video_format->hs)); //hs[7:0]
	regmap_write(lt9211->regmap, 0x19, (u8)(video_format->hfp >> 8)); //hfp[15:8]
	regmap_write(lt9211->regmap, 0x1a, (u8)(video_format->hfp)); //hfp[7:0]

}

int LT9211_TimingSet(struct lt9211 *lt9211) {

	u16 hact;
	u16 vact;
	u8 fmt;
	u8 pa_lpn = 0;

	unsigned int val1, val2;

	msleep(500); //500-->100
	//msleep(100); //500-->100
	regmap_write(lt9211->regmap, 0xff, 0xd0);

	regmap_read(lt9211->regmap, 0x82, &val1);
	regmap_read(lt9211->regmap, 0x83, &val2);
	hact = (val1 << 8) + val2;
	hact = hact / 3;

	regmap_read(lt9211->regmap, 0x84, &val1);
	fmt = (val1 & 0x0f);

	regmap_read(lt9211->regmap, 0x85, &val1);
	regmap_read(lt9211->regmap, 0x86, &val2);

	vact = (val1 << 8) + val2;

	regmap_read(lt9211->regmap, 0x9c, &val1);
	pa_lpn = val1;

	dev_dbg(lt9211->dev, "\r\nhact = %i", hact);
	dev_dbg(lt9211->dev, "\r\nvact = %i", vact);
	dev_dbg(lt9211->dev, "\r\nfmt = %x", fmt);
	dev_dbg(lt9211->dev, "\r\npa_lpn = %x", pa_lpn);

	msleep(100);

	if ((hact == lt9211->video_timing.hact)
			&& (vact == lt9211->video_timing.vact)) {
		dev_dbg(lt9211->dev, "Detected %ix%i\n", hact, vact);
		LT9211_SetVideoTiming(lt9211, &lt9211->video_timing);
		return 0;
	} else {
		dev_err(lt9211->dev, "Could not detect resolution\n");
		return -EPROBE_DEFER;

	}
}

void LT9211_DesscPll_mipi(struct lt9211 *lt9211,
		struct video_timing *video_format) {

	u32 pclk;
	u8 pll_lock_flag;
	u8 i;
	u8 pll_post_div;
	u8 pcr_m;
	unsigned int val;
	pclk = video_format->pclk_khz;

	dev_dbg(lt9211->dev, "\r\n LT9211_DesscPll: set rx pll = %d", pclk);

	regmap_write(lt9211->regmap, 0xff, 0x82);
	regmap_write(lt9211->regmap, 0x2d, 0x48);

	if (pclk > 80000) {
		regmap_write(lt9211->regmap, 0x35, 0x81);
		pll_post_div = 0x01;
	} else if (pclk > 30000) {
		regmap_write(lt9211->regmap, 0x35, 0x82);
		pll_post_div = 0x02;
	} else {
		regmap_write(lt9211->regmap, 0x35, 0x83);
		pll_post_div = 0x04;
	}

	pcr_m = (u8)((pclk * 4 * pll_post_div) / 25000);
	dev_dbg(lt9211->dev, "\r\n LT9211_DesscPll: set rx pll pcr_m = 0x%x",
			pcr_m);

	regmap_write(lt9211->regmap, 0xff, 0xd0);
	regmap_write(lt9211->regmap, 0x2d, 0x40); //M_up_limit
	regmap_write(lt9211->regmap, 0x31, 0x10); //M_low_limit
	regmap_write(lt9211->regmap, 0x26, pcr_m | 0x80);

	regmap_write(lt9211->regmap, 0xff, 0x81); //dessc pll sw rst
	regmap_write(lt9211->regmap, 0x20, 0xef);
	regmap_write(lt9211->regmap, 0x20, 0xff);

	/* pll lock status */
	for (i = 0; i < 6; i++) {
		regmap_write(lt9211->regmap, 0xff, 0x81);
		regmap_write(lt9211->regmap, 0x11, 0xfb); /* pll lock logic reset */
		regmap_write(lt9211->regmap, 0x11, 0xff);

		regmap_write(lt9211->regmap, 0xff, 0x87);
		regmap_read(lt9211->regmap, 0x04, &val);
		pll_lock_flag = val;
		if (pll_lock_flag & 0x01) {
			dev_dbg(lt9211->dev, "\r\n LT9211_DesscPll: dessc pll locked");
			break;
		} else {
			regmap_write(lt9211->regmap, 0xff, 0x81); //dessc pll sw rst
			regmap_write(lt9211->regmap, 0x20, 0xef);
			regmap_write(lt9211->regmap, 0x20, 0xff);
			dev_dbg(lt9211->dev,
					"\r\n LT9211_DesscPll: dessc pll unlocked,sw reset");
		}
	}
}

void LT9211_MipiPcr(struct lt9211 *lt9211) {

	u8 loopx;
	u8 pcr_m;
	unsigned int val;

	regmap_write(lt9211->regmap, 0xff, 0xd0);
	regmap_write(lt9211->regmap, 0x0c, 0x60);  //fifo position
	regmap_write(lt9211->regmap, 0x1c, 0x60);  //fifo position
	regmap_write(lt9211->regmap, 0x24, 0x70);  //pcr mode( de hs vs)
	regmap_write(lt9211->regmap, 0x2d, 0x30); //M up limit
	regmap_write(lt9211->regmap, 0x31, 0x08); //M down limit

	/*stage1 hs mode*/
	regmap_write(lt9211->regmap, 0x25, 0xf0);  //line limit
	regmap_write(lt9211->regmap, 0x2a, 0x30);  //step in limit
	regmap_write(lt9211->regmap, 0x21, 0x4f);  //hs_step
	regmap_write(lt9211->regmap, 0x22, 0x00);

	/*stage2 hs mode*/
	regmap_write(lt9211->regmap, 0x1e, 0x01); //RGD_DIFF_SND[7:4],RGD_DIFF_FST[3:0]
	regmap_write(lt9211->regmap, 0x23, 0x80);  //hs_step
	/*stage2 de mode*/
	regmap_write(lt9211->regmap, 0x0a, 0x02); //de adjust pre line
	regmap_write(lt9211->regmap, 0x38, 0x02); //de_threshold 1
	regmap_write(lt9211->regmap, 0x39, 0x04); //de_threshold 2
	regmap_write(lt9211->regmap, 0x3a, 0x08); //de_threshold 3
	regmap_write(lt9211->regmap, 0x3b, 0x10); //de_threshold 4

	regmap_write(lt9211->regmap, 0x3f, 0x04); //de_step 1
	regmap_write(lt9211->regmap, 0x40, 0x08); //de_step 2
	regmap_write(lt9211->regmap, 0x41, 0x10); //de_step 3
	regmap_write(lt9211->regmap, 0x42, 0x20); //de_step 4

	regmap_write(lt9211->regmap, 0x2b, 0xa0); //stable out
	msleep(100);
	regmap_write(lt9211->regmap, 0xff, 0xd0);   //enable HW pcr_m
	regmap_read(lt9211->regmap, 0x26, &val);
	pcr_m = val;
	pcr_m &= 0x7f;
	msleep(1);
	regmap_write(lt9211->regmap, 0x26, pcr_m);
	regmap_write(lt9211->regmap, 0x27, 0x0f);

	regmap_write(lt9211->regmap, 0xff, 0x81);  //pcr reset
	regmap_write(lt9211->regmap, 0x20, 0xbf); // mipi portB div issue
	regmap_write(lt9211->regmap, 0x20, 0xff);
	msleep(5);
	regmap_write(lt9211->regmap, 0x0B, 0x6F);
	regmap_write(lt9211->regmap, 0x0B, 0xFF);

	msleep(120); //800->120
	for (loopx = 0; loopx < 10; loopx++) { //Check pcr_stable 10
		//msleep(200);
		regmap_write(lt9211->regmap, 0xff, 0xd0);
		regmap_read(lt9211->regmap, 0x87, &val);
		if (val & 0x08) {
			dev_dbg(lt9211->dev, "\r\nLT9211 pcr stable");
			break;
		} else {
			dev_dbg(lt9211->dev, "\r\nLT9211 pcr unstable %x !!!!", val);
		}
	}

	regmap_write(lt9211->regmap, 0xff, 0xd0);
	regmap_read(lt9211->regmap, 0x94, &val);
	dev_dbg(lt9211->dev, "LT9211 pcr_stable_M=%x\n", (val & 0x7F));
}

void LT9211_RXCSC(struct lt9211 *lt9211) {

	regmap_write(lt9211->regmap, 0xff, 0xf9);
	if ( LT9211_OutPutModde == OUTPUT_RGB888) {
		if ( Video_Input_Mode == INPUT_RGB888) {
			regmap_write(lt9211->regmap, 0x86, 0x00);
			regmap_write(lt9211->regmap, 0x87, 0x00);
		} else if ( Video_Input_Mode == INPUT_YCbCr444) {
			regmap_write(lt9211->regmap, 0x86, 0x0f);
			regmap_write(lt9211->regmap, 0x87, 0x00);
		} else if ( Video_Input_Mode == INPUT_YCbCr422_16BIT) {
			regmap_write(lt9211->regmap, 0x86, 0x00);
			regmap_write(lt9211->regmap, 0x87, 0x03);
		}
	} else if ((LT9211_OutPutModde == OUTPUT_BT656_8BIT)
			|| (LT9211_OutPutModde == OUTPUT_BT1120_16BIT)
			|| (LT9211_OutPutModde == OUTPUT_YCbCr422_16BIT)) {
		if ( Video_Input_Mode == INPUT_RGB888) {
			regmap_write(lt9211->regmap, 0x86, 0x0f);
			regmap_write(lt9211->regmap, 0x87, 0x30);
		} else if ( Video_Input_Mode == INPUT_YCbCr444) {
			regmap_write(lt9211->regmap, 0x86, 0x00);
			regmap_write(lt9211->regmap, 0x87, 0x30);
		} else if ( Video_Input_Mode == INPUT_YCbCr422_16BIT) {
			regmap_write(lt9211->regmap, 0x86, 0x00);
			regmap_write(lt9211->regmap, 0x87, 0x00);
		}
	} else if ( LT9211_OutPutModde == OUTPUT_YCbCr444) {
		if ( Video_Input_Mode == INPUT_RGB888) {
			regmap_write(lt9211->regmap, 0x86, 0x0f);
			regmap_write(lt9211->regmap, 0x87, 0x00);
		} else if ( Video_Input_Mode == INPUT_YCbCr444) {
			regmap_write(lt9211->regmap, 0x86, 0x00);
			regmap_write(lt9211->regmap, 0x87, 0x00);
		} else if ( Video_Input_Mode == INPUT_YCbCr422_16BIT) {
			regmap_write(lt9211->regmap, 0x86, 0x00);
			regmap_write(lt9211->regmap, 0x87, 0x03);
		}
	}
}

#if 0
void LT9211_BT_Set(struct lt9211 *lt9211) {

	u16 tmp_data;
	if( (LT9211_OutPutModde == OUTPUT_BT1120_16BIT) || (LT9211_OutPutModde == OUTPUT_BT656_8BIT) )
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

void LT9211_ClockCheckDebug(struct lt9211 *lt9211) {

	u32 val1, val2, val3;
	u32 fm_value;

	regmap_write(lt9211->regmap, 0xff, 0x86);
	regmap_write(lt9211->regmap, 0x00, 0x0a);
	msleep(300);
	fm_value = 0;
	regmap_read(lt9211->regmap, 0x08, &val1);
	val1 = val1 & (0x0f);
	fm_value = (val1 << 8);
	regmap_read(lt9211->regmap, 0x09, &val2);
	fm_value = fm_value + val2;
	fm_value = (fm_value << 8);
	regmap_read(lt9211->regmap, 0x0a, &val3);
	fm_value = fm_value + val3;
	dev_dbg(lt9211->dev, "\r\ndessc pixel clock: %i", fm_value);

}

void LT9211_VideoCheckDebug(struct lt9211 *lt9211) {

	unsigned int sync_polarity, vs, hs, vbp, vfp, hbp, hfp, vtotal, htotal,
			vact, hact;
	unsigned int val1, val2;

	regmap_write(lt9211->regmap, 0xff, 0x86);
	regmap_read(lt9211->regmap, 0x70, &sync_polarity);
	regmap_read(lt9211->regmap, 0x71, &vs);
	regmap_read(lt9211->regmap, 0x72, &val1);
	regmap_read(lt9211->regmap, 0x73, &val2);
	hs = (val1 << 8) + val2;

	regmap_read(lt9211->regmap, 0x74, &vbp);
	regmap_read(lt9211->regmap, 0x75, &vfp);

	regmap_read(lt9211->regmap, 0x76, &val1);
	regmap_read(lt9211->regmap, 0x77, &val2);
	hbp = (val1 << 8) + val2;

	regmap_read(lt9211->regmap, 0x78, &val1);
	regmap_read(lt9211->regmap, 0x79, &val2);
	hfp = (val1 << 8) + val2;

	regmap_read(lt9211->regmap, 0x7A, &val1);
	regmap_read(lt9211->regmap, 0x7B, &val2);
	vtotal = (val1 << 8) + val2;

	regmap_read(lt9211->regmap, 0x7C, &val1);
	regmap_read(lt9211->regmap, 0x7D, &val2);
	htotal = (val1 << 8) + val2;

	regmap_read(lt9211->regmap, 0x7E, &val1);
	regmap_read(lt9211->regmap, 0x7F, &val2);
	vact = (val1 << 8) + val2;

	regmap_read(lt9211->regmap, 0x80, &val1);
	regmap_read(lt9211->regmap, 0x81, &val2);
	hact = (val1 << 8) + val2;

	dev_dbg(lt9211->dev, "\r\nsync_polarity = %x", sync_polarity);

	dev_dbg(lt9211->dev, "\r\nhfp %i, hs %i, hbp %i, hact %i, htotal %i ", hfp,
			hs, hbp, hact, htotal);

	dev_dbg(lt9211->dev, "\r\nvfp %i, vs %i, vbp %i, vact %i, vtotal %i ", vfp,
			vs, vbp, vact, vtotal);

}

void LT9211_MipiRxPll(struct lt9211 *lt9211, struct video_timing *video_format) {

	/* dessc pll */
	regmap_write(lt9211->regmap, 0xff, 0x82);
	regmap_write(lt9211->regmap, 0x2d, 0x48);

	if ((video_format->pclk_khz) < PCLK_KHZ_44000) {
		regmap_write(lt9211->regmap, 0x35, PIXCLK_22M_44M);/*0x83*/
	}

	else if (video_format->pclk_khz < PCLK_KHZ_88000) {
		regmap_write(lt9211->regmap, 0x35, PIXCLK_44M_88M); /*0x83*/
	}

	else if (video_format->pclk_khz < PCLK_KHZ_176000) {
		regmap_write(lt9211->regmap, 0x35, PIXCLK_88M_176M); /*0x81*/
	}

	else if (video_format->pclk_khz < PCLK_KHZ_352000) {
		regmap_write(lt9211->regmap, 0x35, PIXCLK_LARGER_THAN_176M);/*0x80*/
	}
}

void LT9211_InvertRGB_HSVSPoarity(struct lt9211 *lt9211) {

	u32 Reg8263_Value, RegD020_Value;

	dev_dbg(lt9211->dev, "\rLT9211 invert HS VS Poarity");

	regmap_write(lt9211->regmap, 0xff, 0x82);
	regmap_read(lt9211->regmap, 0x63, &Reg8263_Value);
	dev_dbg(lt9211->dev, "\r\n reg0x8263= %x", Reg8263_Value); //0xff

	regmap_write(lt9211->regmap, 0xff, 0xd0);
	regmap_read(lt9211->regmap, 0x20, &RegD020_Value);
	dev_dbg(lt9211->dev, "\r\n reg0xD020= %x", RegD020_Value); //0x07

	regmap_write(lt9211->regmap, 0xff, 0x82);
	regmap_write(lt9211->regmap, 0x63, 0xfb); //0x8263 Bit2 =0 disable DE output MODE

	//to invert HS VS Poarity to Negative in REG 0xD020 bit7-6 to 1, Not bit5-DE
	regmap_write(lt9211->regmap, 0xff, 0xD0);
	regmap_write(lt9211->regmap, 0x20, 0x08); //0x08
}

int LT9211_Init(struct lt9211 *lt9211) {

	int ret;
	dev_dbg(lt9211->dev, "\r\n LT9211_mipi to TTL");
	LT9211_SystemInt(lt9211);
	LT9211_MipiRxPhy(lt9211);
	LT9211_MipiRxDigital(lt9211);
	ret = LT9211_TimingSet(lt9211);
	if (ret)
		return ret;

	//LT9211_MipiRxPll(lt9211, &lt9211->video_timing);
	LT9211_DesscPll_mipi(lt9211, &lt9211->video_timing);
	LT9211_MipiPcr(lt9211);
	LT9211_TxDigital(lt9211);
	LT9211_TxPhy(lt9211);
	//LT9211_InvertRGB_HSVSPoarity(lt9211);//only for TTL output
	msleep(10);
	LT9211_Txpll(lt9211);
	LT9211_RXCSC(lt9211);
	LT9211_ClockCheckDebug(lt9211);
	LT9211_VideoCheckDebug(lt9211);
	//LT9211_BT_Set(lt9211);
	return ret;
}

static void LT9211_Patten_debug_M2TTL(struct lt9211 *lt9211) {

	dev_dbg(lt9211->dev, "\r\n LT9211_Patten_debug");
	LT9211_SystemInt(lt9211);
	msleep(100);
	LT9211_TxDigital(lt9211);
	LT9211_TxPhy(lt9211);
	LT9211_Pattern(lt9211, &lt9211->video_timing);
	msleep(10);
	LT9211_Txpll(lt9211);
	msleep(10);
	LT9211_ClockCheckDebug(lt9211);
}

static void lt9211_reset(struct lt9211 *lt9211) {

	gpiod_set_value_cansleep(lt9211->reset_gpio, 1);
	msleep(20);

	gpiod_set_value_cansleep(lt9211->reset_gpio, 0);
	msleep(20);

	gpiod_set_value_cansleep(lt9211->reset_gpio, 1);
	msleep(100);
}

static int lt9211_regulator_init(struct lt9211 *lt9211) {

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

	return 0;
}

static int lt9211_parse_dt(struct device *dev, struct lt9211 *lt9211) {

	lt9211->debug_mode = of_property_read_bool(dev->of_node, "lt,debug-mode");

	if (of_property_read_u32(lt9211->panel->dev->of_node, "dsi,lanes",
			&lt9211->num_mipi_lanes)) {
		dev_err(dev, "failed to find number of mipi lanes\n");
		return -ENODEV;
	}
	if (lt9211->num_mipi_lanes > 4) {
		dev_err(dev, "invalid number of mipi lanes\n");
		return -ENODEV;
	}
	/* 0: 4 Lane / 1: 1 Lane / 2 : 2 Lane / 3: 3 Lane */
	if (lt9211->num_mipi_lanes == 4)
		lt9211->num_mipi_lanes = 0;

	lt9211->host_node = of_graph_get_remote_node(dev->of_node, /*0*/1, 0);
	if (!lt9211->host_node) {
		dev_dbg(lt9211->dev, "parent np %s %s\n", dev->of_node->name,
				dev->of_node->full_name);
		return -ENODEV;
	}

	return 0;
}

static int lt9211_gpio_init(struct lt9211 *lt9211) {

	struct device *dev = lt9211->dev;

	lt9211->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(lt9211->reset_gpio)) {
		dev_err(dev, "failed to acquire reset gpio\n");
		return PTR_ERR(lt9211->reset_gpio);
	}

	return 0;
}

static int lt9211_read_device_rev(struct lt9211 *lt9211) {

	uint32_t rev = 0;
	uint32_t rev_tmp;
	int ret;

	regmap_write(lt9211->regmap, 0xff, 0x81);
	ret = regmap_read(lt9211->regmap, 0x00, &rev_tmp);
	rev |= rev_tmp << 16;
	ret = regmap_read(lt9211->regmap, 0x01, &rev_tmp);
	rev |= rev_tmp << 8;
	ret = regmap_read(lt9211->regmap, 0x02, &rev_tmp);
	rev |= rev_tmp;
	if (ret)
		dev_err(lt9211->dev, "failed to read revision: %d\n", ret);
	else
		dev_info(lt9211->dev, "lt9211 revision: 0x%x\n", rev);

	return ret;
}

static int lt9211_set_display_timings(struct lt9211 *lt9211) {

	struct display_timing timing;
	struct drm_panel * panel = lt9211->panel;

	panel->funcs->get_timings(panel, 1, &timing);

	lt9211->video_timing.hfp = timing.hfront_porch.typ;
	lt9211->video_timing.hs = timing.hsync_len.typ;
	lt9211->video_timing.hbp = timing.hback_porch.typ;
	lt9211->video_timing.hact = timing.hactive.typ;
	lt9211->video_timing.htotal = timing.hfront_porch.typ + timing.hsync_len.typ
			+ timing.hback_porch.typ + timing.hactive.typ;

	lt9211->video_timing.vfp = timing.vfront_porch.typ;
	lt9211->video_timing.vs = timing.vsync_len.typ;
	lt9211->video_timing.vbp = timing.vback_porch.typ;
	lt9211->video_timing.vact = timing.vactive.typ;
	lt9211->video_timing.vtotal = timing.vfront_porch.typ + timing.vsync_len.typ
			+ timing.vback_porch.typ + timing.vactive.typ;

	lt9211->video_timing.pclk_khz = timing.pixelclock.typ / 1000;

	dev_dbg(lt9211->dev, "#### vtotal %i, htotal %i pclk_khz %i\n",
			lt9211->video_timing.vtotal, lt9211->video_timing.htotal,
			lt9211->video_timing.pclk_khz);
	return 0;
}

static int lt9211_probe(struct i2c_client *client,
		const struct i2c_device_id *id) {

	struct lt9211 *lt9211;
	struct device *dev = &client->dev;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(dev, "device doesn't support I2C\n");
		return -ENODEV;
	}

	lt9211 = devm_kzalloc(dev, sizeof(*lt9211), GFP_KERNEL);
	if (!lt9211)
		return -ENOMEM;

	ret = drm_of_find_panel_or_bridge(dev->of_node, 2, 0, &lt9211->panel, NULL);
	if (ret) {
		DRM_ERROR("could not find any panel node\n");
		return -EPROBE_DEFER;
	}

	lt9211_set_display_timings(lt9211);

	lt9211->dev = &client->dev;
	lt9211->client = client;
	lt9211->sleep = false;

	lt9211->regmap = devm_regmap_init_i2c(client, &lt9211_regmap_config);
	if (IS_ERR(lt9211->regmap)) {
		dev_err(lt9211->dev, "regmap i2c init failed\n");
		return PTR_ERR(lt9211->regmap);
	}

	ret = lt9211_parse_dt(&client->dev, lt9211);
	if (ret) {
		dev_err(dev, "failed to parse device tree\n");
		return ret;
	}

	ret = lt9211_gpio_init(lt9211);
	if (ret < 0)
		goto err_of_put;

	ret = lt9211_regulator_init(lt9211);
	if (ret < 0)
		goto err_of_put;

	ret = lt9211_regulator_enable(lt9211);
	if (ret)
		goto err_of_put;

	lt9211_reset(lt9211);

	i2c_set_clientdata(client, lt9211);

	ret = lt9211_read_device_rev(lt9211);
	if (ret) {
		dev_err(dev, "failed to read chip rev\n");
		goto err_disable_regulators;
	}

	lt9211->bridge.of_node = client->dev.of_node;

	drm_bridge_add(&lt9211->bridge);

	if (lt9211->debug_mode) {
		LT9211_Patten_debug_M2TTL(lt9211);
	} else {
		ret = LT9211_Init(lt9211);
		if (ret)
			goto err_disable_regulators;
	}
	return ret;

	err_disable_regulators:
	regulator_bulk_disable(ARRAY_SIZE(lt9211->supplies),
			lt9211->supplies);

	err_of_put:
	of_node_put(lt9211->dsi1_node);
	of_node_put(lt9211->dsi0_node);

	return ret;
}

static int lt9211_remove(struct i2c_client *client) {

	struct lt9211 *lt9211 = i2c_get_clientdata(client);

	disable_irq(client->irq);

	drm_bridge_remove(&lt9211->bridge);

	regulator_bulk_disable(ARRAY_SIZE(lt9211->supplies), lt9211->supplies);

	of_node_put(lt9211->dsi1_node);
	of_node_put(lt9211->dsi0_node);

	return 0;
}

static struct i2c_device_id lt9211_id[] = { { "lontium,lt9211", 0 }, { } };

static const struct of_device_id lt9211_match_table[] = { { .compatible =
		"lontium,lt9211" }, { } };
MODULE_DEVICE_TABLE( of, lt9211_match_table);

static struct i2c_driver lt9211_driver = { .driver = { .name = "lt9211",
		.of_match_table = lt9211_match_table, }, .probe = lt9211_probe,
		.remove = lt9211_remove, .id_table = lt9211_id, };
module_i2c_driver( lt9211_driver);

MODULE_AUTHOR("Philipp Gerbach <gerbach@fs-net.de>");
MODULE_DESCRIPTION("LONTIUM LT9211 MIPI2RGB Converter Driver");
MODULE_LICENSE("GPL v2");
