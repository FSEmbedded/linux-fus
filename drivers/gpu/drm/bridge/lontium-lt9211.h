/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Author: Philipp Gerbach , <gerbach@fs-net.de>
 *
 * Based on example code by Lontium.
 *
 * This is a driver for the lontium-lt9211 as MIPI2RGB converter.
 */

#ifndef __LONTIUM_LT9211_H__
#define __LONTIUM_LT9211_H__


#define MIPI_SETTLE_VALUE 	0x05 //0x05  0x0a
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

#define LT9211_Output_Mode  OUTPUT_RGB888

typedef enum VIDEO_INPUTMODE_ENUM {
	INPUT_RGB888 = 1,
	INPUT_YCbCr444 = 2,
	INPUT_YCbCr422_16BIT = 3
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

enum lt9211_ports {
	LT9211_DSI_IN,
	LT9211_RGB_OUT0,
};

struct lt9211 {
	struct device *dev;
	struct drm_bridge bridge;
	struct drm_bridge	*panel_bridge;
	struct regmap *regmap;
	struct device_node *host_node;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *enable_gpio;
	struct regulator_bulk_data supplies[2];
	struct drm_display_mode mode;
	bool debug_pattern;
	bool pclk_invert;
	u32 chip_rev;
	u8 num_dsi_lanes;
	u8 rx_source;
	u8 bpc;
	u8 bus_fmt;
};

static const struct regmap_range_cfg lt9211_ranges[] = {
	{
		.name =	"register_range",
		.range_min = 0,
		.range_max = 0xffff,
		.selector_reg = lt9211_PAGE_CONTROL,
		.selector_mask = 0xff,
		.selector_shift = 0,
		.window_start = 0,
		.window_len = 0x100,
	},
};

static const struct regmap_config lt9211_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xffff,
//	.ranges = lt9211_ranges,
//	.num_ranges = ARRAY_SIZE(lt9211_ranges),
};

#endif /* __LONTIUM_LT9211_H__ */