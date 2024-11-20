// SPDX-License-Identifier: GPL-2.0
/*
 * Author: Philipp Gerbach , <gerbach@fs-net.de>
 *
 * Based on example code by Lontium.
 *
 * This is a driver for the lontium-lt9211 as RGB2LVDS converter.
 */

#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/fb.h>
#include <linux/of.h>
#include <video/of_display_timing.h>
#include <video/videomode.h>
#include <video/display_timing.h>

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

typedef enum VIDEO_INPUTMODE_ENUM {
	INPUT_RGB888 = 1,
	INPUT_YCbCr444 = 2,
	INPUT_YCbCr422_16BIT = 3
} Video_Input_Mode_TypeDef;

struct lt9211 {
	struct device *dev;
	struct i2c_client *client;
	struct regmap *regmap;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *enable_gpio;
	struct regulator_bulk_data supplies[2];
	struct videomode video_timing;
	bool debug_mode;
	bool pattern_mode;
	bool lvds_format_jeida;
	u32 lvds_channel;
	u32 vic;
} lt9211;

static const struct regmap_range_cfg lt9211_ranges[] = {
	{
		.name = "register_range",
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

static int LT9211_SystemInt(void) {
	int ret;

	/* System initialization */
	const struct reg_sequence seq[] = {
		{ 0xff, 0x82 }, /* switch to register group 0x82 */
		{ 0x01, 0x18 },
		{ 0xff, 0x86 }, /* switch to register group 0x86 */
		{ 0x06, 0x61 },
		{ 0x07, 0xa8 },
		{ 0xff, 0x87 }, /* switch to register group 0x87 */
		{ 0x14, 0x08 },
		{ 0x15, 0x00 },
		{ 0x18, 0x0f },
		{ 0x22, 0x08 },
		{ 0x23, 0x00 },
		{ 0x26, 0x0f },
	};

	ret = regmap_multi_reg_write(lt9211.regmap, seq, ARRAY_SIZE(seq));
	if (ret) {
		dev_err(lt9211.dev, "LT9211_SystemInt failed!\n");
		return ret;
	}

	return ret;
}

static int LT9211_TxDigital(void) {
	int ret = 0;

	const struct reg_sequence seq_1[] = {
		{ 0xff, 0x85 }, /* switch to register group 0x85 */
		{ 0x59, 0x50 },
		{ 0x5a, 0xaa },
		{ 0x5b, 0xaa },
	};

	const struct reg_sequence seq_2[] = {
		{ 0x88, 0x80 },
		{ 0xa1, 0x77 },
		{ 0xff, 0x86 }, /* switch to register group 0x86 */
		{ 0x40, 0x40 },
		{ 0x41, 0x34 },
		{ 0x42, 0x10 },
		{ 0x43, 0x23 },
		{ 0x44, 0x41 },
		{ 0x45, 0x02 },
	};

	ret = regmap_multi_reg_write(lt9211.regmap, seq_1, ARRAY_SIZE(seq_1));
	if (ret) {
		dev_err(lt9211.dev, "LT9211_TxDigital failed!\n");
		return ret;
	}

	if (lt9211.lvds_channel == 0) {
		ret = regmap_write(lt9211.regmap, 0x5c, 0x00);
		if (ret) {
			dev_err(lt9211.dev, "LT9211_TxDigital failed!\n");
			return ret;
		}
	} else {
		ret = regmap_write(lt9211.regmap, 0x5c, 0x01);
		if (ret) {
			dev_err(lt9211.dev, "LT9211_TxDigital failed!\n");
			return ret;
		}
	}

	ret = regmap_multi_reg_write(lt9211.regmap, seq_2, ARRAY_SIZE(seq_2));
	if (ret) {
		dev_err(lt9211.dev, "LT9211_TxDigital failed!\n");
		return ret;
	}

	if (lt9211.lvds_format_jeida) {
		const struct reg_sequence jeida[] = {
			{ 0xff, 0x85 },
			{ 0x59, 0xd0 },
			{ 0xff, 0xd8 },
			{ 0x11, 0x40 },
		};

		ret = regmap_multi_reg_write(lt9211.regmap, jeida, ARRAY_SIZE(jeida));
		if (ret) {
			dev_err(lt9211.dev, "LT9211_TxDigital failed!\n");
			return ret;
		}
	}

	return ret;
}

static int LT9211_TxPhy(void) {
	int ret = 0;

	const struct reg_sequence seq_1[] = {
		{ 0xff, 0x82 }, /* switch to register group 0x82 */
		{ 0x62, 0x00 },
	};

	const struct reg_sequence seq_2[] = {
		{ 0x3e, 0x92 },
		{ 0x3f, 0x48 },
		{ 0x40, 0x31 },
		{ 0x43, 0x80 },
		{ 0x44, 0x00 },
		{ 0x45, 0x00 },
		{ 0x49, 0x00 },
		{ 0x4a, 0x01 },
		{ 0x4e, 0x00 },
		{ 0x4f, 0x00 },
		{ 0x50, 0x00 },
		{ 0x53, 0x00 },
		{ 0x54, 0x01 },
		{ 0xff, 0x81 }, /* switch to register group 0x81 */
		{ 0x20, 0x7b },
		{ 0x20, 0xff },
	};

	ret = regmap_multi_reg_write(lt9211.regmap, seq_1, ARRAY_SIZE(seq_1));
	if (ret) {
		dev_err(lt9211.dev, "LT9211_TxPhy failed!\n");
		return ret;
	}

	if (lt9211.lvds_channel == 0) {
		ret = regmap_write(lt9211.regmap, 0x3b, 0x38);
		if (ret) {
			dev_err(lt9211.dev, "LT9211_TxPhy failed!\n");
			return ret;
		}
	} else {
		ret = regmap_write(lt9211.regmap, 0x3b, 0xb8);
		if (ret) {
			dev_err(lt9211.dev, "LT9211_TxPhy failed!\n");
			return ret;
		};
	}

	ret = regmap_multi_reg_write(lt9211.regmap, seq_2, ARRAY_SIZE(seq_2));
	if (ret) {
		dev_err(lt9211.dev, "LT9211_TxPhy failed!\n");
		return ret;
	}

	return ret;
}

static int LT9211_Pattern(struct videomode *video_format) {
	u32 pclk_khz;
	u8 dessc_pll_post_div = 0;
	u32 h_start = video_format->hsync_len + video_format->hback_porch;
	u32 v_start = video_format->vsync_len + video_format->vback_porch;
	u32 h_active = video_format->hactive;
	u32 v_active = video_format->vactive;
	u32 h_total = video_format->hactive + video_format->hfront_porch + video_format->hback_porch + video_format->hsync_len;
	u32 v_total = video_format->vactive + video_format->vfront_porch + video_format->vback_porch + video_format->vsync_len;
	u32 hsync_len = video_format->hsync_len;
	u32 vsync_len = video_format->vsync_len;
	u32 pcr_m, pcr_k;
	int ret = 0;

	const struct reg_sequence seq_disp_setup[] = {
		{ 0xff, 0xf9}, /* switch to register group 0xf9 */
		{ 0x3e, 0x80},
		{ 0xff, 0x85}, /* switch to register group 0x85 */
		{ 0x88, 0xc0 }, /* Chip active RX source select: Chip Video pattern gen. */
		{ 0xa1, 0x04 }, /* DATA from register PTN_DATA_VALUE. (R,G,B) DP_DATA, 00, 00; */
//		{ 0xa1, 0x02 }, /* DATA from register PTN_DATA_VALUE. (R,G,B) 00, DP_DATA, 00; */
//		{ 0xa1, 0x01 }, /* DATA from register PTN_DATA_VALUE. (R,G,B) 00, 00, DP_DATA; */
		{ 0xa2, 0xff }, /* PTN_DATA_VALUE */
		{ 0xa3, (u8)(h_start / 256) },
		{ 0xa4, (u8)(h_start % 256) },
		{ 0xa5, (u8)(v_start % 256) },
		{ 0xa6, (u8)(h_active / 256) },
		{ 0xa7, (u8)(h_active % 256) },
		{ 0xa8, (u8)(v_active / 256) },
		{ 0xa9, (u8)(v_active % 256) },
		{ 0xaa, (u8)(h_total / 256) },
		{ 0xab, (u8)(h_total % 256) },
		{ 0xac, (u8)(v_total / 256) },
		{ 0xad, (u8)(v_total % 256) },
		{ 0xae, (u8)(hsync_len / 256) },
		{ 0xaf, (u8)(hsync_len % 256) },
		{ 0xb0, (u8)(vsync_len % 256) },
	};

	const struct reg_sequence seq_clk_setup[] = {
		/* lt9211_System_Init */
		{ 0xff, 0xd0},
		{ 0x2d, 0x7f},
		{ 0x31, 0x00},
	};

	pclk_khz = video_format->pixelclock / 1000;

	ret = regmap_multi_reg_write(lt9211.regmap, seq_disp_setup, ARRAY_SIZE(seq_disp_setup));
	if (ret)
		return ret;
	ret = regmap_write(lt9211.regmap, 0xff, 0x82);
	if (ret)
		return ret;
	ret = regmap_write(lt9211.regmap, 0x2d, 0x48);
	if (ret)
		return ret;

	if (pclk_khz < 44000) {
		ret = regmap_write(lt9211.regmap, 0x35, 0x83);
		if (ret)
			return ret;
		dessc_pll_post_div = 16;
	}

	else if (pclk_khz < 88000) {
		ret = regmap_write(lt9211.regmap, 0x35, 0x82);
		if (ret)
			return ret;
		dessc_pll_post_div = 8;
	}

	else if (pclk_khz < 176000) {
		ret = regmap_write(lt9211.regmap, 0x35, 0x81);
		if (ret)
			return ret;
		dessc_pll_post_div = 4;
	}

	else if (pclk_khz < 352000) {
		ret = regmap_write(lt9211.regmap, 0x35, 0x80);
		if (ret)
			return ret;
		dessc_pll_post_div = 0;
	}

	pcr_m = (pclk_khz * dessc_pll_post_div) / 25;
	pcr_k = pcr_m % 1000;
	pcr_m = pcr_m / 1000;
	pcr_k <<= 14;

	ret = regmap_multi_reg_write(lt9211.regmap, seq_clk_setup, ARRAY_SIZE(seq_clk_setup));
	if (ret)
		return ret;
	ret = regmap_write(lt9211.regmap, 0x26, 0x80 | ((u8) pcr_m));
	if (ret)
		return ret;
	ret = regmap_write(lt9211.regmap, 0x27, (u8)((pcr_k >> 16) & 0xff));
	if (ret)
		return ret;
	ret = regmap_write(lt9211.regmap, 0x28, (u8)((pcr_k >> 8) & 0xff));
	if (ret)
		return ret;
	ret = regmap_write(lt9211.regmap, 0x29, (u8)(pcr_k & 0xff));
	if (ret)
		return ret;


	if (lt9211.debug_mode) {
		int val;
		regmap_read(lt9211.regmap, 0x26, &val);
		dev_info(lt9211.dev, "0xd026: %x\n", val);
		regmap_read(lt9211.regmap, 0x27, &val);
		dev_info(lt9211.dev, "0xd027: %x\n", val);
		regmap_read(lt9211.regmap, 0x28, &val);
		dev_info(lt9211.dev, "0xd028: %x\n", val);
		regmap_read(lt9211.regmap, 0x29, &val);
		dev_info(lt9211.dev, "0xd029: %x\n", val);
	}

	return 0;
}

static int LT9211_Txpll(void) {

	u8 loopx;
	unsigned int val;
	int ret = 0;

	const struct reg_sequence seq_1[] = {
		{ 0xff, 0x82 }, /* switch to register group 0x82 */
		{ 0x36, 0x01 },
	};

	const struct reg_sequence seq_2[] = {
		{ 0x38, 0x06 },
		{ 0x39, 0x30 },
		{ 0x3a, 0x8e },
		{ 0xff, 0x87 }, /* switch to register group 0x87 */
		{ 0x37, 0x14 },
		{ 0x13, 0x00 },
		{ 0x13, 0x80 },
	};

	ret = regmap_multi_reg_write(lt9211.regmap, seq_1, ARRAY_SIZE(seq_1));
	if (ret) {
		dev_err(lt9211.dev, "LT9211_Txpll failed!\n");
		return ret;
	}

	if (lt9211.lvds_channel == 0) {
		ret = regmap_write(lt9211.regmap, 0x37, 0x29);
		if (ret) {
			dev_err(lt9211.dev, "LT9211_Txpll failed!\n");
			return ret;
		}
	} else {
		ret = regmap_write(lt9211.regmap, 0x37, 0x2a);
		if (ret) {
			dev_err(lt9211.dev, "LT9211_Txpll failed!\n");
			return ret;
		}
	}

	ret = regmap_multi_reg_write(lt9211.regmap, seq_2, ARRAY_SIZE(seq_2));
	if (ret) {
		dev_err(lt9211.dev, "LT9211_Txpll failed!\n");
		return ret;
	}
	msleep(100);

	if (lt9211.debug_mode) {
		for (loopx = 0; loopx < 10; loopx++) { //Check Tx PLL cal
			regmap_read(lt9211.regmap, 0x1f, &val);
			if (val & 0x80) {
				regmap_read(lt9211.regmap, 0x20, &val);
				if (val & 0x80) {
					dev_info(lt9211.dev, "LT9211 tx pll lock\n");
				} else {
					dev_info(lt9211.dev, "LT9211 tx pll unlocked\n");
				}
				dev_info(lt9211.dev, "LT9211 tx pll cal done");
				break;
			} else {
				dev_info(lt9211.dev, "LT9211 tx pll unlocked %x\n", val);
			}
		}
	}

	return ret;
}

static int LT9211_TTLRxPhy(void) {
	int ret = 0;

	const struct reg_sequence seq[] = {
		{ 0xff, 0x82 }, /* switch to register group 0x82 */
		{ 0x28, 0x40 },
		{ 0x61, 0x01 },
		{ 0xff, 0x85 }, /* switch to register group 0x85 */
		{ 0x88, 0x80 },
		{ 0x45, 0x70 },
	};

	ret = regmap_multi_reg_write(lt9211.regmap, seq, ARRAY_SIZE(seq));
	if (ret) {
		dev_err(lt9211.dev, "LT9211_TTLRxPhy failed!\n");
		return ret;
	}

	return ret;
}

static void LT9211_ClockCheckDebug(void) {

	u32 val1, val2, val3;
	u32 fm_value;

	regmap_write(lt9211.regmap, 0xff, 0x86);
	regmap_write(lt9211.regmap, 0x00, 0x0a);
	msleep(300);
	fm_value = 0;
	regmap_read(lt9211.regmap, 0x08, &val1);
	val1 = val1 & (0x0f);
	fm_value = (val1 << 8);
	regmap_read(lt9211.regmap, 0x09, &val2);
	fm_value = fm_value + val2;
	fm_value = (fm_value << 8);
	regmap_read(lt9211.regmap, 0x0a, &val3);
	fm_value = fm_value + val3;

	dev_info(lt9211.dev, "\r\ndessc pixel clock: %i", fm_value);
}

static void LT9211_VideoCheckDebug(void) {

	unsigned int sync_polarity, vs, hs, vbp, vfp, hbp, hfp, vtotal, htotal,
			vact, hact;
	unsigned int val1, val2;

	regmap_write(lt9211.regmap, 0xff, 0x86);

	regmap_read(lt9211.regmap, 0x70, &sync_polarity);
	regmap_read(lt9211.regmap, 0x71, &vs);
	regmap_read(lt9211.regmap, 0x72, &val1);
	regmap_read(lt9211.regmap, 0x73, &val2);
	hs = (val1 << 8) + val2;

	regmap_read(lt9211.regmap, 0x74, &vbp);
	regmap_read(lt9211.regmap, 0x75, &vfp);

	regmap_read(lt9211.regmap, 0x76, &val1);
	regmap_read(lt9211.regmap, 0x77, &val2);
	hbp = (val1 << 8) + val2;

	regmap_read(lt9211.regmap, 0x78, &val1);
	regmap_read(lt9211.regmap, 0x79, &val2);
	hfp = (val1 << 8) + val2;

	regmap_read(lt9211.regmap, 0x7A, &val1);
	regmap_read(lt9211.regmap, 0x7B, &val2);
	vtotal = (val1 << 8) + val2;

	regmap_read(lt9211.regmap, 0x7C, &val1);
	regmap_read(lt9211.regmap, 0x7D, &val2);
	htotal = (val1 << 8) + val2;

	regmap_read(lt9211.regmap, 0x7E, &val1);
	regmap_read(lt9211.regmap, 0x7F, &val2);
	vact = (val1 << 8) + val2;

	regmap_read(lt9211.regmap, 0x80, &val1);
	regmap_read(lt9211.regmap, 0x81, &val2);
	hact = (val1 << 8) + val2;

	dev_info(lt9211.dev, "\r\nsync_polarity = %x", sync_polarity);

	dev_info(lt9211.dev, "\r\nhfp %i, hs %i, hbp %i, hact %i, htotal %i ", hfp,
			hs, hbp, hact, htotal);

	dev_info(lt9211.dev, "\r\nvfp %i, vs %i, vbp %i, vact %i, vtotal %i ", vfp,
			vs, vbp, vact, vtotal);
}

static int LT9211_HW_Init(void) {
	int ret = 0;

	ret = LT9211_SystemInt();
	if (ret)
		goto failed;
	ret = LT9211_TTLRxPhy();
	if (ret)
		goto failed;
	msleep(100);
	ret = LT9211_TxDigital();
	if (ret)
		goto failed;
	ret = LT9211_TxPhy();
	if (ret)
		goto failed;

	if(lt9211.pattern_mode) {
		LT9211_Pattern(&lt9211.video_timing);
		if (ret)
			goto failed;
	}

	msleep(10);
	ret = LT9211_Txpll();
	if (ret)
		goto failed;

	if(lt9211.debug_mode) {
		LT9211_VideoCheckDebug();
		LT9211_ClockCheckDebug();
	}

	return ret;
failed:
	dev_err(lt9211.dev, "failed init LT9211\n");
	return ret;
}

static void lt9211_reset(void) {

	gpiod_set_value_cansleep(lt9211.reset_gpio, 1);
	msleep(20);

	gpiod_set_value_cansleep(lt9211.reset_gpio, 0);
	msleep(20);

	gpiod_set_value_cansleep(lt9211.reset_gpio, 1);
	msleep(100);
}

static int lt9211_regulator_init(void) {

	int ret;

	lt9211.supplies[0].supply = "vdd";
	lt9211.supplies[1].supply = "vcc";

	ret = devm_regulator_bulk_get(lt9211.dev, 2, lt9211.supplies);
	if (ret < 0)
		return ret;

	return regulator_set_load(lt9211.supplies[0].consumer, 300000);
}

static int lt9211_regulator_enable(void) {

	int ret;

	ret = regulator_enable(lt9211.supplies[0].consumer);
	if (ret < 0)
		return ret;

	usleep_range(1000, 10000);

	ret = regulator_enable(lt9211.supplies[1].consumer);
	if (ret < 0) {
		regulator_disable(lt9211.supplies[0].consumer);
		return ret;
	}

	return 0;
}

static int lt9211_gpio_init(void) {
	lt9211.reset_gpio = devm_gpiod_get(lt9211.dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(lt9211.reset_gpio)) {
		dev_err(lt9211.dev, "failed to acquire reset gpio\n");
		return PTR_ERR(lt9211.reset_gpio);
	}

	return 0;
}

static int lt9211_read_device_rev(void) {

	uint32_t rev = 0;
	uint32_t rev_tmp;
	int ret;

	regmap_write(lt9211.regmap, 0xff, 0x81);
	ret = regmap_read(lt9211.regmap, 0x00, &rev_tmp);
	rev |= rev_tmp << 16;
	ret = regmap_read(lt9211.regmap, 0x01, &rev_tmp);
	rev |= rev_tmp << 8;
	ret = regmap_read(lt9211.regmap, 0x02, &rev_tmp);
	rev |= rev_tmp;
	if (ret)
		dev_err(lt9211.dev, "failed to read revision: %d\n", ret);
	else
		dev_info(lt9211.dev, "lt9211 revision: 0x%x\n", rev);

	return ret;
}

static int lt9211_fb_event(struct notifier_block *nb, unsigned long val, void *v)
{
	int ret = 0;

	switch (val) {
	case FB_EVENT_FB_REGISTERED:
		ret = LT9211_HW_Init();
		if (ret) {
			dev_err(lt9211.dev, "LT9211_HW_Init failed %d\n", ret);
			return ret;
		}
		break;
	case FB_EVENT_MODE_CHANGE:
	case FB_EVENT_BLANK:
	default:
		break;
	}

	return 0;
}

static struct notifier_block nb = {
	.notifier_call = lt9211_fb_event,
};

static int lt9211_parse_dt(void)
{
	struct display_timings *timings = NULL;
	int ret = 0;
	u32 val = 0;

	lt9211.debug_mode = of_property_read_bool(lt9211.dev->of_node, "lt,debug-mode");
	lt9211.pattern_mode = of_property_read_bool(lt9211.dev->of_node, "lt,pattern-mode");

	if (of_property_read_u32(lt9211.dev->of_node, "lvds-channel", &val) == 0)
		lt9211.lvds_channel = val;
	else
		lt9211.lvds_channel = 0;

	lt9211.lvds_format_jeida = of_property_read_bool(lt9211.dev->of_node, "lvds-format-jeida");

	timings = of_get_display_timings(lt9211.dev->of_node);
	if (!timings) {
		dev_err(lt9211.dev, "failed to get display timings\n");
		ret = -ENOENT;
		goto put_display_node;
	}

	ret = videomode_from_timings(timings, &lt9211.video_timing, 0);
	if (ret < 0)
		goto put_display_node;

put_display_node:
	return ret;
}

static int lt9211_probe(struct i2c_client *client) {

	struct device *dev = &client->dev;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(dev, "device doesn't support I2C\n");
		return -ENODEV;
	}

	lt9211.dev = &client->dev;
	lt9211.client = client;

	lt9211.regmap = devm_regmap_init_i2c(client, &lt9211_regmap_config);
	if (IS_ERR(lt9211.regmap)) {
		dev_err(lt9211.dev, "regmap i2c init failed\n");
		return PTR_ERR(lt9211.regmap);
	}

	ret = lt9211_parse_dt();
	if (ret) {
		dev_err(dev, "failed to parse device tree\n");
		return ret;
	}

	ret = lt9211_gpio_init();
	if (ret < 0)
		return -ENODEV;

	ret = lt9211_regulator_init();
	if (ret < 0)
		return ret;

	ret = lt9211_regulator_enable();
	if (ret)
		return ret;

	lt9211_reset();

	i2c_set_clientdata(client, &lt9211);

	ret = lt9211_read_device_rev();
	if (ret) {
		dev_err(dev, "failed to read chip rev\n");
		goto err_disable_regulators;
	}

	return fb_register_client(&nb);

err_disable_regulators:
	regulator_bulk_disable(ARRAY_SIZE(lt9211.supplies), lt9211.supplies);

	return ret;
}

static void lt9211_remove(struct i2c_client *client) {
	fb_unregister_client(&nb);
	disable_irq(client->irq);
	regulator_bulk_disable(ARRAY_SIZE(lt9211.supplies), lt9211.supplies);
}

static const struct i2c_device_id lt9211_id[] = {
	{ "lontium,lt9211", 0},
	{ },
};
MODULE_DEVICE_TABLE(i2c, lt9211_id);

static const struct of_device_id lt9211_match_table[] = {
	{ .compatible = "lontium,lt9211", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, lt9211_match_table);

static struct i2c_driver lt9211_driver = {
	.driver = {
		.name = "lt9211",
		.owner = THIS_MODULE,
		.of_match_table = lt9211_match_table,
	},
	.probe		= lt9211_probe,
	.remove		= lt9211_remove,
	.id_table	= lt9211_id,
};

module_i2c_driver(lt9211_driver);

MODULE_AUTHOR("Philipp Gerbach <gerbach@fs-net.de>");
MODULE_DESCRIPTION("LONTIUM LT9211 RGB2LVDS Converter Driver");
MODULE_LICENSE("GPL v2");
