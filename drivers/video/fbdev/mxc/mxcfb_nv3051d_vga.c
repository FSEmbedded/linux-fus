/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/types.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/console.h>
#include <linux/io.h>
#include <linux/bitops.h>
#include <linux/spinlock.h>
#include <linux/mipi_dsi.h>
#include <linux/mxcfb.h>
#include <linux/backlight.h>
#include <video/mipi_display.h>

#include "mipi_dsi.h"

#define NV3051D_TWO_DATA_LANE					(0x2)
#define NV3051D_MAX_DPHY_CLK					(800)

#define CHECK_RETCODE(ret)					\
do {								\
	if (ret < 0) {						\
		dev_err(&mipi_dsi->pdev->dev,			\
			"%s ERR: ret:%d, line:%d.\n",		\
			__func__, ret, __LINE__);		\
		return ret;					\
	}							\
} while (0)

static void parse_variadic(int n, u8 *buf, ...)
{
	int i = 0;
	va_list args;

	if (unlikely(!n)) return;

	va_start(args, buf);

	for (i = 0; i < n; i++)
		buf[i + 1] = (u8)va_arg(args, int);

	va_end(args);
}

#define TC358763_DCS_write_1A_nP(n, addr, ...) {		\
	int err;						\
								\
	buf[0] = addr;						\
	parse_variadic(n, buf, ##__VA_ARGS__);			\
								\
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi,		\
		MIPI_DSI_GENERIC_LONG_WRITE, (u32*)buf, n + 1);	\
	CHECK_RETCODE(err);					\
}

#define TC358763_DCS_write_1A_0P(addr)		\
	TC358763_DCS_write_1A_nP(0, addr)

#define TC358763_DCS_write_1A_1P(addr, ...)	\
	TC358763_DCS_write_1A_nP(1, addr, __VA_ARGS__)

static int nv3051d_brightness;

#define    LCM_CLK_FREQ    50000 /* 1/20000 MHz */
#define    LCM_REFRESH     60
#define    LCM_HACT        640
#define    LCM_VACT        480
#define    LCM_HFP         20
#define    LCM_HBP         20
#define    LCM_HSA         2
#define    LCM_VFP         12
#define    LCM_VBP         4
#define    LCM_VSA         2

static struct fb_videomode truly_lcd_modedb[] = {
	{
	"EE0350ET-2CP",LCM_REFRESH, LCM_HACT , LCM_VACT, LCM_CLK_FREQ,
	 LCM_HBP, LCM_HFP,
	 LCM_VBP, LCM_VFP,
	 LCM_HSA, LCM_VSA,
	 0x0, /* Active High */
	 //FB_SYNC_OE_LOW_ACT, /* Active Low */
	 FB_VMODE_NONINTERLACED,
	 0,
	},
};

static struct mipi_lcd_config lcd_config = {
	.virtual_ch	= 0x0,
	.data_lane_num  = NV3051D_TWO_DATA_LANE,
	.max_phy_clk    = NV3051D_MAX_DPHY_CLK,
	.dpi_fmt	= MIPI_RGB888,
};

void mipid_nv3051d_get_lcd_videomode(struct fb_videomode **mode, int *size,
		struct mipi_lcd_config **data)
{
	*mode = &truly_lcd_modedb[0];
	*size = ARRAY_SIZE(truly_lcd_modedb);
	*data = &lcd_config;
}

int mipid_nv3051d_lcd_setup(struct mipi_dsi_info *mipi_dsi)
{
	u8 buf[DSI_CMD_BUF_MAXSIZE];
	int err = 0;

	dev_dbg(&mipi_dsi->pdev->dev, "MIPI DSI LCD setup.\n");

	TC358763_DCS_write_1A_1P(0xFF,0x30);
	TC358763_DCS_write_1A_1P(0xFF,0x52);
	TC358763_DCS_write_1A_1P(0xFF,0x01);
	TC358763_DCS_write_1A_1P(0xE3,0x00);

	TC358763_DCS_write_1A_1P(0x40,0x0a);

	TC358763_DCS_write_1A_1P(0x03,0x40);
	TC358763_DCS_write_1A_1P(0x04,0x00);
	TC358763_DCS_write_1A_1P(0x05,0x03);

	TC358763_DCS_write_1A_1P(0x20,0x90);//2LANE

	TC358763_DCS_write_1A_1P(0x24,0x0c);
	TC358763_DCS_write_1A_1P(0x25,0x06);
	TC358763_DCS_write_1A_1P(0x26,0x14);
	TC358763_DCS_write_1A_1P(0x27,0x14);

	TC358763_DCS_write_1A_1P(0x28,0x57);
	TC358763_DCS_write_1A_1P(0x29,0x01);
	TC358763_DCS_write_1A_1P(0x2A,0xdf);

	TC358763_DCS_write_1A_1P(0x38,0x9C);
	TC358763_DCS_write_1A_1P(0x39,0xA7);
	TC358763_DCS_write_1A_1P(0x3A,0x53);

	TC358763_DCS_write_1A_1P(0x44,0x00);
	TC358763_DCS_write_1A_1P(0x49,0x3C);
	TC358763_DCS_write_1A_1P(0x59,0xfe);
	TC358763_DCS_write_1A_1P(0x5c,0x00);

	TC358763_DCS_write_1A_1P(0x91,0x57);
	TC358763_DCS_write_1A_1P(0x92,0x57);
	TC358763_DCS_write_1A_1P(0xA0,0x55);
	TC358763_DCS_write_1A_1P(0xA1,0x50);

	TC358763_DCS_write_1A_1P(0xA4,0x9C);
	TC358763_DCS_write_1A_1P(0xA7,0x02);
	TC358763_DCS_write_1A_1P(0xA8,0x01);
	TC358763_DCS_write_1A_1P(0xA9,0x01);
	TC358763_DCS_write_1A_1P(0xAA,0xFC);
	TC358763_DCS_write_1A_1P(0xAB,0x28);
	TC358763_DCS_write_1A_1P(0xAC,0x06);
	TC358763_DCS_write_1A_1P(0xAD,0x06);
	TC358763_DCS_write_1A_1P(0xAE,0x06);
	TC358763_DCS_write_1A_1P(0xAF,0x03);
	TC358763_DCS_write_1A_1P(0xB0,0x08);
	TC358763_DCS_write_1A_1P(0xB1,0x26);
	TC358763_DCS_write_1A_1P(0xB2,0x28);
	TC358763_DCS_write_1A_1P(0xB3,0x28);
	TC358763_DCS_write_1A_1P(0xB4,0x33);
	TC358763_DCS_write_1A_1P(0xB5,0x08);
	TC358763_DCS_write_1A_1P(0xB6,0x26);
	TC358763_DCS_write_1A_1P(0xB7,0x08);
	TC358763_DCS_write_1A_1P(0xB8,0x26);

	TC358763_DCS_write_1A_1P(0xFF,0x30);
	TC358763_DCS_write_1A_1P(0xFF,0x52);
	TC358763_DCS_write_1A_1P(0xFF,0x02);
	TC358763_DCS_write_1A_1P(0xB0,0x0B);
	TC358763_DCS_write_1A_1P(0xB1,0x16);
	TC358763_DCS_write_1A_1P(0xB2,0x17);
	TC358763_DCS_write_1A_1P(0xB3,0x2C);
	TC358763_DCS_write_1A_1P(0xB4,0x32);
	TC358763_DCS_write_1A_1P(0xB5,0x3B);
	TC358763_DCS_write_1A_1P(0xB6,0x29);
	TC358763_DCS_write_1A_1P(0xB7,0x40);
	TC358763_DCS_write_1A_1P(0xB8,0x0d);
	TC358763_DCS_write_1A_1P(0xB9,0x05);
	TC358763_DCS_write_1A_1P(0xBA,0x12);
	TC358763_DCS_write_1A_1P(0xBB,0x10);
	TC358763_DCS_write_1A_1P(0xBC,0x12);
	TC358763_DCS_write_1A_1P(0xBD,0x15);
	TC358763_DCS_write_1A_1P(0xBE,0x19);
	TC358763_DCS_write_1A_1P(0xBF,0x0E);
	TC358763_DCS_write_1A_1P(0xC0,0x16);
	TC358763_DCS_write_1A_1P(0xC1,0x0A);
	TC358763_DCS_write_1A_1P(0xD0,0x0C);
	TC358763_DCS_write_1A_1P(0xD1,0x17);
	TC358763_DCS_write_1A_1P(0xD2,0x14);
	TC358763_DCS_write_1A_1P(0xD3,0x2E);
	TC358763_DCS_write_1A_1P(0xD4,0x32);
	TC358763_DCS_write_1A_1P(0xD5,0x3C);
	TC358763_DCS_write_1A_1P(0xD6,0x22);
	TC358763_DCS_write_1A_1P(0xD7,0x3D);
	TC358763_DCS_write_1A_1P(0xD8,0x0D);
	TC358763_DCS_write_1A_1P(0xD9,0x07);
	TC358763_DCS_write_1A_1P(0xDA,0x13);
	TC358763_DCS_write_1A_1P(0xDB,0x13);
	TC358763_DCS_write_1A_1P(0xDC,0x11);
	TC358763_DCS_write_1A_1P(0xDD,0x15);
	TC358763_DCS_write_1A_1P(0xDE,0x19);
	TC358763_DCS_write_1A_1P(0xDF,0x10);
	TC358763_DCS_write_1A_1P(0xE0,0x17);
	TC358763_DCS_write_1A_1P(0xE1,0x0A);

	TC358763_DCS_write_1A_1P(0xFF,0x30);
	TC358763_DCS_write_1A_1P(0xFF,0x52);
	TC358763_DCS_write_1A_1P(0xFF,0x03);

	TC358763_DCS_write_1A_1P(0x00,0x2A);
	TC358763_DCS_write_1A_1P(0x01,0x2A);
	TC358763_DCS_write_1A_1P(0x02,0x2A);
	TC358763_DCS_write_1A_1P(0x03,0x2A);
	TC358763_DCS_write_1A_1P(0x04,0x61);
	TC358763_DCS_write_1A_1P(0x05,0x80);
	TC358763_DCS_write_1A_1P(0x06,0xc7);
	TC358763_DCS_write_1A_1P(0x07,0x01);

	TC358763_DCS_write_1A_1P(0x08,0x82);
	TC358763_DCS_write_1A_1P(0x09,0x83);

	TC358763_DCS_write_1A_1P(0x30,0x2A);
	TC358763_DCS_write_1A_1P(0x31,0x2A);
	TC358763_DCS_write_1A_1P(0x32,0x2A);
	TC358763_DCS_write_1A_1P(0x33,0x2A);
	TC358763_DCS_write_1A_1P(0x34,0x61);
	TC358763_DCS_write_1A_1P(0x35,0xc5);
	TC358763_DCS_write_1A_1P(0x36,0x80);
	TC358763_DCS_write_1A_1P(0x37,0x23);

	TC358763_DCS_write_1A_1P(0x40,0x82);
	TC358763_DCS_write_1A_1P(0x41,0x83);
	TC358763_DCS_write_1A_1P(0x42,0x80);
	TC358763_DCS_write_1A_1P(0x43,0x81);

	TC358763_DCS_write_1A_1P(0x44,0x11);
	TC358763_DCS_write_1A_1P(0x45,0xe6);
	TC358763_DCS_write_1A_1P(0x46,0xe5);
	TC358763_DCS_write_1A_1P(0x47,0x11);
	TC358763_DCS_write_1A_1P(0x48,0xe8);
	TC358763_DCS_write_1A_1P(0x49,0xe7);

	TC358763_DCS_write_1A_1P(0x50,0x02);
	TC358763_DCS_write_1A_1P(0x51,0x01);
	TC358763_DCS_write_1A_1P(0x52,0x04);
	TC358763_DCS_write_1A_1P(0x53,0x03);

	TC358763_DCS_write_1A_1P(0x54,0x11);
	TC358763_DCS_write_1A_1P(0x55,0xea);
	TC358763_DCS_write_1A_1P(0x56,0xe9);
	TC358763_DCS_write_1A_1P(0x57,0x11);
	TC358763_DCS_write_1A_1P(0x58,0xec);
	TC358763_DCS_write_1A_1P(0x59,0xeb);

	TC358763_DCS_write_1A_1P(0x7e,0x02);
	TC358763_DCS_write_1A_1P(0x7f,0x80);
	TC358763_DCS_write_1A_1P(0xe0,0x5a);

	TC358763_DCS_write_1A_1P(0xB1,0x00);
	TC358763_DCS_write_1A_1P(0xB4,0x0e);
	TC358763_DCS_write_1A_1P(0xB5,0x0f);
	TC358763_DCS_write_1A_1P(0xB6,0x04);
	TC358763_DCS_write_1A_1P(0xB7,0x07);
	TC358763_DCS_write_1A_1P(0xB8,0x06);
	TC358763_DCS_write_1A_1P(0xB9,0x05);
	TC358763_DCS_write_1A_1P(0xBA,0x0f);
	TC358763_DCS_write_1A_1P(0xC7,0x00);
	TC358763_DCS_write_1A_1P(0xCA,0x0e);
	TC358763_DCS_write_1A_1P(0xCB,0x0f);
	TC358763_DCS_write_1A_1P(0xCC,0x04);
	TC358763_DCS_write_1A_1P(0xCD,0x07);
	TC358763_DCS_write_1A_1P(0xCE,0x06);
	TC358763_DCS_write_1A_1P(0xCF,0x05);
	TC358763_DCS_write_1A_1P(0xD0,0x0f);

	TC358763_DCS_write_1A_1P(0x81,0x0f);
	TC358763_DCS_write_1A_1P(0x84,0x0e);
	TC358763_DCS_write_1A_1P(0x85,0x0f);
	TC358763_DCS_write_1A_1P(0x86,0x07);
	TC358763_DCS_write_1A_1P(0x87,0x04);
	TC358763_DCS_write_1A_1P(0x88,0x05);
	TC358763_DCS_write_1A_1P(0x89,0x06);
	TC358763_DCS_write_1A_1P(0x8A,0x00);
	TC358763_DCS_write_1A_1P(0x97,0x0f);
	TC358763_DCS_write_1A_1P(0x9A,0x0e);
	TC358763_DCS_write_1A_1P(0x9B,0x0f);
	TC358763_DCS_write_1A_1P(0x9C,0x07);
	TC358763_DCS_write_1A_1P(0x9D,0x04);
	TC358763_DCS_write_1A_1P(0x9E,0x05);
	TC358763_DCS_write_1A_1P(0x9F,0x06);
	TC358763_DCS_write_1A_1P(0xA0,0x00);

	TC358763_DCS_write_1A_1P(0xFF,0x30);
	TC358763_DCS_write_1A_1P(0xFF,0x52);
	TC358763_DCS_write_1A_1P(0xFF,0x02);
	TC358763_DCS_write_1A_1P(0x01,0x01);
	TC358763_DCS_write_1A_1P(0x02,0xDA);
	TC358763_DCS_write_1A_1P(0x03,0xBA);
	TC358763_DCS_write_1A_1P(0x04,0xA8);
	TC358763_DCS_write_1A_1P(0x05,0x9A);
	TC358763_DCS_write_1A_1P(0x06,0x70);
	TC358763_DCS_write_1A_1P(0x07,0xFF);
	TC358763_DCS_write_1A_1P(0x08,0x91);
	TC358763_DCS_write_1A_1P(0x09,0x90);
	TC358763_DCS_write_1A_1P(0x0A,0xFF);
	TC358763_DCS_write_1A_1P(0x0B,0x8F);
	TC358763_DCS_write_1A_1P(0x0C,0x60);
	TC358763_DCS_write_1A_1P(0x0D,0x58);
	TC358763_DCS_write_1A_1P(0x0E,0x48);
	TC358763_DCS_write_1A_1P(0x0F,0x38);
	TC358763_DCS_write_1A_1P(0x10,0x2B);

	TC358763_DCS_write_1A_1P(0xFF,0x30);
	TC358763_DCS_write_1A_1P(0xFF,0x52);
	TC358763_DCS_write_1A_1P(0xFF,0x00);
	TC358763_DCS_write_1A_1P(0x36,0x02);

	TC358763_DCS_write_1A_0P(0x11);
	msleep(200);

	TC358763_DCS_write_1A_0P(0x29);
	msleep(10);

	return err;
}

static int mipid_bl_update_status(struct backlight_device *bl)
{
	return 0;
}

static int mipid_bl_get_brightness(struct backlight_device *bl)
{
	return nv3051d_brightness;
}

static int mipi_bl_check_fb(struct backlight_device *bl, struct fb_info *fbi)
{
	return 0;
}

static const struct backlight_ops mipid_lcd_bl_ops = {
	.update_status = mipid_bl_update_status,
	.get_brightness = mipid_bl_get_brightness,
	.check_fb = mipi_bl_check_fb,
};
