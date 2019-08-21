/*
 * Copyright (C) 2016-2019 F&S Elektronik Systeme GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
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

#define MIPI_DSI_MAX_RET_PACK_SIZE				(0x4)

#define HX8379_MAX_DPHY_CLK					(800)
#define HX8379_ONE_DATA_LANE					(0x1)
#define HX8379_TWO_DATA_LANE					(0x2)

#define HX8379_CMD_GETHXID					(0xF4)
#define HX8379_CMD_GETHXID_LEN					(0x4)
#define HX8379_ID						(0x79)
#define HX8379_ID_MASK						(0xFF)

#define HX8379_CMD_SETEXTC					(0xB9)
#define HX8379_CMD_SETEXTC_LEN					(4)
#define HX8379_CMD_SETEXTC_PARAM_1				(0x7983ff)

#define HX8379_CMD_SETPOWER					(0xB1)
#define HX8379_CMD_SETPOWER_LEN					(17)
#define HX8379_CMD_SETPOWER_PARAM_1				(0x181844)
#define HX8379_CMD_SETPOWER_PARAM_2				(0xD0505131)
#define HX8379_CMD_SETPOWER_PARAM_3				(0x388058D8)
#define HX8379_CMD_SETPOWER_PARAM_4				(0x3233F838)
#define HX8379_CMD_SETPOWER_PARAM_5				(0x22)

#define HX8379_CMD_SETDISP					(0xB2)
#define HX8379_CMD_SETDISP_LEN					(10)
#define HX8379_CMD_SETDISP_PARAM_1				(0x0A3C80)
#define HX8379_CMD_SETDISP_PARAM_2				(0x11507003)
#define HX8379_CMD_SETDISP_PARAM_3				(0x1D42)

#define HX8379_CMD_SETCYC					(0xB4)
#define HX8379_CMD_SETCYC_LEN					(11)
#define HX8379_CMD_SETCYC_PARAM_1				(0x027C02)
#define HX8379_CMD_SETCYC_PARAM_2				(0x227C027C)
#define HX8379_CMD_SETCYC_PARAM_3				(0x862386)

#define HX8379_CMD_SETTCON					(0xC7)
#define HX8379_CMD_SETTCON_LEN					(5)
#define HX8379_CMD_SETTCON_PARAM_1				(0x000000)
#define HX8379_CMD_SETTCON_PARAM_2				(0xC0)

#define HX8379_CMD_SETPANEL					(0xCC)
#define HX8379_CMD_SETPANEL_PARAM_1				(0x02)

#define HX8379_CMD_SETOFFSET					(0xD2)
#define HX8379_CMD_SETOFFSET_PARAM_1				(0x77)

#define HX8379_CMD_SETGIP0					(0xD3)
#define HX8379_CMD_SETGIP0_LEN					(38)
#define HX8379_CMD_SETGIP0_PARAM_1				(0x000700)
#define HX8379_CMD_SETGIP0_PARAM_2				(0x08080000)
#define HX8379_CMD_SETGIP0_PARAM_3				(0x00011032)
#define HX8379_CMD_SETGIP0_PARAM_4				(0x03720301)
#define HX8379_CMD_SETGIP0_PARAM_5				(0x00080072)
#define HX8379_CMD_SETGIP0_PARAM_6				(0x05333308)
#define HX8379_CMD_SETGIP0_PARAM_7				(0x05053705)
#define HX8379_CMD_SETGIP0_PARAM_8				(0x00000837)
#define HX8379_CMD_SETGIP0_PARAM_9				(0x01000A00)
#define HX8379_CMD_SETGIP0_PARAM_10				(0x0F01)

#define HX8379_CMD_SETGIP1					(0xD5)
#define HX8379_CMD_SETGIP1_LEN					(35)
#define HX8379_CMD_SETGIP1_PARAM_1				(0x181818)
#define HX8379_CMD_SETGIP1_PARAM_2				(0x07181818)
#define HX8379_CMD_SETGIP1_PARAM_3				(0x03040506)
#define HX8379_CMD_SETGIP1_PARAM_4				(0x18000102)
#define HX8379_CMD_SETGIP1_PARAM_5				(0x18202118)
#define HX8379_CMD_SETGIP1_PARAM_6				(0x23191918)
#define HX8379_CMD_SETGIP1_PARAM_7				(0x78383822)
#define HX8379_CMD_SETGIP1_PARAM_8				(0x18181878)
#define HX8379_CMD_SETGIP1_PARAM_9				(0x000018)

#define HX8379_CMD_SETGIP2					(0xD6)
#define HX8379_CMD_SETGIP2_LEN					(33)
#define HX8379_CMD_SETGIP2_PARAM_1				(0x181818)
#define HX8379_CMD_SETGIP2_PARAM_2				(0x00181818)
#define HX8379_CMD_SETGIP2_PARAM_3				(0x04030201)
#define HX8379_CMD_SETGIP2_PARAM_4				(0x18070605)
#define HX8379_CMD_SETGIP2_PARAM_5				(0x19232218)
#define HX8379_CMD_SETGIP2_PARAM_6				(0x20181819)
#define HX8379_CMD_SETGIP2_PARAM_7				(0x38383821)
#define HX8379_CMD_SETGIP2_PARAM_8				(0x18181838)
#define HX8379_CMD_SETGIP2_PARAM_9				(0x18)

#define HX8379_CMD_SETGAMMA					(0xE0)
#define HX8379_CMD_SETGAMMA_LEN					(43)
#define HX8379_CMD_SETGAMMA_PARAM_1				(0x040100)
#define HX8379_CMD_SETGAMMA_PARAM_2				(0x113F2420)
#define HX8379_CMD_SETGAMMA_PARAM_3				(0x0C0A0933)
#define HX8379_CMD_SETGAMMA_PARAM_4				(0x15120F17)
#define HX8379_CMD_SETGAMMA_PARAM_5				(0x150A1413)
#define HX8379_CMD_SETGAMMA_PARAM_6				(0x01001816)
#define HX8379_CMD_SETGAMMA_PARAM_7				(0x3F242004)
#define HX8379_CMD_SETGAMMA_PARAM_8				(0x0B093311)
#define HX8379_CMD_SETGAMMA_PARAM_9				(0x110E170C)
#define HX8379_CMD_SETGAMMA_PARAM_10				(0x0A141314)
#define HX8379_CMD_SETGAMMA_PARAM_11				(0x181615)

#define HX8379_CMD_SETVCOM					(0xB6)
#define HX8379_CMD_SETVCOM_LEN					(3)
#define HX8379_CMD_SETVCOM_PARAM_1				(0x5E5E)

#define CHECK_RETCODE(ret)					\
do {								\
	if (ret < 0) {						\
		dev_err(&mipi_dsi->pdev->dev,			\
			"%s ERR: ret:%d, line:%d.\n",		\
			__func__, ret, __LINE__);		\
		return ret;					\
	}							\
} while (0)

static int HX8379bl_brightness;

/* normally we need 28,774 MHz but the display doesnt work correctly so we
 * have set the frequency to 22,4 MHz. This value works for the display.
 */
static struct fb_videomode truly_lcd_modedb[] = {
	{
	"Yes-Optoelectronics", 60, 480, 800, 44641 /* 22,4 MHz */,
	 35, 35,
	 10, 5,
	 37, 2,
	 0x0,
	 FB_VMODE_NONINTERLACED,
	 0,
	},
};

static struct mipi_lcd_config lcd_config = {
	.virtual_ch		= 0x0,
	.data_lane_num  = HX8379_TWO_DATA_LANE,
	.max_phy_clk    = HX8379_MAX_DPHY_CLK,
	.dpi_fmt		= MIPI_RGB888,
};

void mipid_hx8379_get_lcd_videomode(struct fb_videomode **mode, int *size,
		struct mipi_lcd_config **data)
{
	*mode = &truly_lcd_modedb[0];
	*size = ARRAY_SIZE(truly_lcd_modedb);
	*data = &lcd_config;
}

int mipid_hx8379_lcd_setup(struct mipi_dsi_info *mipi_dsi)
{
	u32 buf[DSI_CMD_BUF_MAXSIZE];
	int err = 0;

	dev_dbg(&mipi_dsi->pdev->dev, "MIPI DSI LCD setup HX8379.\n");
	/* Set EXTC */
	buf[0] = HX8379_CMD_SETEXTC | (HX8379_CMD_SETEXTC_PARAM_1 << 8);
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE,
					buf, HX8379_CMD_SETEXTC_LEN);
	CHECK_RETCODE(err);

	buf[0] = MIPI_DSI_MAX_RET_PACK_SIZE;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi,
				MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE,
				buf, 0);
	CHECK_RETCODE(err);

	/* Check ID */
	buf[0] = HX8379_CMD_GETHXID;
	err =  mipi_dsi->mipi_dsi_pkt_read(mipi_dsi,
			MIPI_DSI_GENERIC_READ_REQUEST_2_PARAM,
			buf, HX8379_CMD_GETHXID_LEN);

	if (!err && ((buf[0] & HX8379_ID_MASK) == HX8379_ID)) {
		dev_info(&mipi_dsi->pdev->dev,
				"MIPI DSI LCD ID:0x%x.\n", buf[0]);
	} else {
		dev_err(&mipi_dsi->pdev->dev,
			"mipi_dsi_pkt_read err:%d, data:0x%x.\n",
			err, buf[0]);
		dev_info(&mipi_dsi->pdev->dev,
				"MIPI DSI LCD not detected!\n");
		return err;
	}

	/* Set power: standby, DC etc. */
	buf[0] = HX8379_CMD_SETPOWER | (HX8379_CMD_SETPOWER_PARAM_1 << 8);
	buf[1] = HX8379_CMD_SETPOWER_PARAM_2;
	buf[2] = HX8379_CMD_SETPOWER_PARAM_3;
	buf[3] = HX8379_CMD_SETPOWER_PARAM_4;
	buf[4] = HX8379_CMD_SETPOWER_PARAM_5;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE, buf,
				HX8379_CMD_SETPOWER_LEN);
	CHECK_RETCODE(err);

	/* Set display settings */
	buf[0] = HX8379_CMD_SETDISP | (HX8379_CMD_SETDISP_PARAM_1 << 8);
	buf[1] = HX8379_CMD_SETDISP_PARAM_2;
	buf[2] = HX8379_CMD_SETDISP_PARAM_3;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE,
						buf, HX8379_CMD_SETDISP_LEN);
	CHECK_RETCODE(err);

	/* Set display waveform cycle */
	buf[0] = HX8379_CMD_SETCYC | (HX8379_CMD_SETCYC_PARAM_1 << 8);
	buf[1] = HX8379_CMD_SETCYC_PARAM_2;
	buf[2] = HX8379_CMD_SETCYC_PARAM_3;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE,
						buf, HX8379_CMD_SETCYC_LEN);
	CHECK_RETCODE(err);

	/* Set TCON */
	buf[0] = HX8379_CMD_SETTCON | (HX8379_CMD_SETTCON_PARAM_1 << 8);
	buf[1] = HX8379_CMD_SETTCON_PARAM_2;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE,
						buf, HX8379_CMD_SETTCON_LEN);
	CHECK_RETCODE(err);

	/* Set Panel: BGR/RGB or Inversion. */
	buf[0] = HX8379_CMD_SETPANEL | (HX8379_CMD_SETPANEL_PARAM_1 << 8);
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi,
		MIPI_DSI_GENERIC_SHORT_WRITE_2_PARAM, buf, 0);
	CHECK_RETCODE(err);

	/* Set Offset */
	buf[0] = HX8379_CMD_SETOFFSET | (HX8379_CMD_SETOFFSET_PARAM_1 << 8);
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi,
		MIPI_DSI_GENERIC_SHORT_WRITE_2_PARAM, buf, 0);
	CHECK_RETCODE(err);

	/* Set GIP_0 timing output control */
	buf[0] = HX8379_CMD_SETGIP0 | (HX8379_CMD_SETGIP0_PARAM_1 << 8);
	buf[1] = HX8379_CMD_SETGIP0_PARAM_2;
	buf[2] = HX8379_CMD_SETGIP0_PARAM_3;
	buf[3] = HX8379_CMD_SETGIP0_PARAM_4;
	buf[4] = HX8379_CMD_SETGIP0_PARAM_5;
	buf[5] = HX8379_CMD_SETGIP0_PARAM_6;
	buf[6] = HX8379_CMD_SETGIP0_PARAM_7;
	buf[7] = HX8379_CMD_SETGIP0_PARAM_8;
	buf[8] = HX8379_CMD_SETGIP0_PARAM_9;
	buf[9] = HX8379_CMD_SETGIP0_PARAM_10;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE, buf,
				HX8379_CMD_SETGIP0_LEN);
	CHECK_RETCODE(err);

	/* Set GIP_1 timing output control */
	buf[0] = HX8379_CMD_SETGIP1 | (HX8379_CMD_SETGIP1_PARAM_1 << 8);
	buf[1] = HX8379_CMD_SETGIP1_PARAM_2;
	buf[2] = HX8379_CMD_SETGIP1_PARAM_3;
	buf[3] = HX8379_CMD_SETGIP1_PARAM_4;
	buf[4] = HX8379_CMD_SETGIP1_PARAM_5;
	buf[5] = HX8379_CMD_SETGIP1_PARAM_6;
	buf[6] = HX8379_CMD_SETGIP1_PARAM_7;
	buf[7] = HX8379_CMD_SETGIP1_PARAM_8;
	buf[8] = HX8379_CMD_SETGIP1_PARAM_9;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE, buf,
				HX8379_CMD_SETGIP1_LEN);
	CHECK_RETCODE(err);

	/* Set GIP_2 timing output control */
	buf[0] = HX8379_CMD_SETGIP2 | (HX8379_CMD_SETGIP2_PARAM_1 << 8);
	buf[1] = HX8379_CMD_SETGIP2_PARAM_2;
	buf[2] = HX8379_CMD_SETGIP2_PARAM_3;
	buf[3] = HX8379_CMD_SETGIP2_PARAM_4;
	buf[4] = HX8379_CMD_SETGIP2_PARAM_5;
	buf[5] = HX8379_CMD_SETGIP2_PARAM_6;
	buf[6] = HX8379_CMD_SETGIP2_PARAM_7;
	buf[7] = HX8379_CMD_SETGIP2_PARAM_8;
	buf[8] = HX8379_CMD_SETGIP2_PARAM_9;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE, buf,
				HX8379_CMD_SETGIP2_LEN);
	CHECK_RETCODE(err);

	/* Set gamma curve related setting */
	buf[0] = HX8379_CMD_SETGAMMA | (HX8379_CMD_SETGAMMA_PARAM_1 << 8);
	buf[1] = HX8379_CMD_SETGAMMA_PARAM_2;
	buf[2] = HX8379_CMD_SETGAMMA_PARAM_3;
	buf[3] = HX8379_CMD_SETGAMMA_PARAM_4;
	buf[4] = HX8379_CMD_SETGAMMA_PARAM_5;
	buf[5] = HX8379_CMD_SETGAMMA_PARAM_6;
	buf[6] = HX8379_CMD_SETGAMMA_PARAM_7;
	buf[7] = HX8379_CMD_SETGAMMA_PARAM_8;
	buf[8] = HX8379_CMD_SETGAMMA_PARAM_9;
	buf[9] = HX8379_CMD_SETGAMMA_PARAM_10;
	buf[10] = HX8379_CMD_SETGAMMA_PARAM_11;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE, buf,
				HX8379_CMD_SETGAMMA_LEN);
	CHECK_RETCODE(err);

	/* Set VCOM voltage. */
	buf[0] = HX8379_CMD_SETVCOM | (HX8379_CMD_SETVCOM_PARAM_1 << 8);
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE, buf,
				HX8379_CMD_SETVCOM_LEN);
	CHECK_RETCODE(err);

	/* exit sleep mode and set display on */
	buf[0] = MIPI_DCS_EXIT_SLEEP_MODE;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_SHORT_WRITE_1_PARAM,
		buf, 0);
	CHECK_RETCODE(err);
	/* To allow time for the supply voltages
	 * and clock circuits to stabilize.
	 */
	msleep(120);

	buf[0] = MIPI_DCS_SET_DISPLAY_ON;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_SHORT_WRITE_1_PARAM,
		buf, 0);
	CHECK_RETCODE(err);
	/* To allow time for the supply voltages
	 * and clock circuits to stabilize.
	 */
	msleep(120);

	return err;
}

static int mipid_bl_update_status(struct backlight_device *bl)
{
	return 0;
}

static int mipid_bl_get_brightness(struct backlight_device *bl)
{
	return HX8379bl_brightness;
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
