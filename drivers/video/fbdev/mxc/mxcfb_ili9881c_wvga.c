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

#define ILI9881C_MAX_DPHY_CLK					(800)
#define ILI9881C_ONE_DATA_LANE					(0x1)
#define ILI9881C_TWO_DATA_LANE					(0x2)

#define    LCM_REFRESH     50
#define    LCM_HACT        720
#define    LCM_VACT        1280
#define    LCM_HFP         80
#define    LCM_HBP         150
#define    LCM_HSA         10
#define    LCM_VFP         20
#define    LCM_VBP         20
#define    LCM_VSA         10

static int ILI9881C_brightness;

static struct fb_videomode truly_lcd_modedb[] = {
	{
	"TIANMA",LCM_REFRESH, LCM_HACT , LCM_VACT, 15664 /* 1/63,84 MHz */,
	 LCM_HBP, LCM_HFP,
	 LCM_VBP, LCM_VFP,
	 LCM_HSA, LCM_VSA,
	 0x0,
	 FB_VMODE_NONINTERLACED,
	 0,
	},
};

static struct mipi_lcd_config lcd_config = {
	.virtual_ch		= 0x0,
	.data_lane_num  = ILI9881C_TWO_DATA_LANE,
	.max_phy_clk    = ILI9881C_MAX_DPHY_CLK,
	.dpi_fmt		= MIPI_RGB888,
};

void mipid_ili9881c_get_lcd_videomode(struct fb_videomode **mode, int *size,
		struct mipi_lcd_config **data)
{
	*mode = &truly_lcd_modedb[0];
	*size = ARRAY_SIZE(truly_lcd_modedb);
	*data = &lcd_config;
}
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

#define CHECK_RETCODE(ret)					\
do {								\
	if (ret < 0) {						\
		dev_err(&mipi_dsi->pdev->dev,			\
			"%s ERR: ret:%d, line:%d.\n",		\
			__func__, ret, __LINE__);		\
		return ret;					\
	}							\
} while (0)

#define LCD_Gen_write_1A_nP(n, addr, ...) {		\
	int err;						\
								\
	buf[0] = addr;						\
	parse_variadic(n, buf, ##__VA_ARGS__);			\
								\
	if (n >= 2)						\
		err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi,		\
			MIPI_DSI_DCS_LONG_WRITE, (u32*)buf, n + 1);	\
	else if (n == 1)					\
		err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi,	\
			MIPI_DSI_DCS_SHORT_WRITE_PARAM, (u32*)buf, 0);	\
	else if (n == 0)					\
	{							\
		buf[1] = 0;					\
		err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi,	\
			MIPI_DSI_DCS_SHORT_WRITE, (u32*)buf, 0);	\
	}							\
	CHECK_RETCODE(err);					\
}

#define LCD_Gen_write_1A_0P(addr)		\
	LCD_Gen_write_1A_nP(0, addr)

#define LCD_Gen_write_1A_1P(addr, ...)	\
	LCD_Gen_write_1A_nP(1, addr, __VA_ARGS__)

#define LCD_Gen_write_1A_2P(addr, ...)	\
	LCD_Gen_write_1A_nP(2, addr, __VA_ARGS__)

#define LCD_Gen_write_1A_3P(addr, ...)	\
	LCD_Gen_write_1A_nP(3, addr, __VA_ARGS__)

#define LCD_Gen_write_1A_5P(addr, ...)	\
	LCD_Gen_write_1A_nP(5, addr, __VA_ARGS__)

#define LCD_Gen_write_1A_6P(addr, ...)	\
	LCD_Gen_write_1A_nP(6, addr, __VA_ARGS__)

#define LCD_Gen_write_1A_7P(addr, ...)	\
	LCD_Gen_write_1A_nP(7, addr, __VA_ARGS__)

#define LCD_Gen_write_1A_12P(addr, ...)	\
	LCD_Gen_write_1A_nP(12, addr, __VA_ARGS__)

#define LCD_Gen_write_1A_13P(addr, ...)	\
	LCD_Gen_write_1A_nP(13, addr, __VA_ARGS__)

#define LCD_Gen_write_1A_14P(addr, ...)	\
	LCD_Gen_write_1A_nP(14, addr, __VA_ARGS__)

#define LCD_Gen_write_1A_19P(addr, ...)	\
	LCD_Gen_write_1A_nP(19, addr, __VA_ARGS__)

#define LCD_Gen_write_1A_34P(addr, ...)	\
	LCD_Gen_write_1A_nP(34, addr, __VA_ARGS__)

#define LCD_Gen_write_1A_127P(addr, ...)	\
	LCD_Gen_write_1A_nP(127, addr, __VA_ARGS__)

static int read_register(struct mipi_dsi_info *mipi_dsi, int page, int command, u32* result){
	u32 buf32[4];
	u8 buf[DSI_CMD_BUF_MAXSIZE];
	int err = 0;

	LCD_Gen_write_1A_3P(0xFF, 0x98, 0x81, page);    // Page
	buf32[0] = 0x1;
	mipi_dsi->mipi_dsi_pkt_write(mipi_dsi,
			0x37, buf32, 0);
	buf32[0] = command;
	err = mipi_dsi->mipi_dsi_pkt_read(mipi_dsi,
			0x06, buf32, 0x1);
	*result = buf32 [0];
	if (err)
    	return ENXIO;
	return 0;
}

int mipid_ili9881c_lcd_setup(struct mipi_dsi_info *mipi_dsi)
{

	u8 buf[DSI_CMD_BUF_MAXSIZE];
	int err = 0;
	u32 result[3];


//****************************************************************************//
//***************************** Check Display ID *****************************//
	dev_info(&mipi_dsi->pdev->dev, "MIPI DSI LCD setup ili988.\n");
	err = read_register(mipi_dsi, 0x1, 0x0, &result[0]);
 	if (err){
		dev_err(&mipi_dsi->pdev->dev, "No MIPI display connected!\n");
		return err;
	}
	read_register(mipi_dsi, 0x1, 0x1, &result[1]);
	read_register(mipi_dsi, 0x1, 0x02, &result[2]);
	if ( result[0] != 0x98 || result[1] != 0x81 || result[2] != 0x0c){
		dev_err(&mipi_dsi->pdev->dev, "Wrong MIPI display connected!\n");
		return EACCES;
	}

//****************************************************************************//
//****************************** Page 3 Command ******************************//

	LCD_Gen_write_1A_3P(0xFF,0x98,0x81,0x03);    // Page 3

	LCD_Gen_write_1A_1P(0x01,0x00);           //Software Reset

	/* 5inch module register setting as below
	(2lane:  D0N/P and D1N/P and CLKN/P enable) */
	LCD_Gen_write_1A_3P(0xFF, 0x98, 0x81, 0x01);
	LCD_Gen_write_1A_1P(0xB6,0xF0);
	LCD_Gen_write_1A_1P(0xB7,0x03);
	LCD_Gen_write_1A_3P(0xFF, 0x98, 0x81, 0x04);
	LCD_Gen_write_1A_1P(0x00,0x80);

	//----------GIP_1--------------
	LCD_Gen_write_1A_3P(0xFF,0x98,0x81,0x03);    // Page 3
	LCD_Gen_write_1A_1P(0x02,0x00);
	LCD_Gen_write_1A_1P(0x03,0x73);
	LCD_Gen_write_1A_1P(0x04,0xD3);
	LCD_Gen_write_1A_1P(0x05,0x00);
	LCD_Gen_write_1A_1P(0x06,0x0A);
	LCD_Gen_write_1A_1P(0x07,0x0E);
	LCD_Gen_write_1A_1P(0x08,0x00);
	LCD_Gen_write_1A_1P(0x09,0x01);
	LCD_Gen_write_1A_1P(0x0A,0x01);
	LCD_Gen_write_1A_1P(0x0B,0x01);
	LCD_Gen_write_1A_1P(0x0C,0x01);
	LCD_Gen_write_1A_1P(0x0D,0x01);
	LCD_Gen_write_1A_1P(0x0E,0x01);

	LCD_Gen_write_1A_1P(0x0F,0x01);
	LCD_Gen_write_1A_1P(0x10,0x01);

	LCD_Gen_write_1A_1P(0x11,0x00);
	LCD_Gen_write_1A_1P(0x12,0x00);
	LCD_Gen_write_1A_1P(0x13,0x00);
	LCD_Gen_write_1A_1P(0x14,0x00);
	LCD_Gen_write_1A_1P(0x15,0x00);
	LCD_Gen_write_1A_1P(0x16,0x00);
	LCD_Gen_write_1A_1P(0x17,0x00);
	LCD_Gen_write_1A_1P(0x18,0x00);
	LCD_Gen_write_1A_1P(0x19,0x00);
	LCD_Gen_write_1A_1P(0x1A,0x00);
	LCD_Gen_write_1A_1P(0x1B,0x00);
	LCD_Gen_write_1A_1P(0x1C,0x00);
	LCD_Gen_write_1A_1P(0x1D,0x00);
	LCD_Gen_write_1A_1P(0x1E,0x40);
	LCD_Gen_write_1A_1P(0x1F,0x80);
	LCD_Gen_write_1A_1P(0x20,0x06);
	LCD_Gen_write_1A_1P(0x21,0x01);
	LCD_Gen_write_1A_1P(0x22,0x00);
	LCD_Gen_write_1A_1P(0x23,0x00);
	LCD_Gen_write_1A_1P(0x24,0x00);
	LCD_Gen_write_1A_1P(0x25,0x00);
	LCD_Gen_write_1A_1P(0x26,0x00);
	LCD_Gen_write_1A_1P(0x27,0x00);
	LCD_Gen_write_1A_1P(0x28,0x33);
	LCD_Gen_write_1A_1P(0x29,0x03);
	LCD_Gen_write_1A_1P(0x2A,0x00);
	LCD_Gen_write_1A_1P(0x2B,0x00);
	LCD_Gen_write_1A_1P(0x2C,0x00);
   	LCD_Gen_write_1A_1P(0x2D,0x00);
	LCD_Gen_write_1A_1P(0x2E,0x00);
	LCD_Gen_write_1A_1P(0x2F,0x00);

	LCD_Gen_write_1A_1P(0x30,0x00);
	LCD_Gen_write_1A_1P(0x31,0x00);
	LCD_Gen_write_1A_1P(0x32,0x00);
	LCD_Gen_write_1A_1P(0x33,0x00);
	LCD_Gen_write_1A_1P(0x34,0x03);
	LCD_Gen_write_1A_1P(0x35,0x00);
	LCD_Gen_write_1A_1P(0x36,0x03);
	LCD_Gen_write_1A_1P(0x37,0x00);
	LCD_Gen_write_1A_1P(0x38,0x00);
	LCD_Gen_write_1A_1P(0x39,0x00);
	LCD_Gen_write_1A_1P(0x3A,0x40);
	LCD_Gen_write_1A_1P(0x3B,0x40);
	LCD_Gen_write_1A_1P(0x3C,0x00);
	LCD_Gen_write_1A_1P(0x3D,0x00);
	LCD_Gen_write_1A_1P(0x3E,0x00);
	LCD_Gen_write_1A_1P(0x3F,0x00);

	LCD_Gen_write_1A_1P(0x40,0x00);
	LCD_Gen_write_1A_1P(0x41,0x00);
	LCD_Gen_write_1A_1P(0x42,0x00);
	LCD_Gen_write_1A_1P(0x43,0x00);
	LCD_Gen_write_1A_1P(0x44,0x00);

	//----------GIP_2--------------
	LCD_Gen_write_1A_1P(0x50,0x01);
	LCD_Gen_write_1A_1P(0x51,0x23);
	LCD_Gen_write_1A_1P(0x52,0x45);
	LCD_Gen_write_1A_1P(0x53,0x67);
	LCD_Gen_write_1A_1P(0x54,0x89);
	LCD_Gen_write_1A_1P(0x55,0xab);
	LCD_Gen_write_1A_1P(0x56,0x01);
	LCD_Gen_write_1A_1P(0x57,0x23);
	LCD_Gen_write_1A_1P(0x58,0x45);
	LCD_Gen_write_1A_1P(0x59,0x67);
	LCD_Gen_write_1A_1P(0x5a,0x89);
	LCD_Gen_write_1A_1P(0x5b,0xab);
	LCD_Gen_write_1A_1P(0x5c,0xcd);
	LCD_Gen_write_1A_1P(0x5d,0xef);

	//---------GIP_3--------------

	LCD_Gen_write_1A_1P(0x5e,0x11);	    //01
	LCD_Gen_write_1A_1P(0x5f,0x08);
	LCD_Gen_write_1A_1P(0x60,0x02);
	LCD_Gen_write_1A_1P(0x61,0x00);
	LCD_Gen_write_1A_1P(0x62,0x01);
	LCD_Gen_write_1A_1P(0x63,0x0D);
	LCD_Gen_write_1A_1P(0x64,0x0C);
	LCD_Gen_write_1A_1P(0x65,0x02);
	LCD_Gen_write_1A_1P(0x66,0x02);
	LCD_Gen_write_1A_1P(0x67,0x02);
	LCD_Gen_write_1A_1P(0x68,0x02);
	LCD_Gen_write_1A_1P(0x69,0x02);
	LCD_Gen_write_1A_1P(0x6a,0x02);
	LCD_Gen_write_1A_1P(0x6b,0x0F);
	LCD_Gen_write_1A_1P(0x6c,0x02);
	LCD_Gen_write_1A_1P(0x6d,0x02);
	LCD_Gen_write_1A_1P(0x6e,0x02);
	LCD_Gen_write_1A_1P(0x6f,0x02);

	LCD_Gen_write_1A_1P(0x70,0x02);
	LCD_Gen_write_1A_1P(0x71,0x02);
	LCD_Gen_write_1A_1P(0x72,0x0E);
	LCD_Gen_write_1A_1P(0x73,0x06);
	LCD_Gen_write_1A_1P(0x74,0x07);
	LCD_Gen_write_1A_1P(0x75,0x08);
	LCD_Gen_write_1A_1P(0x76,0x02);
	LCD_Gen_write_1A_1P(0x77,0x00);
	LCD_Gen_write_1A_1P(0x78,0x01);
	LCD_Gen_write_1A_1P(0x79,0x0D);
	LCD_Gen_write_1A_1P(0x7a,0x0C);
	LCD_Gen_write_1A_1P(0x7b,0x02);
	LCD_Gen_write_1A_1P(0x7c,0x02);
	LCD_Gen_write_1A_1P(0x7d,0x02);
	LCD_Gen_write_1A_1P(0x7e,0x02);
	LCD_Gen_write_1A_1P(0x7f,0x02);

	LCD_Gen_write_1A_1P(0x80,0x02);
	LCD_Gen_write_1A_1P(0x81,0x0F);
	LCD_Gen_write_1A_1P(0x82,0x02);
	LCD_Gen_write_1A_1P(0x83,0x02);
	LCD_Gen_write_1A_1P(0x84,0x02);
	LCD_Gen_write_1A_1P(0x85,0x02);
	LCD_Gen_write_1A_1P(0x86,0x02);
	LCD_Gen_write_1A_1P(0x87,0x02);
	LCD_Gen_write_1A_1P(0x88,0x0E);
	LCD_Gen_write_1A_1P(0x89,0x06);
	LCD_Gen_write_1A_1P(0x8a,0x07);


//****************************************************************************//
//****************************** Page 4 Command ******************************//
	LCD_Gen_write_1A_3P(0xFF,0x98,0x81,0x04);  // Page 4

	LCD_Gen_write_1A_1P(0x6c,0x15);	  	// VCORE

	LCD_Gen_write_1A_1P(0x6e,0x1a);		// VGH clamp level

	LCD_Gen_write_1A_1P(0x6f,0x33);		// VGL regulator: disable VGH:3x VGL:-2x VCL:REG
	LCD_Gen_write_1A_1P(0x8d,0x15);		// VGL

	LCD_Gen_write_1A_1P(0x3a,0xa4);		// Power saving

	LCD_Gen_write_1A_1P(0x87,0x2a);
	LCD_Gen_write_1A_1P(0x63,0xC0);	   //Source Timing Adjust

	LCD_Gen_write_1A_1P(0x26,0x76);
	LCD_Gen_write_1A_1P(0xb2,0xd1);    //reload gamma setting

	//----------- Page 1 Command -------------
	LCD_Gen_write_1A_3P(0xFF,0x98,0x81,0x01);
	LCD_Gen_write_1A_1P(0x22,0x09);	   	  // F:0x0A;	B:0x09  panel:normal black
	LCD_Gen_write_1A_1P(0x31,0x00);		  // Inversion:00 Column

	LCD_Gen_write_1A_1P(0x50,0x9E);
	LCD_Gen_write_1A_1P(0x51,0x9E);
//	LCD_Gen_write_1A_1P(0x53,0x5D);
//	LCD_Gen_write_1A_1P(0x55,0x61);

	LCD_Gen_write_1A_1P(0x60,0x14);	   	  // Source Timing

/*-----------------------GAMMA SETTING---------------------------------*/
/*-------------------------P-tive setting---------------------------*/
	LCD_Gen_write_1A_1P(0xA0,0x00);	//255             //gamma
	LCD_Gen_write_1A_1P(0xA1,0x1B);	//251
	LCD_Gen_write_1A_1P(0xA2,0x2B); //247
	LCD_Gen_write_1A_1P(0xA3,0x15); //243
	LCD_Gen_write_1A_1P(0xA4,0x19); //239
	LCD_Gen_write_1A_1P(0xA5,0x2C); //231
	LCD_Gen_write_1A_1P(0xA6,0x20); //219
	LCD_Gen_write_1A_1P(0xA7,0x1D); //203
	LCD_Gen_write_1A_1P(0xA8,0x98);	//175
	LCD_Gen_write_1A_1P(0xA9,0x1D); //144
	LCD_Gen_write_1A_1P(0xAA,0x29); //111
	LCD_Gen_write_1A_1P(0xAB,0x8A); //80
	LCD_Gen_write_1A_1P(0xAC,0x1D); //52
	LCD_Gen_write_1A_1P(0xAD,0x1C); //36
	LCD_Gen_write_1A_1P(0xAE,0x50); //24
	LCD_Gen_write_1A_1P(0xAF,0x24); //16
	LCD_Gen_write_1A_1P(0xB0,0x2C); //12
	LCD_Gen_write_1A_1P(0xB1,0x52); //8
	LCD_Gen_write_1A_1P(0xB2,0x60); //4
	LCD_Gen_write_1A_1P(0xB3,0x2A); //0

/*------------------------Negitive setting---------------------------*/
	LCD_Gen_write_1A_1P(0xC0,0x00);	//255
	LCD_Gen_write_1A_1P(0xC1,0x1B);	//251
	LCD_Gen_write_1A_1P(0xC2,0x2C); //247
	LCD_Gen_write_1A_1P(0xC3,0x16); //243
	LCD_Gen_write_1A_1P(0xC4,0x18); //239
	LCD_Gen_write_1A_1P(0xC5,0x2C); //231
	LCD_Gen_write_1A_1P(0xC6,0x20); //219
	LCD_Gen_write_1A_1P(0xC7,0x23); //203
	LCD_Gen_write_1A_1P(0xC8,0x98);	//175
	LCD_Gen_write_1A_1P(0xC9,0x1D); //144
	LCD_Gen_write_1A_1P(0xCA,0x2A); //111
	LCD_Gen_write_1A_1P(0xCB,0x8A); //80
	LCD_Gen_write_1A_1P(0xCC,0x1E); //52
	LCD_Gen_write_1A_1P(0xCD,0x1D); //36
	LCD_Gen_write_1A_1P(0xCE,0x50); //24
	LCD_Gen_write_1A_1P(0xCF,0x24); //16
	LCD_Gen_write_1A_1P(0xD0,0x2C); //12
	LCD_Gen_write_1A_1P(0xD1,0x52); //8
	LCD_Gen_write_1A_1P(0xD2,0x60); //4
	LCD_Gen_write_1A_1P(0xD3,0x2A); //0

	//****************************************************************************//
	//****************************** Page 0 Command ******************************//
	LCD_Gen_write_1A_3P(0xFF, 0x98, 0x81, 0x00);  //Page 0
	LCD_Gen_write_1A_2P(0x51, 0x0F, 0xFF);
	LCD_Gen_write_1A_1P(0x53, 0x24);
	LCD_Gen_write_1A_1P(0x55, 0x01);

//	LCD_Gen_write_1A_1P(0x35, 0x00);
	LCD_Gen_write_1A_0P(0x34);
//	LCD_Gen_write_1A_1P(0x36, 0x03);
	msleep(10);


	LCD_Gen_write_1A_0P(0x11);                    //Sleep out
	msleep(120);
	LCD_Gen_write_1A_0P(0x29);    			      //Display on
	msleep(20);

	return err;
}

static int mipid_bl_update_status(struct backlight_device *bl)
{
	return 0;
}

static int mipid_bl_get_brightness(struct backlight_device *bl)
{
	return ILI9881C_brightness;
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
