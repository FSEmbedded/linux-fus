/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2012 F&S Elektronik Systeme GmbH, Inc. All Rights Reserved.
 */

/*
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/of_device.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/fsl_devices.h>
#include <media/v4l2-chip-ident.h>
#include "v4l2-int-device.h"
#include "mxc_v4l2_capture.h"

#define OV9665_VOLTAGE_ANALOG               2800000
#define OV9665_VOLTAGE_DIGITAL_CORE         1500000
#define OV9665_VOLTAGE_DIGITAL_IO           1800000

#define MIN_FPS 15
#define MAX_FPS 30
#define DEFAULT_FPS 30

#define OV9665_XCLK_MIN 6000000
#define OV9665_XCLK_MAX 24000000

#define OV9665_CHIP_ID_HIGH_BYTE	0x0A
#define OV9665_CHIP_ID_LOW_BYTE		0x0B

enum ov9665_mode {
	ov9665_mode_MIN = 0,
	ov9665_mode_VGA_640_480 = 0,
	ov9665_mode_SXGA_1280_1024 = 1,
	ov9665_mode_MAX = 1
};

enum ov9665_frame_rate {
	ov9665_15_fps,
	ov9665_30_fps
};

struct reg_value {
	u16 u16RegAddr;
	u8 u8Val;
	u8 u8Mask;
	u32 u32Delay_ms;
};

struct ov9665_mode_info {
	enum ov9665_mode mode;
	u32 width;
	u32 height;
	struct reg_value *init_data_ptr;
	u32 init_data_size;
};

/*!
 * Maintains the information on the current state of the sesor.
 */
static struct sensor_data ov9665_data;
static int pwn_gpio, rst_gpio;

static struct reg_value ov9665_rotate_none[] = {
	{0x04, 0x0, 0x3f, 1}, 
};

static struct reg_value ov9665_rotate_vert_flip[] = {
	{0x04, 0x40, 0xff, 0x00},
};

static struct reg_value ov9665_rotate_horiz_flip[] = {
	{0x04, 0x80, 0xff, 0x00},
	{0x33, 0x40, 0xff, 0x00},
};

static struct reg_value ov9665_initial_setting[] = {
       { 0x3E, 0x80 ,0 ,1 },
       { 0x12, 0x80 ,0 ,1 },
       { 0xd5, 0xff ,0 ,1 },
       { 0xd6, 0x3f ,0 ,1 },
       { 0x3d, 0x3c ,0 ,1 },
       { 0x11, 0x81 ,0 ,1 },
       { 0x2a, 0x00 ,0 ,1 },
       { 0x2b, 0x00 ,0 ,1 },
       { 0x3a, 0xf1 ,0 ,1 },
       { 0x3b, 0x00 ,0 ,1 },
       { 0x3c, 0x58 ,0 ,1 },
       { 0x3e, 0x50 ,0 ,1 },
       { 0x71, 0x00 ,0 ,1 },
       { 0x15, 0x00 ,0 ,1 },
       { 0x6a, 0x24 ,0 ,1 },
       { 0x85, 0xe7 ,0 ,1 },
       { 0x63, 0x01 ,0 ,1 },
       { 0x17, 0x0c ,0 ,1 },
       { 0x18, 0x5c ,0 ,1 },
       { 0x19, 0x01 ,0 ,1 },
       { 0x1a, 0x82 ,0 ,1 },
       { 0x03, 0x03 ,0 ,1 },
       { 0x2b, 0x00 ,0 ,1 },
       { 0x36, 0xb4 ,0 ,1 },
       { 0x65, 0x10 ,0 ,1 },
       { 0x70, 0x02 ,0 ,1 },
       { 0x71, 0x9f ,0 ,1 },
       { 0x64, 0x24 ,0 ,1 },
       { 0x43, 0x00 ,0 ,1 },
       { 0x5D, 0x55 ,0 ,1 },
       { 0x5E, 0x57 ,0 ,1 },
       { 0x5F, 0x21 ,0 ,1 },
       { 0x24, 0x3e ,0 ,1 },
       { 0x25, 0x38 ,0 ,1 },
       { 0x26, 0x72 ,0 ,1 },
       { 0x14, 0x68 ,0 ,1 },
       { 0x0C, 0x3a ,0 ,1 },
       { 0x4F, 0x9E ,0 ,1 },
       { 0x50, 0x84 ,0 ,1 },
       { 0x5A, 0x67 ,0 ,1 },
       { 0x7d, 0x30 ,0 ,1 },
       { 0x7e, 0x00 ,0 ,1 },
       { 0x82, 0x03 ,0 ,1 },
       { 0x7f, 0x00 ,0 ,1 },
       { 0x83, 0x07 ,0 ,1 },
       { 0x80, 0x03 ,0 ,1 },
       { 0x81, 0x04 ,0 ,1 },
       { 0x96, 0xf0 ,0 ,1 },
       { 0x97, 0x00 ,0 ,1 },
       { 0x92, 0x33 ,0 ,1 },
       { 0x94, 0x5a ,0 ,1 },
       { 0x93, 0x3a ,0 ,1 },
       { 0x95, 0x48 ,0 ,1 },
       { 0x91, 0xfc ,0 ,1 },
       { 0x90, 0xff ,0 ,1 },
       { 0x8e, 0x4e ,0 ,1 },
       { 0x8f, 0x4e ,0 ,1 },
       { 0x8d, 0x13 ,0 ,1 },
       { 0x8c, 0x0c ,0 ,1 },
       { 0x8b, 0x0c ,0 ,1 },
       { 0x86, 0x9e ,0 ,1 },
       { 0x87, 0x11 ,0 ,1 },
       { 0x88, 0x22 ,0 ,1 },
       { 0x89, 0x05 ,0 ,1 },
       { 0x8a, 0x03 ,0 ,1 },
       { 0x9b, 0x0e ,0 ,1 },
       { 0x9c, 0x1c ,0 ,1 },
       { 0x9d, 0x34 ,0 ,1 },
       { 0x9e, 0x5a ,0 ,1 },
       { 0x9f, 0x68 ,0 ,1 },
       { 0xa0, 0x76 ,0 ,1 },
       { 0xa1, 0x82 ,0 ,1 },
       { 0xa2, 0x8e ,0 ,1 },
       { 0xa3, 0x98 ,0 ,1 },
       { 0xa4, 0xa0 ,0 ,1 },
       { 0xa5, 0xb0 ,0 ,1 },
       { 0xa6, 0xbe ,0 ,1 },
       { 0xa7, 0xd2 ,0 ,1 },
       { 0xa8, 0xe2 ,0 ,1 },
       { 0xa9, 0xee ,0 ,1 },
       { 0xaa, 0x18 ,0 ,1 },
       { 0xAB, 0xe7 ,0 ,1 },
       { 0xb0, 0x43 ,0 ,1 },
       { 0xac, 0x04 ,0 ,1 },
       { 0x84, 0x40 ,0 ,1 },
       { 0xad, 0x84 ,0 ,1 },
       { 0xd9, 0x24 ,0 ,1 },
       { 0xda, 0x00 ,0 ,1 },
       { 0xae, 0x10 ,0 ,1 },
       { 0xab, 0xe7 ,0 ,1 },
       { 0xb9, 0xa0 ,0 ,1 },
       { 0xba, 0x80 ,0 ,1 },
       { 0xbb, 0xa0 ,0 ,1 },
       { 0xbc, 0x80 ,0 ,1 },
       { 0xbd, 0x08 ,0 ,1 },
       { 0xbe, 0x19 ,0 ,1 },
       { 0xbf, 0x02 ,0 ,1 },
       { 0xc0, 0x08 ,0 ,1 },
       { 0xc1, 0x2a ,0 ,1 },
       { 0xc2, 0x34 ,0 ,1 },
       { 0xc3, 0x2d ,0 ,1 },
       { 0xc4, 0x2d ,0 ,1 },
       { 0xc5, 0x00 ,0 ,1 },
       { 0xc6, 0x98 ,0 ,1 },
       { 0xc7, 0x18 ,0 ,1 },
       { 0x69, 0x48 ,0 ,1 },
       { 0x74, 0xc0 ,0 ,1 },
       { 0x7c, 0x18 ,0 ,1 },
       { 0x65, 0x11 ,0 ,1 },
       { 0x66, 0x00 ,0 ,1 },
       { 0x41, 0xa0 ,0 ,1 },
       { 0x5b, 0x28 ,0 ,1 },
       { 0x60, 0x84 ,0 ,1 },
       { 0x05, 0x07 ,0 ,1 },
       { 0x03, 0x03 ,0 ,1 },
       { 0xd2, 0x8c ,0 ,1 },
       { 0xc7, 0x90 ,0 ,1 },
       { 0xc8, 0x06 ,0 ,1 },
       { 0xcb, 0x40 ,0 ,1 },
       { 0xcc, 0x40 ,0 ,1 },
       { 0xcf, 0x00 ,0 ,1 },
       { 0xd0, 0x20 ,0 ,1 },
       { 0xd1, 0x00 ,0 ,1 },
       { 0xc7, 0x18 ,0 ,1 },
       { 0x0d, 0x82 ,0 ,1 },
       { 0x0d, 0x80 ,0 ,1 },
       { 0x09, 0x01 ,0 ,1 },
       { 0xff, 0xff ,0 ,1 },
};

static struct reg_value ov9665_setting_30fps_VGA_640_480[] = {
	{ 0xd5, 0xfc, 0, 1 },
	{ 0xd6, 0x3f, 0, 1 },
	{ 0x3d, 0x3c, 0, 1 },
	{ 0x11, 0x81, 0, 1 },
	{ 0x2a, 0x0 , 0, 1 },
	{ 0x2b, 0x0 , 0, 1 },
	{ 0x3a, 0xd9, 0, 1 },
	{ 0x3b, 0x0 , 0, 1 },
	{ 0x3c, 0x58, 0, 1 },
	{ 0x3e, 0x50, 0, 1 },
	{ 0x71, 0x0 , 0, 1 },
	{ 0x15, 0x0 , 0, 1 },
	{ 0xd7, 0x10, 0, 1 },
	{ 0x6a, 0x24, 0, 1 },
	{ 0x85, 0xe7, 0, 1 },
	{ 0x63, 0x0 , 0, 1 },
	{ 0x12, 0x40, 0, 1 },
	{ 0x4d, 0x9 , 0, 1 },
	{ 0x17, 0xc , 0, 1 },
	{ 0x18, 0x5c, 0, 1 },
	{ 0x19, 0x2 , 0, 1 },
	{ 0x1a, 0x3f, 0, 1 },
	{ 0x03, 0x3 , 0, 1 },
	{ 0x32, 0xb4, 0, 1 },
	{ 0x2b, 0x0 , 0, 1 },
	{ 0x5c, 0x80, 0, 1 },
	{ 0x36, 0xb4, 0, 1 },
	{ 0x65, 0x10, 0, 1 },
	{ 0x70, 0x2 , 0, 1 },
	{ 0x71, 0x9f, 0, 1 },
	{ 0x64, 0xa4, 0, 1 },
	{ 0x5c, 0x80, 0, 1 },
	{ 0x43, 0x0 , 0, 1 },
	{ 0x5d, 0x55, 0, 1 },
	{ 0x5e, 0x57, 0, 1 },
	{ 0x5f, 0x21, 0, 1 },
	{ 0x24, 0x3e, 0, 1 },
	{ 0x25, 0x38, 0, 1 },
	{ 0x26, 0x72, 0, 1 },
	{ 0x14, 0x68, 0, 1 },
	{ 0x0c, 0x38, 0, 1 },
	{ 0x4f, 0x4f, 0, 1 },
	{ 0x50, 0x42, 0, 1 },
	{ 0x5a, 0x67, 0, 1 },
	{ 0x7d, 0x30, 0, 1 },
	{ 0x7e, 0x0 , 0, 1 },
	{ 0x82, 0x3 , 0, 1 },
	{ 0x7f, 0x0 , 0, 1 },
	{ 0x83, 0x7 , 0, 1 },
	{ 0x80, 0x3 , 0, 1 },
	{ 0x81, 0x4 , 0, 1 },
	{ 0x96, 0xf0, 0, 1 },
	{ 0x97, 0x0 , 0, 1 },
	{ 0x92, 0x33, 0, 1 },
	{ 0x94, 0x5a, 0, 1 },
	{ 0x93, 0x3a, 0, 1 },
	{ 0x95, 0x48, 0, 1 },
	{ 0x91, 0xfc, 0, 1 },
	{ 0x90, 0xff, 0, 1 },
	{ 0x8e, 0x4e, 0, 1 },
	{ 0x8f, 0x4e, 0, 1 },
	{ 0x8d, 0x13, 0, 1 },
	{ 0x8c, 0xc , 0, 1 },
	{ 0x8b, 0xc , 0, 1 },
	{ 0x86, 0x9e, 0, 1 },
	{ 0x87, 0x11, 0, 1 },
	{ 0x88, 0x22, 0, 1 },
	{ 0x89, 0x5 , 0, 1 },
	{ 0x8a, 0x3 , 0, 1 },
	{ 0x9b, 0xe , 0, 1 },
	{ 0x9c, 0x1c, 0, 1 },
	{ 0x9d, 0x34, 0, 1 },
	{ 0x9e, 0x5a, 0, 1 },
	{ 0x9f, 0x68, 0, 1 },
	{ 0xa0, 0x76, 0, 1 },
	{ 0xa1, 0x82, 0, 1 },
	{ 0xa2, 0x8e, 0, 1 },
	{ 0xa3, 0x98, 0, 1 },
	{ 0xa4, 0xa0, 0, 1 },
	{ 0xa5, 0xb0, 0, 1 },
	{ 0xa6, 0xbe, 0, 1 },
	{ 0xa7, 0xd2, 0, 1 },
	{ 0xa8, 0xe2, 0, 1 },
	{ 0xa9, 0xee, 0, 1 },
	{ 0xaa, 0x18, 0, 1 },
	{ 0xab, 0xe7, 0, 1 },
	{ 0xb0, 0x43, 0, 1 },
	{ 0xac, 0x4 , 0, 1 },
	{ 0x84, 0x40, 0, 1 },
	{ 0xad, 0x82, 0, 1 },
	{ 0xd9, 0x11, 0, 1 },
	{ 0xda, 0x0 , 0, 1 },
	{ 0xae, 0x10, 0, 1 },
	{ 0xab, 0xe7, 0, 1 },
	{ 0xb9, 0x50, 0, 1 },
	{ 0xba, 0x3c, 0, 1 },
	{ 0xbb, 0x50, 0, 1 },
	{ 0xbc, 0x3c, 0, 1 },
	{ 0xbd, 0x8 , 0, 1 },
	{ 0xbe, 0x19, 0, 1 },
	{ 0xbf, 0x2 , 0, 1 },
	{ 0xc0, 0x8 , 0, 1 },
	{ 0xc1, 0x2a, 0, 1 },
	{ 0xc2, 0x34, 0, 1 },
	{ 0xc3, 0x2d, 0, 1 },
	{ 0xc4, 0x2d, 0, 1 },
	{ 0xc5, 0x0 , 0, 1 },
	{ 0xc6, 0x98, 0, 1 },
	{ 0xc7, 0x18, 0, 1 },
	{ 0x69, 0x48, 0, 1 },
	{ 0x74, 0xc0, 0, 1 },
	{ 0x7c, 0x28, 0, 1 },
	{ 0x65, 0x11, 0, 1 },
	{ 0x66, 0x0 , 0, 1 },
	{ 0x41, 0xc0, 0, 1 },
	{ 0x5b, 0x24, 0, 1 },
	{ 0x60, 0x82, 0, 1 },
	{ 0x05, 0x7 , 0, 1 },
	{ 0x03, 0x3 , 0, 1 },
	{ 0xd2, 0x94, 0, 1 },
	{ 0xc8, 0x6 , 0, 1 },
	{ 0xcb, 0x40, 0, 1 },
	{ 0xcc, 0x40, 0, 1 },
	{ 0xcf, 0x0 , 0, 1 },
	{ 0xd0, 0x20, 0, 1 },
	{ 0xd1, 0x0 , 0, 1 },
	{ 0xc7, 0x18, 0, 1 },
	{ 0x0d, 0x92, 0, 1 },
	{ 0x0d, 0x90, 0, 1 },
	{ 0xab, 0xe7, 0, 1 },
	{ 0xb0, 0x43, 0, 1 },
	{ 0xac, 0x4 , 0, 1 },
	{ 0x84, 0x40, 0, 1 },
	{ 0xc7, 0x3c, 0, 1 },
	{ 0xc8, 0x6 , 0, 1 },
	{ 0x64, 0xa6, 0, 1 },
	{ 0xd0, 0x20, 0, 1 },
	{ 0xad, 0x98, 0, 1 },
	{ 0xd9, 0x49, 0, 1 },
	{ 0xda, 0x2 , 0, 1 },
	{ 0xae, 0x10, 0, 1 },
};

static struct reg_value ov9665_setting_15fps_VGA_640_480[] = {
	{ 0xd5, 0xfc, 0, 1 },
	{ 0xd6, 0x3f, 0, 1 },
	{ 0x3d, 0x3c, 0, 1 },
	{ 0x11, 0x81, 0, 1 },
	{ 0x2a, 0x0 , 0, 1 },
	{ 0x2b, 0x0 , 0, 1 },
	{ 0x3a, 0xd9, 0, 1 },
	{ 0x3b, 0x0 , 0, 1 },
	{ 0x3c, 0x58, 0, 1 },
	{ 0x3e, 0x50, 0, 1 },
	{ 0x71, 0x0 , 0, 1 },
	{ 0x15, 0x0 , 0, 1 },
	{ 0xd7, 0x10, 0, 1 },
	{ 0x6a, 0x24, 0, 1 },
	{ 0x85, 0xe7, 0, 1 },
	{ 0x63, 0x0 , 0, 1 },
	{ 0x12, 0x40, 0, 1 },
	{ 0x4d, 0x9 , 0, 1 },
	{ 0x17, 0xc , 0, 1 },
	{ 0x18, 0x5c, 0, 1 },
	{ 0x19, 0x2 , 0, 1 },
	{ 0x1a, 0x3f, 0, 1 },
	{ 0x03, 0x3 , 0, 1 },
	{ 0x32, 0xb4, 0, 1 },
	{ 0x2b, 0x0 , 0, 1 },
	{ 0x5c, 0x80, 0, 1 },
	{ 0x36, 0xb4, 0, 1 },
	{ 0x65, 0x10, 0, 1 },
	{ 0x70, 0x2 , 0, 1 },
	{ 0x71, 0x9f, 0, 1 },
	{ 0x64, 0xa4, 0, 1 },
	{ 0x5c, 0x80, 0, 1 },
	{ 0x43, 0x0 , 0, 1 },
	{ 0x5d, 0x55, 0, 1 },
	{ 0x5e, 0x57, 0, 1 },
	{ 0x5f, 0x21, 0, 1 },
	{ 0x24, 0x3e, 0, 1 },
	{ 0x25, 0x38, 0, 1 },
	{ 0x26, 0x72, 0, 1 },
	{ 0x14, 0x68, 0, 1 },
	{ 0x0c, 0x38, 0, 1 },
	{ 0x4f, 0x4f, 0, 1 },
	{ 0x50, 0x42, 0, 1 },
	{ 0x5a, 0x67, 0, 1 },
	{ 0x7d, 0x30, 0, 1 },
	{ 0x7e, 0x0 , 0, 1 },
	{ 0x82, 0x3 , 0, 1 },
	{ 0x7f, 0x0 , 0, 1 },
	{ 0x83, 0x7 , 0, 1 },
	{ 0x80, 0x3 , 0, 1 },
	{ 0x81, 0x4 , 0, 1 },
	{ 0x96, 0xf0, 0, 1 },
	{ 0x97, 0x0 , 0, 1 },
	{ 0x92, 0x33, 0, 1 },
	{ 0x94, 0x5a, 0, 1 },
	{ 0x93, 0x3a, 0, 1 },
	{ 0x95, 0x48, 0, 1 },
	{ 0x91, 0xfc, 0, 1 },
	{ 0x90, 0xff, 0, 1 },
	{ 0x8e, 0x4e, 0, 1 },
	{ 0x8f, 0x4e, 0, 1 },
	{ 0x8d, 0x13, 0, 1 },
	{ 0x8c, 0xc , 0, 1 },
	{ 0x8b, 0xc , 0, 1 },
	{ 0x86, 0x9e, 0, 1 },
	{ 0x87, 0x11, 0, 1 },
	{ 0x88, 0x22, 0, 1 },
	{ 0x89, 0x5 , 0, 1 },
	{ 0x8a, 0x3 , 0, 1 },
	{ 0x9b, 0xe , 0, 1 },
	{ 0x9c, 0x1c, 0, 1 },
	{ 0x9d, 0x34, 0, 1 },
	{ 0x9e, 0x5a, 0, 1 },
	{ 0x9f, 0x68, 0, 1 },
	{ 0xa0, 0x76, 0, 1 },
	{ 0xa1, 0x82, 0, 1 },
	{ 0xa2, 0x8e, 0, 1 },
	{ 0xa3, 0x98, 0, 1 },
	{ 0xa4, 0xa0, 0, 1 },
	{ 0xa5, 0xb0, 0, 1 },
	{ 0xa6, 0xbe, 0, 1 },
	{ 0xa7, 0xd2, 0, 1 },
	{ 0xa8, 0xe2, 0, 1 },
	{ 0xa9, 0xee, 0, 1 },
	{ 0xaa, 0x18, 0, 1 },
	{ 0xab, 0xe7, 0, 1 },
	{ 0xb0, 0x43, 0, 1 },
	{ 0xac, 0x4 , 0, 1 },
	{ 0x84, 0x40, 0, 1 },
	{ 0xad, 0x82, 0, 1 },
	{ 0xd9, 0x11, 0, 1 },
	{ 0xda, 0x0 , 0, 1 },
	{ 0xae, 0x10, 0, 1 },
	{ 0xab, 0xe7, 0, 1 },
	{ 0xb9, 0x50, 0, 1 },
	{ 0xba, 0x3c, 0, 1 },
	{ 0xbb, 0x50, 0, 1 },
	{ 0xbc, 0x3c, 0, 1 },
	{ 0xbd, 0x8 , 0, 1 },
	{ 0xbe, 0x19, 0, 1 },
	{ 0xbf, 0x2 , 0, 1 },
	{ 0xc0, 0x8 , 0, 1 },
	{ 0xc1, 0x2a, 0, 1 },
	{ 0xc2, 0x34, 0, 1 },
	{ 0xc3, 0x2d, 0, 1 },
	{ 0xc4, 0x2d, 0, 1 },
	{ 0xc5, 0x0 , 0, 1 },
	{ 0xc6, 0x98, 0, 1 },
	{ 0xc7, 0x18, 0, 1 },
	{ 0x69, 0x48, 0, 1 },
	{ 0x74, 0xc0, 0, 1 },
	{ 0x7c, 0x28, 0, 1 },
	{ 0x65, 0x11, 0, 1 },
	{ 0x66, 0x0 , 0, 1 },
	{ 0x41, 0xc0, 0, 1 },
	{ 0x5b, 0x24, 0, 1 },
	{ 0x60, 0x82, 0, 1 },
	{ 0x05, 0x7 , 0, 1 },
	{ 0x03, 0x3 , 0, 1 },
	{ 0xd2, 0x94, 0, 1 },
	{ 0xc8, 0x6 , 0, 1 },
	{ 0xcb, 0x40, 0, 1 },
	{ 0xcc, 0x40, 0, 1 },
	{ 0xcf, 0x0 , 0, 1 },
	{ 0xd0, 0x20, 0, 1 },
	{ 0xd1, 0x0 , 0, 1 },
	{ 0xc7, 0x18, 0, 1 },
	{ 0x0d, 0x92, 0, 1 },
	{ 0x0d, 0x90, 0, 1 },
	{ 0xab, 0xe7, 0, 1 },
	{ 0xb0, 0x43, 0, 1 },
	{ 0xac, 0x4 , 0, 1 },
	{ 0x84, 0x40, 0, 1 },
	{ 0xc7, 0x3c, 0, 1 },
	{ 0xc8, 0x6 , 0, 1 },
	{ 0x64, 0xa6, 0, 1 },
	{ 0xd0, 0x20, 0, 1 },
	{ 0xad, 0x98, 0, 1 },
	{ 0xd9, 0x49, 0, 1 },
	{ 0xda, 0x2 , 0, 1 },
	{ 0xae, 0x10, 0, 1 },
};

static struct reg_value ov9665_setting_15fps_SXGA_1280_1024[] = {
	{ 0x0c, 0xbc, 0, 1 },
	{ 0xff, 0xff, 0, 1 },
};

static struct ov9665_mode_info ov9665_mode_info_data[2][ov9665_mode_MAX + 1] = {
	{
		{ov9665_mode_VGA_640_480,    640,  480,
		ov9665_setting_15fps_VGA_640_480,
		ARRAY_SIZE(ov9665_setting_15fps_VGA_640_480)},
		{ov9665_mode_SXGA_1280_1024,   1280, 1024,
		ov9665_setting_15fps_SXGA_1280_1024,
		ARRAY_SIZE(ov9665_setting_15fps_SXGA_1280_1024)},
	},
	{
		{ov9665_mode_VGA_640_480,    640,  480,
		ov9665_setting_30fps_VGA_640_480,
		ARRAY_SIZE(ov9665_setting_30fps_VGA_640_480)},
		{ov9665_mode_SXGA_1280_1024,   1280, 1024,
		ov9665_setting_15fps_SXGA_1280_1024,
		ARRAY_SIZE(ov9665_setting_15fps_SXGA_1280_1024)},
	},
};

static struct regulator *io_regulator;
static struct regulator *core_regulator;
static struct regulator *analog_regulator;
static struct regulator *gpo_regulator;

static int ov9665_probe(struct i2c_client *adapter,
				const struct i2c_device_id *device_id);
static int ov9665_remove(struct i2c_client *client);

static s32 ov9665_read_reg(u16 reg, u8 *val);
static s32 ov9665_write_reg(u16 reg, u8 val);

static const struct i2c_device_id ov9665_id[] = {
	{"ov9665", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, ov9665_id);

static struct i2c_driver ov9665_i2c_driver = {
	.driver = {
		  .owner = THIS_MODULE,
		  .name  = "ov9665",
		  },
	.probe  = ov9665_probe,
	.remove = ov9665_remove,
	.id_table = ov9665_id,
};

static void ov9665_standby(s32 enable)
{
	if (enable)
		gpio_set_value(pwn_gpio, 1);
	else
		gpio_set_value(pwn_gpio, 0);

	msleep(2);
}

static void ov9665_reset(void)
{
	/* camera reset */
	gpio_set_value(rst_gpio, 1);

	/* camera power down */
	gpio_set_value(pwn_gpio, 1);
	msleep(5);

	gpio_set_value(pwn_gpio, 0);
	msleep(5);

	gpio_set_value(rst_gpio, 0);
	msleep(1);

	gpio_set_value(rst_gpio, 1);
	msleep(5);

	gpio_set_value(pwn_gpio, 1);
}

static int ov9665_power_on(struct device *dev)
{
	int ret = 0;

	io_regulator = devm_regulator_get(dev, "DOVDD");
	if (!IS_ERR(io_regulator)) {
		regulator_set_voltage(io_regulator,
				      OV9665_VOLTAGE_DIGITAL_IO,
					  OV9665_VOLTAGE_DIGITAL_IO);
		ret = regulator_enable(io_regulator);
		if (ret) {
			pr_err("%s:io set voltage error\n", __func__);
			return ret;
		} else {
			dev_dbg(dev,
				"%s:io set voltage ok\n", __func__);
		}
	} else {
		pr_err("%s: cannot get io voltage error\n", __func__);
		io_regulator = NULL;
	}

	core_regulator = devm_regulator_get(dev, "DVDD");
	if (!IS_ERR(core_regulator)) {
		regulator_set_voltage(core_regulator,
				OV9665_VOLTAGE_DIGITAL_CORE,
				OV9665_VOLTAGE_DIGITAL_CORE);
		ret = regulator_enable(core_regulator);
		if (ret) {
			pr_err("%s:core set voltage error\n", __func__);
			return ret;
		} else {
			dev_dbg(dev,
				"%s:core set voltage ok\n", __func__);
		}
	} else {
		core_regulator = NULL;
		pr_err("%s: cannot get core voltage error\n", __func__);
	}

	analog_regulator = devm_regulator_get(dev, "AVDD");
	if (!IS_ERR(analog_regulator)) {
		regulator_set_voltage(analog_regulator,
				OV9665_VOLTAGE_ANALOG,
				OV9665_VOLTAGE_ANALOG);
		ret = regulator_enable(analog_regulator);
		if (ret) {
			pr_err("%s:analog set voltage error\n",
				__func__);
			return ret;
		} else {
			dev_dbg(dev,
				"%s:analog set voltage ok\n", __func__);
		}
	} else {
		analog_regulator = NULL;
		pr_err("%s: cannot get analog voltage error\n", __func__);
	}

	return ret;
}

static s32 ov9665_write_reg(u16 reg, u8 val)
{
	int ret = i2c_smbus_write_byte_data(ov9665_data.i2c_client, reg, val);

	return ret;
}

static s32 ov9665_read_reg(u16 reg, u8 *val)
{
	int ret;

	ret = i2c_smbus_read_byte_data(ov9665_data.i2c_client, reg);
	if (ret >= 0) {
		*val = (unsigned char) ret;
		ret = 0;
	}

	return ret;
}

static int ov9665_set_rotate_mode(struct reg_value *rotate_mode)
{
	s32 i = 0;
	s32 iModeSettingArySize = 2;
	register u32 Delay_ms = 0;
	register u16 RegAddr = 0;
	register u8 Mask = 0;
	register u8 Val = 0;
	u8 RegVal = 0;
	int retval = 0;

	for (i = 0; i < iModeSettingArySize; ++i, ++rotate_mode) {
		Delay_ms = rotate_mode->u32Delay_ms;
		RegAddr = rotate_mode->u16RegAddr;
		Val = rotate_mode->u8Val;
		Mask = rotate_mode->u8Mask;

		if (Mask) {
			retval = ov9665_read_reg(RegAddr, &RegVal);
			if (retval < 0) {
				pr_err("%s, read reg 0x%x failed\n", __FUNCTION__, RegAddr);
				goto err;
			}

			Val |= RegVal;
			Val &= Mask;
		}

		retval = ov9665_write_reg(RegAddr, Val);
		if (retval < 0) {
			pr_err("%s, write reg 0x%x failed\n", __FUNCTION__, RegAddr);
			goto err;
		}

		if (Delay_ms)
			mdelay(Delay_ms);
	}
err:
	return retval;
}
static int ov9665_init_mode(enum ov9665_frame_rate frame_rate,
		enum ov9665_mode mode);
static int ov9665_change_mode(enum ov9665_frame_rate new_frame_rate,
		enum ov9665_frame_rate old_frame_rate,
		enum ov9665_mode new_mode,
		enum ov9665_mode orig_mode)
{
	//struct reg_value *pModeSetting = NULL;
	int retval = 0;

	if (new_mode > ov9665_mode_MAX || new_mode < ov9665_mode_MIN) {
		pr_err("Wrong ov9665 mode detected!\n");
		return -1;
	}

	if( (new_frame_rate == old_frame_rate) &&
	    (new_mode == orig_mode) ) {
		return 0;
	}

	retval = ov9665_init_mode(new_frame_rate, new_mode);

	return retval;
}
static int ov9665_init_mode(enum ov9665_frame_rate frame_rate,
			    enum ov9665_mode mode)
{
	struct reg_value *pModeSetting = NULL;
	s32 i = 0;
	s32 iModeSettingArySize = 0;
	register u32 Delay_ms = 0;
	register u16 RegAddr = 0;
	register u8 Mask = 0;
	register u8 Val = 0;
	u8 RegVal = 0;
	int retval = 0;

	printk("%s: mode=%d, framerate=%d\n", __func__, mode, frame_rate);

	if (mode > ov9665_mode_MAX || mode < ov9665_mode_MIN) {
		pr_err("Wrong ov9665 mode detected!\n");
		return -1;
	}

	pModeSetting = ov9665_mode_info_data[frame_rate][mode].init_data_ptr;
	iModeSettingArySize =
		ov9665_mode_info_data[frame_rate][mode].init_data_size;

	ov9665_data.pix.width = ov9665_mode_info_data[frame_rate][mode].width;
	ov9665_data.pix.height = ov9665_mode_info_data[frame_rate][mode].height;

	if (ov9665_data.pix.width == 0 || ov9665_data.pix.height == 0 ||
	    pModeSetting == NULL || iModeSettingArySize == 0)
		return -EINVAL;

	for (i = 0; i < iModeSettingArySize; ++i, ++pModeSetting) {
		Delay_ms = pModeSetting->u32Delay_ms;
		RegAddr = pModeSetting->u16RegAddr;
		Val = pModeSetting->u8Val;
		Mask = pModeSetting->u8Mask;

		if (Mask) {
			retval = ov9665_read_reg(RegAddr, &RegVal);
			if (retval < 0) {
				pr_err("read reg error addr=0x%x", RegAddr);
				goto err;
			}

			RegVal &= ~(u8)Mask;
			Val &= Mask;
			Val |= RegVal;
		}

		retval = ov9665_write_reg(RegAddr, Val);
		if (retval < 0) {
			pr_err("write reg error addr=0x%x", RegAddr);
			goto err;
		}

		if (Delay_ms)
			msleep(Delay_ms);
	}
err:
	return retval;
}


/* --------------- IOCTL functions from v4l2_int_ioctl_desc --------------- */

static int ioctl_g_ifparm(struct v4l2_int_device *s, struct v4l2_ifparm *p)
{
	if (s == NULL) {
		pr_err("   ERROR!! no slave device set!\n");
		return -1;
	}

	memset(p, 0, sizeof(*p));
	p->u.bt656.clock_curr = ov9665_data.mclk;
	pr_debug("   clock_curr=mclk=%d\n", ov9665_data.mclk);
	p->if_type = V4L2_IF_TYPE_BT656;
	p->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT;
	p->u.bt656.clock_min = OV9665_XCLK_MIN;
	p->u.bt656.clock_max = OV9665_XCLK_MAX;
	p->u.bt656.bt_sync_correct = 1;  /* Indicate external vsync */

	return 0;
}

/*!
 * ioctl_s_power - V4L2 sensor interface handler for VIDIOC_S_POWER ioctl
 * @s: pointer to standard V4L2 device structure
 * @on: indicates power mode (on or off)
 *
 * Turns the power on or off, depending on the value of on and returns the
 * appropriate error code.
 */
static int ioctl_s_power(struct v4l2_int_device *s, int on)
{
	struct sensor_data *sensor = s->priv;

	if (on && !sensor->on) {
		if (io_regulator)
			if (regulator_enable(io_regulator) != 0)
				return -EIO;
		if (core_regulator)
			if (regulator_enable(core_regulator) != 0)
				return -EIO;
		if (gpo_regulator)
			if (regulator_enable(gpo_regulator) != 0)
				return -EIO;
		if (analog_regulator)
			if (regulator_enable(analog_regulator) != 0)
				return -EIO;
		/* Make sure power on */
		ov9665_standby(0);
	} else if (!on && sensor->on) {
		if (analog_regulator)
			regulator_disable(analog_regulator);
		if (core_regulator)
			regulator_disable(core_regulator);
		if (io_regulator)
			regulator_disable(io_regulator);
		if (gpo_regulator)
			regulator_disable(gpo_regulator);

		ov9665_standby(1);
	}

	sensor->on = on;

	return 0;
}

/*!
 * ioctl_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int ioctl_g_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct sensor_data *sensor = s->priv;
	struct v4l2_captureparm *cparm = &a->parm.capture;
	int ret = 0;

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		memset(a, 0, sizeof(*a));
		a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cparm->capability = sensor->streamcap.capability;
		cparm->timeperframe = sensor->streamcap.timeperframe;
		cparm->capturemode = sensor->streamcap.capturemode;
		ret = 0;
		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		ret = -EINVAL;
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

/*!
 * ioctl_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 */
static int ioctl_s_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct sensor_data *sensor = s->priv;
	struct v4l2_fract *timeperframe = &a->parm.capture.timeperframe;
	u32 tgt_fps, old_fps;	/* target frames per secound */
	enum ov9665_frame_rate new_frame_rate, old_frame_rate;
	int ret = 0;

	/* Make sure power on */
	ov9665_standby(0);

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		/* Check that the new frame rate is allowed. */
		if ((timeperframe->numerator == 0) ||
		    (timeperframe->denominator == 0)) {
			timeperframe->denominator = DEFAULT_FPS;
			timeperframe->numerator = 1;
		}

		tgt_fps = timeperframe->denominator /
			  timeperframe->numerator;

		if (tgt_fps > MAX_FPS) {
			timeperframe->denominator = MAX_FPS;
			timeperframe->numerator = 1;
		} else if (tgt_fps < MIN_FPS) {
			timeperframe->denominator = MIN_FPS;
			timeperframe->numerator = 1;
		}

		/* Actual frame rate we use */
		tgt_fps = timeperframe->denominator /
			  timeperframe->numerator;

		if (tgt_fps == 15)
			new_frame_rate = ov9665_15_fps;
		else if (tgt_fps == 30)
			new_frame_rate = ov9665_30_fps;
		else {
			pr_err(" The camera frame rate is not supported!\n");
			return -EINVAL;
		}

		if (sensor->streamcap.timeperframe.numerator != 0)
			old_fps = sensor->streamcap.timeperframe.denominator /
				sensor->streamcap.timeperframe.numerator;
		else
			old_fps = 30;

		if (old_fps == 15)
			old_frame_rate = ov9665_15_fps;
		else if (old_fps == 30)
			old_frame_rate = ov9665_30_fps;
		else {
			pr_warning(" No valid frame rate set!\n");
			old_frame_rate = ov9665_30_fps;
		}

		ret = ov9665_change_mode(new_frame_rate, old_frame_rate,
				a->parm.capture.capturemode,
				sensor->streamcap.capturemode);
		if (ret < 0)
			return ret;

		sensor->streamcap.timeperframe = *timeperframe;
		sensor->streamcap.capturemode =
				(u32)a->parm.capture.capturemode;
		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		pr_debug("   type is not " \
			"V4L2_BUF_TYPE_VIDEO_CAPTURE but %d\n",
			a->type);
		ret = -EINVAL;
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

/*!
 * ioctl_g_fmt_cap - V4L2 sensor interface handler for ioctl_g_fmt_cap
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 v4l2_format structure
 *
 * Returns the sensor's current pixel format in the v4l2_format
 * parameter.
 */
static int ioctl_g_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	struct sensor_data *sensor = s->priv;

	f->fmt.pix = sensor->pix;

	return 0;
}

/*!
 * ioctl_g_ctrl - V4L2 sensor interface handler for VIDIOC_G_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_G_CTRL ioctl structure
 *
 * If the requested control is supported, returns the control's current
 * value from the video_control[] array.  Otherwise, returns -EINVAL
 * if the control is not supported.
 */
static int ioctl_g_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	int ret = 0;

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		vc->value = ov9665_data.brightness;
		break;
	case V4L2_CID_HUE:
		vc->value = ov9665_data.hue;
		break;
	case V4L2_CID_CONTRAST:
		vc->value = ov9665_data.contrast;
		break;
	case V4L2_CID_SATURATION:
		vc->value = ov9665_data.saturation;
		break;
	case V4L2_CID_RED_BALANCE:
		vc->value = ov9665_data.red;
		break;
	case V4L2_CID_BLUE_BALANCE:
		vc->value = ov9665_data.blue;
		break;
	case V4L2_CID_EXPOSURE:
		vc->value = ov9665_data.ae_mode;
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

/*!
 * ioctl_s_ctrl - V4L2 sensor interface handler for VIDIOC_S_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_S_CTRL ioctl structure
 *
 * If the requested control is supported, sets the control's current
 * value in HW (and updates the video_control[] array).  Otherwise,
 * returns -EINVAL if the control is not supported.
 */
static int ioctl_s_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	int retval = 0;
	//struct sensor_data *sensor = s->priv;
	//__u32 captureMode = sensor->streamcap.capturemode;

	pr_debug("In ov9665:ioctl_s_ctrl %d\n",
		 vc->id);

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		break;
	case V4L2_CID_CONTRAST:
		break;
	case V4L2_CID_SATURATION:
		break;
	case V4L2_CID_HUE:
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		break;
	case V4L2_CID_DO_WHITE_BALANCE:
		break;
	case V4L2_CID_RED_BALANCE:
		break;
	case V4L2_CID_BLUE_BALANCE:
		break;
	case V4L2_CID_GAMMA:
		break;
	case V4L2_CID_EXPOSURE:
		break;
	case V4L2_CID_AUTOGAIN:
		break;
	case V4L2_CID_GAIN:
		break;
	case V4L2_CID_HFLIP:
		break;
	case V4L2_CID_VFLIP:
		break;
	case V4L2_CID_MXC_ROT:
	case V4L2_CID_MXC_VF_ROT:
		switch (vc->value) {
		case V4L2_MXC_ROTATE_NONE:
			if (ov9665_set_rotate_mode(ov9665_rotate_none))
				retval = -EPERM;
			break;
		case V4L2_MXC_ROTATE_VERT_FLIP:
			if (ov9665_set_rotate_mode(ov9665_rotate_vert_flip))
				retval = -EPERM;
			break;
		case V4L2_MXC_ROTATE_HORIZ_FLIP:
			if (ov9665_set_rotate_mode(ov9665_rotate_horiz_flip))
				retval = -EPERM;

			break;
		case V4L2_MXC_ROTATE_180:
			retval = -EPERM;
			break;
		default:
			retval = -EPERM;
			break;
		}
		break;
	default:
		retval = -EPERM;
		break;
	}

	return retval;
}

/*!
 * ioctl_enum_framesizes - V4L2 sensor interface handler for
 *			   VIDIOC_ENUM_FRAMESIZES ioctl
 * @s: pointer to standard V4L2 device structure
 * @fsize: standard V4L2 VIDIOC_ENUM_FRAMESIZES ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int ioctl_enum_framesizes(struct v4l2_int_device *s,
				 struct v4l2_frmsizeenum *fsize)
{
	if (fsize->index > ov9665_mode_MAX)
		return -EINVAL;

	fsize->pixel_format = ov9665_data.pix.pixelformat;
	fsize->discrete.width =
			max(ov9665_mode_info_data[0][fsize->index].width,
			    ov9665_mode_info_data[1][fsize->index].width);
	fsize->discrete.height =
			max(ov9665_mode_info_data[0][fsize->index].height,
			    ov9665_mode_info_data[1][fsize->index].height);
	return 0;
}

/*!
 * ioctl_g_chip_ident - V4L2 sensor interface handler for
 *			VIDIOC_DBG_G_CHIP_IDENT ioctl
 * @s: pointer to standard V4L2 device structure
 * @id: pointer to int
 *
 * Return 0.
 */
static int ioctl_g_chip_ident(struct v4l2_int_device *s, int *id)
{
	((struct v4l2_dbg_chip_ident *)id)->match.type =
					V4L2_CHIP_MATCH_I2C_DRIVER;
	strcpy(((struct v4l2_dbg_chip_ident *)id)->match.name, "ov9665_camera");

	return 0;
}

/*!
 * ioctl_init - V4L2 sensor interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 */
static int ioctl_init(struct v4l2_int_device *s)
{
	return 0;
}

/*!
 * ioctl_enum_fmt_cap - V4L2 sensor interface handler for VIDIOC_ENUM_FMT
 * @s: pointer to standard V4L2 device structure
 * @fmt: pointer to standard V4L2 fmt description structure
 *
 * Return 0.
 */
static int ioctl_enum_fmt_cap(struct v4l2_int_device *s,
			      struct v4l2_fmtdesc *fmt)
{
	if (fmt->index > ov9665_mode_MAX)
		return -EINVAL;

	fmt->pixelformat = ov9665_data.pix.pixelformat;

	return 0;
}

/*!
 * ioctl_dev_init - V4L2 sensor interface handler for vidioc_int_dev_init_num
 * @s: pointer to standard V4L2 device structure
 *
 * Initialise the device when slave attaches to the master.
 */
static int ioctl_dev_init(struct v4l2_int_device *s)
{
	struct reg_value *pModeSetting = NULL;
	s32 i = 0;
	s32 iModeSettingArySize = 0;
	register u32 Delay_ms = 0;
	register u16 RegAddr = 0;
	register u8 Mask = 0;
	register u8 Val = 0;
	u8 RegVal = 0;
	int retval = 0;

	struct sensor_data *sensor = s->priv;
	u32 tgt_xclk;	/* target xclk */
	u32 tgt_fps;	/* target frames per secound */
	enum ov9665_frame_rate frame_rate;

	ov9665_data.on = true;

	/* mclk */
	tgt_xclk = ov9665_data.mclk;
	tgt_xclk = min(tgt_xclk, (u32)OV9665_XCLK_MAX);
	tgt_xclk = max(tgt_xclk, (u32)OV9665_XCLK_MIN);
	ov9665_data.mclk = tgt_xclk;

	pr_debug("   Setting mclk to %d MHz\n", tgt_xclk / 1000000);
	//set_mclk_rate(&ov9665_data.mclk, ov9665_data.mclk_source);

	/* Default camera frame rate is set in probe */
	tgt_fps = sensor->streamcap.timeperframe.denominator /
		  sensor->streamcap.timeperframe.numerator;

	if (tgt_fps == 15)
		frame_rate = ov9665_15_fps;
	else if (tgt_fps == 30)
		frame_rate = ov9665_30_fps;
	else
		return -EINVAL; /* Only support 15fps or 30fps now. */

	pModeSetting = ov9665_initial_setting;
	iModeSettingArySize = ARRAY_SIZE(ov9665_initial_setting);

	for (i = 0; i < iModeSettingArySize; ++i, ++pModeSetting) {
		Delay_ms = pModeSetting->u32Delay_ms;
		RegAddr = pModeSetting->u16RegAddr;
		Val = pModeSetting->u8Val;
		Mask = pModeSetting->u8Mask;
		if (Mask) {
			retval = ov9665_read_reg(RegAddr, &RegVal);
			if (retval < 0)
				goto err;

			RegVal &= ~(u8)Mask;
			Val &= Mask;
			Val |= RegVal;
		}

		retval = ov9665_write_reg(RegAddr, Val);
		if (retval < 0)
			goto err;

		if (Delay_ms)
			msleep(Delay_ms);
	}
err:
	return retval;
}

/*!
 * ioctl_dev_exit - V4L2 sensor interface handler for vidioc_int_dev_exit_num
 * @s: pointer to standard V4L2 device structure
 *
 * Delinitialise the device when slave detaches to the master.
 */
static int ioctl_dev_exit(struct v4l2_int_device *s)
{
	return 0;
}

/*!
 * This structure defines all the ioctls for this module and links them to the
 * enumeration.
 */
static struct v4l2_int_ioctl_desc ov9665_ioctl_desc[] = {
	{vidioc_int_dev_init_num, (v4l2_int_ioctl_func *)ioctl_dev_init},
	{vidioc_int_dev_exit_num, ioctl_dev_exit},
	{vidioc_int_s_power_num, (v4l2_int_ioctl_func *)ioctl_s_power},
	{vidioc_int_g_ifparm_num, (v4l2_int_ioctl_func *)ioctl_g_ifparm},
/*	{vidioc_int_g_needs_reset_num,
				(v4l2_int_ioctl_func *)ioctl_g_needs_reset}, */
/*	{vidioc_int_reset_num, (v4l2_int_ioctl_func *)ioctl_reset}, */
	{vidioc_int_init_num, (v4l2_int_ioctl_func *)ioctl_init},
	{vidioc_int_enum_fmt_cap_num,
				(v4l2_int_ioctl_func *)ioctl_enum_fmt_cap},
/*	{vidioc_int_try_fmt_cap_num,
				(v4l2_int_ioctl_func *)ioctl_try_fmt_cap}, */
	{vidioc_int_g_fmt_cap_num, (v4l2_int_ioctl_func *)ioctl_g_fmt_cap},
/*	{vidioc_int_s_fmt_cap_num, (v4l2_int_ioctl_func *)ioctl_s_fmt_cap}, */
	{vidioc_int_g_parm_num, (v4l2_int_ioctl_func *)ioctl_g_parm},
	{vidioc_int_s_parm_num, (v4l2_int_ioctl_func *)ioctl_s_parm},
/*	{vidioc_int_queryctrl_num, (v4l2_int_ioctl_func *)ioctl_queryctrl}, */
	{vidioc_int_g_ctrl_num, (v4l2_int_ioctl_func *)ioctl_g_ctrl},
	{vidioc_int_s_ctrl_num, (v4l2_int_ioctl_func *)ioctl_s_ctrl},
	{vidioc_int_enum_framesizes_num,
				(v4l2_int_ioctl_func *)ioctl_enum_framesizes},
	{vidioc_int_g_chip_ident_num,
				(v4l2_int_ioctl_func *)ioctl_g_chip_ident},
};

static struct v4l2_int_slave ov9665_slave = {
	.ioctls = ov9665_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(ov9665_ioctl_desc),
};

static struct v4l2_int_device ov9665_int_device = {
	.module = THIS_MODULE,
	.name = "ov9665",
	.type = v4l2_int_type_slave,
	.u = {
		.slave = &ov9665_slave,
	},
};

/*!
 * ov9665 I2C probe function
 *
 * @param adapter            struct i2c_adapter *
 * @return  Error code indicating success or failure
 */
static int ov9665_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct pinctrl *pinctrl;
	struct device *dev = &client->dev;
	int retval;
	u8 chip_id_high, chip_id_low;
	
	/* ov9665 pinctrl */
	pinctrl = devm_pinctrl_get_select_default(dev);
	if (IS_ERR(pinctrl)) {
		dev_err(dev, "ov9665 setup pinctrl failed!");
		return PTR_ERR(pinctrl);
	}

	/* request power down pin */
	pwn_gpio = of_get_named_gpio(dev->of_node, "pwn-gpios", 0);
	if (!gpio_is_valid(pwn_gpio)) {
		dev_warn(dev, "no sensor pwdn pin available");
		return -EINVAL;
	}
	retval = devm_gpio_request_one(dev, pwn_gpio, GPIOF_OUT_INIT_HIGH,
					"ov9665_pwdn");
	if (retval < 0)
		return retval;

	/* request reset pin */
	rst_gpio = of_get_named_gpio(dev->of_node, "rst-gpios", 0);
	if (!gpio_is_valid(rst_gpio)) {
		dev_warn(dev, "no sensor reset pin available");
		return -EINVAL;
	}
	retval = devm_gpio_request_one(dev, rst_gpio, GPIOF_OUT_INIT_HIGH,
					"ov9665_reset");
	if (retval < 0)
		return retval;

	/* Set initial values for the sensor struct. */
	memset(&ov9665_data, 0, sizeof(ov9665_data));
	ov9665_data.sensor_clk = devm_clk_get(dev, "csi_mclk");
	if (IS_ERR(ov9665_data.sensor_clk)) {
		/* assuming clock enabled by default */
		ov9665_data.sensor_clk = NULL;
		dev_err(dev, "clock-frequency missing or invalid\n");
		return PTR_ERR(ov9665_data.sensor_clk);
	}

	retval = of_property_read_u32(dev->of_node, "mclk",
					(u32 *) &(ov9665_data.mclk));
	if (retval) {
		dev_err(dev, "mclk missing or invalid\n");
		return retval;
	}

	retval = of_property_read_u32(dev->of_node, "mclk_source",
					(u32 *) &(ov9665_data.mclk_source));
	if (retval) {
		dev_err(dev, "mclk_source missing or invalid\n");
		return retval;
	}

	retval = of_property_read_u32(dev->of_node, "csi_id",
					&(ov9665_data.csi));
	if (retval) {
		dev_err(dev, "csi_id missing or invalid\n");
		return retval;
	}
	
	clk_prepare_enable(ov9665_data.sensor_clk);
	
	ov9665_data.io_init =  ov9665_reset;
	ov9665_data.i2c_client = client;
	ov9665_data.pix.pixelformat = V4L2_PIX_FMT_YUYV;
	ov9665_data.pix.width = 640;
	ov9665_data.pix.height = 480;
	ov9665_data.streamcap.capability = V4L2_MODE_HIGHQUALITY |
					   V4L2_CAP_TIMEPERFRAME;
	ov9665_data.streamcap.capturemode = 0;
	ov9665_data.streamcap.timeperframe.denominator = DEFAULT_FPS;
	ov9665_data.streamcap.timeperframe.numerator = 1;
	
	ov9665_power_on(dev);

	ov9665_reset();

	ov9665_standby(0);

	retval = ov9665_read_reg(OV9665_CHIP_ID_HIGH_BYTE, &chip_id_high);
	if (retval < 0 || chip_id_high != 0x96) {
		pr_warning("camera ov9665 is not found\n");
		pr_warning("high byte: %d\n", retval);
		clk_disable_unprepare(ov9665_data.sensor_clk);
		return -ENODEV;
	}
	retval = ov9665_read_reg(OV9665_CHIP_ID_LOW_BYTE, &chip_id_low);
	if (retval < 0 || chip_id_low != 0x63) {
		pr_warning("camera ov9665 is not found\n");
		clk_disable_unprepare(ov9665_data.sensor_clk);
		return -ENODEV;
	}

	ov9665_standby(1);

	ov9665_int_device.priv = &ov9665_data;
	retval = v4l2_int_device_register(&ov9665_int_device);
	
	clk_disable_unprepare(ov9665_data.sensor_clk);

	pr_info("camera ov9665 is found\n");

	return retval;
}

/*!
 * ov9665 I2C detach function
 *
 * @param client            struct i2c_client *
 * @return  Error code indicating success or failure
 */
static int ov9665_remove(struct i2c_client *client)
{
	v4l2_int_device_unregister(&ov9665_int_device);

	if (gpo_regulator) {
		regulator_disable(gpo_regulator);
	}

	if (analog_regulator) {
		regulator_disable(analog_regulator);
	}

	if (core_regulator) {
		regulator_disable(core_regulator);
	}

	if (io_regulator) {
		regulator_disable(io_regulator);
	}

	return 0;
}

/*!
 * ov9665 init function
 * Called by insmod ov9665_camera.ko.
 *
 * @return  Error code indicating success or failure
 */
static __init int ov9665_init(void)
{
	u8 err;

	err = i2c_add_driver(&ov9665_i2c_driver);
	if (err != 0)
		pr_err("%s:driver registration failed, error=%d \n",
			__func__, err);

	return err;
}

/*!
 * OV9665 cleanup function
 * Called on rmmod ov9665_camera.ko
 *
 * @return  Error code indicating success or failure
 */
static void __exit ov9665_clean(void)
{
	i2c_del_driver(&ov9665_i2c_driver);
}

module_init(ov9665_init);
module_exit(ov9665_clean);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("OV9665 Camera Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
MODULE_ALIAS("CSI");
