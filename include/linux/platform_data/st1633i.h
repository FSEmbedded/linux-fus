/*
 * drivers/input/touchscreen/sitronix_i2c_touch.h
 *
 * Touchscreen driver for Sitronix
 *
 * Copyright (C) 2011 Sitronix Technology Co., Ltd.
 *	Rudy Huang <rudy_huang@sitronix.com.tw>
 */
/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 */

#ifndef __SITRONIX_I2C_TOUCH_h
#define __SITRONIX_I2C_TOUCH_h

#include <linux/ioctl.h> /* needed for the _IOW etc stuff used later */

#define SITRONIX_TOUCH_DRIVER_VERSION 0x03
#define SITRONIX_MAX_SUPPORTED_POINT 10
#define SITRONIX_I2C_TOUCH_DRV_NAME "sitronix"
#define SITRONIX_I2C_TOUCH_DEV_NAME "sitronixDev"
#define SITRONIX_I2C_TOUCH_MT_INPUT_DEV_NAME "sitronix-i2c-touch-mt"
#define SITRONIX_I2C_TOUCH_KEY_INPUT_DEV_NAME "sitronix-i2c-touch-key"

#ifdef CONFIG_MACH_DEVKIT8000
#define SITRONIX_RESET_GPIO	170
#define SITRONIX_INT_GPIO	157
#elif defined(CONFIG_MACH_OMAP4_PANDA)
#define SITRONIX_RESET_GPIO	44
#define SITRONIX_INT_GPIO	47
#elif defined(CONFIG_ARCH_MSM8X60)
#define SITRONIX_RESET_GPIO	58
#define SITRONIX_INT_GPIO	61
#endif // CONFIG_MACH_DEVKIT8000

// MT SLOT feature is implmented in linux kernel 2.6.38 and later. Make sure that version of your linux kernel before using this feature.
//#define SITRONIX_SUPPORT_MT_SLOT
//#define SITRONIX_I2C_COMBINED_MESSAGE
#ifndef SITRONIX_I2C_COMBINED_MESSAGE
#define SITRONIX_I2C_SINGLE_MESSAGE
#endif // SITRONIX_I2C_COMBINED_MESSAGE
//#define SITRONIX_MONITOR_THREAD
#define DELAY_MONITOR_THREAD_START_PROBE 10000
#define DELAY_MONITOR_THREAD_START_RESUME 3000
#define SITRONIX_FW_UPGRADE_FEATURE
//#define SITRONIX_PERMISSION_THREAD
#define SITRONIX_SYSFS
#define SITRONIX_LEVEL_TRIGGERED
//#define SITRONIX_SMART_WAKE_UP

// When enable_irq() is invoked, irq will be sent once while INT is not triggered if CONFIG_HARDIRQS_SW_RESEND is set.
// This behavior is implemented by linux kernel, it is used to prevent irq from losting when irq is edge-triggered mode.
#ifndef SITRONIX_LEVEL_TRIGGERED
#define SITRONIX_INT_POLLING_MODE
#define INT_POLLING_MODE_INTERVAL 14
#endif // SITRONIX_LEVEL_TRIGGERED
//#define SITRONIX_IDENTIFY_ID
//#define SITRONIX_MULTI_SLAVE_ADDR

//#define EnableDbgMsg 1
//#define EnableUpgradeMsg 1

#ifdef EnableDbgMsg
#define DbgMsg(arg...) printk(arg)
#else
#define DbgMsg(arg...)
#endif

#ifdef EnableUpgradeMsg
#define UpgradeMsg(arg...) printk(arg)
#else
#define UpgradeMsg(arg...)
#endif

typedef enum{
	FIRMWARE_VERSION,
	STATUS_REG,
	DEVICE_CONTROL_REG,
	TIMEOUT_TO_IDLE_REG,
	XY_RESOLUTION_HIGH,
	X_RESOLUTION_LOW,
	Y_RESOLUTION_LOW,
	DEVICE_CONTROL_REG2 = 0x09,
	FIRMWARE_REVISION_3 = 0x0C,
	FIRMWARE_REVISION_2,
	FIRMWARE_REVISION_1,
	FIRMWARE_REVISION_0,
	FINGERS,
	KEYS_REG,
	XY0_COORD_H,
	X0_COORD_L,
	Y0_COORD_L,
	I2C_PROTOCOL = 0x3E,
	MAX_NUM_TOUCHES,
	DATA_0_HIGH,
	DATA_0_LOW,
	MISC_CONTROL = 0xF1,
	SMART_WAKE_UP_REG = 0xF2,
	CHIP_ID = 0xF4,
	PAGE_REG = 0xff,
}RegisterOffset;

typedef enum{
	NO = 0x0,
	CHARACTER_C	    = 0x63,
	CHARACTER_E	    = 0x65,
	CHARACTER_M	    = 0x6D,
	CHARACTER_O	    = 0x6F,
	CHARACTER_S	    = 0x73,
	CHARACTER_V	    = 0x76,
	CHARACTER_W	    = 0x77,
	CHARACTER_Z	    = 0x7A,
	LEFT_TO_RIGHT_SLIDE = 0xB0,
	RIGHT_TO_LEFT_SLIDE = 0xB4,
	TOP_TO_DOWN_SLIDE   = 0xB8,
	DOWN_TO_UP_SLIDE    = 0xBC,
	DOUBLE_CLICK	    = 0xC0,
}SWK_ID;

#define SITRONIX_TS_CHANGE_MODE_DELAY 150

typedef enum{
	XY_COORD_H,
	X_COORD_L,
	Y_COORD_L,
	PIXEL_DATA_LENGTH_B,
	PIXEL_DATA_LENGTH_A,
}PIXEL_DATA_FORMAT;

#define X_RES_H_SHFT 4
#define X_RES_H_BMSK 0xf
#define Y_RES_H_SHFT 0
#define Y_RES_H_BMSK 0xf
#define FINGERS_SHFT 0
#define FINGERS_BMSK 0xf
#define X_COORD_VALID_SHFT 7
#define X_COORD_VALID_BMSK 0x1
#define X_COORD_H_SHFT 4
#define X_COORD_H_BMSK 0x7
#define Y_COORD_H_SHFT 0
#define Y_COORD_H_BMSK 0x7

typedef enum{
	SITRONIX_RESERVED_TYPE_0,
	SITRONIX_A_TYPE,
	SITRONIX_B_TYPE,
}I2C_PROTOCOL_TYPE;

#define I2C_PROTOCOL_SHFT 0x0
#define I2C_PROTOCOL_BMSK 0x3

typedef enum{
	SENSING_BOTH,
	SENSING_X_ONLY,
	SENSING_Y_ONLY,
	SENSING_BOTH_NOT,
}ONE_D_SENSING_CONTROL_MODE;

#define ONE_D_SENSING_CONTROL_SHFT 0x2
#define ONE_D_SENSING_CONTROL_BMSK 0x3

#define SMT_IOC_MAGIC   0xf1

enum{
	SMT_GET_DRIVER_REVISION = 1,
	SMT_GET_FW_REVISION,
	SMT_ENABLE_IRQ,
	SMT_DISABLE_IRQ,
	SMT_RESUME,
	SMT_SUSPEND,
	SMT_HW_RESET,
	SMT_REPROBE,
	SMT_IOC_MAXNR,
};

#define IOCTL_SMT_GET_DRIVER_REVISION				_IOC(_IOC_READ,  SMT_IOC_MAGIC, SMT_GET_DRIVER_REVISION, 			1)
#define IOCTL_SMT_GET_FW_REVISION					_IOC(_IOC_READ,  SMT_IOC_MAGIC, SMT_GET_FW_REVISION, 				4)
#define IOCTL_SMT_ENABLE_IRQ					_IOC(_IOC_NONE, SMT_IOC_MAGIC, SMT_ENABLE_IRQ, 0)
#define IOCTL_SMT_DISABLE_IRQ					_IOC(_IOC_NONE, SMT_IOC_MAGIC, SMT_DISABLE_IRQ, 0)
#define IOCTL_SMT_RESUME					_IOC(_IOC_NONE, SMT_IOC_MAGIC, SMT_RESUME, 0)
#define IOCTL_SMT_SUSPEND					_IOC(_IOC_NONE, SMT_IOC_MAGIC, SMT_SUSPEND, 0)
#define IOCTL_SMT_HW_RESET					_IOC(_IOC_NONE, SMT_IOC_MAGIC, SMT_HW_RESET, 0)
#define IOCTL_SMT_REPROBE					_IOC(_IOC_NONE, SMT_IOC_MAGIC, SMT_REPROBE, 0)

//#define SITRONIX_AA_KEY
#define SITRONIX_KEY_BOUNDARY_MANUAL_SPECIFY

typedef struct {
	u8 y_h		:3,
	   reserved	:1,
	   x_h		:3,
	   valid	:1;
	u8 x_l;
	u8 y_l;
	u8 z;
}xy_data_t;

typedef struct {
	xy_data_t	xy_data[SITRONIX_MAX_SUPPORTED_POINT];
}stx_report_data_t;

struct sitronix_sensor_key_t{
	unsigned int code;
};

#ifndef SITRONIX_AA_KEY
enum{
	AREA_NONE,
	AREA_DISPLAY,
};
#else
enum{
	AREA_NONE,
	AREA_DISPLAY,
	AREA_KEY,
	AREA_INVALID,
};

struct sitronix_AA_key{
	int x_low;
	int x_high;
	int y_low;
	int y_high;
	unsigned int code;
};
#endif // SITRONIX_AA_KEY

typedef struct {
	uint8_t offset;
	uint8_t shft;
	uint8_t bmsk;
}sitronix_reg_field;

typedef struct {
	sitronix_reg_field dis_coord_flag;
}sitronix_i2c_protocol_map;
/*
static sitronix_i2c_protocol_map sitronix_i2c_ptcl_v1 = {
	.dis_coord_flag = {
		.offset = 0x09,
		.shft = 0,
		.bmsk = 0x1,
	},
};

static sitronix_i2c_protocol_map sitronix_i2c_ptcl_v2 = {
	.dis_coord_flag = {
		.offset = 0xF1,
		.shft = 2,
		.bmsk = 0x1,
	},
};
*/
struct sitronix_i2c_touch_platform_data {
	uint32_t version;	/* Use this entry for panels with */
				/* (major << 8 | minor) version or above. */
				/* If non-zero another array entry follows */
	void (*reset_ic)(void);
};

//petitk add

//#define CONFIG_TOUCHSCREEN_SITRONIX_I2C_TOUCH
//#define KERN_DEBUG    "<7>"    /* debug-level messages */
////////////

#endif // __SITRONIX_I2C_TOUCH_h
