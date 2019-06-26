/*
 * Copyright (C) 2015-2016 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/mipi_csi2.h>
#include <media/v4l2-chip-ident.h>
#include "v4l2-int-device.h"
#include "mxc_v4l2_capture.h"

#define CHIP_ID_79985		0x85
#define CHIP_ID_79987		0x87

#define ISL7998X_STD_NTSC	0
#define ISL7998X_STD_PAL	1
#define ISL7998X_STD_SECAM	2
#define ISL7998X_STD_NTSC_443	3
#define ISL7998X_STD_PAL_M	4
#define ISL7998X_STD_PAL_CN	5
#define ISL7998X_STD_PAL_60	6
#define ISL7998X_STD_AUTO	7

#define ISL7997X_FRAME_TYPES	2

#define SENSOR_NUM 4

static struct isl7998x_frametype {
	unsigned int width;
	unsigned int height;
	unsigned int rate;
} g_isl7998x_frametype[ISL7997X_FRAME_TYPES];

/*!
 * Maintains the information on the current state of the sesor.
 */
static struct sensor_data isl7998x_data[SENSOR_NUM];
static unsigned int chip_id = 0;
static unsigned int sensor_on;		/* One bit for each sensor */
static spinlock_t lock;

static int isl7998x_probe(struct i2c_client *adapter,
				const struct i2c_device_id *device_id);
static int isl7998x_remove(struct i2c_client *client);

static int ioctl_dev_init(struct v4l2_int_device *s);

static const struct i2c_device_id isl7998x_id[] = {
	{"isl7998x_mipi", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, isl7998x_id);

static struct i2c_driver isl7998x_i2c_driver = {
	.driver = {
		  .owner = THIS_MODULE,
		  .name  = "isl7998x_mipi",
		  },
	.probe  = isl7998x_probe,
	.remove = isl7998x_remove,
	.id_table = isl7998x_id,
};


/*! Read one register from a ISL7998x i2c slave device.
 *
 *  @param *reg		register in the device we wish to access.
 *
 *  @return		       0 if success, an error code otherwise.
 */
static inline int isl7998x_read_reg(u8 reg)
{
	int val;

	val = i2c_smbus_read_byte_data(isl7998x_data[0].i2c_client, reg);
	if (val < 0) {
		dev_info(&isl7998x_data[0].i2c_client->dev,
			"%s:read reg error: reg=%2x\n", __func__, reg);
		return -1;
	}
	return val;
}

/*! Write one register of a ISL7998x i2c slave device.
 *
 *  @param *reg		register in the device we wish to access.
 *
 *  @return		       0 if success, an error code otherwise.
 */
static int isl7998x_write_reg(u8 reg, u8 val)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(isl7998x_data[0].i2c_client, reg, val);
	if (ret < 0) {
		dev_info(&isl7998x_data[0].i2c_client->dev,
			"%s:write reg error:reg=%2x,val=%2x\n", __func__,
			reg, val);
		return -1;
	}
	return 0;
}

/* Update sensor info for frame type and frame rate */
static void isl7998x_update_sensor(struct sensor_data *sensor, int i)
{
	sensor->streamcap.timeperframe.denominator =
						g_isl7998x_frametype[i].rate;
	sensor->streamcap.timeperframe.numerator = 1;
	sensor->pix.width = g_isl7998x_frametype[i].width;
	sensor->pix.height = g_isl7998x_frametype[i].height;

	/* Do we need to set bytesperline, screensize, etc.? */
}

/* Get index for frametype array corresponding to given std_mode */
static int isl7998x_get_frametype_index(int std_mode)
{
	if ((std_mode == ISL7998X_STD_PAL)
	    || (std_mode == ISL7998X_STD_PAL_CN)
	    || (std_mode == ISL7998X_STD_SECAM))
		return 1;		/* 720x576@25 */

	return 0;			/* 720x480@30 (also PAL-M/PAL-60) */
}

/* Update frame size and frame rate according to currently active standard. */
static int isl7998x_update_mode(struct sensor_data *sensor)
{
	int std_mode;

	isl7998x_write_reg(0xFF, sensor->v_channel + 1);
	std_mode = isl7998x_read_reg(0x1C);
	if (std_mode < 0)
		return std_mode;

	std_mode = (std_mode >> 4) & 0x07;
	isl7998x_update_sensor(sensor, isl7998x_get_frametype_index(std_mode));

	return 0;
}

static int isl7998x_start_mipi(struct sensor_data *sensor)
{
	void *mipi_csi2_info;
	u32 mipi_reg;
	int i, lanes;

	mipi_csi2_info = mipi_csi2_get_info();

	/* initial mipi dphy */
	if (!mipi_csi2_info) {
		printk(KERN_ERR "%s() in %s: Fail to get s_mipi_csi2_info!\n",
		       __func__, __FILE__);
		return -1;
	}

	if (!mipi_csi2_get_status(mipi_csi2_info))
		mipi_csi2_enable(mipi_csi2_info);

	if (!mipi_csi2_get_status(mipi_csi2_info)) {
		pr_err("Can not enable mipi csi2 driver!\n");
		return -1;
	}

	lanes = mipi_csi2_set_lanes(mipi_csi2_info);
	if (lanes > 1) {
		pr_err("ISL7998x doesn't support lanes = %d.\n", lanes + 1);
		return -1;
	}

	/* Page 0: Set clock */
	isl7998x_write_reg(0xFF, 0x00);
	if (lanes == 1)
		isl7998x_write_reg(0x0B, 0x41);
	else
		isl7998x_write_reg(0x0B, 0x40);

	/* Page 5: Power up PLL and MIPI */
	isl7998x_write_reg(0xFF, 0x05);
	isl7998x_write_reg(0x34, 0x18);
	isl7998x_write_reg(0x35, 0x00);
	if (lanes == 1)
		isl7998x_write_reg(0x00, 0x02);
	else
		isl7998x_write_reg(0x00, 0x01);

	/* Only reset MIPI CSI2 HW at sensor initialize */
	/* 13.5MHz pixel clock (720*480@30fps) * 16 bits per pixel (YUV422) = 216Mbps mipi data rate for each camera */
	mipi_csi2_reset(mipi_csi2_info, (216 * SENSOR_NUM) / (lanes + 1));

	if (sensor->pix.pixelformat == V4L2_PIX_FMT_UYVY) {
		for (i=0; i<SENSOR_NUM; i++)
			mipi_csi2_set_datatype(mipi_csi2_info, i, MIPI_DT_YUV422);
	} else
		pr_err("currently this sensor format can not be supported!\n");

	i = 0;

	/* wait for mipi stable */
	mipi_reg = mipi_csi2_get_error1(mipi_csi2_info);
	while ((mipi_reg != 0x0) && (i < 10)) {
		mipi_reg = mipi_csi2_get_error1(mipi_csi2_info);
		i++;
		msleep(10);
	}

	if (i >= 10) {
		pr_err("mipi csi2 can not reveive data correctly! MIPI_CSI_ERR1 = 0x%x.\n", mipi_reg);
		return -1;
	}

	return 0;
}

static void isl7998x_stop_mipi(struct sensor_data *sensor)
{
	void *mipi_csi2_info;

	mipi_csi2_info = mipi_csi2_get_info();

	/* disable mipi csi2 */
	if (!mipi_csi2_info)
		return;

	if (mipi_csi2_get_status(mipi_csi2_info)) {
		mipi_csi2_disable(mipi_csi2_info);

		/* Page 5: power down PLL and MIPI */
		isl7998x_write_reg(0xFF, 0x05);
		isl7998x_write_reg(0x34, 0x02);
		isl7998x_write_reg(0x35, 0x31);
		isl7998x_write_reg(0x00, 0x82);
	}
}


static void isl7998x_hardware_init(unsigned int std_mode)
{
	// Init the isl7998x
	if (chip_id == CHIP_ID_79985) {
		// Page 0
		isl7998x_write_reg(0xFF, 0x00);
		isl7998x_write_reg(0x03, 0x00);
#if 0
		if (lanes == 1)
			isl7998x_write_reg(0x0B, 0x41);
		else
			isl7998x_write_reg(0x0B, 0x40);
#endif
		isl7998x_write_reg(0x0D, 0xC9);
		isl7998x_write_reg(0x0E, 0xC9);
		isl7998x_write_reg(0x10, 0x01);
		isl7998x_write_reg(0x11, 0x03);
		isl7998x_write_reg(0x12, 0x00);
		isl7998x_write_reg(0x13, 0x00);
		isl7998x_write_reg(0x14, 0x00);
		isl7998x_write_reg(0xFF, 0x00);

		// Page 1-4 for all decoders
		isl7998x_write_reg(0xFF, 0x0F);
		isl7998x_write_reg(0x2F, 0xE6);
		isl7998x_write_reg(0x33, 0x85);
		/* PAL-shadow register: VACTIVE=288 lines */
		isl7998x_write_reg(0x1C, 0x01);
		isl7998x_write_reg(0x07, 0x12);
		isl7998x_write_reg(0x09, 0x20);
		/* NTSC: VACTIVE=240 lines */
		isl7998x_write_reg(0x1C, 0x00);
		isl7998x_write_reg(0x07, 0x02);
		isl7998x_write_reg(0x08, 0x12);
		isl7998x_write_reg(0x09, 0xF0);
		isl7998x_write_reg(0xFF, 0x0F);
		isl7998x_write_reg(0x1C, std_mode);

		// Page 5
		isl7998x_write_reg(0xFF, 0x05);
		isl7998x_write_reg(0x01, 0x85);
		isl7998x_write_reg(0x02, 0xA0);
		isl7998x_write_reg(0x03, 0x08);
		isl7998x_write_reg(0x04, 0xE4);
		isl7998x_write_reg(0x05, 0x00);
		isl7998x_write_reg(0x06, 0x00);
		isl7998x_write_reg(0x07, 0x46);
		isl7998x_write_reg(0x08, 0x02);
		isl7998x_write_reg(0x09, 0x00);
		isl7998x_write_reg(0x0A, 0x68);
		isl7998x_write_reg(0x0B, 0x02);
		isl7998x_write_reg(0x0C, 0x00);
		isl7998x_write_reg(0x0D, 0x06);
		isl7998x_write_reg(0x0E, 0x00);
		isl7998x_write_reg(0x0F, 0x00);
		isl7998x_write_reg(0x10, 0x05);
		isl7998x_write_reg(0x11, 0xA0);
		isl7998x_write_reg(0x12, 0x76);
		isl7998x_write_reg(0x13, 0x2F);
		isl7998x_write_reg(0x14, 0x0E);
		isl7998x_write_reg(0x15, 0x36);
		isl7998x_write_reg(0x16, 0x12);
		isl7998x_write_reg(0x17, 0xF6);
		isl7998x_write_reg(0x18, 0x00);
		isl7998x_write_reg(0x19, 0x17);
		isl7998x_write_reg(0x1A, 0x0A);
		isl7998x_write_reg(0x1B, 0x61);
		isl7998x_write_reg(0x1C, 0x7A);
		isl7998x_write_reg(0x1D, 0x0F);
		isl7998x_write_reg(0x1E, 0x8C);
		isl7998x_write_reg(0x1F, 0x02);
		isl7998x_write_reg(0x20, 0x00);
		isl7998x_write_reg(0x21, 0x0C);
//read-only	isl7998x_write_reg(0x22, 0x00);
//unknown reg.	isl7998x_write_reg(0x23, 0x00);
//read-only	isl7998x_write_reg(0x24, 0x00);
//read-only	isl7998x_write_reg(0x25, 0xF0);
		isl7998x_write_reg(0x26, 0x00);
		isl7998x_write_reg(0x27, 0x00);
		isl7998x_write_reg(0x28, 0x01);
		isl7998x_write_reg(0x29, 0x0E);
		isl7998x_write_reg(0x2A, 0x00);
		isl7998x_write_reg(0x2B, 0x19);
		isl7998x_write_reg(0x2C, 0x18);
		isl7998x_write_reg(0x2D, 0xF1);
		isl7998x_write_reg(0x2E, 0x00);
		isl7998x_write_reg(0x2F, 0xF1);
		isl7998x_write_reg(0x30, 0x00);
		isl7998x_write_reg(0x31, 0x00);
		isl7998x_write_reg(0x32, 0x00);
		isl7998x_write_reg(0x33, 0xC0);
		isl7998x_write_reg(0x36, 0x00);

#if 0
		isl7998x_write_reg(0x34, 0x18);
		isl7998x_write_reg(0x35, 0x00);
		if (lanes == 1)
			isl7998x_write_reg(0x00, 0x02);
		else
			isl7998x_write_reg(0x00, 0x01);
#endif
		isl7998x_write_reg(0xFF, 0x05);
	} else if (chip_id == CHIP_ID_79987) {
		// Page 5
		isl7998x_write_reg(0xFF, 0x05);

		// Page 0
		isl7998x_write_reg(0xFF, 0x00);
		isl7998x_write_reg(0x02, 0x1F);  /* set Reset */

		// Default setting
		isl7998x_write_reg(0xFF, 0x00);
		isl7998x_write_reg(0x03, 0x00);
		isl7998x_write_reg(0x0D, 0xC9);
		isl7998x_write_reg(0x0E, 0xC9);
		isl7998x_write_reg(0x10, 0x01);
		isl7998x_write_reg(0x11, 0x03);
		isl7998x_write_reg(0x12, 0x00);
		isl7998x_write_reg(0x13, 0x00);
		isl7998x_write_reg(0x14, 0x00);

		isl7998x_write_reg(0xFF, 0x05);
		isl7998x_write_reg(0x00, 0x02);
		isl7998x_write_reg(0x01, 0x85);
		isl7998x_write_reg(0x02, 0xA0);
		isl7998x_write_reg(0x03, 0x18);
		isl7998x_write_reg(0x05, 0x40);
		isl7998x_write_reg(0x06, 0x40);
		isl7998x_write_reg(0x10, 0x05);
		isl7998x_write_reg(0x11, 0xA0);
		isl7998x_write_reg(0x20, 0x00);
		isl7998x_write_reg(0x21, 0x0C);
		isl7998x_write_reg(0x22, 0x00);
		isl7998x_write_reg(0x23, 0x00);
		isl7998x_write_reg(0x24, 0x00);
		isl7998x_write_reg(0x25, 0xF0);
		isl7998x_write_reg(0x26, 0x00);
		isl7998x_write_reg(0x27, 0x00);
		isl7998x_write_reg(0x2A, 0x00);
		isl7998x_write_reg(0x2B, 0x19);
		isl7998x_write_reg(0x2C, 0x18);
		isl7998x_write_reg(0x2D, 0xF1);
		isl7998x_write_reg(0x2E, 0x00);
		isl7998x_write_reg(0x2F, 0xF1);
		isl7998x_write_reg(0x30, 0x00);
		isl7998x_write_reg(0x31, 0x00);
		isl7998x_write_reg(0x32, 0x00);
		isl7998x_write_reg(0x33, 0xC0);

		// Page 0
		isl7998x_write_reg(0xFF, 0x00);
		isl7998x_write_reg(0x02, 0x10);  /* clear CReset_CH1 ~ Reset_CH4 */

		// Page 1-4 for all decoders
		isl7998x_write_reg(0xFF, 0x0F);
		isl7998x_write_reg(0x2F, 0xE6);
		isl7998x_write_reg(0x33, 0x85);
		/* PAL-shadow register: VACTIVE=288 lines */
		isl7998x_write_reg(0x1C, 0x01);
		isl7998x_write_reg(0x07, 0x12);
		isl7998x_write_reg(0x09, 0x20);
		/* NTSC: VACTIVE=240 lines */
		isl7998x_write_reg(0x1C, 0x00);
		isl7998x_write_reg(0x07, 0x02);
//###		isl7998x_write_reg(0x08, 0x12);
		isl7998x_write_reg(0x08, 0x14);
		isl7998x_write_reg(0x09, 0xF0);
		isl7998x_write_reg(0x45, 0x11);
		isl7998x_write_reg(0x1C, std_mode);
		isl7998x_write_reg(0xE7, 0x00);

		// Page 0
		isl7998x_write_reg(0xFF, 0x00);
		isl7998x_write_reg(0x07, 0x02);
		isl7998x_write_reg(0x08, 0x1F);
		isl7998x_write_reg(0x09, 0x43);
		isl7998x_write_reg(0x0A, 0x4F);

		// Page 5
		isl7998x_write_reg(0xFF, 0x05);
//		isl7998x_write_reg(0x01, 0x05);  //For field mode
		isl7998x_write_reg(0x01, 0x25);  //For frame mode
		isl7998x_write_reg(0x02, 0xA0);
		isl7998x_write_reg(0x03, 0x10);
		isl7998x_write_reg(0x04, 0xE4);
		isl7998x_write_reg(0x05, 0x00);
		isl7998x_write_reg(0x06, 0x60);
		isl7998x_write_reg(0x07, 0x2B);
		isl7998x_write_reg(0x08, 0x02);
		isl7998x_write_reg(0x09, 0x00);
		isl7998x_write_reg(0x0A, 0x62);
		isl7998x_write_reg(0x0B, 0x02);
		isl7998x_write_reg(0x0C, 0x36);
		isl7998x_write_reg(0x0D, 0x00);
		isl7998x_write_reg(0x0E, 0x6C);
		isl7998x_write_reg(0x0F, 0x00);
		isl7998x_write_reg(0x10, 0x05);
		isl7998x_write_reg(0x11, 0xA0);
		isl7998x_write_reg(0x12, 0x77);
		isl7998x_write_reg(0x13, 0x17);
		isl7998x_write_reg(0x14, 0x08);
		isl7998x_write_reg(0x15, 0x38);
		isl7998x_write_reg(0x16, 0x14);
		isl7998x_write_reg(0x17, 0xF6);
		isl7998x_write_reg(0x18, 0x00);
		isl7998x_write_reg(0x19, 0x17);
		isl7998x_write_reg(0x1A, 0x0A);
		isl7998x_write_reg(0x1B, 0x71);
		isl7998x_write_reg(0x1C, 0x7A);
		isl7998x_write_reg(0x1D, 0x0F);
		isl7998x_write_reg(0x1E, 0x8C);
		isl7998x_write_reg(0x23, 0x0A);
		isl7998x_write_reg(0x26, 0x08);
		isl7998x_write_reg(0x28, 0x01);
		isl7998x_write_reg(0x29, 0x0E);
		isl7998x_write_reg(0x2A, 0x00);
		isl7998x_write_reg(0x2B, 0x00);
		isl7998x_write_reg(0x38, 0x03);
		isl7998x_write_reg(0x39, 0xC0);
		isl7998x_write_reg(0x3A, 0x06);
		isl7998x_write_reg(0x3B, 0xB3);
		isl7998x_write_reg(0x3C, 0x00);
		isl7998x_write_reg(0x3D, 0xF1);

		/* change by BKang */
		isl7998x_write_reg(0x06, 0x00);
		isl7998x_write_reg(0x35, 0x00);

		// Page 0
		isl7998x_write_reg(0xFF, 0x00);
		isl7998x_write_reg(0x02, 0x00);  /* clear Reset */
	}
}

/* --------------- IOCTL functions from v4l2_int_ioctl_desc --------------- */

static int ioctl_g_ifparm(struct v4l2_int_device *s, struct v4l2_ifparm *p)
{
	struct sensor_data *sensor = s->priv;

	if (s == NULL) {
		pr_err("   ERROR!! no slave device set!\n");
		return -1;
	}

	memset(p, 0, sizeof(*p));
	p->u.bt656.clock_curr = sensor->mclk;
	pr_debug("   clock_curr=mclk=%d\n", sensor->mclk);
	p->if_type = V4L2_IF_TYPE_BT656;
	p->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT;
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
	struct v4l2_captureparm *cparm;

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	isl7998x_update_mode(sensor);
	memset(a, 0, sizeof(*a));
	cparm = &a->parm.capture;
	cparm->capability = sensor->streamcap.capability;
	cparm->timeperframe = sensor->streamcap.timeperframe;
	cparm->capturemode = sensor->streamcap.capturemode;

	return 0;
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
	struct v4l2_fract *tpf;

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	isl7998x_update_mode(sensor);
	tpf = &a->parm.capture.timeperframe;
	if ((tpf->denominator != sensor->streamcap.timeperframe.denominator)
	    || (tpf->numerator != sensor->streamcap.timeperframe.numerator))
		return -EINVAL;

	return 0;
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

	isl7998x_update_mode(sensor);
	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		f->fmt.pix = sensor->pix;

		return 0;
	}
	/*
	 * V4L2_BUF_TYPE_PRIVATE is misused for video standard because there
	 * is no interface function to pass this information between the slave
	 * and the master capture driver in mxc_v4l2_capture.c. However the
	 * mxc_v4l2_capture driver is not really interested in the TV
	 * standard, it just needs a hint for the resolution and frame rate to
	 * manipulate buffer croppings. So return V4L2_STD_PAL for all
	 * standards with 720x576@50 and V4L2_STD_NTSC for all standards with
	 * 720x480@60.
	 */
	if (f->type == V4L2_BUF_TYPE_PRIVATE) {
		if (sensor->streamcap.timeperframe.denominator == 50)
			f->fmt.pix.pixelformat = V4L2_STD_PAL;
		else
			f->fmt.pix.pixelformat = V4L2_STD_NTSC;

		return 0;
	}

	return -EINVAL;
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

	ret = -EINVAL;

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
	int ret = 0;
	return ret;
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
	struct sensor_data *sensor = s->priv;

	if (fsize->index > 0)
		return -EINVAL;

#if 0 // imxv4l2src passes a bad pixelformat in some cases, so skip test
	if (fsize->pixel_format != sensor->pix.pixelformat)
		return -EINVAL;
#endif

	isl7998x_update_mode(sensor);

	fsize->discrete.width = sensor->pix.width;
	fsize->discrete.height = sensor->pix.height;

	return 0;
}

/*!
 * ioctl_enum_frameintervals - V4L2 sensor interface handler for
 *			       VIDIOC_ENUM_FRAMEINTERVALS ioctl
 * @s: pointer to standard V4L2 device structure
 * @fival: standard V4L2 VIDIOC_ENUM_FRAMEINTERVALS ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int ioctl_enum_frameintervals(struct v4l2_int_device *s,
					 struct v4l2_frmivalenum *fival)
{
	struct sensor_data *sensor = s->priv;

	if (fival->index > 0)
		return -EINVAL;
	if (fival->pixel_format != sensor->pix.pixelformat)
		return -EINVAL;
	if ((fival->width != sensor->pix.width)
	    || (fival->height != sensor->pix.height))
		return -EINVAL;

	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->discrete = sensor->streamcap.timeperframe;

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
	strcpy(((struct v4l2_dbg_chip_ident *)id)->match.name,
		"ovisl7998x_mipi_decoder");

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
	struct sensor_data *sensor = s->priv;

	if (fmt->index > 0) /* only 1 pixelformat support so far */
		return -EINVAL;

	fmt->pixelformat = sensor->pix.pixelformat;

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
	struct sensor_data *sensor = s->priv;
	int ret;

	sensor->on = true;

	spin_lock(&lock);
	if (!sensor_on) {
		sensor_on |= 1 << sensor->v_channel;
		spin_unlock(&lock);
		ret = isl7998x_start_mipi(sensor);
		if (ret < 0)
			return ret;
	} else
		spin_unlock(&lock);

	return 0;
}

/*!
 * ioctl_dev_exit - V4L2 sensor interface handler for vidioc_int_dev_exit_num
 * @s: pointer to standard V4L2 device structure
 *
 * Delinitialise the device when slave detaches to the master.
 */
static int ioctl_dev_exit(struct v4l2_int_device *s)
{
	struct sensor_data *sensor = s->priv;

	spin_lock(&lock);
	sensor_on &= ~(1 << sensor->v_channel);
	if (!sensor_on) {
		spin_unlock(&lock);
		isl7998x_stop_mipi(sensor);
	} else
		spin_unlock(&lock);

	return 0;
}

/*!
 * This structure defines all the ioctls for this module and links them to the
 * enumeration.
 */
static struct v4l2_int_ioctl_desc isl7998x_ioctl_desc[] = {
	{vidioc_int_dev_init_num, (v4l2_int_ioctl_func *) ioctl_dev_init},
	{vidioc_int_dev_exit_num, ioctl_dev_exit},
	{vidioc_int_s_power_num, (v4l2_int_ioctl_func *) ioctl_s_power},
	{vidioc_int_g_ifparm_num, (v4l2_int_ioctl_func *) ioctl_g_ifparm},
/*	{vidioc_int_g_needs_reset_num,
				(v4l2_int_ioctl_func *)ioctl_g_needs_reset}, */
/*	{vidioc_int_reset_num, (v4l2_int_ioctl_func *)ioctl_reset}, */
	{vidioc_int_init_num, (v4l2_int_ioctl_func *) ioctl_init},
	{vidioc_int_enum_fmt_cap_num,
				(v4l2_int_ioctl_func *) ioctl_enum_fmt_cap},
/*	{vidioc_int_try_fmt_cap_num,
				(v4l2_int_ioctl_func *)ioctl_try_fmt_cap}, */
	{vidioc_int_g_fmt_cap_num, (v4l2_int_ioctl_func *) ioctl_g_fmt_cap},
/*	{vidioc_int_s_fmt_cap_num, (v4l2_int_ioctl_func *) ioctl_s_fmt_cap}, */
	{vidioc_int_g_parm_num, (v4l2_int_ioctl_func *) ioctl_g_parm},
	{vidioc_int_s_parm_num, (v4l2_int_ioctl_func *) ioctl_s_parm},
/*	{vidioc_int_queryctrl_num, (v4l2_int_ioctl_func *)ioctl_queryctrl}, */
	{vidioc_int_g_ctrl_num, (v4l2_int_ioctl_func *) ioctl_g_ctrl},
	{vidioc_int_s_ctrl_num, (v4l2_int_ioctl_func *) ioctl_s_ctrl},
	{vidioc_int_enum_framesizes_num,
				(v4l2_int_ioctl_func *) ioctl_enum_framesizes},
	{vidioc_int_enum_frameintervals_num,
				(v4l2_int_ioctl_func *) ioctl_enum_frameintervals},
	{vidioc_int_g_chip_ident_num,
				(v4l2_int_ioctl_func *) ioctl_g_chip_ident},
};

static struct v4l2_int_slave isl7998x_slave[SENSOR_NUM] = {
	{
	.ioctls = isl7998x_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(isl7998x_ioctl_desc),
	},

	{
	.ioctls = isl7998x_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(isl7998x_ioctl_desc),
	},

	{
	.ioctls = isl7998x_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(isl7998x_ioctl_desc),
	},

	{
	.ioctls = isl7998x_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(isl7998x_ioctl_desc),
	}
};

static struct v4l2_int_device isl7998x_int_device[SENSOR_NUM] = {
	{
		.module = THIS_MODULE,
		.name = "isl7998x",
		.type = v4l2_int_type_slave,
		.u = {
			.slave = &isl7998x_slave[0],
		},
	}, 

	{
		.module = THIS_MODULE,
		.name = "isl7998x",
		.type = v4l2_int_type_slave,
		.u = {
			.slave = &isl7998x_slave[1],
		},
	}, 

	{
		.module = THIS_MODULE,
		.name = "isl7998x",
		.type = v4l2_int_type_slave,
		.u = {
			.slave = &isl7998x_slave[2],
		},
	}, 

	{
		.module = THIS_MODULE,
		.name = "isl7998x",
		.type = v4l2_int_type_slave,
		.u = {
			.slave = &isl7998x_slave[3],
		},
	}
};

/*!
 * isl7998x I2C probe function
 *
 * @param adapter            struct i2c_adapter *
 * @return  Error code indicating success or failure
 */
static int isl7998x_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct sensor_data *sensor = &isl7998x_data[0];
	int retval;
	const char *name;
	const char *standard;
	unsigned int std_mode;

	spin_lock_init(&lock);

	/* Set initial values for the sensor struct. */
	memset(sensor, 0, sizeof(*sensor));
	sensor->sensor_clk = devm_clk_get(dev, "csi_mclk");
	if (IS_ERR(sensor->sensor_clk)) {
		/* assuming clock enabled by default */
		sensor->sensor_clk = NULL;
		dev_err(dev, "clock-frequency missing or invalid\n");
		return PTR_ERR(sensor->sensor_clk);
	}

	retval = of_property_read_u32(dev->of_node, "mclk", &(sensor->mclk));
	if (retval) {
		dev_err(dev, "mclk missing or invalid\n");
		return retval;
	}

	/* Get video standard to use, auto detection by default */
	std_mode = ISL7998X_STD_AUTO;
	if (!of_property_read_string(dev->of_node, "standard", &standard)) {
		if (!strcmp(standard, "NTSC"))
			std_mode = ISL7998X_STD_NTSC;
		else if (!strcmp(standard, "PAL"))
			std_mode = ISL7998X_STD_PAL;
		else if (!strcmp(standard, "SECAM"))
			std_mode = ISL7998X_STD_SECAM;
		else if (!strcmp(standard, "NTSC-4.43"))
			std_mode = ISL7998X_STD_NTSC_443;
		else if (!strcmp(standard, "PAL-M"))
			std_mode = ISL7998X_STD_PAL_M;
		else if (!strcmp(standard, "PAL-CN"))
			std_mode = ISL7998X_STD_PAL_CN;
		else if (!strcmp(standard, "PAL-60"))
			std_mode = ISL7998X_STD_PAL_60;
		else if (strcmp(standard, "AUTO"))
			pr_warning("Invalid video standard %s,"
				   " using AUTO detection\n", standard);
	}

	clk_prepare_enable(sensor->sensor_clk);

	sensor->i2c_client = client;

	isl7998x_write_reg(0xFF, 0x00);
	chip_id = isl7998x_read_reg(0x00);
	if ((chip_id != CHIP_ID_79985) && (chip_id != CHIP_ID_79987)) {
		pr_warning("isl7998x is not found, chip id reg 0x00 = 0x%x.\n", chip_id);
		clk_disable_unprepare(sensor->sensor_clk);
		return -ENODEV;
	}

	if (chip_id == CHIP_ID_79987) {
		name = "ISL79987";
		g_isl7998x_frametype[0].width = 720;
		g_isl7998x_frametype[0].height = 480;
		g_isl7998x_frametype[1].width = 720;
		g_isl7998x_frametype[1].height = 576;
		g_isl7998x_frametype[0].rate = 30;
		g_isl7998x_frametype[1].rate = 25;
		sensor->pix.field = V4L2_FIELD_INTERLACED;
		sensor->is_mipi_interlaced = 1;
	} else if (chip_id == CHIP_ID_79985) {
		name = "ISL79985";
		g_isl7998x_frametype[0].width = 720;
		g_isl7998x_frametype[0].height = 240;
		g_isl7998x_frametype[1].width = 720;
		g_isl7998x_frametype[1].height = 288;
		g_isl7998x_frametype[0].rate = 60;
		g_isl7998x_frametype[1].rate = 50;
		sensor->pix.field = V4L2_FIELD_ALTERNATE;
	}

	sensor->pix.pixelformat = V4L2_PIX_FMT_UYVY;
	sensor->pix.priv = 1;
	sensor->streamcap.capability = V4L2_CAP_TIMEPERFRAME;
	sensor->streamcap.capturemode = 0;
	sensor->is_mipi = 1;
	isl7998x_update_sensor(sensor, isl7998x_get_frametype_index(std_mode));

	memcpy(&isl7998x_data[1], sensor, sizeof(struct sensor_data));
	memcpy(&isl7998x_data[2], sensor, sizeof(struct sensor_data));
	memcpy(&isl7998x_data[3], sensor, sizeof(struct sensor_data));

	isl7998x_data[1].i2c_client = NULL;
	isl7998x_data[2].i2c_client = NULL;
	isl7998x_data[3].i2c_client = NULL;

	isl7998x_data[0].ipu_id = 0;
	isl7998x_data[0].csi = 0;
	isl7998x_data[0].v_channel = 0;

	isl7998x_data[1].ipu_id = 0;
	isl7998x_data[1].csi = 1;
	isl7998x_data[1].v_channel = 1;

	isl7998x_data[2].ipu_id = 1;
	isl7998x_data[2].csi = 0;
	isl7998x_data[2].v_channel = 2;

	isl7998x_data[3].ipu_id = 1;
	isl7998x_data[3].csi = 1;
	isl7998x_data[3].v_channel = 3;

	isl7998x_int_device[0].priv = &isl7998x_data[0];
	isl7998x_int_device[1].priv = &isl7998x_data[1];
	isl7998x_int_device[2].priv = &isl7998x_data[2];
	isl7998x_int_device[3].priv = &isl7998x_data[3];

	isl7998x_hardware_init(std_mode);

	v4l2_int_device_register(&isl7998x_int_device[0]);
	v4l2_int_device_register(&isl7998x_int_device[1]);
	v4l2_int_device_register(&isl7998x_int_device[2]);
	retval = v4l2_int_device_register(&isl7998x_int_device[3]);

	clk_disable_unprepare(sensor->sensor_clk);

	pr_info("isl7998x_mipi (%s) is found\n", name);
	return retval;
}

/*!
 * isl7998x I2C detach function
 *
 * @param client            struct i2c_client *
 * @return  Error code indicating success or failure
 */
static int isl7998x_remove(struct i2c_client *client)
{
	v4l2_int_device_unregister(&isl7998x_int_device[3]);
	v4l2_int_device_unregister(&isl7998x_int_device[2]);
	v4l2_int_device_unregister(&isl7998x_int_device[1]);
	v4l2_int_device_unregister(&isl7998x_int_device[0]);

	return 0;
}

/*!
 * isl7998x init function
 *
 * @return  Error code indicating success or failure
 */
static __init int isl7998x_init(void)
{
	u8 err;

	err = i2c_add_driver(&isl7998x_i2c_driver);
	if (err != 0)
		pr_err("%s:driver registration failed, error=%d\n",
			__func__, err);

	return err;
}

/*!
 * ISL7998x cleanup function
 *
 * @return  Error code indicating success or failure
 */
static void __exit isl7998x_clean(void)
{
	i2c_del_driver(&isl7998x_i2c_driver);
}

module_init(isl7998x_init);
module_exit(isl7998x_clean);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("ISL7998x Video Decoder Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
MODULE_ALIAS("CSI");
