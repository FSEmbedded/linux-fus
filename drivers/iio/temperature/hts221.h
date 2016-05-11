/*
* include/linux/input/hts221.h
*
* STMicroelectronics HTS221 Relative Humidity and Temperature Sensor module driver
*
* Copyright (C) 2014 STMicroelectronics - HESA BU - Application Team
* (lorenzo.sarchi@st.com)
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, see <http://www.gnu.org/licenses> or
* write to the Free Software * Foundation, Inc., 51 Franklin St, Fifth Floor,
* Boston, MA 02110-1301 USA
*
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
*
*******************************************************************************
*******************************************************************************
Version History.

Revision 0-0-0 06/02/2013
 first revision

Revision 0-0-1 04/04/2014
 second revision

******************************************************************************/

#ifndef __HTS221_H__
#define __HTS221_H__

#define HTS221_DEV_NAME  "hts221"

/************************************************/
/*      Output data                             */
/*************************************************
humidity: relative humidity (m RH)
temperature: celsius degree (m DegC)
*************************************************/

/************************************************/
/*      sysfs data                              */
/*************************************************
        - pollrate->ms
humidity:
        - humidity_resolution->number
temperature:
        - temperature_resolution->number
*************************************************/

/* Default values loaded in probe function */
#define WHOIAM_VALUE		0xbc  /** Who Am I default value */

/* Humidity Sensor Resolution */
#define HTS221_H_RESOLUTION_4    0x00  /* Resolution set to 0.4 %RH */
#define HTS221_H_RESOLUTION_8    0x01  /* Resolution set to 0.3 %RH */
#define HTS221_H_RESOLUTION_16   0x02  /* Resolution set to 0.2 %RH */
#define HTS221_H_RESOLUTION_32   0x03  /* Resolution set to 0.15 %RH */
#define HTS221_H_RESOLUTION_64   0x04  /* Resolution set to 0.1 %RH */
#define HTS221_H_RESOLUTION_128  0x05  /* Resolution set to 0.07 %RH */
#define HTS221_H_RESOLUTION_256  0x06  /* Resolution set to 0.05 %RH */
#define HTS221_H_RESOLUTION_512  0x07  /* Resolution set to 0.03 %RH */

/* Temperature Sensor Resolution */
#define HTS221_T_RESOLUTION_2    0x00  /* Resolution set to 0.08 DegC */
#define HTS221_T_RESOLUTION_4    0x08  /* Resolution set to 0.05 DegC */
#define HTS221_T_RESOLUTION_8    0x10  /* Resolution set to 0.04 DegC */
#define HTS221_T_RESOLUTION_16   0x18  /* Resolution set to 0.03 DegC */
#define HTS221_T_RESOLUTION_32   0x20  /* Resolution set to 0.02 DegC */
#define HTS221_T_RESOLUTION_64   0x28  /* Resolution set to 0.015 DegC */
#define HTS221_T_RESOLUTION_128  0x30  /* Resolution set to 0.01 DegC */
#define HTS221_T_RESOLUTION_256  0x38  /* Resolution set to 0.007 DegC */

/* Humidity and Termometer output data rate ODR */
#define	ODR_ONESH	0x00	/* one shot 	*/
#define	ODR_1_1		0x01	/*  1  Hz 	*/
#define	ODR_7_7		0x02	/*  7  Hz 	*/
#define	ODR_12_12	0x03	/* 12.5Hz 	*/

/* Address registers */
#define REG_WHOAMI_ADDR			0x0f	/* Who am i address register */
#define REG_AV_CONF			0x10	/* AV_CONF address register */
#define REG_CNTRL1_ADDR			0x20	/* CNTRL1 address register */
#define REG_CNTRL2_ADDR			0x21	/* CNTRL2 address register */
#define REG_STATUS			0x27	/* Status address register */
#define HTS221_HUMIDITY_OUT_LSB_REG	0x28	/* Humidity out lsb register */
#define HTS221_HUMIDITY_OUT_MSB_REG	0x29	/* Humidity out msb register */
#define HTS221_TEMPERATURE_OUT_LSB_REG	0x2A	/* Temperature out lsb register */
#define HTS221_TEMPERATURE_OUT_MSB_REG	0x2B	/* Temperature out msb register */

/* Calibration Parameters */
#define HTS221_HUMIDITY_CALIB_DIG_H0_OUT_LSB_REG	0x36
#define HTS221_HUMIDITY_CALIB_DIG_H0_OUT_MSB_REG	0x37
#define HTS221_HUMIDITY_CALIB_DIG_H1_OUT_LSB_REG	0x3a
#define HTS221_HUMIDITY_CALIB_DIG_H1_OUT_MSB_REG	0x3b
#define HTS221_HUMIDITY_CALIB_DIG_H0_rH_REG		0x30
#define HTS221_HUMIDITY_CALIB_DIG_H1_rH_REG		0x31
#define HTS221_TEMPERATURE_CALIB_DIG_T0_OUT_LSB_REG	0x3c
#define HTS221_TEMPERATURE_CALIB_DIG_T0_OUT_MSB_REG	0x3d
#define HTS221_TEMPERATURE_CALIB_DIG_T1_OUT_LSB_REG	0x3e
#define HTS221_TEMPERATURE_CALIB_DIG_T1_OUT_MSB_REG	0x3f
#define HTS221_TEMPERATURE_CALIB_DIG_T0_degC_REG	0x32
#define HTS221_TEMPERATURE_CALIB_DIG_T1_degC_REG	0x33
#define HTS221_TEMPERATURE_CALIB_DIG_T1_T0_MSB_REG	0x35

/* mask */
#define HTS221_MASK_HEATER_ON		0x02
#define HTS221_MASK_ODR			0x03
#define HTS221_MASK_T0_MSB		0x03
#define HTS221_MASK_AVG_H		0x07
#define HTS221_MASK_T1_MSB		0x0c
#define HTS221_MASK_AVG_T		0x38
#define HTS221_MASK_POWER_ON		0x80

/* AVG define which sensor to read out and calc */
#define HTS221_AVG_T			1
#define HTS221_AVG_H			2

/* shift definitions*/
#define HTS221_SHIFT_BIT_POSITION_BY_01_BITS	1
#define HTS221_SHIFT_BIT_POSITION_BY_02_BITS	2
#define HTS221_SHIFT_BIT_POSITION_BY_03_BITS	3

struct hts221_calibration_param_t {
	s16 dig_H0_out;		/*<calibration H0 data*/
	s16 dig_H1_out;		/*<calibration H1 data*/
	u8 dig_H0_rH;		/*<calibration H0_rH data*/
	u8 dig_H1_rH;		/*<calibration H1_rH data*/
	s16 dig_T0_out;		/*<calibration T0 data*/
	s16 dig_T1_out;		/*<calibration T1 data*/
	u8 dig_T0_degC;		/*<calibration T0_degC data*/
	u8 dig_T1_degC;		/*<calibration T1_degC data*/
	u8 dig_T1_T0_msb;	/*<calibration T1_T0_msb data*/
};

struct hts221_platform_data {
        unsigned int poll_interval;
	struct hts221_calibration_param_t cal_param;

        u8 h_resolution;
        u8 t_resolution;

        int (*init)(void);
        void (*exit)(void);
};

#endif  /* __HTS221_H__ */
