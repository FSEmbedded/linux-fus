/** \mainpage
*
****************************************************************************
* Copyright (C) 2013 - 2015 Bosch Sensortec GmbH
*
* File : bme280.h
*
* Date : 2015/03/27
*
* Revision : 2.0.4(Pressure and Temperature compensation code revision is 1.1
*               and Humidity compensation code revision is 1.0)
*
* Usage: Sensor Driver for BME280 sensor
*
****************************************************************************
*
* \section License
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*   Redistributions of source code must retain the above copyright
*   notice, this list of conditions and the following disclaimer.
*
*   Redistributions in binary form must reproduce the above copyright
*   notice, this list of conditions and the following disclaimer in the
*   documentation and/or other materials provided with the distribution.
*
*   Neither the name of the copyright holder nor the names of the
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
* OR CONTRIBUTORS BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
* OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
* PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*
* The information provided is believed to be accurate and reliable.
* The copyright holder assumes no responsibility
* for the consequences of use
* of such information nor for any infringement of patents or
* other rights of third parties which may result from its use.
* No license is granted by implication or otherwise under any patent or
* patent rights of the copyright holder.
**************************************************************************/
/*! \file bme280.h
    \brief BME280 Sensor Driver Support Header File */
#ifndef __BME280_H__
#define __BME280_H__

/***************************************************************/
/**\name	COMMON USED CONSTANTS      */
/***************************************************************/
/* shift definitions*/
#define BME280_SHIFT_BIT_POSITION_BY_01_BIT			1
#define BME280_SHIFT_BIT_POSITION_BY_02_BITS			2
#define BME280_SHIFT_BIT_POSITION_BY_03_BITS			3
#define BME280_SHIFT_BIT_POSITION_BY_04_BITS			4
#define BME280_SHIFT_BIT_POSITION_BY_05_BITS			5
#define BME280_SHIFT_BIT_POSITION_BY_07_BITS			7
#define BME280_SHIFT_BIT_POSITION_BY_10_BITS			10
#define BME280_SHIFT_BIT_POSITION_BY_11_BITS			11
#define BME280_SHIFT_BIT_POSITION_BY_12_BITS			12
#define BME280_SHIFT_BIT_POSITION_BY_13_BITS			13
#define BME280_SHIFT_BIT_POSITION_BY_14_BITS			14
#define BME280_SHIFT_BIT_POSITION_BY_15_BITS			15
#define BME280_SHIFT_BIT_POSITION_BY_16_BITS			16
#define BME280_SHIFT_BIT_POSITION_BY_18_BITS			18
#define BME280_SHIFT_BIT_POSITION_BY_20_BITS			20

/****************************************************/
/**\name	POWER MODE DEFINITIONS  */
/***************************************************/
/* Sensor Specific constants */
#define BME280_SLEEP_MODE			0x00
#define BME280_FORCED_MODE			0x01
#define BME280_NORMAL_MODE			0x03
#define BME280_SOFT_RESET_CODE			0xB6

/****************************************************/
/**\name	STANDBY DEFINITIONS  */
/***************************************************/
#define BME280_STANDBY_TIME_1_MS		0x00
#define BME280_STANDBY_TIME_63_MS		0x01
#define BME280_STANDBY_TIME_125_MS		0x02
#define BME280_STANDBY_TIME_250_MS		0x03
#define BME280_STANDBY_TIME_500_MS		0x04
#define BME280_STANDBY_TIME_1000_MS		0x05
#define BME280_STANDBY_TIME_10_MS		0x06
#define BME280_STANDBY_TIME_20_MS		0x07

/****************************************************/
/**\name	OVER SAMPLING DEFINITIONS  */
/***************************************************/
#define BME280_OVERSAMP_SKIPPED			0x00
#define BME280_OVERSAMP_1X			0x01
#define BME280_OVERSAMP_2X			0x02
#define BME280_OVERSAMP_4X			0x03
#define BME280_OVERSAMP_8X			0x04
#define BME280_OVERSAMP_16X			0x05

/****************************************************/
/**\name	FILTER DEFINITIONS  */
/***************************************************/
#define BME280_FILTER_COEFF_OFF			0x00
#define BME280_FILTER_COEFF_2			0x01
#define BME280_FILTER_COEFF_4			0x02
#define BME280_FILTER_COEFF_8			0x03
#define BME280_FILTER_COEFF_16			0x04

/****************************************************/
/**\name	DEFINITIONS FOR ARRAY SIZE OF DATA   */
/***************************************************/
#define	BME280_HUMIDITY_DATA_SIZE		2
#define	BME280_TEMPERATURE_DATA_SIZE		3
#define	BME280_PRESSURE_DATA_SIZE		3

#define	BME280_TEMPERATURE_MSB_DATA		0
#define	BME280_TEMPERATURE_LSB_DATA		1
#define	BME280_TEMPERATURE_XLSB_DATA		2

#define	BME280_PRESSURE_MSB_DATA		0
#define	BME280_PRESSURE_LSB_DATA		1
#define	BME280_PRESSURE_XLSB_DATA	    	2

#define	BME280_HUMIDITY_MSB_DATA		0
#define	BME280_HUMIDITY_LSB_DATA		1

/****************************************************/
/**\name	Mask DEFINITIONS  */
/***************************************************/
#define BME280_MASK_1_0				0x03
#define BME280_MASK_2_0				0x07
#define	BME280_MASK_3_0				0x0F
#define BME280_MASK_4_2				0x1C
#define BME280_MASK_7_5				0xE0
#define	BME280_MASK_7_4				0xF0

/****************************************************/
/**\name	CALIBRATION REGISTER ADDRESS DEFINITIONS  */
/***************************************************/
/*calibration parameters */
#define BME280_TEMPERATURE_CALIB_DIG_T1_LSB_REG		0x88
#define BME280_TEMPERATURE_CALIB_DIG_T1_MSB_REG		0x89
#define BME280_TEMPERATURE_CALIB_DIG_T2_LSB_REG		0x8A
#define BME280_TEMPERATURE_CALIB_DIG_T2_MSB_REG		0x8B
#define BME280_TEMPERATURE_CALIB_DIG_T3_LSB_REG		0x8C
#define BME280_TEMPERATURE_CALIB_DIG_T3_MSB_REG		0x8D
#define BME280_PRESSURE_CALIB_DIG_P1_LSB_REG		0x8E
#define BME280_PRESSURE_CALIB_DIG_P1_MSB_REG		0x8F
#define BME280_PRESSURE_CALIB_DIG_P2_LSB_REG		0x90
#define BME280_PRESSURE_CALIB_DIG_P2_MSB_REG		0x91
#define BME280_PRESSURE_CALIB_DIG_P3_LSB_REG		0x92
#define BME280_PRESSURE_CALIB_DIG_P3_MSB_REG		0x93
#define BME280_PRESSURE_CALIB_DIG_P4_LSB_REG		0x94
#define BME280_PRESSURE_CALIB_DIG_P4_MSB_REG		0x95
#define BME280_PRESSURE_CALIB_DIG_P5_LSB_REG		0x96
#define BME280_PRESSURE_CALIB_DIG_P5_MSB_REG		0x97
#define BME280_PRESSURE_CALIB_DIG_P6_LSB_REG		0x98
#define BME280_PRESSURE_CALIB_DIG_P6_MSB_REG		0x99
#define BME280_PRESSURE_CALIB_DIG_P7_LSB_REG		0x9A
#define BME280_PRESSURE_CALIB_DIG_P7_MSB_REG		0x9B
#define BME280_PRESSURE_CALIB_DIG_P8_LSB_REG		0x9C
#define BME280_PRESSURE_CALIB_DIG_P8_MSB_REG		0x9D
#define BME280_PRESSURE_CALIB_DIG_P9_LSB_REG		0x9E
#define BME280_PRESSURE_CALIB_DIG_P9_MSB_REG		0x9F

#define BME280_HUMIDITY_CALIB_DIG_H1_REG		0xA1

#define BME280_HUMIDITY_CALIB_DIG_H2_LSB_REG		0xE1
#define BME280_HUMIDITY_CALIB_DIG_H2_MSB_REG		0xE2
#define BME280_HUMIDITY_CALIB_DIG_H3_REG		0xE3
#define BME280_HUMIDITY_CALIB_DIG_H4_MSB_REG		0xE4
#define BME280_HUMIDITY_CALIB_DIG_H4_LSB_REG		0xE5
#define BME280_HUMIDITY_CALIB_DIG_H5_MSB_REG		0xE6
#define BME280_HUMIDITY_CALIB_DIG_H6_REG		0xE7

/****************************************************/
/**\name	REGISTER ADDRESS DEFINITIONS  */
/***************************************************/
#define BME280_CHIP_ID_REG		0xD0  /*Chip ID Register */
#define BME280_RST_REG			0xE0  /*Softreset Register */
#define BME280_CTRL_HUMIDITY_REG	0xF2  /*Ctrl Humidity Register*/
#define BME280_STAT_REG			0xF3  /*Status Register */
#define BME280_CTRL_MEAS_REG		0xF4  /*Ctrl Measure Register */
#define BME280_CONFIG_REG		0xF5  /*Configuration Register */
#define BME280_PRESSURE_MSB_REG		0xF7  /*Pressure MSB Register */
#define BME280_PRESSURE_LSB_REG		0xF8  /*Pressure LSB Register */
#define BME280_PRESSURE_XLSB_REG	0xF9  /*Pressure XLSB Register */
#define BME280_TEMPERATURE_MSB_REG	0xFA  /*Temperature MSB Reg */
#define BME280_TEMPERATURE_LSB_REG	0xFB  /*Temperature LSB Reg */
#define BME280_TEMPERATURE_XLSB_REG	0xFC  /*Temperature XLSB Reg */
#define BME280_HUMIDITY_MSB_REG		0xFD  /*Humidity MSB Reg */
#define BME280_HUMIDITY_LSB_REG		0xFE  /*Humidity LSB Reg */

/**************************************************************/
/**\name	STRUCTURE DEFINITIONS                         */
/**************************************************************/
/*!
 * @brief This structure holds all device specific calibration parameters
 */

struct bme280_calibration_param_t {
	u16 dig_T1;/**<calibration T1 data*/
	s16 dig_T2;/**<calibration T2 data*/
	s16 dig_T3;/**<calibration T3 data*/
	u16 dig_P1;/**<calibration P1 data*/
	s16 dig_P2;/**<calibration P2 data*/
	s16 dig_P3;/**<calibration P3 data*/
	s16 dig_P4;/**<calibration P4 data*/
	s16 dig_P5;/**<calibration P5 data*/
	s16 dig_P6;/**<calibration P6 data*/
	s16 dig_P7;/**<calibration P7 data*/
	s16 dig_P8;/**<calibration P8 data*/
	s16 dig_P9;/**<calibration P9 data*/

	u8  dig_H1;/**<calibration H1 data*/
	s16 dig_H2;/**<calibration H2 data*/
	u8  dig_H3;/**<calibration H3 data*/
	s16 dig_H4;/**<calibration H4 data*/
	s16 dig_H5;/**<calibration H5 data*/
	s8  dig_H6;/**<calibration H6 data*/

	s32 t_fine;/**<calibration T_FINE data*/
};
/*!
 * @brief This structure holds BME280 initialization parameters
 */

struct bme280_t {
	struct i2c_client *client;
	struct bme280_calibration_param_t cal_param;

	char *name;

	u8 chip_id; /*< chip id of the sensor*/
	u8 dev_addr; /*< device address of the sensor*/

	u32 oversamp_temperature; /*< temperature over sampling*/
	u32 oversamp_pressure; /*< pressure over sampling*/
	u32 oversamp_humidity; /*< humidity over sampling*/
	u32 t_standby;
	u32 filter;
};

#endif