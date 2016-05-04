/*
****************************************************************************
* Copyright (C) 2013 - 2015 Bosch Sensortec GmbH
*
* bme280.c
* Date: 2015/03/27
* Revision: 2.0.4(Pressure and Temperature compensation code revision is 1.1
*               and Humidity compensation code revision is 1.0)
*
* Usage: Sensor Driver file for BME280 sensor
*
****************************************************************************
* License:
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
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/iio/iio.h>
#include <linux/of.h>
#include <linux/string.h>
#include <linux/iio/sysfs.h>

#include "bme280.h"

static struct bme280_t *p_bme280; /**< pointer to BME280 */

/*!
 *	@brief This API is used to read uncompensated temperature
 *	in the registers 0xFA, 0xFB and 0xFC
 *	@note 0xFA -> MSB -> bit from 0 to 7
 *	@note 0xFB -> LSB -> bit from 0 to 7
 *	@note 0xFC -> LSB -> bit from 4 to 7
 *
 * @param v_uncomp_temperature_s32 : The value of uncompensated temperature
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
static int bme280_read_uncomp_temperature(s32 *v_uncomp_temperature_s32)
{
	int ret = 0;
	/* Array holding the MSB and LSb value
	a_data_u8r[0] - Temperature MSB
	a_data_u8r[1] - Temperature LSB
	a_data_u8r[2] - Temperature XLSB
	*/
	u8 a_data_u8r[BME280_TEMPERATURE_DATA_SIZE] = {0, 0, 0};

	a_data_u8r[0] = i2c_smbus_read_byte_data(p_bme280->client,
						 BME280_TEMPERATURE_MSB_REG);
	a_data_u8r[1] = i2c_smbus_read_byte_data(p_bme280->client,
						 BME280_TEMPERATURE_LSB_REG);
	a_data_u8r[2] = i2c_smbus_read_byte_data(p_bme280->client,
						 BME280_TEMPERATURE_XLSB_REG);

	*v_uncomp_temperature_s32 = (s32)(((
			(u32)(a_data_u8r[BME280_TEMPERATURE_MSB_DATA])) <<
				BME280_SHIFT_BIT_POSITION_BY_12_BITS) | (((u32)
				(a_data_u8r[BME280_TEMPERATURE_LSB_DATA])) <<
				BME280_SHIFT_BIT_POSITION_BY_04_BITS) | ((u32)
				a_data_u8r[BME280_TEMPERATURE_XLSB_DATA] >>
					BME280_SHIFT_BIT_POSITION_BY_04_BITS));

	return ret;
}

/*!
 * @brief Reads actual temperature from uncompensated temperature
 * @note Returns the value in 0.01 degree Centigrade
 * Output value of "5123" equals 51.23 DegC.
 *
 *
 *
 *  @param  v_uncomp_temperature_s32 : value of uncompensated temperature
 *
 *
 *  @return Returns the actual temperature
 *
*/
s32 bme280_compensate_temperature_int32(s32 v_uncomp_temperature_s32)
{
	s32 v_x1_u32r = 0;
	s32 v_x2_u32r = 0;
	s32 temperature = 0;

	/* calculate x1*/
	v_x1_u32r  =
	((((v_uncomp_temperature_s32
	>> BME280_SHIFT_BIT_POSITION_BY_03_BITS) -
	((s32)p_bme280->cal_param.dig_T1
	<< BME280_SHIFT_BIT_POSITION_BY_01_BIT))) *
	((s32)p_bme280->cal_param.dig_T2)) >>
	BME280_SHIFT_BIT_POSITION_BY_11_BITS;
	/* calculate x2*/
	v_x2_u32r  = (((((v_uncomp_temperature_s32
	>> BME280_SHIFT_BIT_POSITION_BY_04_BITS) -
	((s32)p_bme280->cal_param.dig_T1))
	* ((v_uncomp_temperature_s32 >> BME280_SHIFT_BIT_POSITION_BY_04_BITS) -
	((s32)p_bme280->cal_param.dig_T1)))
	>> BME280_SHIFT_BIT_POSITION_BY_12_BITS) *
	((s32)p_bme280->cal_param.dig_T3))
	>> BME280_SHIFT_BIT_POSITION_BY_14_BITS;
	/* calculate t_fine*/
	p_bme280->cal_param.t_fine = v_x1_u32r + v_x2_u32r;
	/* calculate temperature*/
	temperature  = (p_bme280->cal_param.t_fine * 5 + 128)
	>> BME280_SHIFT_BIT_POSITION_BY_08_BITS;

	return temperature;
}

/*!
 *	@brief This API is used to read uncompensated pressure.
 *	in the registers 0xF7, 0xF8 and 0xF9
 *	@note 0xF7 -> MSB -> bit from 0 to 7
 *	@note 0xF8 -> LSB -> bit from 0 to 7
 *	@note 0xF9 -> LSB -> bit from 4 to 7
 *
 *
 *
 *	@param v_uncomp_pressure_s32 : The value of uncompensated pressure
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
static int bme280_read_uncomp_pressure(s32 *v_uncomp_pressure_s32)
{
	int ret = 0;
	/* Array holding the MSB and LSb value
	a_data_u8[0] - Pressure MSB
	a_data_u8[1] - Pressure LSB
	a_data_u8[2] - Pressure XLSB
	*/
	u8 a_data_u8[BME280_PRESSURE_DATA_SIZE] = {0, 0, 0};

	a_data_u8[0] = i2c_smbus_read_byte_data(p_bme280->client,
						BME280_PRESSURE_MSB_REG);
	a_data_u8[1] = i2c_smbus_read_byte_data(p_bme280->client,
						BME280_PRESSURE_LSB_REG);
	a_data_u8[2] = i2c_smbus_read_byte_data(p_bme280->client,
						BME280_PRESSURE_XLSB_REG);

	*v_uncomp_pressure_s32 = (s32)((((u32)
				(a_data_u8[BME280_PRESSURE_MSB_DATA])) <<
				BME280_SHIFT_BIT_POSITION_BY_12_BITS) | (((u32)
				(a_data_u8[BME280_PRESSURE_LSB_DATA])) <<
				BME280_SHIFT_BIT_POSITION_BY_04_BITS) | ((u32)
					a_data_u8[BME280_PRESSURE_XLSB_DATA] >>
					BME280_SHIFT_BIT_POSITION_BY_04_BITS));

	return ret;
}

/*!
 * @brief Reads actual pressure from uncompensated pressure
 * @note Returns the value in Pascal(Pa)
 * Output value of "96386" equals 96386 Pa =
 * 963.86 hPa = 963.86 millibar
 *
 *
 *
 *  @param v_uncomp_pressure_s32 : value of uncompensated pressure
 *
 *
 *
 *  @return Return the actual pressure output as u32
 *
*/
u32 bme280_compensate_pressure_int32(s32 v_uncomp_pressure_s32)
{
	s32 v_x1_u32 = 0;
	s32 v_x2_u32 = 0;
	u32 v_pressure_u32 = 0;

	/* calculate x1*/
	v_x1_u32 = (((s32)p_bme280->cal_param.t_fine)
	>> BME280_SHIFT_BIT_POSITION_BY_01_BIT) - (s32)64000;
	/* calculate x2*/
	v_x2_u32 = (((v_x1_u32 >> BME280_SHIFT_BIT_POSITION_BY_02_BITS)
	* (v_x1_u32 >> BME280_SHIFT_BIT_POSITION_BY_02_BITS)
	) >> BME280_SHIFT_BIT_POSITION_BY_11_BITS)
	* ((s32)p_bme280->cal_param.dig_P6);
	/* calculate x2*/
	v_x2_u32 = v_x2_u32 + ((v_x1_u32 *
	((s32)p_bme280->cal_param.dig_P5))
	<< BME280_SHIFT_BIT_POSITION_BY_01_BIT);
	/* calculate x2*/
	v_x2_u32 = (v_x2_u32 >> BME280_SHIFT_BIT_POSITION_BY_02_BITS) +
	(((s32)p_bme280->cal_param.dig_P4)
	<< BME280_SHIFT_BIT_POSITION_BY_16_BITS);
	/* calculate x1*/
	v_x1_u32 = (((p_bme280->cal_param.dig_P3 *
	(((v_x1_u32 >> BME280_SHIFT_BIT_POSITION_BY_02_BITS) *
	(v_x1_u32 >> BME280_SHIFT_BIT_POSITION_BY_02_BITS))
	>> BME280_SHIFT_BIT_POSITION_BY_13_BITS))
	>> BME280_SHIFT_BIT_POSITION_BY_03_BITS) +
	((((s32)p_bme280->cal_param.dig_P2) *
	v_x1_u32) >> BME280_SHIFT_BIT_POSITION_BY_01_BIT))
	>> BME280_SHIFT_BIT_POSITION_BY_18_BITS;
	/* calculate x1*/
	v_x1_u32 = ((((32768 + v_x1_u32)) *
	((s32)p_bme280->cal_param.dig_P1))
	>> BME280_SHIFT_BIT_POSITION_BY_15_BITS);
	/* calculate pressure*/
	v_pressure_u32 =
	(((u32)(((s32)1048576) - v_uncomp_pressure_s32)
	- (v_x2_u32 >> BME280_SHIFT_BIT_POSITION_BY_12_BITS))) * 3125;
	if (v_pressure_u32
	< 0x80000000)
		/* Avoid exception caused by division by zero */
		if (v_x1_u32 != 0)
			v_pressure_u32 =
			(v_pressure_u32
			<< BME280_SHIFT_BIT_POSITION_BY_01_BIT) /
			((u32)v_x1_u32);
		else
			return 0;
	else
		/* Avoid exception caused by division by zero */
		if (v_x1_u32 != 0)
			v_pressure_u32 = (v_pressure_u32
			/ (u32)v_x1_u32) * 2;
		else
			return 0;

		v_x1_u32 = (((s32)p_bme280->cal_param.dig_P9) *
		((s32)(((v_pressure_u32 >> BME280_SHIFT_BIT_POSITION_BY_03_BITS)
		* (v_pressure_u32 >> BME280_SHIFT_BIT_POSITION_BY_03_BITS))
		>> BME280_SHIFT_BIT_POSITION_BY_13_BITS)))
		>> BME280_SHIFT_BIT_POSITION_BY_12_BITS;
		v_x2_u32 = (((s32)(v_pressure_u32
		>> BME280_SHIFT_BIT_POSITION_BY_02_BITS)) *
		((s32)p_bme280->cal_param.dig_P8))
		>> BME280_SHIFT_BIT_POSITION_BY_13_BITS;
		v_pressure_u32 = (u32)((s32)v_pressure_u32 +
		((v_x1_u32 + v_x2_u32 + p_bme280->cal_param.dig_P7)
		>> BME280_SHIFT_BIT_POSITION_BY_04_BITS));

	return v_pressure_u32;
}

/*!
 *	@brief This API is used to read uncompensated humidity.
 *	in the registers 0xF7, 0xF8 and 0xF9
 *	@note 0xFD -> MSB -> bit from 0 to 7
 *	@note 0xFE -> LSB -> bit from 0 to 7
 *
 *
 *
 *	@param v_uncomp_humidity_s32 : The value of uncompensated humidity
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
static int bme280_read_uncomp_humidity(s32 *v_uncomp_humidity_s32)
{
	int ret = 0;
	/* Array holding the MSB and LSb value
	a_data_u8[0] - Humidity MSB
	a_data_u8[1] - Humidity LSB
	*/
	u8 a_data_u8[BME280_HUMIDITY_DATA_SIZE] = {0, 0};

	a_data_u8[0] = i2c_smbus_read_byte_data(p_bme280->client,
						BME280_HUMIDITY_MSB_REG);
	a_data_u8[1] = i2c_smbus_read_byte_data(p_bme280->client,
						BME280_HUMIDITY_LSB_REG);

	*v_uncomp_humidity_s32 = (s32)((((u32)
				(a_data_u8[BME280_HUMIDITY_MSB_DATA])) <<
	BME280_SHIFT_BIT_POSITION_BY_08_BITS) |
				((u32)(a_data_u8[BME280_HUMIDITY_LSB_DATA])));

	return ret;
}

/*!
 * @brief Reads actual humidity from uncompensated humidity
 * @note Returns the value in %rH as unsigned 32bit integer
 * in Q22.10 format(22 integer 10 fractional bits).
 * @note An output value of 42313
 * represents 42313 / 1024 = 41.321 %rH
 *
 *
 *
 *  @param  v_uncomp_humidity_s32: value of uncompensated humidity
 *
 *  @return Return the actual relative humidity output as u32
 *
*/
u32 bme280_compensate_humidity_int32(s32 v_uncomp_humidity_s32)
{
	s32 v_x1_u32 = 0;

	/* calculate x1 */
	v_x1_u32 = (p_bme280->cal_param.t_fine - ((s32)76800));
	/* calculate x1 */
	v_x1_u32 = (((((v_uncomp_humidity_s32
	<< BME280_SHIFT_BIT_POSITION_BY_14_BITS) -
	(((s32)p_bme280->cal_param.dig_H4)
	<< BME280_SHIFT_BIT_POSITION_BY_20_BITS) -
	(((s32)p_bme280->cal_param.dig_H5) * v_x1_u32)) +
	((s32)16384)) >> BME280_SHIFT_BIT_POSITION_BY_15_BITS)
	* (((((((v_x1_u32 *
	((s32)p_bme280->cal_param.dig_H6))
	>> BME280_SHIFT_BIT_POSITION_BY_10_BITS) *
	(((v_x1_u32 * ((s32)p_bme280->cal_param.dig_H3))
	>> BME280_SHIFT_BIT_POSITION_BY_11_BITS) + ((s32)32768)))
	>> BME280_SHIFT_BIT_POSITION_BY_10_BITS) + ((s32)2097152)) *
	((s32)p_bme280->cal_param.dig_H2) + 8192) >> 14));
	v_x1_u32 = (v_x1_u32 - (((((v_x1_u32
	>> BME280_SHIFT_BIT_POSITION_BY_15_BITS) *
	(v_x1_u32 >> BME280_SHIFT_BIT_POSITION_BY_15_BITS))
	>> BME280_SHIFT_BIT_POSITION_BY_07_BITS) *
	((s32)p_bme280->cal_param.dig_H1))
	>> BME280_SHIFT_BIT_POSITION_BY_04_BITS));
	v_x1_u32 = (v_x1_u32 < 0 ? 0 : v_x1_u32);
	v_x1_u32 = (v_x1_u32 > 419430400 ?
	419430400 : v_x1_u32);

	return (u32)(v_x1_u32 >> BME280_SHIFT_BIT_POSITION_BY_12_BITS);
}

/*!
 *	@brief This API is used to
 *	calibration parameters used for calculation in the registers
 *
 *  parameter | Register address |   bit
 *------------|------------------|----------------
 *	dig_T1    |  0x88 and 0x89   | from 0 : 7 to 8: 15
 *	dig_T2    |  0x8A and 0x8B   | from 0 : 7 to 8: 15
 *	dig_T3    |  0x8C and 0x8D   | from 0 : 7 to 8: 15
 *	dig_P1    |  0x8E and 0x8F   | from 0 : 7 to 8: 15
 *	dig_P2    |  0x90 and 0x91   | from 0 : 7 to 8: 15
 *	dig_P3    |  0x92 and 0x93   | from 0 : 7 to 8: 15
 *	dig_P4    |  0x94 and 0x95   | from 0 : 7 to 8: 15
 *	dig_P5    |  0x96 and 0x97   | from 0 : 7 to 8: 15
 *	dig_P6    |  0x98 and 0x99   | from 0 : 7 to 8: 15
 *	dig_P7    |  0x9A and 0x9B   | from 0 : 7 to 8: 15
 *	dig_P8    |  0x9C and 0x9D   | from 0 : 7 to 8: 15
 *	dig_P9    |  0x9E and 0x9F   | from 0 : 7 to 8: 15
 *	dig_H1    |         0xA1     | from 0 to 7
 *	dig_H2    |  0xE1 and 0xE2   | from 0 : 7 to 8: 15
 *	dig_H3    |         0xE3     | from 0 to 7
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
static int bme280_get_calib_param(void)
{
	int ret = 0;

	p_bme280->cal_param.dig_T1 = (i2c_smbus_read_byte_data(
		p_bme280->client, BME280_TEMPERATURE_CALIB_DIG_T1_LSB_REG) |
		(i2c_smbus_read_byte_data (p_bme280->client,
		BME280_TEMPERATURE_CALIB_DIG_T1_MSB_REG) <<
					BME280_SHIFT_BIT_POSITION_BY_08_BITS));

	p_bme280->cal_param.dig_T2 = (i2c_smbus_read_byte_data(
		p_bme280->client, BME280_TEMPERATURE_CALIB_DIG_T2_LSB_REG) |
		(i2c_smbus_read_byte_data (p_bme280->client,
		BME280_TEMPERATURE_CALIB_DIG_T2_MSB_REG) <<
					BME280_SHIFT_BIT_POSITION_BY_08_BITS));

	p_bme280->cal_param.dig_T3 = (i2c_smbus_read_byte_data(
		p_bme280->client, BME280_TEMPERATURE_CALIB_DIG_T3_LSB_REG) |
		(i2c_smbus_read_byte_data (p_bme280->client,
		BME280_TEMPERATURE_CALIB_DIG_T3_MSB_REG) <<
					BME280_SHIFT_BIT_POSITION_BY_08_BITS));

	p_bme280->cal_param.dig_P1 = (i2c_smbus_read_byte_data(
		p_bme280->client, BME280_PRESSURE_CALIB_DIG_P1_LSB_REG) |
		(i2c_smbus_read_byte_data (p_bme280->client,
		BME280_PRESSURE_CALIB_DIG_P1_MSB_REG) <<
					BME280_SHIFT_BIT_POSITION_BY_08_BITS));

	p_bme280->cal_param.dig_P2 = (i2c_smbus_read_byte_data(
		p_bme280->client, BME280_PRESSURE_CALIB_DIG_P2_LSB_REG) |
		(i2c_smbus_read_byte_data (p_bme280->client,
		BME280_PRESSURE_CALIB_DIG_P2_MSB_REG) <<
					BME280_SHIFT_BIT_POSITION_BY_08_BITS));

	p_bme280->cal_param.dig_P3 = (i2c_smbus_read_byte_data(
		p_bme280->client, BME280_PRESSURE_CALIB_DIG_P3_LSB_REG) |
		(i2c_smbus_read_byte_data (p_bme280->client,
		BME280_PRESSURE_CALIB_DIG_P3_MSB_REG) <<
					BME280_SHIFT_BIT_POSITION_BY_08_BITS));

	p_bme280->cal_param.dig_P4 = (i2c_smbus_read_byte_data(
		p_bme280->client, BME280_PRESSURE_CALIB_DIG_P4_LSB_REG) |
		(i2c_smbus_read_byte_data (p_bme280->client,
		BME280_PRESSURE_CALIB_DIG_P4_MSB_REG) <<
					BME280_SHIFT_BIT_POSITION_BY_08_BITS));

	p_bme280->cal_param.dig_P5 = (i2c_smbus_read_byte_data(
		p_bme280->client, BME280_PRESSURE_CALIB_DIG_P5_LSB_REG) |
		(i2c_smbus_read_byte_data (p_bme280->client,
		BME280_PRESSURE_CALIB_DIG_P5_MSB_REG) <<
					BME280_SHIFT_BIT_POSITION_BY_08_BITS));

	p_bme280->cal_param.dig_P6 = (i2c_smbus_read_byte_data(
		p_bme280->client, BME280_PRESSURE_CALIB_DIG_P6_LSB_REG) |
		(i2c_smbus_read_byte_data (p_bme280->client,
		BME280_PRESSURE_CALIB_DIG_P6_MSB_REG) <<
					BME280_SHIFT_BIT_POSITION_BY_08_BITS));

	p_bme280->cal_param.dig_P7 = (i2c_smbus_read_byte_data(
		p_bme280->client, BME280_PRESSURE_CALIB_DIG_P7_LSB_REG) |
		(i2c_smbus_read_byte_data (p_bme280->client,
		BME280_PRESSURE_CALIB_DIG_P7_MSB_REG) <<
					BME280_SHIFT_BIT_POSITION_BY_08_BITS));

	p_bme280->cal_param.dig_P8 = (i2c_smbus_read_byte_data(
		p_bme280->client, BME280_PRESSURE_CALIB_DIG_P8_LSB_REG) |
		(i2c_smbus_read_byte_data (p_bme280->client,
		BME280_PRESSURE_CALIB_DIG_P8_MSB_REG) <<
					BME280_SHIFT_BIT_POSITION_BY_08_BITS));

	p_bme280->cal_param.dig_P9 = (i2c_smbus_read_byte_data(
		p_bme280->client, BME280_PRESSURE_CALIB_DIG_P9_LSB_REG) |
		(i2c_smbus_read_byte_data (p_bme280->client,
		BME280_PRESSURE_CALIB_DIG_P9_MSB_REG) <<
					BME280_SHIFT_BIT_POSITION_BY_08_BITS));

	p_bme280->cal_param.dig_H1 = i2c_smbus_read_byte_data(p_bme280->client,
					BME280_HUMIDITY_CALIB_DIG_H1_REG);

	p_bme280->cal_param.dig_H2 = (i2c_smbus_read_byte_data(
		p_bme280->client, BME280_HUMIDITY_CALIB_DIG_H2_LSB_REG) |
		(i2c_smbus_read_byte_data(p_bme280->client,
		BME280_HUMIDITY_CALIB_DIG_H2_MSB_REG) <<
					BME280_SHIFT_BIT_POSITION_BY_08_BITS));

	p_bme280->cal_param.dig_H3 = i2c_smbus_read_byte_data(p_bme280->client,
					BME280_HUMIDITY_CALIB_DIG_H3_REG);

	p_bme280->cal_param.dig_H4 = ((i2c_smbus_read_byte_data(
		p_bme280->client, BME280_HUMIDITY_CALIB_DIG_H4_MSB_REG) <<
		BME280_SHIFT_BIT_POSITION_BY_04_BITS) |
		(i2c_smbus_read_byte_data(p_bme280->client,
		BME280_HUMIDITY_CALIB_DIG_H4_LSB_REG) & BME280_MASK_3_0));

	p_bme280->cal_param.dig_H5 = ((i2c_smbus_read_byte_data(
		p_bme280->client, BME280_HUMIDITY_CALIB_DIG_H4_LSB_REG) &&
		BME280_MASK_7_4) | (i2c_smbus_read_byte_data(p_bme280->client,
		BME280_HUMIDITY_CALIB_DIG_H5_MSB_REG) <<
					BME280_SHIFT_BIT_POSITION_BY_04_BITS));

	p_bme280->cal_param.dig_H6 = i2c_smbus_read_byte_data(p_bme280->client,
					BME280_HUMIDITY_CALIB_DIG_H6_REG);
	return ret;
}

static int convert_oversampling_to_val(u8 val)
{
	if(val == 0x05)
		return 16;
	else if(val == 0x04)
		return 8;
	else if(val == 0x03)
		return 4;
	else if(val == 0x02)
		return 2;
	else if(val == 0x01)
		return 1;
	else
		return 0;
}

static u8 convert_oversampling_to_reg(u32 val)
{
	if(val > 12)
		return BME280_OVERSAMP_16X;
	else if(val > 6)
		return BME280_OVERSAMP_8X;
	else if(val > 2)
		return BME280_OVERSAMP_4X;
	else if(val == 2)
		return BME280_OVERSAMP_2X;
	else if(val == 1)
		return BME280_OVERSAMP_1X;
	else
		return BME280_OVERSAMP_SKIPPED;
}

/*!
 *	@brief This API is used to set
 *	the temperature oversampling setting in the register 0xF4
 *	bits from 5 to 7
 *
 *	value               |   Temperature oversampling
 * ---------------------|---------------------------------
 *	0x00                | Skipped
 *	0x01                | BME280_OVERSAMP_1X
 *	0x02                | BME280_OVERSAMP_2X
 *	0x03                | BME280_OVERSAMP_4X
 *	0x04                | BME280_OVERSAMP_8X
 *	0x05,0x06 and 0x07  | BME280_OVERSAMP_16X
 *
 *
 *  @param v_value_u8 : The value of temperature over sampling
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
static int bme280_set_oversamp_temperature(void)
{
	int ret = 0;
	u8 v_data_u8 = 0;
	u8 oversamp_temp = 0;

	oversamp_temp = convert_oversampling_to_reg(
					p_bme280->oversamp_temperature);

	v_data_u8 = i2c_smbus_read_byte_data(p_bme280->client,
					      BME280_CTRL_MEAS_REG);
	if(v_data_u8 < 0)
		return -EFAULT;

	v_data_u8 &= ~BME280_MASK_7_5;

	v_data_u8 |= ((oversamp_temp << BME280_SHIFT_BIT_POSITION_BY_05_BITS) &
							BME280_MASK_7_5);

	i2c_smbus_write_byte_data(p_bme280->client, BME280_CTRL_MEAS_REG,
								v_data_u8);

	return ret;
}

/*!
 *	@brief This API is used to get
 *	the pressure oversampling setting in the register 0xF4
 *	bits from 2 to 4
 *
 *	value              | Pressure oversampling
 * --------------------|--------------------------
 *	0x00               | Skipped
 *	0x01               | BME280_OVERSAMP_1X
 *	0x02               | BME280_OVERSAMP_2X
 *	0x03               | BME280_OVERSAMP_4X
 *	0x04               | BME280_OVERSAMP_8X
 *	0x05,0x06 and 0x07 | BME280_OVERSAMP_16X
 *
 *
 *  @param v_value_u8 : The value of pressure oversampling
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
static int bme280_set_oversamp_pressure(void)
{
	int ret = 0;
	u8 v_data_u8 = 0;
	u8 oversamp_press = 0;

	oversamp_press = convert_oversampling_to_reg(
						p_bme280->oversamp_pressure);

	v_data_u8 = i2c_smbus_read_byte_data(p_bme280->client,
					      BME280_CTRL_MEAS_REG);
	if(v_data_u8 < 0)
		return -EFAULT;

	v_data_u8 &= ~BME280_MASK_4_2;

	v_data_u8 |= ((oversamp_press <<
		BME280_SHIFT_BIT_POSITION_BY_02_BITS) & BME280_MASK_4_2);

	i2c_smbus_write_byte_data(p_bme280->client, BME280_CTRL_MEAS_REG,
								v_data_u8);
	return ret;
}

/*!
 *	@brief This API is used to set
 *	the humidity oversampling setting in the register 0xF2
 *	bits from 0 to 2
 *
 *	value               | Humidity oversampling
 * ---------------------|-------------------------
 *	0x00                | Skipped
 *	0x01                | BME280_OVERSAMP_1X
 *	0x02                | BME280_OVERSAMP_2X
 *	0x03                | BME280_OVERSAMP_4X
 *	0x04                | BME280_OVERSAMP_8X
 *	0x05,0x06 and 0x07  | BME280_OVERSAMP_16X
 *
 *
 *  @param  v_value_u8 : The value of humidity over sampling
 *
 *
 *
 * @note The "BME280_CTRL_HUMIDITY_REG"
 * register sets the humidity
 * data acquisition options of the device.
 * @note changes to this registers only become
 * effective after a write operation to
 * "BME280_CTRL_MEAS_REG" register.
 * @note In the code automated reading and writing of
 *	"BME280_CTRL_HUMIDITY_REG"
 * @note register first set the
 * "BME280_CTRL_HUMIDITY_REG"
 *  and then read and write
 *  the "BME280_CTRL_MEAS_REG" register in the function.
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
static int bme280_set_oversamp_humidity(void)
{
	int ret = 0;
	u8 v_data_u8 = 0;
	u8 oversamp_hum = 0;

	oversamp_hum = convert_oversampling_to_reg(
						p_bme280->oversamp_humidity);

	v_data_u8 = i2c_smbus_read_byte_data(p_bme280->client,
					      BME280_CTRL_HUMIDITY_REG);
	if(v_data_u8 < 0)
		return -EFAULT;

	v_data_u8 &= ~BME280_MASK_2_0;

	v_data_u8 |= (oversamp_hum & BME280_MASK_2_0);

	i2c_smbus_write_byte_data(p_bme280->client, BME280_CTRL_HUMIDITY_REG,
								v_data_u8);

	/* to make change effective write to ctrl_meas register */
	v_data_u8 = i2c_smbus_read_byte_data(p_bme280->client,
							BME280_CTRL_MEAS_REG);
	if(v_data_u8 < 0)
		return -EFAULT;

	i2c_smbus_write_byte_data(p_bme280->client, BME280_CTRL_MEAS_REG,
								v_data_u8);

	return ret;
}

/*!
 *	@brief This API used to set the
 *	Operational Mode from the sensor in the register 0xF4 bit 0 and 1
 *
 *
 *
 *	@param v_power_mode_u8 : The value of power mode
 *  value           |    mode
 * -----------------|------------------
 *	0x00            | BME280_SLEEP_MODE
 *	0x01 and 0x02   | BME280_FORCED_MODE
 *	0x03            | BME280_NORMAL_MODE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
static int bme280_set_power_mode(u8 v_power_mode_u8)
{
	int ret = 0;
	u8 v_mode_u8r = 0;

	v_mode_u8r = i2c_smbus_read_byte_data(p_bme280->client,
							BME280_CTRL_MEAS_REG);
	if(v_mode_u8r < 0)
		return -EFAULT;

	v_mode_u8r &= ~BME280_MASK_1_0;
	v_mode_u8r |= (v_power_mode_u8 & BME280_MASK_1_0);

	i2c_smbus_write_byte_data(p_bme280->client, BME280_CTRL_MEAS_REG,
								v_mode_u8r);

	return ret;
}

/*!
 *	@brief This API is used to write filter setting
 *	in the register 0xF5 bit 3 and 4
 *
 *
 *
 *	@param v_value_u8 : The value of IIR filter coefficient
 *
 *	value	    |	Filter coefficient
 * -------------|-------------------------
 *	0x00        | BME280_FILTER_COEFF_OFF
 *	0x01        | BME280_FILTER_COEFF_2
 *	0x02        | BME280_FILTER_COEFF_4
 *	0x03        | BME280_FILTER_COEFF_8
 *	0x04        | BME280_FILTER_COEFF_16
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
static int bme280_set_filter(void)
{
	int ret = 0;
	u8 v_data_u8 = 0;
	u8 filter = 0;

	if(p_bme280->filter > 12)
		filter = BME280_FILTER_COEFF_16;
	else if(p_bme280->filter > 6)
		filter = BME280_FILTER_COEFF_8;
	else if(p_bme280->filter > 2)
		filter = BME280_FILTER_COEFF_4;
	else if(p_bme280->filter == 2)
		filter = BME280_FILTER_COEFF_2;
	else
		filter = BME280_FILTER_COEFF_OFF;

	v_data_u8 = i2c_smbus_read_byte_data(p_bme280->client,
					      BME280_CONFIG_REG);
	if(v_data_u8 < 0)
		return -EFAULT;

	v_data_u8 &= ~BME280_MASK_4_2;
	v_data_u8 |= ((filter << BME280_SHIFT_BIT_POSITION_BY_02_BITS) &
							BME280_MASK_4_2);

	i2c_smbus_write_byte_data(p_bme280->client, BME280_CONFIG_REG,
								v_data_u8);

	return ret;
}

/*!
 *	@brief This API used to write the
 *	standby duration time from the sensor in the register 0xF5 bit 5 to 7
 *
 *	@param v_standby_durn_u8 : The value of standby duration time value.
 *  value       | standby duration
 * -------------|-----------------------
 *    0x00      | BME280_STANDBY_TIME_1_MS
 *    0x01      | BME280_STANDBY_TIME_63_MS
 *    0x02      | BME280_STANDBY_TIME_125_MS
 *    0x03      | BME280_STANDBY_TIME_250_MS
 *    0x04      | BME280_STANDBY_TIME_500_MS
 *    0x05      | BME280_STANDBY_TIME_1000_MS
 *    0x06      | BME280_STANDBY_TIME_2000_MS
 *    0x07      | BME280_STANDBY_TIME_4000_MS
 *
 *	@note Normal mode comprises an automated perpetual
 *	cycling between an (active)
 *	Measurement period and an (inactive) standby period.
 *	@note The standby time is determined by
 *	the contents of the register t_sb.
 *	Standby time can be set using BME280_STANDBY_TIME_125_MS.
 *
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/

static int bme280_set_standby_durn(void)
{
	int ret = 0;
	u8 v_data_u8 = 0;
	u8 t_standby = 0;

	if(p_bme280->t_standby > 3000)
		t_standby = BME280_STANDBY_TIME_20_MS;
	else if(p_bme280->t_standby > 1500)
		t_standby = BME280_STANDBY_TIME_10_MS;
	else if(p_bme280->t_standby > 750)
		t_standby = BME280_STANDBY_TIME_1000_MS;
	else if(p_bme280->t_standby > 375)
		t_standby = BME280_STANDBY_TIME_500_MS;
	else if(p_bme280->t_standby > 187)
		t_standby = BME280_STANDBY_TIME_250_MS;
	else if(p_bme280->t_standby > 94)
		t_standby = BME280_STANDBY_TIME_125_MS;
	else if(p_bme280->t_standby > 32)
		t_standby = BME280_STANDBY_TIME_63_MS;
	else
		t_standby = BME280_STANDBY_TIME_1_MS;

	v_data_u8 = i2c_smbus_read_byte_data(p_bme280->client,
							BME280_CONFIG_REG);
	if(v_data_u8 < 0)
		return -EFAULT;

	v_data_u8 &= ~BME280_MASK_7_5;
	v_data_u8 |= ((t_standby << BME280_SHIFT_BIT_POSITION_BY_05_BITS) &
							BME280_MASK_7_5);

	i2c_smbus_write_byte_data(p_bme280->client, BME280_CONFIG_REG,
								v_data_u8);

	return ret;
}

static int bme280_read_raw(struct iio_dev *iio_dev,
			const struct iio_chan_spec *chan,
			int *val, int *val2, long m)
{
	int ret = 0;
	s32 uncomp_temp = 0;
	s32 uncomp_press = 0;
	s32 uncomp_hum = 0;
	if(chan->type == IIO_TEMP)
	{
		dev_info(&p_bme280->client->dev, "temperature in Centigrade\n");
		bme280_read_uncomp_temperature(&uncomp_temp);
		uncomp_temp =
		bme280_compensate_temperature_int32(uncomp_temp);
		*val = uncomp_temp;
		ret = IIO_VAL_INT;
	}
	else if(chan->type == IIO_PRESSURE)
	{
		dev_info(&p_bme280->client->dev, ">pressure in Pascal\n");
		bme280_read_uncomp_pressure(&uncomp_press);
		uncomp_press =
		bme280_compensate_pressure_int32(uncomp_press);
		*val = uncomp_press;
		ret = IIO_VAL_INT;
	}
	else if(chan->type == IIO_HUMIDITYRELATIVE)
	{
		dev_info(&p_bme280->client->dev, "humidity value / 1024 = xxx %%rH\n");
		bme280_read_uncomp_humidity(&uncomp_hum);
		uncomp_hum =
		bme280_compensate_humidity_int32(uncomp_hum);
		*val = uncomp_hum;
		ret = IIO_VAL_INT;
	}
	return ret;
}

#if IS_ENABLED(CONFIG_OF)
static struct bme280_t* bme280_dt_init(struct bme280_t *bme280)
{
	struct device_node *np = bme280->client->dev.of_node;
	u32 val;

	if (of_property_read_u32(np, "temperature_oversampling", &val) == 0)
		bme280->oversamp_temperature = val;
	else
		bme280->oversamp_temperature = 4;

	if (of_property_read_u32(np, "pressure_oversampling", &val) == 0)
		bme280->oversamp_pressure = val;
	else
		bme280->oversamp_pressure = 2;

	if (of_property_read_u32(np, "humidity_oversampling", &val) == 0)
		bme280->oversamp_humidity = val;
	else
		bme280->oversamp_humidity = 1;

	if (of_property_read_u32(np, "t_standby", &val) == 0)
		bme280->t_standby = val;
	else
		bme280->t_standby = 0;

	if (of_property_read_u32(np, "filter", &val) == 0)
		bme280->filter = val;
	else
		bme280->filter = 0;

	return bme280;
}
#else
static struct bme280_t* bme280_dt_init(struct bme280_t *bme280)
{
	return ERR_PTR(-ENODEV);
}
#endif

/*!
 *	@brief This function is used for readout the values from the devicetree
 *      initialize the different register and assign the chip id and I2C address
 *      of the BME280 sensor chip id is read in the register 0xD0 bit
 *      from 0 to 7
 *
 *	 @param bme280 structure pointer.
 *
 *	@note While changing the parameter of the bme280_t
 *	@note consider the following point:
 *	Changing the reference value of the parameter
 *	will changes the local copy or local reference
 *	make sure your changes will not
 *	affect the reference value of the parameter
 *	(Better case don't change the reference value of the parameter)
 *
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
static int bme280_init(struct bme280_t *bme280)
{
	int ret = 0;

	/* readout from device tree */
	bme280 = bme280_dt_init(bme280);
	/* refer lokal pointer to global pointer */
	p_bme280 = bme280;
	/* set name */
	p_bme280->name = "bme280";
	/* read Chip Id */
	p_bme280->chip_id = i2c_smbus_read_byte_data(p_bme280->client, BME280_CHIP_ID_REG);
	if(p_bme280->chip_id < 0)
		return -ENODEV;
	/* readout bme280 calibparam structure */
	ret = bme280_get_calib_param();

	ret = bme280_set_power_mode(BME280_SLEEP_MODE);
	if(ret < 0)
		return ret;

	ret = bme280_set_oversamp_humidity();
	if(ret < 0)
		return ret;

	ret = bme280_set_oversamp_temperature();
	if(ret < 0)
		return ret;

	ret = bme280_set_oversamp_pressure();
	if(ret < 0)
		return ret;

	ret = bme280_set_standby_durn();
	if(ret < 0)
		return ret;

	ret = bme280_set_filter();
	if(ret < 0)
		return ret;

	ret = bme280_set_power_mode(BME280_NORMAL_MODE);
	if(ret < 0)
		return ret;

	return ret;
}

static ssize_t show_t_standby(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int val;
	u8 v_data_u8;

	v_data_u8 = i2c_smbus_read_byte_data(p_bme280->client,
					      BME280_CONFIG_REG);
	v_data_u8 &= BME280_MASK_7_5;
	v_data_u8 = (v_data_u8 >> BME280_SHIFT_BIT_POSITION_BY_05_BITS);

	if(v_data_u8 == 0x07)
		val = 4000;
	else if(v_data_u8 == 0x06)
		val = 2000;
	else if(v_data_u8 == 0x05)
		val = 1000;
	else if(v_data_u8 == 0x04)
		val = 500;
	else if(v_data_u8 == 0x03)
		val = 250;
	else if(v_data_u8 == 0x02)
		val = 125;
	else if(v_data_u8 == 0x01)
		val = 63;
	else
		val = 1;

        return sprintf(buf, "t_standby: %d\n", val);
}

static ssize_t store_t_standby(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	int val = 0;

	ret = kstrtoint(buf, 10, &val);
	if(ret != 0)
		return -EINVAL;
	if(val < 0)
		return -EINVAL;

	p_bme280->t_standby = val;

	ret = bme280_set_standby_durn();

        return count;
}

static ssize_t show_filter(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	int val = 0;
	u8 v_data_u8 = 0;

	v_data_u8 = i2c_smbus_read_byte_data(p_bme280->client,
					      BME280_CONFIG_REG);
	v_data_u8 &= BME280_MASK_4_2;
	v_data_u8 = (v_data_u8 >> BME280_SHIFT_BIT_POSITION_BY_02_BITS);
	if(v_data_u8 > 0)
		v_data_u8++;

	val = convert_oversampling_to_val(v_data_u8);

        return sprintf(buf, "filter: %d\n", val);
}

static ssize_t store_filter(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	int ret = 0;
	int val = 0;

	ret = kstrtoint(buf, 10, &val);
	if(ret != 0)
		return -EINVAL;
	if(val < 0)
		return -EINVAL;

	p_bme280->filter = val;

	ret = bme280_set_filter();

        return count;
}

static ssize_t show_oversamp_pressure(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int val = 0;
	u8 v_data_u8 = 0;

	v_data_u8 = i2c_smbus_read_byte_data(p_bme280->client,
					      BME280_CTRL_MEAS_REG);
	v_data_u8 &= BME280_MASK_4_2;
	v_data_u8 = (v_data_u8 >> BME280_SHIFT_BIT_POSITION_BY_02_BITS);

	val = convert_oversampling_to_val(v_data_u8);

        return sprintf(buf, "oversampling x%d\n", val);
}

static ssize_t store_oversamp_pressure(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	int val = 0;

	ret = kstrtoint(buf, 10, &val);
	if(ret != 0)
		return -EINVAL;
	if(val < 0)
		return -EINVAL;

	p_bme280->oversamp_pressure = val;

	ret = bme280_set_oversamp_pressure();

        return count;
}


static ssize_t show_oversamp_temperature(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int val = 0;
	u8 v_data_u8 = 0;

	v_data_u8 = i2c_smbus_read_byte_data(p_bme280->client,
					      BME280_CTRL_MEAS_REG);
	v_data_u8 &= BME280_MASK_7_5;
	v_data_u8 = (v_data_u8 >> BME280_SHIFT_BIT_POSITION_BY_05_BITS);

	val = convert_oversampling_to_val(v_data_u8);

        return sprintf(buf, "oversampling x%d\n", val);
}

static ssize_t store_oversamp_temperature(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	int val = 0;

	ret = kstrtoint(buf, 10, &val);
	if(ret != 0)
		return -EINVAL;
	if(val < 0)
		return -EINVAL;

	p_bme280->oversamp_temperature = val;

	ret = bme280_set_oversamp_temperature();

        return count;
}

static ssize_t show_oversamp_humidity(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int val = 0;
	u8 v_data_u8 = 0;

	v_data_u8 = i2c_smbus_read_byte_data(p_bme280->client,
					      BME280_CTRL_HUMIDITY_REG);
	v_data_u8 &= BME280_MASK_2_0;

	val = convert_oversampling_to_val(v_data_u8);

        return sprintf(buf, "oversampling x%d\n", val);
}

static ssize_t store_oversamp_humidity(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	int val = 0;

	ret = kstrtoint(buf, 10, &val);
	if(ret != 0)
		return -EINVAL;
	if(val < 0)
		return -EINVAL;

	p_bme280->oversamp_humidity = val;

	ret = bme280_set_oversamp_humidity();

        return count;
}

static struct iio_dev_attr bme280_attr[] = {
	IIO_ATTR(oversamp_humidity, S_IWUSR | S_IRUGO, show_oversamp_humidity,
						store_oversamp_humidity, 0),
	IIO_ATTR(oversamp_temperature, S_IWUSR | S_IRUGO,
		 show_oversamp_temperature, store_oversamp_temperature, 0),
	IIO_ATTR(oversamp_pressure, S_IWUSR | S_IRUGO, show_oversamp_pressure,
						store_oversamp_pressure, 0),
	IIO_ATTR(filter, S_IWUSR | S_IRUGO, show_filter, store_filter, 0),
	IIO_ATTR(t_standby, S_IWUSR | S_IRUGO, show_t_standby,
							store_t_standby, 0),
};

static struct attribute *bme280_attributes[] = {
	&bme280_attr[0].dev_attr.attr,
	&bme280_attr[1].dev_attr.attr,
	&bme280_attr[2].dev_attr.attr,
	&bme280_attr[3].dev_attr.attr,
	&bme280_attr[4].dev_attr.attr,
	NULL
};

static const struct attribute_group bme280_attribute_group = {
	.attrs = bme280_attributes,
};

static const struct iio_info bme280_iio_info = {
	.driver_module		= THIS_MODULE,
	.read_raw		= bme280_read_raw,
	.attrs			= &bme280_attribute_group,
};

static const struct iio_chan_spec bme280_channels[] = {
	{
		.type = IIO_TEMP,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),
	},
	{
		.type = IIO_PRESSURE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),
	},
	{
		.type = IIO_HUMIDITYRELATIVE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),
	}
};

static int bme280_probe(struct i2c_client *client,
                const struct i2c_device_id *id)
{
	int ret;
	struct iio_dev *indio_dev;
	struct bme280_t *bme280_chip;

	bme280_chip = devm_kzalloc(&client->dev, sizeof(*bme280_chip),
								GFP_KERNEL);
	if (!bme280_chip)
		return -ENOMEM;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*bme280_chip));
 	if (!indio_dev)
		return -ENOMEM;

	bme280_chip = iio_priv(indio_dev);
	i2c_set_clientdata(client, indio_dev);
	bme280_chip->client = client;
	ret = bme280_init(bme280_chip);
	if(ret < 0)
		return ret;

	indio_dev->dev.parent = &client->dev;
	indio_dev->name = p_bme280->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &bme280_iio_info;

	indio_dev->channels = bme280_channels;
	indio_dev->num_channels = ARRAY_SIZE(bme280_channels);

	return iio_device_register(indio_dev);
}

static int bme280_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);

	iio_device_unregister(indio_dev);
        return 0;
}

#ifdef CONFIG_PM_SLEEP
static int bme280_suspend(struct device *dev)
{
	return bme280_set_power_mode(BME280_SLEEP_MODE);
}

static int bme280_resume(struct device *dev)
{
	return bme280_set_power_mode(BME280_NORMAL_MODE);
}
#endif

static SIMPLE_DEV_PM_OPS(bme280_pm_ops, bme280_suspend, bme280_resume);

static const struct i2c_device_id bme280_id[] = {
	{ "bme280", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, bme280_id);

static const struct of_device_id bme280_dt_ids[] = {
	{ .compatible = "st,bme280-humid" },
	{},
};

static struct i2c_driver bme280_driver = {
	.driver = {
		.name	= "bme280",
		.pm	= &bme280_pm_ops,
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(bme280_dt_ids),
	},
	.probe = bme280_probe,
	.remove = bme280_remove,
	.id_table = bme280_id,
};
module_i2c_driver(bme280_driver);

MODULE_DESCRIPTION("bme280 Digital Humidity, Pressure and Temperature Sensor");
MODULE_AUTHOR("Patrick Jakob, F&S Elektronik Systeme");
MODULE_LICENSE("GPL");
