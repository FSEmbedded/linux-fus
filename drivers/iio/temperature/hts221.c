/******************** (C) COPYRIGHT 2014 STMicroelectronics *******************
*i2c_read
* File Name          : hts221.c
* Authors            : HESA BU
*                      Lorenzo SARCHI <lorenzo.sarchi@st.com>
*                      Morris Chen (morris.chen@st.com)
* Version            : V.0.0.3
* Date               : 2014/May/12
*
*******************************************************************************
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
** Output data from the device are available from the assigned
* /dev/input/eventX device;
*
* HTS221 can be controlled by sysfs interface looking inside:
* /sys/bus/i2c/devices/<busnum>-<devaddr>/
*
* Read 'relative humidity' and temperatures output can be converted in units of
* measurement by dividing them respectively 1000.
* Temperature values are expressed as Celsius degrees.
*
* To use odr, pls write the corresponding rates in "poll_period_ms".
* 1 hz: 1000; 7hz: 143,  12.5 hz: 80
*
* To enable/disable oneshot mode, set corresponding 1/0 in "oneshot".
* To enable/disable the heater, set corresponding 1/0 in "heater".
*
*******************************************************************************
*******************************************************************************
Version History.

Revision 0.0.1 06/03/2013
 first revision

Revision 0.0.1 07/23/2013
 Optimization process

Revision 0.0.2 04/04/2014
 second revision

Revision 0.0.3 05/12/2014
 ODR

******************************************************************************/

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/of.h>
#include <linux/export.h>

#include "hts221.h"

struct hts221_status {
        struct i2c_client *client;
        struct hts221_platform_data pdata;
};

static int hts221_hw_init(struct hts221_status *stat)
{
	u8 val = 0;
        int err = -1;

	val = i2c_smbus_read_byte_data(stat->client, REG_WHOAMI_ADDR);
	if (val < 0) {
                dev_err(&stat->client->dev, "Error reading WHO_AM_I: is device"
                " available/working?\n");
                goto err_firstread;
        }

	if(val != WHOIAM_VALUE) {
	        dev_err(&stat->client->dev,
                "device unknown. Expected: 0x%02x,"
                " Replies: 0x%02x\n", WHOIAM_VALUE, val);
                err = -1;
                goto err_unknown_device;
        }
	return 0;

err_unknown_device:
err_firstread:
        return err;
}

static int hts221_device_power_off(struct hts221_status *stat)
{
	u8 val = 0;
	int err = -1;

	val = i2c_smbus_read_byte_data(stat->client, REG_CNTRL1_ADDR);
	val &= ~HTS221_MASK_POWER_ON;

	err = i2c_smbus_write_byte_data(stat->client, REG_CNTRL1_ADDR, val);
	if(err < 0)
		dev_err(&stat->client->dev, "device power down error 0x%02x",
									err);
	return err;
}

static int hts221_device_power_on(struct hts221_status *stat)
{
	u8 val = 0;
	int err = -1;

	val = i2c_smbus_read_byte_data(stat->client, REG_CNTRL1_ADDR);
	val |= HTS221_MASK_POWER_ON;

	err = i2c_smbus_write_byte_data(stat->client, REG_CNTRL1_ADDR, val);
	if(err < 0)
		dev_err(&stat->client->dev, "device power down error 0x%02x",
									err);
	return err;
}

static int hts221_update_resolution(struct hts221_status *stat, int avg)
{
	u8 val = 0;
	int err = -1;

	val = i2c_smbus_read_byte_data(stat->client, REG_AV_CONF);
	if(avg == HTS221_AVG_T) {
		val &= ~HTS221_MASK_AVG_T;
		val |= stat->pdata.t_resolution;
	}
	else if(avg == HTS221_AVG_H) {
		val &= ~HTS221_MASK_AVG_H;
		val |= stat->pdata.h_resolution;
	}

	err = i2c_smbus_write_byte_data(stat->client, REG_AV_CONF, val);
	if(err < 0)
		dev_err(&stat->client->dev, "update resolution failed %d",
									err);
        return err;
}

static int hts221_update_odr(struct hts221_status *stat,
                        unsigned int poll_interval_ms)
{
	u8 val = 0;
	int err = -1;

	val = i2c_smbus_read_byte_data(stat->client, REG_CNTRL1_ADDR);
	val &= ~HTS221_MASK_ODR;

	if(poll_interval_ms >= 570)
		val |= ODR_1_1;
	else if(poll_interval_ms >= 112)
		val |= ODR_7_7;
	else if(poll_interval_ms >= 1)
		val |= ODR_12_12;
	else
		val |= ODR_ONESH;

	err = i2c_smbus_write_byte_data(stat->client, REG_CNTRL1_ADDR, val);
        if (err < 0)
		dev_err(&stat->client->dev, "update odr failed %d", err);

	return err;
}

static int convert_resolution_h(int val)
{
	if(val >= 384)
		return HTS221_H_RESOLUTION_512;
	else if(val >= 192)
		return HTS221_H_RESOLUTION_256;
	else if(val >= 96)
		return HTS221_H_RESOLUTION_128;
	else if(val >= 48)
		return HTS221_H_RESOLUTION_64;
	else if(val >= 24)
		return HTS221_H_RESOLUTION_32;
	else if(val >= 12)
		return HTS221_H_RESOLUTION_16;
	else if(val >= 6)
		return HTS221_H_RESOLUTION_8;
	else
		return HTS221_H_RESOLUTION_4;
}

static int convert_resolution_t(int val)
{
	if(val >= 192)
		return HTS221_T_RESOLUTION_256;
	else if(val >= 96)
		return HTS221_T_RESOLUTION_128;
	else if(val >= 48)
		return HTS221_T_RESOLUTION_64;
	else if(val >= 24)
		return HTS221_T_RESOLUTION_32;
	else if(val >= 12)
		return HTS221_T_RESOLUTION_16;
	else if(val >= 6)
		return HTS221_T_RESOLUTION_8;
	else if(val >= 3)
		return HTS221_T_RESOLUTION_4;
	else
		return HTS221_T_RESOLUTION_2;
}

static int hts221_get_data(struct hts221_status *stat, int sensor)
{
	int temp_calc = 0, temp_res = 0;
	int calib_t0 = 0, calib_t1 = 0;
	int calib_h0 = 0, calib_h1 = 0;
	u8 t0_msb = 0, t1_msb = 0;
	s16 x_t = 0, x_h = 0;
	s32 val = 0;

	if(sensor == HTS221_AVG_T) {
		t0_msb = (stat->pdata.cal_param.dig_T1_T0_msb &
							HTS221_MASK_T0_MSB);
		t1_msb = (stat->pdata.cal_param.dig_T1_T0_msb &
							HTS221_MASK_T1_MSB);
		t1_msb = (t1_msb >> HTS221_SHIFT_BIT_POSITION_BY_02_BITS);

		calib_t0 = ((t0_msb << 8) |
					stat->pdata.cal_param.dig_T0_degC);
		calib_t1 = ((t1_msb << 8) |
					stat->pdata.cal_param.dig_T1_degC);

		x_t = (s16)(i2c_smbus_read_byte_data(stat->client,
					HTS221_TEMPERATURE_OUT_LSB_REG) |
		(i2c_smbus_read_byte_data(stat->client,
					HTS221_TEMPERATURE_OUT_MSB_REG) <<
					8));
		/* multiple with 100 for Centigrade */
		temp_calc = (x_t - stat->pdata.cal_param.dig_T1_out) * 100;
		temp_res = (temp_calc*(calib_t1-calib_t0)) /
					(stat->pdata.cal_param.dig_T1_out -
					stat->pdata.cal_param.dig_T0_out);
		/* value is multiplied with 8 so divide trough 8 and round with
		 * +4. Multiple calib_t1 with 100 for Centigrade */
		val = ((temp_res+(calib_t1*100))+4)/8;
	}
	else if(sensor == HTS221_AVG_H) {
		calib_h0 = stat->pdata.cal_param.dig_H0_rH;
		calib_h1 = stat->pdata.cal_param.dig_H1_rH;

		x_h = (s16)(i2c_smbus_read_byte_data(stat->client,
						HTS221_HUMIDITY_OUT_LSB_REG) |
				(i2c_smbus_read_byte_data(stat->client,
						HTS221_HUMIDITY_OUT_MSB_REG) <<
					8));
		/* multiple with 100 for for comma value */
		temp_calc = (x_h - stat->pdata.cal_param.dig_H1_out) * 100;
		temp_res = (temp_calc * (calib_h1-calib_h0)) /
					(stat->pdata.cal_param.dig_H1_out -
					stat->pdata.cal_param.dig_H0_out);
		/* value is multiplied with 2 so divide trough 2 and round with
		 *+1. Multiple calib_h1 with 100 for comma value */
		val = ((temp_res+(calib_h1*100))+1)/2;
	}
	return val;
}

#ifdef CONFIG_OF
static struct hts221_status* hts221_dt_init(struct i2c_client *client,
						struct hts221_status *stat)
{
	u32 val;
	struct device_node *np = client->dev.of_node;

	stat->pdata.poll_interval = ODR_ONESH;
	if (of_property_read_u32(np, "poll_interval", &val) == 0)
			stat->pdata.poll_interval = val;

	stat->pdata.h_resolution = HTS221_H_RESOLUTION_32;
	if (of_property_read_u32(np, "h_resolution", &val) == 0)
		stat->pdata.h_resolution = convert_resolution_h(val);

	stat->pdata.t_resolution = HTS221_T_RESOLUTION_16;
	if (of_property_read_u32(np, "t_resolution", &val) == 0)
		stat->pdata.t_resolution = convert_resolution_t(val);

	return stat;
}
#else
static struct hts221_status* hts221_dt_init(struct i2c_client *client,
						struct hts221_status *stat)
{
	dev_info(&client->dev, "using default plaform_data for "
                                                "humidity\n");
	return stat;
}
#endif

static int hts221_read_raw(struct iio_dev *iio_dev,
			const struct iio_chan_spec *chan,
			int *val, int *val2, long m)
{
	struct hts221_status *stat = iio_priv(iio_dev);
	int temp = 0;
	int hum = 0;
	int ret = 0;

	if(chan->type == IIO_TEMP)
	{
		/* temperature unit is in Centigrade */
		temp = hts221_get_data(stat, HTS221_AVG_T);
		*val = temp;
		ret = IIO_VAL_INT;
	}
	else if(chan->type == IIO_HUMIDITYRELATIVE)
	{
		/* humidity value / 100 = %rH */
		hum = hts221_get_data(stat, HTS221_AVG_H);
		*val = hum;
		ret = IIO_VAL_INT;
	}
	return ret;
}

/* shows the number of averaged temperature samples */
static ssize_t show_resolution_t(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct hts221_status *stat = iio_priv(indio_dev);
        u8 val = 0;
	int resolution = 0;

	val = (i2c_smbus_read_byte_data(stat->client, REG_AV_CONF) &
							HTS221_MASK_AVG_T);
	switch(val) {
		case HTS221_T_RESOLUTION_2:
			resolution = 2; break;
		case HTS221_T_RESOLUTION_4:
			resolution = 4; break;
		case HTS221_T_RESOLUTION_8:
			resolution = 8; break;
		case HTS221_T_RESOLUTION_16:
			resolution = 16; break;
		case HTS221_T_RESOLUTION_32:
			resolution = 32; break;
		case HTS221_T_RESOLUTION_64:
			resolution = 64; break;
		case HTS221_T_RESOLUTION_128:
			resolution = 128; break;
		case HTS221_T_RESOLUTION_256:
			resolution = 256; break;
	}
        return sprintf(buf, "%d\n", resolution);
}

/* set the number of averaged temperature samples */
static ssize_t store_resolution_t(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct hts221_status *stat = iio_priv(indio_dev);
        int val = 0;
	int err = -1;

	err = kstrtoint(buf, 10, &val);
	if(err != 0)
		return -EINVAL;
	if(val < 0)
		return -EINVAL;

	stat->pdata.t_resolution = convert_resolution_t(val);
	hts221_update_resolution(stat, HTS221_AVG_T);

        return count;
}

/* shows the number of averaged humidity samples */
static ssize_t show_resolution_h(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct hts221_status *stat = iio_priv(indio_dev);
        u8 val = 0;
	int resolution = 0;

	val = (i2c_smbus_read_byte_data(stat->client, REG_AV_CONF) &
							HTS221_MASK_AVG_H);
	switch(val) {
		case HTS221_H_RESOLUTION_4:
			resolution = 4; break;
		case HTS221_H_RESOLUTION_8:
			resolution = 8; break;
		case HTS221_H_RESOLUTION_16:
			resolution = 16; break;
		case HTS221_H_RESOLUTION_32:
			resolution = 32; break;
		case HTS221_H_RESOLUTION_64:
			resolution = 64; break;
		case HTS221_H_RESOLUTION_128:
			resolution = 128; break;
		case HTS221_H_RESOLUTION_256:
			resolution = 256; break;
		case HTS221_H_RESOLUTION_512:
			resolution = 512; break;
	}
        return sprintf(buf, "%d\n", resolution);
}

/* set the number of averaged humidity samples */
static ssize_t store_resolution_h(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct hts221_status *stat = iio_priv(indio_dev);
        int val = 0;
	int err = -1;

	err = kstrtoint(buf, 10, &val);
	if(err != 0)
		return -EINVAL;
	if(val < 0)
		return -EINVAL;

	stat->pdata.h_resolution = convert_resolution_h(val);
	hts221_update_resolution(stat, HTS221_AVG_H);

        return count;
}

/* Polling rate value is in milliseconds */
static ssize_t show_polling_rate(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct hts221_status *stat = iio_priv(indio_dev);
	u8 val = 0;
	int result = 0;

	val = (i2c_smbus_read_byte_data(stat->client, REG_CNTRL1_ADDR) &
							HTS221_MASK_ODR);
	if(val == ODR_12_12)
		result = 83;
	else if(val == ODR_7_7)
		result = 142;
	else if(val == ODR_1_1)
		result = 1000;
	else
		result = 0;

        return sprintf(buf, "%u\n", result);
}

/* set the polling rate. Value must be in milliseconds */
static ssize_t store_polling_rate(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct hts221_status *stat = iio_priv(indio_dev);
        int val = 0;
	int err = -1;

	err = kstrtoint(buf, 10, &val);
	if(err != 0)
		return -EINVAL;
	if(val < 0)
		return -EINVAL;

	err = hts221_update_odr(stat, val);
	if(err < 0)
		return -EINVAL;

        return count;
}

/* Value 1 = heating on, 0 = heating off */
static ssize_t show_heating(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct hts221_status *stat = iio_priv(indio_dev);
	u8 val = 0;

	val = (i2c_smbus_read_byte_data(stat->client, REG_CNTRL2_ADDR) &
							HTS221_MASK_HEATER_ON);
	val = val >> HTS221_SHIFT_BIT_POSITION_BY_01_BITS;

        return sprintf(buf, "%d\n", val);
}

/* if value is > 0 heating = on */
static ssize_t store_heating(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct hts221_status *stat = iio_priv(indio_dev);
	u8 v_data_u8 = 0;
	int val = 0;
	int err = -1;

	err = kstrtoint(buf, 10, &val);
	if(err != 0)
		return -EINVAL;
	if(val < 0)
		return -EINVAL;

	v_data_u8 = i2c_smbus_read_byte_data(stat->client, REG_CNTRL2_ADDR);
	v_data_u8 &= ~HTS221_MASK_HEATER_ON;
	if(val > 0) {
		val = HTS221_MASK_HEATER_ON;
		v_data_u8 |= val;
	}

        i2c_smbus_write_byte_data(stat->client, REG_CNTRL2_ADDR, v_data_u8);

        return count;
}

static struct iio_dev_attr hts221_attr[] = {
	IIO_ATTR(heating, S_IWUSR | S_IRUGO, show_heating, store_heating, 0),
	IIO_ATTR(polling_rate, S_IWUSR | S_IRUGO, show_polling_rate,
							store_polling_rate, 0),
	IIO_ATTR(resolution_h, S_IWUSR | S_IRUGO, show_resolution_h,
							store_resolution_h, 0),
	IIO_ATTR(resolution_t, S_IWUSR | S_IRUGO, show_resolution_t,
							store_resolution_t, 0),
};

static struct attribute *hts221_attributes[] = {
	&hts221_attr[0].dev_attr.attr,
	&hts221_attr[1].dev_attr.attr,
	&hts221_attr[2].dev_attr.attr,
	&hts221_attr[3].dev_attr.attr,
	NULL
};

static const struct attribute_group hts221_attribute_group = {
	.attrs = hts221_attributes,
};

static const struct iio_info hts221_iio_info = {
	.read_raw		= hts221_read_raw,
	.attrs = &hts221_attribute_group,
};

static const struct iio_chan_spec hts221_channels[] = {
	{
		.type = IIO_TEMP,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),
	},
	{
		.type = IIO_HUMIDITYRELATIVE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),
	}
};

static void hts221_get_calib_param(struct hts221_status *stat)
{
	stat->pdata.cal_param.dig_H0_out = (i2c_smbus_read_byte_data(
		stat->client, HTS221_HUMIDITY_CALIB_DIG_H0_OUT_LSB_REG) |
		(i2c_smbus_read_byte_data (stat->client,
			HTS221_HUMIDITY_CALIB_DIG_H0_OUT_MSB_REG) << 8));

	stat->pdata.cal_param.dig_H1_out = (i2c_smbus_read_byte_data(
		stat->client, HTS221_HUMIDITY_CALIB_DIG_H1_OUT_LSB_REG) |
		(i2c_smbus_read_byte_data (stat->client,
			HTS221_HUMIDITY_CALIB_DIG_H1_OUT_MSB_REG) << 8));

	stat->pdata.cal_param.dig_H0_rH = i2c_smbus_read_byte_data(
			stat->client, HTS221_HUMIDITY_CALIB_DIG_H0_rH_REG);
	stat->pdata.cal_param.dig_H1_rH = i2c_smbus_read_byte_data(
			stat->client, HTS221_HUMIDITY_CALIB_DIG_H1_rH_REG);
	stat->pdata.cal_param.dig_T0_out = (i2c_smbus_read_byte_data(
		stat->client, HTS221_TEMPERATURE_CALIB_DIG_T0_OUT_LSB_REG) |
		(i2c_smbus_read_byte_data (stat->client,
		HTS221_TEMPERATURE_CALIB_DIG_T0_OUT_MSB_REG) << 8));

	stat->pdata.cal_param.dig_T1_out = (i2c_smbus_read_byte_data(
		stat->client, HTS221_TEMPERATURE_CALIB_DIG_T1_OUT_LSB_REG) |
		(i2c_smbus_read_byte_data (stat->client,
			HTS221_TEMPERATURE_CALIB_DIG_T1_OUT_MSB_REG) << 8));

	stat->pdata.cal_param.dig_T0_degC = i2c_smbus_read_byte_data(
		stat->client, HTS221_TEMPERATURE_CALIB_DIG_T0_degC_REG);
	stat->pdata.cal_param.dig_T1_degC = i2c_smbus_read_byte_data(
		stat->client, HTS221_TEMPERATURE_CALIB_DIG_T1_degC_REG);
	stat->pdata.cal_param.dig_T1_T0_msb = i2c_smbus_read_byte_data(
		stat->client, HTS221_TEMPERATURE_CALIB_DIG_T1_T0_MSB_REG);

}

static int hts221_init(struct hts221_status *stat)
{
	int err = 0;

	err = hts221_device_power_off(stat);
	if (err < 0) {
		dev_err(&stat->client->dev, "device power off failed: %d\n",
									err);
		return err;
	}

	err = hts221_hw_init(stat);
        if (err < 0) {
                dev_err(&stat->client->dev, "hw init failed: %d\n", err);
		return err;
	}

	hts221_get_calib_param(stat);

	err = hts221_update_resolution(stat, HTS221_AVG_T);
	if (err < 0)
		return err;

	err = hts221_update_resolution(stat, HTS221_AVG_H);
	if (err < 0)
		return err;

	err = hts221_update_odr(stat, stat->pdata.poll_interval);
	if(err < 0)
		return err;

	err = hts221_device_power_on(stat);
	if (err < 0) {
		dev_err(&stat->client->dev, "device power on failed: %d\n",
									err);
		return err;
	}
	return err;
}

static int hts221_probe(struct i2c_client *client)
{
        struct hts221_status *stat;
	struct iio_dev *indio_dev;
	int err = -1;
	u32 smbus_func = I2C_FUNC_SMBUS_BYTE_DATA |
                        I2C_FUNC_SMBUS_WORD_DATA | I2C_FUNC_SMBUS_I2C_BLOCK;

	stat = kzalloc(sizeof(struct hts221_status), GFP_KERNEL);
        if (stat == NULL) {
                err = -ENOMEM;
                dev_err(&client->dev,
                                "failed to allocate memory for module data: "
                                        "%d\n", err);
               goto exit_check_functionality_failed;
        }

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*stat));
	if (!indio_dev)
		return -ENOMEM;

	stat = iio_priv(indio_dev);
	i2c_set_clientdata(client, indio_dev);
        stat->client = client;

        if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
                dev_warn(&client->dev, "client not i2c capable\n");
                if (i2c_check_functionality(client->adapter, smbus_func)) {
                        dev_warn(&client->dev, "client using SMBUS\n");
                } else {
                        err = -ENODEV;
                        dev_err(&client->dev, "client not SMBUS capable\n");
                        goto exit_check_functionality_failed;
                }
        }

        if (client->dev.platform_data == NULL)
		stat = hts221_dt_init(client, stat);
        else {
                memcpy(&stat->pdata, client->dev.platform_data,
                                                sizeof(stat->pdata));
                dev_info(&client->dev, "using user plaform_data for "
                                                "humidity\n");
        }

        if (stat->pdata.init) {
                err = stat->pdata.init();
                if (err < 0) {
                        dev_err(&client->dev, "humidity init failed: "
                                                                "%d\n", err);
                        goto err_pdata_init;
                }
        }

        err = hts221_init(stat);
	if (err < 0) {
		dev_err(&client->dev, "hts221 init failed: %d\n", err);
		goto err_hw_init;
	}

	indio_dev->dev.parent = &client->dev;
	indio_dev->name = HTS221_DEV_NAME;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &hts221_iio_info;
	indio_dev->channels = hts221_channels;
	indio_dev->num_channels = ARRAY_SIZE(hts221_channels);

	err = iio_device_register(indio_dev);
	if(err) {
		dev_err(&client->dev, "register iio_device failed: %d\n", err);
		return err;
	}
	return 0;

err_hw_init:
err_pdata_init:
        if (stat->pdata.exit)
                stat->pdata.exit();
exit_check_functionality_failed:
        pr_err("%s: Driver Init failed\n", HTS221_DEV_NAME);
        return err;
}

static void hts221_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct hts221_status *stat = iio_priv(indio_dev);

	hts221_device_power_off(stat);
	iio_device_unregister(indio_dev);

	if (stat->pdata.exit)
		stat->pdata.exit();

        kfree(stat);
}

#ifdef CONFIG_PM_SLEEP
static int hts221_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct hts221_status *stat = iio_priv(indio_dev);

	return hts221_device_power_off(stat);
}

static int hts221_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct hts221_status *stat = iio_priv(indio_dev);

	return hts221_device_power_on(stat);
}
#endif

static SIMPLE_DEV_PM_OPS(hts221_pm_ops, hts221_suspend, hts221_resume);

static const struct i2c_device_id hts221_id[] = {
	{ HTS221_DEV_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, hts221_id);

static const struct of_device_id hts221_dt_ids[] = {
	{ .compatible = "st,hts221-humid" },
	{},
};

static struct i2c_driver hts221_driver = {
        .driver = {
                        .owner = THIS_MODULE,
                        .name = HTS221_DEV_NAME,
			.pm	= &hts221_pm_ops,
			.of_match_table = of_match_ptr(hts221_dt_ids),
                  },
        .probe = hts221_probe,
	.remove = hts221_remove,
        .id_table = hts221_id,
};

module_i2c_driver(hts221_driver);

MODULE_DESCRIPTION("hts221 humidity driver");
MODULE_AUTHOR("HESA BU, STMicroelectronics");
MODULE_LICENSE("GPL");
