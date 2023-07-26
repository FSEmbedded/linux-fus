/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * F&S Elektronik Systeme 2023
 */

#ifndef __TPS65185_H_
#define __TPS65185_H_

/* Reigster Addresses */

#define TPS65185_REG_TMST_VALUE    0x00
#define TPS65185_REG_ENABLE        0x01
#define TPS65185_REG_VADJ          0x02
#define TPS65185_REG_VCOM1         0x03
#define TPS65185_REG_VCOM2         0x04
#define TPS65185_REG_INT_EN1       0x05
#define TPS65185_REG_INT_EN2       0x06
#define TPS65185_REG_INT1          0x07
#define TPS65185_REG_INT8          0x08
#define TPS65185_REG_UPSEQ0        0x09
#define TPS65185_REG_UPSEQ1        0x0A
#define TPS65185_REG_DWNSEQ0       0x0B
#define TPS65185_REG_DWNSEQ1       0x0C
#define TPS65185_REG_TMST1         0x0D
#define TPS65185_REG_TMST2         0x0E
#define TPS65185_REG_PG            0x0F
#define TPS65185_REG_REVID         0x10

/* BIT-FIELD */
#define VPOS_VNEG_SETTING GENMASK(2,0)
#define ENABLE_REG_V3P3_EN BIT(5)
#define ENABLE_REG_VCOM_EN BIT(4)
#define TMST1_READ_THERM BIT(7)
#define TMST1_CONV_END BIT(5)

enum {
	tps65185_DISPLAY,
	tps65185_VPOS,
	tps65185_VNEG,
	tps65185_VGH,
	tps65185_VGL,
	tps65185_VCOM,
	tps65185_V3P3,
};

enum TPS65185_REVID {
     TPS65185_1P0 = 0x45,
     TPS65185_1P1 = 0x55,
     TPS65185_1P2 = 0x65,
     TPS651851_1P0 = 0x66,
};

struct tps65185 {
	struct device *dev;
	struct tps65185_platform_data *pdata;

	struct i2c_client *client;

	/* power up delay time: 0ms, 1ms, 2ms, 4ms */
	unsigned int vgl_pwrup;
	unsigned int vneg_pwrup;
	unsigned int vgh_pwrup;
	unsigned int vpos_pwrup;

	/* GPIOS*/
	int gpio_pmic_wakeup;
	int gpio_pmic_pwrup;
	int gpio_pmic_pwrgood;
};

struct tps65185_platform_data {
	/* PMIC */
	struct tps65185_regulator_data *regulators;
	int num_regulators;
};

struct tps65185_regulator_data {
	int id;
	struct regulator_init_data *initdata;
	struct device_node *reg_node;
};

int tps65185_reg_read(struct i2c_client *client, int reg_num, u8 *reg_val);
int tps65185_reg_write(struct i2c_client *client, int reg_num, u8 reg_val);
#endif
