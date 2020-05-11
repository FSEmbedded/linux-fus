/*
 * Copyright 2005-2015 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2019 NXP
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file ipu_disp.c
 *
 * @brief IPU display submodule API functions
 *
 * @ingroup IPU
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/ipu-v3.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/types.h>

#include <asm/atomic.h>

#include "ipu_param_mem.h"
#include "ipu_regs.h"

struct dp_csc_param_t {
	int mode;
	const void *coeff;
};

#define SYNC_WAVE 0
#define NULL_WAVE (-1)
#define ASYNC_SER_WAVE 6

/* DC display ID assignments */
#define DC_DISP_ID_SYNC(di)	(di)
#define DC_DISP_ID_SERIAL	2
#define DC_DISP_ID_ASYNC	3

/* DC microcode address */
#define DC_MCODE_BT656_EOF			101
#define DC_MCODE_BT656_NF				140
#define DC_MCODE_BT656_EOFIELD		161
#define DC_MCODE_BT656_DATA_W		164
#define DC_MCODE_BT656_NL				168

#define DC_MCODE_BT656_P_EOF			101
#define DC_MCODE_BT656_P_NF			121
#define DC_MCODE_BT656_P_DATA_W		141
#define DC_MCODE_BT656_P_NL			145

#define BT656_IF_DI_MSB			7  /* For 8 bits BT656: 23 for DISP_DAT23 ~ DISP_DAT16; 7 for DISP_DAT7 ~ DISP_DAT0 */
								      /* For 16 bits BT1120: 23 for DISP_DAT23 ~ DISP_DAT8; 15 for DISP_DAT15 ~ DISP_DAT0 */

int dmfc_type_setup;

void _ipu_dmfc_init(struct ipu_soc *ipu, int dmfc_type, int first)
{
	u32 dmfc_wr_chan, dmfc_dp_chan;

	if (first) {
		if (dmfc_type_setup > dmfc_type)
			dmfc_type = dmfc_type_setup;
		else
			dmfc_type_setup = dmfc_type;

		/* disable DMFC-IC channel*/
		ipu_dmfc_write(ipu, 0x2, DMFC_IC_CTRL);
	} else if (dmfc_type_setup >= DMFC_HIGH_RESOLUTION_DC) {
		dev_dbg(ipu->dev, "DMFC high resolution has set, will not change\n");
		return;
	} else
		dmfc_type_setup = dmfc_type;

	if (dmfc_type == DMFC_HIGH_RESOLUTION_DC) {
		/* 1 - segment 0~3;
		 * 5B - segement 4, 5;
		 * 5F - segement 6, 7;
		 * 1C, 2C and 6B, 6F unused;
		 */
		dev_info(ipu->dev, "IPU DMFC DC HIGH RESOLUTION: 1(0~3), 5B(4,5), 5F(6,7)\n");
		dmfc_wr_chan = 0x00000088;
		dmfc_dp_chan = 0x00009694;
		ipu->dmfc_size_28 = 256*4;
		ipu->dmfc_size_29 = 0;
		ipu->dmfc_size_24 = 0;
		ipu->dmfc_size_27 = 128*4;
		ipu->dmfc_size_23 = 128*4;
	} else if (dmfc_type == DMFC_HIGH_RESOLUTION_DP) {
		/* 1 - segment 0, 1;
		 * 5B - segement 2~5;
		 * 5F - segement 6,7;
		 * 1C, 2C and 6B, 6F unused;
		 */
		dev_info(ipu->dev, "IPU DMFC DP HIGH RESOLUTION: 1(0,1), 5B(2~5), 5F(6,7)\n");
		dmfc_wr_chan = 0x00000090;
		dmfc_dp_chan = 0x0000968a;
		ipu->dmfc_size_28 = 128*4;
		ipu->dmfc_size_29 = 0;
		ipu->dmfc_size_24 = 0;
		ipu->dmfc_size_27 = 128*4;
		ipu->dmfc_size_23 = 256*4;
	} else if (dmfc_type == DMFC_HIGH_RESOLUTION_ONLY_DP) {
		/* 5B - segement 0~3;
		 * 5F - segement 4~7;
		 * 1, 1C, 2C and 6B, 6F unused;
		 */
		dev_info(ipu->dev, "IPU DMFC ONLY-DP HIGH RESOLUTION: 5B(0~3), 5F(4~7)\n");
		dmfc_wr_chan = 0x00000000;
		dmfc_dp_chan = 0x00008c88;
		ipu->dmfc_size_28 = 0;
		ipu->dmfc_size_29 = 0;
		ipu->dmfc_size_24 = 0;
		ipu->dmfc_size_27 = 256*4;
		ipu->dmfc_size_23 = 256*4;
	} else {
		/* 1 - segment 0, 1;
		 * 5B - segement 4, 5;
		 * 5F - segement 6, 7;
		 * 1C, 2C and 6B, 6F unused;
		 */
		dev_info(ipu->dev, "IPU DMFC NORMAL mode: 1(0~1), 5B(4,5), 5F(6,7)\n");
		dmfc_wr_chan = 0x00000090;
		dmfc_dp_chan = 0x00009694;
		ipu->dmfc_size_28 = 128*4;
		ipu->dmfc_size_29 = 0;
		ipu->dmfc_size_24 = 0;
		ipu->dmfc_size_27 = 128*4;
		ipu->dmfc_size_23 = 128*4;
	}
	ipu_dmfc_write(ipu, dmfc_wr_chan, DMFC_WR_CHAN);
	ipu_dmfc_write(ipu, 0x202020F6, DMFC_WR_CHAN_DEF);
	ipu_dmfc_write(ipu, dmfc_dp_chan, DMFC_DP_CHAN);
	/* Enable chan 5 watermark set at 5 bursts and clear at 7 bursts */
	ipu_dmfc_write(ipu, 0x2020F6F6, DMFC_DP_CHAN_DEF);
}

static int __init dmfc_setup(char *options)
{
	get_option(&options, &dmfc_type_setup);
	if (dmfc_type_setup > DMFC_HIGH_RESOLUTION_ONLY_DP)
		dmfc_type_setup = DMFC_HIGH_RESOLUTION_ONLY_DP;
	return 1;
}
__setup("dmfc=", dmfc_setup);

void _ipu_dmfc_set_wait4eot(struct ipu_soc *ipu, int dma_chan, int width)
{
	u32 dmfc_gen1 = ipu_dmfc_read(ipu, DMFC_GENERAL1);

	if (width >= HIGH_RESOLUTION_WIDTH) {
		if (dma_chan == 23)
			_ipu_dmfc_init(ipu, DMFC_HIGH_RESOLUTION_DP, 0);
		else if (dma_chan == 28)
			_ipu_dmfc_init(ipu, DMFC_HIGH_RESOLUTION_DC, 0);
	}

	if (dma_chan == 23) { /*5B*/
		if (ipu->dmfc_size_23/width > 3)
			dmfc_gen1 |= 1UL << 20;
		else
			dmfc_gen1 &= ~(1UL << 20);
	} else if (dma_chan == 24) { /*6B*/
		if (ipu->dmfc_size_24/width > 1)
			dmfc_gen1 |= 1UL << 22;
		else
			dmfc_gen1 &= ~(1UL << 22);
	} else if (dma_chan == 27) { /*5F*/
		if (ipu->dmfc_size_27/width > 2)
			dmfc_gen1 |= 1UL << 21;
		else
			dmfc_gen1 &= ~(1UL << 21);
	} else if (dma_chan == 28) { /*1*/
		if (ipu->dmfc_size_28/width > 2)
			dmfc_gen1 |= 1UL << 16;
		else
			dmfc_gen1 &= ~(1UL << 16);
	} else if (dma_chan == 29) { /*6F*/
		if (ipu->dmfc_size_29/width > 1)
			dmfc_gen1 |= 1UL << 23;
		else
			dmfc_gen1 &= ~(1UL << 23);
	}

	ipu_dmfc_write(ipu, dmfc_gen1, DMFC_GENERAL1);
}

void _ipu_dmfc_set_burst_size(struct ipu_soc *ipu, int dma_chan, int burst_size)
{
	u32 dmfc_wr_chan = ipu_dmfc_read(ipu, DMFC_WR_CHAN);
	u32 dmfc_dp_chan = ipu_dmfc_read(ipu, DMFC_DP_CHAN);
	int dmfc_bs = 0;

	switch (burst_size) {
	case 64:
		dmfc_bs = 0x40;
		break;
	case 32:
	case 20:
		dmfc_bs = 0x80;
		break;
	case 16:
		dmfc_bs = 0xc0;
		break;
	default:
		dev_err(ipu->dev, "Unsupported burst size %d\n",
			burst_size);
		return;
	}

	if (dma_chan == 23) { /*5B*/
		dmfc_dp_chan &= ~(0xc0);
		dmfc_dp_chan |= dmfc_bs;
	} else if (dma_chan == 27) { /*5F*/
		dmfc_dp_chan &= ~(0xc000);
		dmfc_dp_chan |= (dmfc_bs << 8);
	} else if (dma_chan == 28) { /*1*/
		dmfc_wr_chan &= ~(0xc0);
		dmfc_wr_chan |= dmfc_bs;
	}

	ipu_dmfc_write(ipu, dmfc_wr_chan, DMFC_WR_CHAN);
	ipu_dmfc_write(ipu, dmfc_dp_chan, DMFC_DP_CHAN);
}

static void _ipu_di_data_wave_config(struct ipu_soc *ipu,
				int di, int wave_gen,
				int access_size, int component_size)
{
	u32 reg;
	reg = (access_size << DI_DW_GEN_ACCESS_SIZE_OFFSET) |
	    (component_size << DI_DW_GEN_COMPONENT_SIZE_OFFSET);
	ipu_di_write(ipu, di, reg, DI_DW_GEN(wave_gen));
}

static void _ipu_di_data_pin_config(struct ipu_soc *ipu,
			int di, int wave_gen, int di_pin, int set,
			int up, int down)
{
	u32 reg;

	reg = ipu_di_read(ipu, di, DI_DW_GEN(wave_gen));
	reg &= ~(0x3 << (di_pin * 2));
	reg |= set << (di_pin * 2);
	ipu_di_write(ipu, di, reg, DI_DW_GEN(wave_gen));

	ipu_di_write(ipu, di, (down << 16) | up, DI_DW_SET(wave_gen, set));
}

static void _ipu_di_sync_config(struct ipu_soc *ipu,
				int di, int wave_gen,
				int run_count, int run_src,
				int offset_count, int offset_src,
				int repeat_count, int cnt_clr_src,
				int cnt_polarity_gen_en,
				int cnt_polarity_clr_src,
				int cnt_polarity_trigger_src,
				int cnt_up, int cnt_down)
{
	u32 reg;

	if ((run_count >= 0x1000) || (offset_count >= 0x1000) || (repeat_count >= 0x1000) ||
		(cnt_up >= 0x400) || (cnt_down >= 0x400)) {
		dev_err(ipu->dev, "DI%d counters out of range.\n", di);
		return;
	}

	reg = (run_count << 19) | (++run_src << 16) |
	    (offset_count << 3) | ++offset_src;
	ipu_di_write(ipu, di, reg, DI_SW_GEN0(wave_gen));
	reg = (cnt_polarity_gen_en << 29) | (++cnt_clr_src << 25) |
	    (++cnt_polarity_trigger_src << 12) | (++cnt_polarity_clr_src << 9);
	reg |= (cnt_down << 16) | cnt_up;
	if (repeat_count == 0) {
		/* Enable auto reload */
		reg |= 0x10000000;
	}
	ipu_di_write(ipu, di, reg, DI_SW_GEN1(wave_gen));
	reg = ipu_di_read(ipu, di, DI_STP_REP(wave_gen));
	reg &= ~(0xFFFF << (16 * ((wave_gen - 1) & 0x1)));
	reg |= repeat_count << (16 * ((wave_gen - 1) & 0x1));
	ipu_di_write(ipu, di, reg, DI_STP_REP(wave_gen));
}

static void _ipu_dc_map_link(struct ipu_soc *ipu,
		int current_map,
		int base_map_0, int buf_num_0,
		int base_map_1, int buf_num_1,
		int base_map_2, int buf_num_2)
{
	int ptr_0 = base_map_0 * 3 + buf_num_0;
	int ptr_1 = base_map_1 * 3 + buf_num_1;
	int ptr_2 = base_map_2 * 3 + buf_num_2;
	int ptr;
	u32 reg;
	ptr = (ptr_2 << 10) +  (ptr_1 << 5) + ptr_0;

	reg = ipu_dc_read(ipu, DC_MAP_CONF_PTR(current_map));
	reg &= ~(0x1F << ((16 * (current_map & 0x1))));
	reg |= ptr << ((16 * (current_map & 0x1)));
	ipu_dc_write(ipu, reg, DC_MAP_CONF_PTR(current_map));
}

static void _ipu_dc_map_config(struct ipu_soc *ipu,
		int map, int byte_num, int offset, int mask)
{
	int ptr = map * 3 + byte_num;
	u32 reg;

	reg = ipu_dc_read(ipu, DC_MAP_CONF_VAL(ptr));
	reg &= ~(0xFFFF << (16 * (ptr & 0x1)));
	reg |= ((offset << 8) | mask) << (16 * (ptr & 0x1));
	ipu_dc_write(ipu, reg, DC_MAP_CONF_VAL(ptr));

	reg = ipu_dc_read(ipu, DC_MAP_CONF_PTR(map));
	reg &= ~(0x1F << ((16 * (map & 0x1)) + (5 * byte_num)));
	reg |= ptr << ((16 * (map & 0x1)) + (5 * byte_num));
	ipu_dc_write(ipu, reg, DC_MAP_CONF_PTR(map));
}

static void _ipu_dc_map_clear(struct ipu_soc *ipu, int map)
{
	u32 reg = ipu_dc_read(ipu, DC_MAP_CONF_PTR(map));
	ipu_dc_write(ipu, reg & ~(0xFFFF << (16 * (map & 0x1))),
		     DC_MAP_CONF_PTR(map));
}

static void _ipu_dc_write_tmpl(struct ipu_soc *ipu,
			int word, u32 opcode, u32 operand, int map,
			int wave, int glue, int sync, int stop, int lf, int af)
{
	u32 reg, opcmd;

	switch(opcode) {
		case WROD:
			reg = sync;
			reg |= (glue << 4);
			reg |= (++wave << 11);
			reg |= (++map << 15);
			reg |= (operand << 20) & 0xFFF00000;
			ipu_dc_tmpl_write(ipu, reg, word * 8);

			opcmd = 0x18 | (lf << 1);
			reg = (operand >> 12);
			reg |= opcmd << 4;
			reg |= (stop << 9);
			ipu_dc_tmpl_write(ipu, reg, word * 8 + 4);
			break;

		case WRG:
			reg = sync;
			reg |= (glue << 4);
			reg |= (++wave << 11);
			reg |= ((operand & 0x1FFFF) << 15);
			ipu_dc_tmpl_write(ipu, reg, word * 8);

			opcmd = 0x01;
			reg = (operand >> 17);
			reg |= opcmd << 7;
			reg |= (stop << 9);
			ipu_dc_tmpl_write(ipu, reg, word * 8 + 4);
			break;

		case HMA:
			reg = (operand << 5);
			ipu_dc_tmpl_write(ipu, reg, word * 8);

			opcmd = 0x02;
			reg = (opcmd << 5);
			reg |= (stop << 9);
			ipu_dc_tmpl_write(ipu, reg, word * 8 + 4);
			break;

		case HMA1:
			reg = (operand << 5);
			ipu_dc_tmpl_write(ipu, reg, word * 8);

			opcmd = 0x01;
			reg = (opcmd << 5);
			reg |= (stop << 9);
			ipu_dc_tmpl_write(ipu, reg, word * 8 + 4);
			break;

		case BMA:
			reg = sync;
			reg |= (operand << 5);
			ipu_dc_tmpl_write(ipu, reg, word * 8);

			opcmd = 0x03;
			reg = (af << 3);
			reg |= (lf << 4);
			reg |= (opcmd << 5);
			reg |= (stop << 9);
			ipu_dc_tmpl_write(ipu, reg, word * 8 + 4);
			break;

		default:
			dev_err(ipu->dev, "WARNING: unsupported opcode.\n");
			break;
	}
}

static void _ipu_dc_link_event(struct ipu_soc *ipu,
		int chan, int event, int addr, int priority)
{
	u32 reg;
	u32 address_shift;
	if (event < DC_EVEN_UGDE0) {
		reg = ipu_dc_read(ipu, DC_RL_CH(chan, event));
		reg &= ~(0xFFFF << (16 * (event & 0x1)));
		reg |= ((addr << 8) | priority) << (16 * (event & 0x1));
		ipu_dc_write(ipu, reg, DC_RL_CH(chan, event));
	} else {
		reg = ipu_dc_read(ipu, DC_UGDE_0((event - DC_EVEN_UGDE0) / 2));
		if ((event - DC_EVEN_UGDE0) & 0x1) {
			reg &= ~(0x2FF << 16);
			reg |= (addr << 16);
			reg |= priority ? (2 << 24) : 0x0;
		} else {
			reg &= ~0xFC00FFFF;
			if (priority)
				chan = (chan >> 1) +
					((((chan & 0x1) + ((chan & 0x2) >> 1))) | (chan >> 3));
			else
				chan = 0x7;
			address_shift = ((event - DC_EVEN_UGDE0) >> 1) ? 7 : 8;
			reg |= (addr << address_shift) | (priority << 3) | chan;
		}
		ipu_dc_write(ipu, reg, DC_UGDE_0((event - DC_EVEN_UGDE0) / 2));
	}
}

/*     Y = R *  1.200 + G *  2.343 + B *  .453 + 0.250;
       U = R * -.672 + G * -1.328 + B *  2.000 + 512.250.;
       V = R *  2.000 + G * -1.672 + B * -.328 + 512.250.;*/
static const int rgb2ycbcr_coeff[5][3] = {
	{0x4D, 0x96, 0x1D},
	{-0x2B, -0x55, 0x80},
	{0x80, -0x6B, -0x15},
	{0x0000, 0x0200, 0x0200},	/* B0, B1, B2 */
	{0x2, 0x2, 0x2},	/* S0, S1, S2 */
};

/*     R = (1.164 * (Y - 16)) + (1.596 * (Cr - 128));
       G = (1.164 * (Y - 16)) - (0.392 * (Cb - 128)) - (0.813 * (Cr - 128));
       B = (1.164 * (Y - 16)) + (2.017 * (Cb - 128); */
static const int ycbcr2rgb_coeff[5][3] = {
	{0x095, 0x000, 0x0CC},
	{0x095, 0x3CE, 0x398},
	{0x095, 0x0FF, 0x000},
	{0x3E42, 0x010A, 0x3DD6},	/*B0,B1,B2 */
	{0x1, 0x1, 0x1},	/*S0,S1,S2 */
};

#define mask_a(a) ((u32)(a) & 0x3FF)
#define mask_b(b) ((u32)(b) & 0x3FFF)

/* Pls keep S0, S1 and S2 as 0x2 by using this convertion */
static int _rgb_to_yuv(int n, int red, int green, int blue)
{
	int c;
	c = red * rgb2ycbcr_coeff[n][0];
	c += green * rgb2ycbcr_coeff[n][1];
	c += blue * rgb2ycbcr_coeff[n][2];
	c /= 16;
	c += rgb2ycbcr_coeff[3][n] * 4;
	c += 8;
	c /= 16;
	if (c < 0)
		c = 0;
	if (c > 255)
		c = 255;
	return c;
}

/*
 * Row is for BG: 	RGB2YUV YUV2RGB RGB2RGB YUV2YUV CSC_NONE
 * Column is for FG:	RGB2YUV YUV2RGB RGB2RGB YUV2YUV CSC_NONE
 */
static struct dp_csc_param_t dp_csc_array[CSC_NUM][CSC_NUM] = {
{
	{DP_COM_CONF_CSC_DEF_BOTH, (void *)&rgb2ycbcr_coeff},
	{0, 0}, {0, 0},
	{DP_COM_CONF_CSC_DEF_BG, (void *)&rgb2ycbcr_coeff},
	{DP_COM_CONF_CSC_DEF_BG, (void *)&rgb2ycbcr_coeff}
},
{
	{0, 0},
	{DP_COM_CONF_CSC_DEF_BOTH, (void *)&ycbcr2rgb_coeff},
	{DP_COM_CONF_CSC_DEF_BG, (void *)&ycbcr2rgb_coeff},
	{0, 0},
	{DP_COM_CONF_CSC_DEF_BG, (void *)&ycbcr2rgb_coeff}
},
{
	{0, 0},
	{DP_COM_CONF_CSC_DEF_FG, (void *)&ycbcr2rgb_coeff},
	{0, 0}, {0, 0}, {0, 0}
},
{
	{DP_COM_CONF_CSC_DEF_FG, (void *)&rgb2ycbcr_coeff},
	{0, 0}, {0, 0}, {0, 0}, {0, 0}
},
{
	{DP_COM_CONF_CSC_DEF_FG, (void *)&rgb2ycbcr_coeff},
	{DP_COM_CONF_CSC_DEF_FG, (void *)&ycbcr2rgb_coeff},
	{0, 0}, {0, 0}, {0, 0}
}
};

void __ipu_dp_csc_setup(struct ipu_soc *ipu,
		int dp, struct dp_csc_param_t dp_csc_param,
		bool srm_mode_update)
{
	u32 reg;
	const int (*coeff)[5][3];

	if (dp_csc_param.mode >= 0) {
		reg = ipu_dp_read(ipu, DP_COM_CONF(dp));
		reg &= ~DP_COM_CONF_CSC_DEF_MASK;
		reg |= dp_csc_param.mode;
		reg |= (0x1 << 11);  // For BT656 display, Y range 16-235, U/V range 16-240.
		ipu_dp_write(ipu, reg, DP_COM_CONF(dp));
	}

	coeff = dp_csc_param.coeff;

	if (coeff) {
		ipu_dp_write(ipu, mask_a((*coeff)[0][0]) |
				(mask_a((*coeff)[0][1]) << 16), DP_CSC_A_0(dp));
		ipu_dp_write(ipu, mask_a((*coeff)[0][2]) |
				(mask_a((*coeff)[1][0]) << 16), DP_CSC_A_1(dp));
		ipu_dp_write(ipu, mask_a((*coeff)[1][1]) |
				(mask_a((*coeff)[1][2]) << 16), DP_CSC_A_2(dp));
		ipu_dp_write(ipu, mask_a((*coeff)[2][0]) |
				(mask_a((*coeff)[2][1]) << 16), DP_CSC_A_3(dp));
		ipu_dp_write(ipu, mask_a((*coeff)[2][2]) |
				(mask_b((*coeff)[3][0]) << 16) |
				((*coeff)[4][0] << 30), DP_CSC_0(dp));
		ipu_dp_write(ipu, mask_b((*coeff)[3][1]) | ((*coeff)[4][1] << 14) |
				(mask_b((*coeff)[3][2]) << 16) |
				((*coeff)[4][2] << 30), DP_CSC_1(dp));
	}

	if (srm_mode_update) {
		reg = ipu_cm_read(ipu, IPU_SRM_PRI2) | 0x8;
		ipu_cm_write(ipu, reg, IPU_SRM_PRI2);
	}
}

int _ipu_dp_init(struct ipu_soc *ipu,
		ipu_channel_t channel, uint32_t in_pixel_fmt,
		uint32_t out_pixel_fmt)
{
	int in_fmt, out_fmt;
	int dp;
	int partial = false;
	uint32_t reg;

	if (channel == MEM_FG_SYNC) {
		dp = DP_SYNC;
		partial = true;
	} else if (channel == MEM_BG_SYNC) {
		dp = DP_SYNC;
		partial = false;
	} else if (channel == MEM_BG_ASYNC0) {
		dp = DP_ASYNC0;
		partial = false;
	} else {
		return -EINVAL;
	}

	in_fmt = format_to_colorspace(in_pixel_fmt);
	out_fmt = format_to_colorspace(out_pixel_fmt);

	if (partial) {
		if (in_fmt == RGB) {
			if (out_fmt == RGB)
				ipu->fg_csc_type = RGB2RGB;
			else
				ipu->fg_csc_type = RGB2YUV;
		} else {
			if (out_fmt == RGB)
				ipu->fg_csc_type = YUV2RGB;
			else
				ipu->fg_csc_type = YUV2YUV;
		}
	} else {
		if (in_fmt == RGB) {
			if (out_fmt == RGB)
				ipu->bg_csc_type = RGB2RGB;
			else
				ipu->bg_csc_type = RGB2YUV;
		} else {
			if (out_fmt == RGB)
				ipu->bg_csc_type = YUV2RGB;
			else
				ipu->bg_csc_type = YUV2YUV;
		}
	}

	/* Transform color key from rgb to yuv if CSC is enabled */
	reg = ipu_dp_read(ipu, DP_COM_CONF(dp));
	if (ipu->color_key_4rgb && (reg & DP_COM_CONF_GWCKE) &&
			(((ipu->fg_csc_type == RGB2YUV) && (ipu->bg_csc_type == YUV2YUV)) ||
			 ((ipu->fg_csc_type == YUV2YUV) && (ipu->bg_csc_type == RGB2YUV)) ||
			 ((ipu->fg_csc_type == YUV2YUV) && (ipu->bg_csc_type == YUV2YUV)) ||
			 ((ipu->fg_csc_type == YUV2RGB) && (ipu->bg_csc_type == YUV2RGB)))) {
		int red, green, blue;
		int y, u, v;
		uint32_t color_key = ipu_dp_read(ipu, DP_GRAPH_WIND_CTRL(dp)) & 0xFFFFFFL;

		dev_dbg(ipu->dev, "_ipu_dp_init color key 0x%x need change to yuv fmt!\n", color_key);

		red = (color_key >> 16) & 0xFF;
		green = (color_key >> 8) & 0xFF;
		blue = color_key & 0xFF;

		y = _rgb_to_yuv(0, red, green, blue);
		u = _rgb_to_yuv(1, red, green, blue);
		v = _rgb_to_yuv(2, red, green, blue);
		color_key = (y << 16) | (u << 8) | v;

		reg = ipu_dp_read(ipu, DP_GRAPH_WIND_CTRL(dp)) & 0xFF000000L;
		ipu_dp_write(ipu, reg | color_key, DP_GRAPH_WIND_CTRL(dp));
		ipu->color_key_4rgb = false;

		dev_dbg(ipu->dev, "_ipu_dp_init color key change to yuv fmt 0x%x!\n", color_key);
	}

	__ipu_dp_csc_setup(ipu, dp,
			   dp_csc_array[ipu->bg_csc_type][ipu->fg_csc_type],
			   false);

	return 0;
}

void _ipu_dp_uninit(struct ipu_soc *ipu, ipu_channel_t channel)
{
	int dp;
	int partial = false;

	if (channel == MEM_FG_SYNC) {
		dp = DP_SYNC;
		partial = true;
	} else if (channel == MEM_BG_SYNC) {
		dp = DP_SYNC;
		partial = false;
	} else if (channel == MEM_BG_ASYNC0) {
		dp = DP_ASYNC0;
		partial = false;
	} else {
		return;
	}

	if (partial)
		ipu->fg_csc_type = CSC_NONE;
	else
		ipu->bg_csc_type = CSC_NONE;

	__ipu_dp_csc_setup(ipu, dp, dp_csc_array[ipu->bg_csc_type][ipu->fg_csc_type], false);
}

void _ipu_dc_init(struct ipu_soc *ipu, int dc_chan, int di, bool interlaced, uint32_t pixel_fmt)
{
	u32 reg = 0;
	int mcode_bt656_nl, mcode_bt656_nf, mcode_bt656_eof, mcode_bt656_eofield, mcode_bt656_data_w;

	if((pixel_fmt == IPU_PIX_FMT_BT656) || (pixel_fmt == IPU_PIX_FMT_BT1120)) {
		if (interlaced) {
			mcode_bt656_nl = DC_MCODE_BT656_NL;
			mcode_bt656_nf = DC_MCODE_BT656_NF;
			mcode_bt656_eof = DC_MCODE_BT656_EOF;
			mcode_bt656_eofield = DC_MCODE_BT656_EOFIELD;
			mcode_bt656_data_w = DC_MCODE_BT656_DATA_W;
		} else {
			mcode_bt656_nl = DC_MCODE_BT656_P_NL;
			mcode_bt656_nf = DC_MCODE_BT656_P_NF;
			mcode_bt656_eof = DC_MCODE_BT656_P_EOF;
			mcode_bt656_eofield = 0;
			mcode_bt656_data_w = DC_MCODE_BT656_P_DATA_W;
		}
	}

	if ((dc_chan == 1) || (dc_chan == 5)) {
		if (interlaced) {
			if((pixel_fmt == IPU_PIX_FMT_BT656) || (pixel_fmt == IPU_PIX_FMT_BT1120)) {
				_ipu_dc_link_event(ipu, dc_chan, DC_EVT_NL, mcode_bt656_nl, 2);
				_ipu_dc_link_event(ipu, dc_chan, DC_EVT_NF, mcode_bt656_nf, 5);
				_ipu_dc_link_event(ipu, dc_chan, DC_EVT_EOF, mcode_bt656_eof, 4);
				_ipu_dc_link_event(ipu, dc_chan, DC_EVT_EOFIELD, mcode_bt656_eofield, 3);
				_ipu_dc_link_event(ipu, dc_chan, DC_EVT_NEW_DATA, mcode_bt656_data_w, 0);
			} else {
				if (di) {
					_ipu_dc_link_event(ipu, dc_chan, DC_EVT_NL, 1, 3);
					_ipu_dc_link_event(ipu, dc_chan, DC_EVT_EOL, 1, 2);
					_ipu_dc_link_event(ipu, dc_chan, DC_EVT_NEW_DATA, 1, 1);
					if ((pixel_fmt == IPU_PIX_FMT_YUYV) ||
					(pixel_fmt == IPU_PIX_FMT_UYVY) ||
					(pixel_fmt == IPU_PIX_FMT_YVYU) ||
					(pixel_fmt == IPU_PIX_FMT_VYUY)) {
						_ipu_dc_link_event(ipu, dc_chan, DC_ODD_UGDE1, 9, 5);
						_ipu_dc_link_event(ipu, dc_chan, DC_EVEN_UGDE1, 8, 5);
					}
				} else {
					_ipu_dc_link_event(ipu, dc_chan, DC_EVT_NL, 0, 3);
					_ipu_dc_link_event(ipu, dc_chan, DC_EVT_EOL, 0, 2);
					_ipu_dc_link_event(ipu, dc_chan, DC_EVT_NEW_DATA, 0, 1);
					if ((pixel_fmt == IPU_PIX_FMT_YUYV) ||
					(pixel_fmt == IPU_PIX_FMT_UYVY) ||
					(pixel_fmt == IPU_PIX_FMT_YVYU) ||
					(pixel_fmt == IPU_PIX_FMT_VYUY)) {
						_ipu_dc_link_event(ipu, dc_chan, DC_ODD_UGDE0, 10, 5);
						_ipu_dc_link_event(ipu, dc_chan, DC_EVEN_UGDE0, 11, 5);
					}
				}
			}
		} else {
			if((pixel_fmt == IPU_PIX_FMT_BT656) || (pixel_fmt == IPU_PIX_FMT_BT1120)) {
				_ipu_dc_link_event(ipu, dc_chan, DC_EVT_NL, mcode_bt656_nl, 2);
				_ipu_dc_link_event(ipu, dc_chan, DC_EVT_NF, mcode_bt656_nf, 4);
				_ipu_dc_link_event(ipu, dc_chan, DC_EVT_EOF, mcode_bt656_eof, 3);
				_ipu_dc_link_event(ipu, dc_chan, DC_EVT_NEW_DATA, mcode_bt656_data_w, 0);
			} else {
				if (di) {
					_ipu_dc_link_event(ipu, dc_chan, DC_EVT_NL, 2, 3);
					_ipu_dc_link_event(ipu, dc_chan, DC_EVT_EOL, 3, 2);
					_ipu_dc_link_event(ipu, dc_chan, DC_EVT_NEW_DATA, 1, 1);
					if ((pixel_fmt == IPU_PIX_FMT_YUYV) ||
					(pixel_fmt == IPU_PIX_FMT_UYVY) ||
					(pixel_fmt == IPU_PIX_FMT_YVYU) ||
					(pixel_fmt == IPU_PIX_FMT_VYUY)) {
						_ipu_dc_link_event(ipu, dc_chan, DC_ODD_UGDE1, 9, 5);
						_ipu_dc_link_event(ipu, dc_chan, DC_EVEN_UGDE1, 8, 5);
					}
				} else {
					_ipu_dc_link_event(ipu, dc_chan, DC_EVT_NL, 5, 3);
					_ipu_dc_link_event(ipu, dc_chan, DC_EVT_EOL, 6, 2);
					_ipu_dc_link_event(ipu, dc_chan, DC_EVT_NEW_DATA, 12, 1);
					if ((pixel_fmt == IPU_PIX_FMT_YUYV) ||
					(pixel_fmt == IPU_PIX_FMT_UYVY) ||
					(pixel_fmt == IPU_PIX_FMT_YVYU) ||
					(pixel_fmt == IPU_PIX_FMT_VYUY)) {
						_ipu_dc_link_event(ipu, dc_chan, DC_ODD_UGDE0, 10, 5);
						_ipu_dc_link_event(ipu, dc_chan, DC_EVEN_UGDE0, 11, 5);
					}
				}
			}
		}

		if((pixel_fmt == IPU_PIX_FMT_BT656) || (pixel_fmt == IPU_PIX_FMT_BT1120)) {
			_ipu_dc_link_event(ipu, dc_chan, DC_EVT_EOL, 0, 0);
			_ipu_dc_link_event(ipu, dc_chan, DC_EVT_NFIELD, 0, 0);
			_ipu_dc_link_event(ipu, dc_chan, DC_EVT_NEW_CHAN, 0, 0);
			_ipu_dc_link_event(ipu, dc_chan, DC_EVT_NEW_ADDR, 0, 0);
			if (!interlaced)
				_ipu_dc_link_event(ipu, dc_chan, DC_EVT_EOFIELD, 0, 0);

			if (di) {
				_ipu_dc_link_event(ipu, dc_chan, DC_ODD_UGDE1, mcode_bt656_data_w, 1);
				if(pixel_fmt == IPU_PIX_FMT_BT656)
					_ipu_dc_link_event(ipu, dc_chan, DC_EVEN_UGDE1, mcode_bt656_data_w + 3, 1);
				else
					_ipu_dc_link_event(ipu, dc_chan, DC_EVEN_UGDE1, mcode_bt656_data_w + 1, 1);
			} else {
				_ipu_dc_link_event(ipu, dc_chan, DC_ODD_UGDE0, mcode_bt656_data_w, 1);
				if(pixel_fmt == IPU_PIX_FMT_BT656)
					_ipu_dc_link_event(ipu, dc_chan, DC_EVEN_UGDE0, mcode_bt656_data_w + 3, 1);
				else
					_ipu_dc_link_event(ipu, dc_chan, DC_EVEN_UGDE0, mcode_bt656_data_w + 1, 1);
			}
		} else {
			_ipu_dc_link_event(ipu, dc_chan, DC_EVT_NF, 0, 0);
			_ipu_dc_link_event(ipu, dc_chan, DC_EVT_NFIELD, 0, 0);
			_ipu_dc_link_event(ipu, dc_chan, DC_EVT_EOF, 0, 0);
			_ipu_dc_link_event(ipu, dc_chan, DC_EVT_EOFIELD, 0, 0);
			_ipu_dc_link_event(ipu, dc_chan, DC_EVT_NEW_CHAN, 0, 0);
			_ipu_dc_link_event(ipu, dc_chan, DC_EVT_NEW_ADDR, 0, 0);
		}

		reg = 0x2;
		reg |= DC_DISP_ID_SYNC(di) << DC_WR_CH_CONF_PROG_DISP_ID_OFFSET;
		reg |= di << 2;
		if (interlaced)
			reg |= DC_WR_CH_CONF_FIELD_MODE;
	} else if ((dc_chan == 8) || (dc_chan == 9)) {
		/* async channels */
		_ipu_dc_link_event(ipu, dc_chan, DC_EVT_NEW_DATA_W_0, 0x64, 1);
		_ipu_dc_link_event(ipu, dc_chan, DC_EVT_NEW_DATA_W_1, 0x64, 1);

		reg = 0x3;
		reg |= DC_DISP_ID_SERIAL << DC_WR_CH_CONF_PROG_DISP_ID_OFFSET;
	}
	ipu_dc_write(ipu, reg, DC_WR_CH_CONF(dc_chan));

	ipu_dc_write(ipu, 0x00000000, DC_WR_CH_ADDR(dc_chan));

	ipu_dc_write(ipu, 0x00000084, DC_GEN);
}

void _ipu_dc_uninit(struct ipu_soc *ipu, int dc_chan, int di)
{
	if ((dc_chan == 1) || (dc_chan == 5)) {
		_ipu_dc_link_event(ipu, dc_chan, DC_EVT_NL, 0, 0);
		_ipu_dc_link_event(ipu, dc_chan, DC_EVT_EOL, 0, 0);
		_ipu_dc_link_event(ipu, dc_chan, DC_EVT_NEW_DATA, 0, 0);
		_ipu_dc_link_event(ipu, dc_chan, DC_EVT_NF, 0, 0);
		_ipu_dc_link_event(ipu, dc_chan, DC_EVT_NFIELD, 0, 0);
		_ipu_dc_link_event(ipu, dc_chan, DC_EVT_EOF, 0, 0);
		_ipu_dc_link_event(ipu, dc_chan, DC_EVT_EOFIELD, 0, 0);
		_ipu_dc_link_event(ipu, dc_chan, DC_EVT_NEW_CHAN, 0, 0);
		_ipu_dc_link_event(ipu, dc_chan, DC_EVT_NEW_ADDR, 0, 0);
		if (di == 0) {
			_ipu_dc_link_event(ipu, dc_chan, DC_ODD_UGDE0, 0, 0);
			_ipu_dc_link_event(ipu, dc_chan, DC_EVEN_UGDE0, 0, 0);
		} else if (di == 1) {
			_ipu_dc_link_event(ipu, dc_chan, DC_ODD_UGDE1, 0, 0);
			_ipu_dc_link_event(ipu, dc_chan, DC_EVEN_UGDE1, 0, 0);
		}
	} else if ((dc_chan == 8) || (dc_chan == 9)) {
		_ipu_dc_link_event(ipu, dc_chan, DC_EVT_NEW_ADDR_W_0, 0, 0);
		_ipu_dc_link_event(ipu, dc_chan, DC_EVT_NEW_ADDR_W_1, 0, 0);
		_ipu_dc_link_event(ipu, dc_chan, DC_EVT_NEW_CHAN_W_0, 0, 0);
		_ipu_dc_link_event(ipu, dc_chan, DC_EVT_NEW_CHAN_W_1, 0, 0);
		_ipu_dc_link_event(ipu, dc_chan, DC_EVT_NEW_DATA_W_0, 0, 0);
		_ipu_dc_link_event(ipu, dc_chan, DC_EVT_NEW_DATA_W_1, 0, 0);
		_ipu_dc_link_event(ipu, dc_chan, DC_EVT_NEW_ADDR_R_0, 0, 0);
		_ipu_dc_link_event(ipu, dc_chan, DC_EVT_NEW_ADDR_R_1, 0, 0);
		_ipu_dc_link_event(ipu, dc_chan, DC_EVT_NEW_CHAN_R_0, 0, 0);
		_ipu_dc_link_event(ipu, dc_chan, DC_EVT_NEW_CHAN_R_1, 0, 0);
		_ipu_dc_link_event(ipu, dc_chan, DC_EVT_NEW_DATA_R_0, 0, 0);
		_ipu_dc_link_event(ipu, dc_chan, DC_EVT_NEW_DATA_R_1, 0, 0);
	}
}

int _ipu_disp_chan_is_interlaced(struct ipu_soc *ipu, ipu_channel_t channel)
{
	if (channel == MEM_DC_SYNC)
		return !!(ipu_dc_read(ipu, DC_WR_CH_CONF_1) &
			  DC_WR_CH_CONF_FIELD_MODE);
	else if ((channel == MEM_BG_SYNC) || (channel == MEM_FG_SYNC))
		return !!(ipu_dc_read(ipu, DC_WR_CH_CONF_5) &
			  DC_WR_CH_CONF_FIELD_MODE);
	return 0;
}

void _ipu_dp_dc_enable(struct ipu_soc *ipu, ipu_channel_t channel)
{
	int di;
	uint32_t reg;
	uint32_t dc_chan;
	int irq = 0;

	if (channel == MEM_FG_SYNC)
		irq = IPU_IRQ_DP_SF_END;
	else if (channel == MEM_DC_SYNC)
		dc_chan = 1;
	else if (channel == MEM_BG_SYNC)
		dc_chan = 5;
	else
		return;

	if (channel == MEM_FG_SYNC) {
		/* Enable FG channel */
		reg = ipu_dp_read(ipu, DP_COM_CONF(DP_SYNC));
		ipu_dp_write(ipu, reg | DP_COM_CONF_FG_EN, DP_COM_CONF(DP_SYNC));

		reg = ipu_cm_read(ipu, IPU_SRM_PRI2) | 0x8;
		ipu_cm_write(ipu, reg, IPU_SRM_PRI2);
		return;
	} else if (channel == MEM_BG_SYNC) {
		reg = ipu_cm_read(ipu, IPU_SRM_PRI2) | 0x8;
		ipu_cm_write(ipu, reg, IPU_SRM_PRI2);
	}

	di = ipu->dc_di_assignment[dc_chan];

	/* Make sure other DC sync channel is not assigned same DI */
	reg = ipu_dc_read(ipu, DC_WR_CH_CONF(6 - dc_chan));
	if ((di << 2) == (reg & DC_WR_CH_CONF_PROG_DI_ID)) {
		reg &= ~DC_WR_CH_CONF_PROG_DI_ID;
		reg |= di ? 0 : DC_WR_CH_CONF_PROG_DI_ID;
		ipu_dc_write(ipu, reg, DC_WR_CH_CONF(6 - dc_chan));
	}

	reg = ipu_dc_read(ipu, DC_WR_CH_CONF(dc_chan));
	reg |= 4 << DC_WR_CH_CONF_PROG_TYPE_OFFSET;
	ipu_dc_write(ipu, reg, DC_WR_CH_CONF(dc_chan));

	clk_prepare_enable(ipu->pixel_clk[di]);
	ipu->pixel_clk_en[ipu->dc_di_assignment[dc_chan]] = true;
}

static irqreturn_t dc_irq_handler(int irq, void *dev_id)
{
	struct ipu_soc *ipu = dev_id;
	struct completion *comp = &ipu->dc_comp;
	uint32_t reg;
	uint32_t dc_chan;

	if (irq == IPU_IRQ_DC_FC_1)
		dc_chan = 1;
	else
		dc_chan = 5;

	if (!ipu->dc_swap) {
		reg = ipu_dc_read(ipu, DC_WR_CH_CONF(dc_chan));
		reg &= ~DC_WR_CH_CONF_PROG_TYPE_MASK;
		ipu_dc_write(ipu, reg, DC_WR_CH_CONF(dc_chan));

		reg = ipu_cm_read(ipu, IPU_DISP_GEN);
		if (ipu->dc_di_assignment[dc_chan])
			reg &= ~DI1_COUNTER_RELEASE;
		else
			reg &= ~DI0_COUNTER_RELEASE;
		ipu_cm_write(ipu, reg, IPU_DISP_GEN);
	}

	complete(comp);
	return IRQ_HANDLED;
}

void _ipu_dp_dc_disable(struct ipu_soc *ipu, ipu_channel_t channel, bool swap)
{
	int ret;
	uint32_t reg;
	uint32_t csc;
	uint32_t dc_chan;
	int irq = 0;
	int timeout = 50;

	ipu->dc_swap = swap;

	if (channel == MEM_DC_SYNC) {
		dc_chan = 1;
		irq = IPU_IRQ_DC_FC_1;
	} else if (channel == MEM_BG_SYNC) {
		dc_chan = 5;
		irq = IPU_IRQ_DP_SF_END;
	} else if (channel == MEM_FG_SYNC) {
		/* Disable FG channel */
		dc_chan = 5;

		reg = ipu_dp_read(ipu, DP_COM_CONF(DP_SYNC));
		csc = reg & DP_COM_CONF_CSC_DEF_MASK;
		if (csc == DP_COM_CONF_CSC_DEF_FG)
			reg &= ~DP_COM_CONF_CSC_DEF_MASK;

		reg &= ~DP_COM_CONF_FG_EN;
		ipu_dp_write(ipu, reg, DP_COM_CONF(DP_SYNC));

		reg = ipu_cm_read(ipu, IPU_SRM_PRI2) | 0x8;
		ipu_cm_write(ipu, reg, IPU_SRM_PRI2);

		if (ipu_is_channel_busy(ipu, MEM_BG_SYNC)) {
			ipu_cm_write(ipu, IPUIRQ_2_MASK(IPU_IRQ_DP_SF_END),
				IPUIRQ_2_STATREG(ipu->devtype,
							IPU_IRQ_DP_SF_END));
			while ((ipu_cm_read(ipu,
				IPUIRQ_2_STATREG(ipu->devtype,
							IPU_IRQ_DP_SF_END)) &
				IPUIRQ_2_MASK(IPU_IRQ_DP_SF_END)) == 0) {
				msleep(2);
				timeout -= 2;
				if (timeout <= 0)
					break;
			}
		}
		return;
	} else {
		return;
	}

	init_completion(&ipu->dc_comp);
	ret = ipu_request_irq(ipu, irq, dc_irq_handler, 0, NULL, ipu);
	if (ret < 0) {
		dev_err(ipu->dev, "DC irq %d in use\n", irq);
		return;
	}
	ret = wait_for_completion_timeout(&ipu->dc_comp, msecs_to_jiffies(50));
	ipu_free_irq(ipu, irq, ipu);
	dev_dbg(ipu->dev, "DC stop timeout - %d * 10ms\n", 5 - ret);

	if (ipu->dc_swap) {
		/* Swap DC channel 1 and 5 settings, and disable old dc chan */
		reg = ipu_dc_read(ipu, DC_WR_CH_CONF(dc_chan));
		ipu_dc_write(ipu, reg, DC_WR_CH_CONF(6 - dc_chan));
		reg &= ~DC_WR_CH_CONF_PROG_TYPE_MASK;
		reg ^= DC_WR_CH_CONF_PROG_DI_ID;
		ipu_dc_write(ipu, reg, DC_WR_CH_CONF(dc_chan));
	}
}

void _ipu_init_dc_mappings(struct ipu_soc *ipu)
{
	/* IPU_PIX_FMT_RGB24 */
	_ipu_dc_map_clear(ipu, 0);
	_ipu_dc_map_config(ipu, 0, 0, 7, 0xFF);
	_ipu_dc_map_config(ipu, 0, 1, 15, 0xFF);
	_ipu_dc_map_config(ipu, 0, 2, 23, 0xFF);

	/* IPU_PIX_FMT_RGB666 */
	_ipu_dc_map_clear(ipu, 1);
	_ipu_dc_map_config(ipu, 1, 0, 5, 0xFC);
	_ipu_dc_map_config(ipu, 1, 1, 11, 0xFC);
	_ipu_dc_map_config(ipu, 1, 2, 17, 0xFC);

	/* IPU_PIX_FMT_YUV444 */
	_ipu_dc_map_clear(ipu, 2);
	_ipu_dc_map_config(ipu, 2, 0, 15, 0xFF);
	_ipu_dc_map_config(ipu, 2, 1, 23, 0xFF);
	_ipu_dc_map_config(ipu, 2, 2, 7, 0xFF);

	/* IPU_PIX_FMT_RGB565 */
	_ipu_dc_map_clear(ipu, 3);
	_ipu_dc_map_config(ipu, 3, 0, 4, 0xF8);
	_ipu_dc_map_config(ipu, 3, 1, 10, 0xFC);
	_ipu_dc_map_config(ipu, 3, 2, 15, 0xF8);

	/* IPU_PIX_FMT_LVDS666 */
	_ipu_dc_map_clear(ipu, 4);
	_ipu_dc_map_config(ipu, 4, 0, 5, 0xFC);
	_ipu_dc_map_config(ipu, 4, 1, 13, 0xFC);
	_ipu_dc_map_config(ipu, 4, 2, 21, 0xFC);

#ifdef BT656_IF_DI_MSB
	/* IPU_PIX_FMT_VYUY 16bit width */
	_ipu_dc_map_clear(ipu, 5);
	_ipu_dc_map_config(ipu, 5, 0, BT656_IF_DI_MSB - 8, 0xFF);
	_ipu_dc_map_config(ipu, 5, 1, 0, 0x0);
	_ipu_dc_map_config(ipu, 5, 2, BT656_IF_DI_MSB, 0xFF);
	_ipu_dc_map_clear(ipu, 6);
	_ipu_dc_map_config(ipu, 6, 0, 0, 0x0);
	_ipu_dc_map_config(ipu, 6, 1, BT656_IF_DI_MSB - 8, 0xFF);
	_ipu_dc_map_config(ipu, 6, 2, BT656_IF_DI_MSB, 0xFF);

	// IPU_PIX_FMT_UYVY 16bit width for BT1120
	_ipu_dc_map_clear(ipu, 7);	//UY
	_ipu_dc_map_link(ipu, 7, 6, 0, 6, 1, 6, 2);
	_ipu_dc_map_clear(ipu, 8);	//VY
	_ipu_dc_map_link(ipu, 8, 5, 0, 5, 1, 5, 2);

	// IPU_PIX_FMT_UYVY 8bit width for BT656
	_ipu_dc_map_clear(ipu, 9);	//U
	_ipu_dc_map_link(ipu, 9, 6, 0, 6, 2, 6, 0);
	_ipu_dc_map_clear(ipu, 10);  //Y
	_ipu_dc_map_link(ipu, 10, 6, 0, 6, 0, 6, 2);
	_ipu_dc_map_clear(ipu, 11);  //V
	_ipu_dc_map_link(ipu, 11, 6, 2, 6, 0, 6, 0);
#else
	/* IPU_PIX_FMT_VYUY 16bit width */
	_ipu_dc_map_clear(ipu, 5);
	_ipu_dc_map_config(ipu, 5, 0, 7, 0xFF);
	_ipu_dc_map_config(ipu, 5, 1, 0, 0x0);
	_ipu_dc_map_config(ipu, 5, 2, 15, 0xFF);
	_ipu_dc_map_clear(ipu, 6);
	_ipu_dc_map_config(ipu, 6, 0, 0, 0x0);
	_ipu_dc_map_config(ipu, 6, 1, 7, 0xFF);
	_ipu_dc_map_config(ipu, 6, 2, 15, 0xFF);

	/* IPU_PIX_FMT_UYVY 16bit width */
	_ipu_dc_map_clear(ipu, 7);
	_ipu_dc_map_link(ipu, 7, 6, 0, 6, 1, 6, 2);
	_ipu_dc_map_clear(ipu, 8);
	_ipu_dc_map_link(ipu, 8, 5, 0, 5, 1, 5, 2);

	/* IPU_PIX_FMT_YUYV 16bit width */
	_ipu_dc_map_clear(ipu, 9);
	_ipu_dc_map_link(ipu, 9, 5, 2, 5, 1, 5, 0);
	_ipu_dc_map_clear(ipu, 10);
	_ipu_dc_map_link(ipu, 10, 5, 1, 5, 2, 5, 0);

	/* IPU_PIX_FMT_YVYU 16bit width */
	_ipu_dc_map_clear(ipu, 11);
	_ipu_dc_map_link(ipu, 11, 5, 1, 5, 2, 5, 0);
	_ipu_dc_map_clear(ipu, 12);
	_ipu_dc_map_link(ipu, 12, 5, 2, 5, 1, 5, 0);
#endif

	/* IPU_PIX_FMT_GBR24 */
	/* IPU_PIX_FMT_VYU444 */
	_ipu_dc_map_clear(ipu, 13);
	_ipu_dc_map_link(ipu, 13, 0, 2, 0, 0, 0, 1);

	/* IPU_PIX_FMT_BGR24 */
	_ipu_dc_map_clear(ipu, 14);
	_ipu_dc_map_link(ipu, 14, 0, 2, 0, 1, 0, 0);

	/* IPU_PIX_FMT_BGR666 */
	_ipu_dc_map_clear(ipu, 15);
	_ipu_dc_map_link(ipu, 15, 1, 2, 1, 1, 1, 0);
}

int _ipu_pixfmt_to_map(uint32_t fmt)
{
	switch (fmt) {
	case IPU_PIX_FMT_GENERIC:
	case IPU_PIX_FMT_RGB24:
		return 0;
	case IPU_PIX_FMT_RGB666:
		return 1;
	case IPU_PIX_FMT_YUV444:
		return 2;
	case IPU_PIX_FMT_RGB565:
		return 3;
	case IPU_PIX_FMT_LVDS666:
		return 4;
	case IPU_PIX_FMT_VYUY:
		return 6;
#ifdef BT656_IF_DI_MSB
	case IPU_PIX_FMT_UYVY:
	case IPU_PIX_FMT_BT1120:
		return 8;
	case IPU_PIX_FMT_BT656:
		return 11;
#else
	case IPU_PIX_FMT_UYVY:
		return 8;
	case IPU_PIX_FMT_YUYV:
		return 10;
	case IPU_PIX_FMT_YVYU:
		return 12;
#endif
	case IPU_PIX_FMT_GBR24:
	case IPU_PIX_FMT_VYU444:
		return 13;
	case IPU_PIX_FMT_BGR24:
		return 14;
	case IPU_PIX_FMT_BGR666:
		return 15;
	}

	return -1;
}

/*!
 * This function sets the colorspace for of dp.
 * modes.
 *
 * @param	ipu		ipu handler
 * @param       channel         Input parameter for the logical channel ID.
 *
 * @param       param         	If it's not NULL, update the csc table
 *                              with this parameter.
 *
 * @return      N/A
 */
void _ipu_dp_set_csc_coefficients(struct ipu_soc *ipu, ipu_channel_t channel, int32_t param[][3])
{
	int dp;
	struct dp_csc_param_t dp_csc_param;

	if (channel == MEM_FG_SYNC)
		dp = DP_SYNC;
	else if (channel == MEM_BG_SYNC)
		dp = DP_SYNC;
	else if (channel == MEM_BG_ASYNC0)
		dp = DP_ASYNC0;
	else
		return;

	dp_csc_param.mode = -1;
	dp_csc_param.coeff = param;
	__ipu_dp_csc_setup(ipu, dp, dp_csc_param, true);
}

void ipu_set_csc_coefficients(struct ipu_soc *ipu, ipu_channel_t channel, int32_t param[][3])
{
	_ipu_dp_set_csc_coefficients(ipu, channel, param);
}
EXPORT_SYMBOL(ipu_set_csc_coefficients);

static void _ipu_dc_setup_bt656_progressive(struct ipu_soc *ipu,
			    int u_map, int y_map, int v_map,
			    bool is_bt1120, int di_msb,
			    uint32_t bt656_h_start_width,
			    uint32_t bt656_v_start_width,
			    uint32_t bt656_v_end_width)
{
	uint32_t microcode_addr_NL, microcode_addr_DataW;
	uint32_t microcode_addr_NF, microcode_addr_EOF, microcode_addr_BlankDone;
	uint32_t microcode_addr_EAV_FF0000B6, microcode_addr_SAV_FF0000AB;
	uint32_t microcode_addr_EAV_FF00009D, microcode_addr_SAV_FF000080;
	uint32_t loop_blank_video_times;
	uint32_t loop_frame_blank_line_times;
	uint32_t microcode_start_addr, microcode_addr;
	uint32_t loop_N_mode, temp, i;

	microcode_addr = microcode_start_addr = DC_MCODE_BT656_P_EOF;

	microcode_addr_EOF = microcode_start_addr + 0x0;  //offset 0 //DC_MCODE_BT656_P_EOF
	microcode_addr_EAV_FF0000B6 = microcode_start_addr + 0x1;  //offset 1
	microcode_addr_SAV_FF0000AB = microcode_start_addr + 0xC;  //offset 12
	microcode_addr_NF = microcode_start_addr + 0x14;  //offset 20, DC_MCODE_BT656_P_NF
	microcode_addr_EAV_FF00009D = microcode_start_addr + 0x17;  //offset 23
	microcode_addr_SAV_FF000080 = microcode_start_addr + 0x22;  //offset 34
	microcode_addr_DataW = microcode_start_addr + 0x28;  //offset 40 //DC_MCODE_BT656_P_DATA_W
	microcode_addr_NL = microcode_start_addr + 0x2C;  //offset 44 //DC_MCODE_BT656_P_NL
	microcode_addr_BlankDone = microcode_start_addr + 0x2E;  //offset 46

	// loop_blank_video_times max value is 255, 8 bits
	if(is_bt1120)
		temp = bt656_h_start_width - 8;  //horizontal blanking + 1
	else
		temp = (bt656_h_start_width - 8) / 2;  //horizontal blanking + 1

	if (temp < 0x100) {
		loop_N_mode = 1;
		loop_blank_video_times = temp - 1;
	} else if (temp < 0x200) {
		if ((temp & 0x1) != 0) {
			dev_err(ipu->dev, "_ipu_dc_setup_bt656_progressive: bt656_h_start_width is not aligned in mode 2.\n");
			return;
		}
		loop_N_mode = 2;
		loop_blank_video_times = (temp / 2) - 1;
	} else if (temp < 0x400) {
		if ((temp & 0x3) != 0) {
			dev_err(ipu->dev, "_ipu_dc_setup_bt656_progressive: bt656_h_start_width is not aligned in mode 4.\n");
			return;
		}
		if (!is_bt1120) {
			dev_err(ipu->dev, "_ipu_dc_setup_bt656_progressive: bt656 interface is not supported in mode 4.\n");
			return;
		}
		loop_N_mode = 4;
		loop_blank_video_times = (temp / 4) - 1;
	} else {
		dev_err(ipu->dev, "_ipu_dc_setup_bt656_progressive: bt656_h_start_width = %d is too big.\n", bt656_h_start_width);
		return;
	}

	loop_frame_blank_line_times = bt656_v_start_width + bt656_v_end_width - 1;  //Vertical Blanking lines for one frame

	// offset 0, microcode_addr_EOF
	microcode_addr = microcode_addr_EOF;
	if(is_bt1120) {
		//Send data VY
		_ipu_dc_write_tmpl(ipu, microcode_addr, WROD, 0, v_map, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	} else {
		//Send data Y
		_ipu_dc_write_tmpl(ipu, microcode_addr, WROD, 0, y_map, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	}

	//Vertical Blanking, EAV_FF0000B6
	// offset 1
	microcode_addr = microcode_addr_EAV_FF0000B6;
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0xFF << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_HSYNC, 0, 0, 0);
	microcode_addr ++;

	// offset 2
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, 0x00, 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	//Store jump address 1, microcode_addr_SAV_FF0000AB
	// offset 3
	_ipu_dc_write_tmpl(ipu, microcode_addr, HMA1, microcode_addr_SAV_FF0000AB, 0, 0, 0, 0, 0, 0, 0);
	microcode_addr ++;

	// offset 4
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, 0x00, 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	// offset 5
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0xB6 << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	//Store jump address 0, microcode_addr + 1 for blank video
	// offset 6
	_ipu_dc_write_tmpl(ipu, microcode_addr, HMA, microcode_addr + 1, 0, 0, 0, 0, 0, 0, 0);
	microcode_addr ++;

	//Blank Video between EAV and SAV, UYVY 80 10 80 10
	// offset 7,8,9,10
	for (i=0; i<loop_N_mode; i++) {
		if(is_bt1120) {
			//Send 0x1080
			_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0x1080 << (di_msb - 15)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
			microcode_addr ++;
		} else {
			//Send 0x80
			_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0x80 << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
			microcode_addr ++;

			//Send 0x10
			_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0x10 << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
			microcode_addr ++;
		}
	}

	//LOOP:
	//Jump Store jump address 0 loop_blank_video_times, then jump Store jump address 1
	// offset 11
	_ipu_dc_write_tmpl(ipu, microcode_addr, BMA, loop_blank_video_times, 0, 0, 0, 0, 0, 1, 1);

	//Vertical Blanking, SAV_FF0000AB
	// offset 12
	microcode_addr = microcode_addr_SAV_FF0000AB;
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0xFF << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	// offset 13
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, 0x00, 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	//Store jump address 1: microcode_addr_EAV_FF0000B6
	// offset 14
	_ipu_dc_write_tmpl(ipu, microcode_addr, HMA1, microcode_addr_EAV_FF0000B6, 0, 0, 0, 0, 0, 0, 0);
	microcode_addr ++;

	// offset 15
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, 0x00, 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	// offset 16
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0xAB << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	//Store jump address 0: microcode_addr_BlankDone
	// offset 17
	_ipu_dc_write_tmpl(ipu, microcode_addr, HMA, microcode_addr_BlankDone, 0, 0, 0, 0, 0, 0, 0);
	microcode_addr ++;

	// offset 18, Blank data in Active Video
	if(is_bt1120) {
		//Send 0x1080 and keep
		_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0x1080 << (di_msb - 15)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
		microcode_addr ++;
	} else {
		//Send 0x10 and keep
		_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0x10 << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
		microcode_addr ++;
	}

	//Jump Store jump address 1 loop_frame_blank_line_times then jump Store jump address 0
	// offset 19
	_ipu_dc_write_tmpl(ipu, microcode_addr, BMA, loop_frame_blank_line_times, 0, 0, 0, 0, 0, 0, 0);

	//New Frame, start from Acvite Video, EAV = FF 00 00 9D
	// microcode_addr_NF
	//Store jump address 0, microcode_addr_EAV_FF00009D + 1
	// offset 20
	microcode_addr = microcode_addr_NF;
	_ipu_dc_write_tmpl(ipu, microcode_addr, HMA, microcode_addr_EAV_FF00009D + 1, 0, 0, 0, 0, 0, 0, 0);
	microcode_addr ++;

	// offset 21
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0xFF << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_AFIELD, 0, 0, 0);
	microcode_addr ++;

	//Jump Store jump address 0, microcode_addr_EAV_FF00009D + 1
	// offset 22
	_ipu_dc_write_tmpl(ipu, microcode_addr, BMA, 0, 0, 0, 0, 0, 0, 0, 0);

	//Acvite Video, EAV = FF 00 00 9D
	// microcode_addr_EAV_FF00009D
	// offset 23
	microcode_addr = microcode_addr_EAV_FF00009D;
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0xFF << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_HSYNC, 0, 0, 0);
	microcode_addr ++;

	// offset 24
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, 0x00, 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	//Store jump address 0: microcode_addr_SAV_FF000080
	// offset 25
	_ipu_dc_write_tmpl(ipu, microcode_addr, HMA, microcode_addr_SAV_FF000080, 0, 0, 0, 0, 0, 0, 0);
	microcode_addr ++;

	// offset 26
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, 0x00, 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	// offset 27
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0x9D << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	//Store jump address 1: microcode_addr + 1
	// offset 28
	_ipu_dc_write_tmpl(ipu, microcode_addr, HMA1, microcode_addr + 1, 0, 0, 0, 0, 0, 0, 0);
	microcode_addr ++;

	//Blank Video between EAV and SAV, UYVY 80 10 80 10
	// offset 29,30,31,32
	for (i=0; i<loop_N_mode; i++) {
		if(is_bt1120) {
			//Send 0x1080
			_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0x1080 << (di_msb - 15)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
			microcode_addr ++;
		} else {
			//Send 0x80
			_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0x80 << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
			microcode_addr ++;

			//Send 0x10
			_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0x10 << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
			microcode_addr ++;
		}
	}

	//LOOP:
	//Jump Store jump address 1 loop_blank_video_times, else jump Store jump address 0
	// offset 33
	_ipu_dc_write_tmpl(ipu, microcode_addr, BMA, loop_blank_video_times, 0, 0, 0, 0, 0, 0, 0);

	//Active Video, microcode_addr_SAV_FF000080
	// offset 34
	microcode_addr = microcode_addr_SAV_FF000080;
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0xFF << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	// offset 35
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, 0x00, 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	//Store jump address 1: microcode_addr_DataW
	// offset 36
	_ipu_dc_write_tmpl(ipu, microcode_addr, HMA1, microcode_addr_DataW, 0, 0, 0, 0, 0, 0, 0);
	microcode_addr ++;

	// offset 37
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, 0x00, 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	// offset 38
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0x80 << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	//Jump Store jump address 1, microcode_addr_DataW
	// offset 39
	_ipu_dc_write_tmpl(ipu, microcode_addr, BMA, 0, 0, 0, 0, 0, 0, 1, 1);

	//microcode_addr_DataW
	// offset 40,41,42,43
	microcode_addr = microcode_addr_DataW;
	if(is_bt1120) {
		//Send data UY
		_ipu_dc_write_tmpl(ipu, microcode_addr, WROD, 0, u_map, 0, 0, DI_BT656_SYNC_BASECLK, 1, 0, 0);
		microcode_addr ++;

		//Send data VY
		_ipu_dc_write_tmpl(ipu, microcode_addr, WROD, 0, v_map, 0, 0, DI_BT656_SYNC_BASECLK, 1, 0, 0);
	} else {
		//Send data U
		_ipu_dc_write_tmpl(ipu, microcode_addr, WROD, 0, u_map, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
		microcode_addr ++;

		//Send data Y
		_ipu_dc_write_tmpl(ipu, microcode_addr, WROD, 0, y_map, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
		microcode_addr ++;

		//Send data V
		_ipu_dc_write_tmpl(ipu, microcode_addr, WROD, 0, v_map, 0, 0, DI_BT656_SYNC_BASECLK, 1, 0, 0);
		microcode_addr ++;

		//Send data Y
		_ipu_dc_write_tmpl(ipu, microcode_addr, WROD, 0, y_map, 0, 0, DI_BT656_SYNC_BASECLK, 1, 0, 0);
	}

	// New line, microcode_addr_NL
	//Store jump address 0, microcode_addr_EAV_FF00009D
	// offset 44
	microcode_addr = microcode_addr_NL;
	_ipu_dc_write_tmpl(ipu, microcode_addr, HMA, microcode_addr_EAV_FF00009D, 0, 0, 0, 0, 0, 0, 0);
	microcode_addr ++;

	//Jump Store jump address 0
	//Active video microcode_addr_EAV_FF00009D
	// offset 45
	_ipu_dc_write_tmpl(ipu, microcode_addr, BMA, 0, 0, 0, 0, 0, 0, 0, 0);

	// offset 46, Stop
	microcode_addr = microcode_addr_BlankDone;
	if(is_bt1120) {
		//Send 0x1080 and keep
		_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0x1080 << (di_msb - 15)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 1, 0, 0);
	} else {
		//Send 0x10 and keep
		_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0x10 << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 1, 0, 0);
	}

	//Note, DC max word address is 255, so "last offset + microcode_start_addr" should be less than 255.
}

static void _ipu_dc_setup_bt656_interlaced(struct ipu_soc *ipu,
			    int u_map, int y_map, int v_map,
			    bool is_bt1120, int di_msb,
			    uint32_t bt656_h_start_width,
			    uint32_t bt656_v_start_width_field0,
			    uint32_t bt656_v_end_width_field0,
			    uint32_t bt656_v_start_width_field1,
			    uint32_t bt656_v_end_width_field1,
			    bool need_insert_active_video)
{
	uint32_t microcode_addr_NL, microcode_addr_EOFIELD, microcode_addr_DataW;
	uint32_t microcode_addr_NF, microcode_addr_EOF, microcode_addr_BlankDone;
	uint32_t microcode_addr_EAV0_FF0000B6_First, microcode_addr_SAV0_FF0000AB_First;
	uint32_t microcode_addr_EAV0_FF00009D, microcode_addr_SAV0_FF000080;
	uint32_t microcode_addr_EAV0_FF00009D_Blank, microcode_addr_SAV0_FF000080_Blank;
	uint32_t microcode_addr_EAV0_FF0000B6_Second, microcode_addr_SAV0_FF0000AB_Second;
	uint32_t microcode_addr_EAV1_FF0000F1_First, microcode_addr_SAV1_FF0000EC_First;
	uint32_t microcode_addr_EAV1_FF0000DA, microcode_addr_SAV1_FF0000C7;
	uint32_t microcode_addr_EAV1_FF0000F1_Second, microcode_addr_SAV1_FF0000EC_Second;
	uint32_t loop_blank_video_times;
	uint32_t loop_field0_start_blank_line_times, loop_field0_end_blank_line_times;
	uint32_t loop_field1_start_blank_line_times, loop_field1_end_blank_line_times;
	uint32_t microcode_start_addr, microcode_addr;
	uint32_t loop_N_mode, temp, i;

	microcode_addr = microcode_start_addr = DC_MCODE_BT656_EOF;

	microcode_addr_EOF = microcode_start_addr + 0x0;  //offset 0 //DC_MCODE_BT656_EOF
	microcode_addr_EAV1_FF0000F1_Second = microcode_start_addr + 0x2;  //offset 2
	microcode_addr_SAV1_FF0000EC_Second = microcode_start_addr + 0xC;  //offset 12
	microcode_addr_EAV0_FF0000B6_First = microcode_start_addr + 0x14;  //offset 20
	microcode_addr_SAV0_FF0000AB_First = microcode_start_addr + 0x1F;  //offset 31
	microcode_addr_NF = microcode_start_addr + 0x27;  //offset 39, DC_MCODE_BT656_NF
	microcode_addr_EAV0_FF00009D = microcode_start_addr + 0x2A;  //offset 42
	microcode_addr_SAV0_FF000080 = microcode_start_addr + 0x35;  //offset 53
	microcode_addr_EOFIELD = microcode_start_addr + 0x3C;  //offset 60 //DC_MCODE_BT656_EOFIELD
	microcode_addr_DataW = microcode_start_addr + 0x3F;  //offset 63 //DC_MCODE_BT656_DATA_W
	microcode_addr_NL = microcode_start_addr + 0x43;  //offset 67 //DC_MCODE_BT656_NL
	microcode_addr_EAV0_FF00009D_Blank = microcode_start_addr + 0x44;  //offset 68
	microcode_addr_SAV0_FF000080_Blank = microcode_start_addr + 0x4F;  //offset 79
	microcode_addr_EAV0_FF0000B6_Second = microcode_start_addr + 0x55;  //offset 85
	microcode_addr_SAV0_FF0000AB_Second = microcode_start_addr + 0x5F;  //offset 95
	microcode_addr_EAV1_FF0000F1_First = microcode_start_addr + 0x67;  //offset 103
	microcode_addr_SAV1_FF0000EC_First = microcode_start_addr + 0x72;  //offset 114
	microcode_addr_BlankDone = microcode_start_addr + 0x7A;  //offset 122
	microcode_addr_EAV1_FF0000DA = microcode_start_addr + 0x7C;  //offset 124
	microcode_addr_SAV1_FF0000C7 = microcode_start_addr + 0x87;  //offset 135

	// loop_blank_video_times max value is 255, 8 bits
	if(is_bt1120)
		temp = bt656_h_start_width - 8;  //horizontal blanking + 1
	else
		temp = (bt656_h_start_width - 8) / 2;  //horizontal blanking + 1

	if (temp < 0x100) {
		loop_N_mode = 1;
		loop_blank_video_times = temp - 1;
	} else if (temp < 0x200) {
		if ((temp & 0x1) != 0) {
			dev_err(ipu->dev, "_ipu_dc_setup_bt656_interlaced: bt656_h_start_width is not aligned in mode 2.\n");
			return;
		}
		loop_N_mode = 2;
		loop_blank_video_times = (temp / 2) - 1;
	} else if (temp < 0x400) {
		if ((temp & 0x3) != 0) {
			dev_err(ipu->dev, "_ipu_dc_setup_bt656_interlaced: bt656_h_start_width is not aligned in mode 4.\n");
			return;
		}
		if (!is_bt1120) {
			dev_err(ipu->dev, "_ipu_dc_setup_bt656_interlaced: bt656 interface is not supported in mode 4.\n");
			return;
		}
		loop_N_mode = 4;
		loop_blank_video_times = (temp / 4) - 1;
	} else {
		dev_err(ipu->dev, "_ipu_dc_setup_bt656_interlaced: bt656_h_start_width = %d is too big.\n", bt656_h_start_width);
		return;
	}

	loop_field0_start_blank_line_times = bt656_v_start_width_field0 - 1;  //Field 0 - First Vertical Blanking(Top) lines
	loop_field0_end_blank_line_times = bt656_v_end_width_field0 - 1;  //Field 0 - Second Vertical Blanking(Bottom) lines
	loop_field1_start_blank_line_times = bt656_v_start_width_field1 - 1;  //Field 1 - First Vertical Blanking(Top) lines
	loop_field1_end_blank_line_times = bt656_v_end_width_field1 - 1;  //Field 1 - Second Vertical Blanking(Bottom) lines

	// offset 0, microcode_addr_EOF
	microcode_addr = microcode_addr_EOF;
	if(is_bt1120) {
		//Send data VY
		_ipu_dc_write_tmpl(ipu, microcode_addr, WROD, 0, v_map, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	} else {
		//Send data Y
		_ipu_dc_write_tmpl(ipu, microcode_addr, WROD, 0, y_map, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	}

	//Store jump address 1, microcode_addr_SAV1_FF0000EC_Second
	// offset 1
	microcode_addr = microcode_addr_EAV1_FF0000F1_Second - 1;
	_ipu_dc_write_tmpl(ipu, microcode_addr, HMA1, microcode_addr_SAV1_FF0000EC_Second, 0, 0, 0, 0, 0, 0, 0);

	//Field 1, Second Vertical Blanking(Bottom), EAV1_FF0000F1
	// offset 2
	microcode_addr = microcode_addr_EAV1_FF0000F1_Second;
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0xFF << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_HSYNC, 0, 0, 0);
	microcode_addr ++;

	// offset 3
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, 0x00, 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	// offset 4
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, 0x00, 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	// offset 5
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0xF1 << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	//Store jump address 0, microcode_addr + 1 for blank video
	// offset 6
	_ipu_dc_write_tmpl(ipu, microcode_addr, HMA, microcode_addr + 1, 0, 0, 0, 0, 0, 0, 0);
	microcode_addr ++;

	//Blank Video between EAV and SAV, UYVY 80 10 80 10
	// offset 7,8,9,10
	for (i=0; i<loop_N_mode; i++) {
		if(is_bt1120) {
			//Send 0x1080
			_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0x1080 << (di_msb - 15)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
			microcode_addr ++;
		} else {
			//Send 0x80
			_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0x80 << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
			microcode_addr ++;

			//Send 0x10
			_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0x10 << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
			microcode_addr ++;
		}
	}

	//LOOP:
	//Jump Store jump address 0 loop_blank_video_times, then jump Store jump address 1
	// offset 11
	_ipu_dc_write_tmpl(ipu, microcode_addr, BMA, loop_blank_video_times, 0, 0, 0, 0, 0, 1, 1);

	//Field 1, Second Vertical Blanking(Bottom), SAV1_FF0000EC
	// offset 12
	microcode_addr = microcode_addr_SAV1_FF0000EC_Second;
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0xFF << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	// offset 13
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, 0x00, 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	//Store jump address 1: microcode_addr_EAV1_FF0000F1_Second - 1
	// offset 14
	_ipu_dc_write_tmpl(ipu, microcode_addr, HMA1, microcode_addr_EAV1_FF0000F1_Second - 1, 0, 0, 0, 0, 0, 0, 0);
	microcode_addr ++;

	// offset 15
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, 0x00, 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	// offset 16
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0xEC << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	//Store jump address 0: microcode_addr_EAV0_FF0000B6_First
	// offset 17
	_ipu_dc_write_tmpl(ipu, microcode_addr, HMA, microcode_addr_EAV0_FF0000B6_First, 0, 0, 0, 0, 0, 0, 0);
	microcode_addr ++;

	// offset 18, Blank data in Active Video
	if(is_bt1120) {
		//Send 0x1080 and keep
		_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0x1080 << (di_msb - 15)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
		microcode_addr ++;
	} else {
		//Send 0x10 and keep
		_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0x10 << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
		microcode_addr ++;
	}

	//Jump Store jump address 1 loop_field1_end_blank_line_times then jump Store jump address 0
	// offset 19
	_ipu_dc_write_tmpl(ipu, microcode_addr, BMA, loop_field1_end_blank_line_times, 0, 0, 0, 0, 0, 0, 0);

	//Field 0, First Vertical Blanking(Top), EAV_FF0000B6
	// offset 20
	microcode_addr = microcode_addr_EAV0_FF0000B6_First;
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0xFF << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_HSYNC, 0, 0, 0);
	microcode_addr ++;

	// offset 21
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, 0x00, 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	//Store jump address 0, microcode_addr_SAV0_FF0000AB_First
	// offset 22
	_ipu_dc_write_tmpl(ipu, microcode_addr, HMA, microcode_addr_SAV0_FF0000AB_First, 0, 0, 0, 0, 0, 0, 0);
	microcode_addr ++;

	// offset 23
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, 0x00, 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	// offset 24
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0xB6 << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	//Store jump address 1, microcode_addr + 1 for blank video
	// offset 25
	_ipu_dc_write_tmpl(ipu, microcode_addr, HMA1, microcode_addr + 1, 0, 0, 0, 0, 0, 0, 0);
	microcode_addr ++;

	//Blank Video between EAV and SAV, UYVY 80 10 80 10
	// offset 26,27,28,29
	for (i=0; i<loop_N_mode; i++) {
		if(is_bt1120) {
			//Send 0x1080
			_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0x1080 << (di_msb - 15)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
			microcode_addr ++;
		} else {
			//Send 0x80
			_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0x80 << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
			microcode_addr ++;

			//Send 0x10
			_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0x10 << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
			microcode_addr ++;
		}
	}

	//LOOP:
	//Jump Store jump address 1 loop_blank_video_times, else jump Store jump address 0
	// offset 30
	_ipu_dc_write_tmpl(ipu, microcode_addr, BMA, loop_blank_video_times, 0, 0, 0, 0, 0, 0, 0);

	//Field 0, First Vertical Blanking(Top), SAV0_FF0000AB
	// offset 31
	microcode_addr = microcode_addr_SAV0_FF0000AB_First;
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0xFF << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	// offset 32
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, 0x00, 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	//Store jump address 0: microcode_addr_EAV0_FF0000B6_First
	// offset 33
	_ipu_dc_write_tmpl(ipu, microcode_addr, HMA, microcode_addr_EAV0_FF0000B6_First, 0, 0, 0, 0, 0, 0, 0);
	microcode_addr ++;

	// offset 34
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, 0x00, 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	// offset 35
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0xAB << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	//Store jump address 1, microcode_addr_BlankDone
	// offset 36
	_ipu_dc_write_tmpl(ipu, microcode_addr, HMA1, microcode_addr_BlankDone, 0, 0, 0, 0, 0, 0, 0);
	microcode_addr ++;

	// offset 37, Blank data in Active Video
	if(is_bt1120) {
		//Send 0x1080 and keep
		_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0x1080 << (di_msb - 15)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
		microcode_addr ++;
	} else {
		//Send 0x10 and keep
		_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0x10 << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
		microcode_addr ++;
	}

	//Jump Store jump address 0 loop_field0_start_blank_line_times, else jump Store jump address 1
	// offset 38
	_ipu_dc_write_tmpl(ipu, microcode_addr, BMA, loop_field0_start_blank_line_times, 0, 0, 0, 0, 0, 1, 1);

	//New Frame, start from field 0 Acvite Video, EAV = FF 00 00 9D
	// microcode_addr_NF
	//Store jump address 0, microcode_addr_EAV0_FF00009D + 1
	// offset 39
	microcode_addr = microcode_addr_NF;
	_ipu_dc_write_tmpl(ipu, microcode_addr, HMA, microcode_addr_EAV0_FF00009D + 1, 0, 0, 0, 0, 0, 0, 0);
	microcode_addr ++;

	// offset 40
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0xFF << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_AFIELD, 0, 0, 0);
	microcode_addr ++;

	//Jump Store jump address 0, microcode_addr_EAV0_FF00009D + 1
	// offset 41
	_ipu_dc_write_tmpl(ipu, microcode_addr, BMA, 0, 0, 0, 0, 0, 0, 0, 0);

	//Field 0 Acvite Video, EAV = FF 00 00 9D
	// microcode_addr_EAV0_FF00009D
	// offset 42
	microcode_addr = microcode_addr_EAV0_FF00009D;
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0xFF << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_HSYNC, 0, 0, 0);
	microcode_addr ++;

	// offset 43
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, 0x00, 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	//Store jump address 0: microcode_addr_SAV0_FF000080
	// offset 44
	_ipu_dc_write_tmpl(ipu, microcode_addr, HMA, microcode_addr_SAV0_FF000080, 0, 0, 0, 0, 0, 0, 0);
	microcode_addr ++;

	// offset 45
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, 0x00, 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	// offset 46
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0x9D << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	//Store jump address 1: microcode_addr + 1
	// offset 47
	_ipu_dc_write_tmpl(ipu, microcode_addr, HMA1, microcode_addr + 1, 0, 0, 0, 0, 0, 0, 0);
	microcode_addr ++;

	//Blank Video between EAV and SAV, UYVY 80 10 80 10
	// offset 48,49,50,51
	for (i=0; i<loop_N_mode; i++) {
		if(is_bt1120) {
			//Send 0x1080
			_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0x1080 << (di_msb - 15)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
			microcode_addr ++;
		} else {
			//Send 0x80
			_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0x80 << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
			microcode_addr ++;

			//Send 0x10
			_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0x10 << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
			microcode_addr ++;
		}
	}

	//LOOP:
	//Jump Store jump address 1 loop_blank_video_times, else jump Store jump address 0
	// offset 52
	_ipu_dc_write_tmpl(ipu, microcode_addr, BMA, loop_blank_video_times, 0, 0, 0, 0, 0, 0, 0);

	//Field 0, microcode_addr_SAV0_FF000080
	// offset 53
	microcode_addr = microcode_addr_SAV0_FF000080;
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0xFF << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	// offset 54
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, 0x00, 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	//Store jump address 1: microcode_addr_DataW
	// offset 55
	_ipu_dc_write_tmpl(ipu, microcode_addr, HMA1, microcode_addr_DataW, 0, 0, 0, 0, 0, 0, 0);
	microcode_addr ++;

	// offset 56
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, 0x00, 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	// offset 57
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0x80 << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	//Store jump address 0, microcode_addr_EAV0_FF00009D
	// offset 58
	_ipu_dc_write_tmpl(ipu, microcode_addr, HMA, microcode_addr_EAV0_FF00009D, 0, 0, 0, 0, 0, 0, 0);
	microcode_addr ++;

	//Jump Store jump address 1, microcode_addr_DataW
	// offset 59
	_ipu_dc_write_tmpl(ipu, microcode_addr, BMA, 0, 0, 0, 0, 0, 0, 1, 1);

	//Share routine
	//EOFIELD: last data of field 0 if not NF
	// offset 60
	microcode_addr = microcode_addr_EOFIELD;
	if(is_bt1120) {
		//Send data VY
		_ipu_dc_write_tmpl(ipu, microcode_addr, WROD, 0, v_map, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	} else {
		//Send data Y
		_ipu_dc_write_tmpl(ipu, microcode_addr, WROD, 0, y_map, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	}
	microcode_addr ++;

	// Check whether a active video line needs be inserted or not.
	// offset 61,62
	if (need_insert_active_video) {
		//Store jump address 1, microcode_addr_EAV0_FF00009D_Blank
		_ipu_dc_write_tmpl(ipu, microcode_addr, HMA1, microcode_addr_EAV0_FF00009D_Blank, 0, 0, 0, 0, 0, 0, 0);
		microcode_addr ++;

		//Jump Store jump address 1, microcode_addr_EAV0_FF00009D_Blank
		_ipu_dc_write_tmpl(ipu, microcode_addr, BMA, 0, 0, 0, 0, 0, 0, 1, 1);
	} else {
		//Store jump address 1, microcode_addr_EAV0_FF0000B6_Second - 1
		_ipu_dc_write_tmpl(ipu, microcode_addr, HMA1, microcode_addr_EAV0_FF0000B6_Second - 1, 0, 0, 0, 0, 0, 0, 0);
		microcode_addr ++;

		//Jump Store jump address 1, microcode_addr_EAV0_FF0000B6_Second - 1
		_ipu_dc_write_tmpl(ipu, microcode_addr, BMA, 0, 0, 0, 0, 0, 0, 1, 1);
	}

	//microcode_addr_DataW
	// offset 63,64,65,66
	microcode_addr = microcode_addr_DataW;
	if(is_bt1120) {
		//Send data UY
		_ipu_dc_write_tmpl(ipu, microcode_addr, WROD, 0, u_map, 0, 0, DI_BT656_SYNC_BASECLK, 1, 0, 0);
		microcode_addr ++;

		//Send data VY
		_ipu_dc_write_tmpl(ipu, microcode_addr, WROD, 0, v_map, 0, 0, DI_BT656_SYNC_BASECLK, 1, 0, 0);
	} else {
		//Send data U
		_ipu_dc_write_tmpl(ipu, microcode_addr, WROD, 0, u_map, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
		microcode_addr ++;

		//Send data Y
		_ipu_dc_write_tmpl(ipu, microcode_addr, WROD, 0, y_map, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
		microcode_addr ++;

		//Send data V
		_ipu_dc_write_tmpl(ipu, microcode_addr, WROD, 0, v_map, 0, 0, DI_BT656_SYNC_BASECLK, 1, 0, 0);
		microcode_addr ++;

		//Send data Y
		_ipu_dc_write_tmpl(ipu, microcode_addr, WROD, 0, y_map, 0, 0, DI_BT656_SYNC_BASECLK, 1, 0, 0);
	}

	// New line, microcode_addr_NL
	//Jump Store jump address 0,
	//Field 0 microcode_addr_EAV0_FF00009D  or field 1 microcode_addr_EAV1_FF0000DA
	// offset 67
	microcode_addr = microcode_addr_NL;
	_ipu_dc_write_tmpl(ipu, microcode_addr, BMA, 0, 0, 0, 0, 0, 0, 0, 0);

	// Insert one active video line
	// offset 68
	microcode_addr = microcode_addr_EAV0_FF00009D_Blank;
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0xFF << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_HSYNC, 0, 0, 0);
	microcode_addr ++;

	// offset 69
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, 0x00, 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	//Store jump address 0, microcode_addr_SAV0_FF000080_Blank
	// offset 70
	_ipu_dc_write_tmpl(ipu, microcode_addr, HMA, microcode_addr_SAV0_FF000080_Blank, 0, 0, 0, 0, 0, 0, 0);
	microcode_addr ++;

	// offset 71
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, 0x00, 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	// offset 72
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0x9D << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	//Store jump address 1, microcode_addr + 1 for blank video
	// offset 73
	_ipu_dc_write_tmpl(ipu, microcode_addr, HMA1, microcode_addr + 1, 0, 0, 0, 0, 0, 0, 0);
	microcode_addr ++;

	//Blank Video between EAV and SAV, UYVY 80 10 80 10
	// offset 74,75,76,77
	for (i=0; i<loop_N_mode; i++) {
		if(is_bt1120) {
			//Send 0x1080
			_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0x1080 << (di_msb - 15)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
			microcode_addr ++;
		} else {
			//Send 0x80
			_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0x80 << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
			microcode_addr ++;

			//Send 0x10
			_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0x10 << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
			microcode_addr ++;
		}
	}

	//LOOP:
	//Jump Store jump address 1 loop_blank_video_times, else jump Store jump address 0
	// offset 78
	_ipu_dc_write_tmpl(ipu, microcode_addr, BMA, loop_blank_video_times, 0, 0, 0, 0, 0, 0, 0);

	//SAV_FF000080, microcode_addr_SAV0_FF000080_Blank
	// offset 79
	microcode_addr = microcode_addr_SAV0_FF000080_Blank;
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0xFF << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	// offset 80
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, 0x00, 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	// offset 81
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, 0x00, 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	// offset 82
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0x80 << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	// offset 83, Blank data in Active Video
	if(is_bt1120) {
		//Send 0x1080 and keep
		_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0x1080 << (di_msb - 15)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
		microcode_addr ++;
	} else {
		//Send 0x10 and keep
		_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0x10 << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
		microcode_addr ++;
	}

	//Store jump address 1, microcode_addr_SAV0_FF0000AB_Second
	// offset 84
	microcode_addr = microcode_addr_EAV0_FF0000B6_Second - 1;
	_ipu_dc_write_tmpl(ipu, microcode_addr, HMA1, microcode_addr_SAV0_FF0000AB_Second, 0, 0, 0, 0, 0, 0, 0);

	//Field 0, Second Vertical Blanking(Bottom), EAV_FF0000B6
	// offset 85
	microcode_addr = microcode_addr_EAV0_FF0000B6_Second;
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0xFF << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_HSYNC, 0, 0, 0);
	microcode_addr ++;

	// offset 86
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, 0x00, 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	// offset 87
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, 0x00, 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	// offset 88
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0xB6 << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	//Store jump address 0, microcode_addr + 1 for blank video
	// offset 89
	_ipu_dc_write_tmpl(ipu, microcode_addr, HMA, microcode_addr + 1, 0, 0, 0, 0, 0, 0, 0);
	microcode_addr ++;

	//Blank Video between EAV and SAV, UYVY 80 10 80 10
	// offset 90,91,92,93
	for (i=0; i<loop_N_mode; i++) {
		if(is_bt1120) {
			//Send 0x1080
			_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0x1080 << (di_msb - 15)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
			microcode_addr ++;
		} else {
			//Send 0x80
			_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0x80 << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
			microcode_addr ++;

			//Send 0x10
			_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0x10 << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
			microcode_addr ++;
		}
	}

	//LOOP:
	//Jump Store jump address 0 loop_blank_video_times, else jump Store jump address 1
	// offset 94
	_ipu_dc_write_tmpl(ipu, microcode_addr, BMA, loop_blank_video_times, 0, 0, 0, 0, 0, 1, 1);

	//Field 0, Second Vertical Blanking(Bottom), SAV_FF0000AB
	// offset 95
	microcode_addr = microcode_addr_SAV0_FF0000AB_Second;
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0xFF << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	// offset 96
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, 0x00, 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	//Store jump address 1: microcode_addr_EAV0_FF0000B6_Second - 1
	// offset 97
	_ipu_dc_write_tmpl(ipu, microcode_addr, HMA1, microcode_addr_EAV0_FF0000B6_Second - 1, 0, 0, 0, 0, 0, 0, 0);
	microcode_addr ++;

	// offset 98
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, 0x00, 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	// offset 99
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0xAB << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	//Store jump address 0, microcode_addr_EAV1_FF0000F1_First
	// offset 100
	_ipu_dc_write_tmpl(ipu, microcode_addr, HMA, microcode_addr_EAV1_FF0000F1_First, 0, 0, 0, 0, 0, 0, 0);
	microcode_addr ++;

	// offset 101, Blank data in Active Video
	if(is_bt1120) {
		//Send 0x1080 and keep
		_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0x1080 << (di_msb - 15)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
		microcode_addr ++;
	} else {
		//Send 0x10 and keep
		_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0x10 << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
		microcode_addr ++;
	}

	//Jump Store jump address 1 loop_field0_end_blank_line_times, then jump Store jump address 0
	// offset 102
	_ipu_dc_write_tmpl(ipu, microcode_addr, BMA, loop_field0_end_blank_line_times, 0, 0, 0, 0, 0, 0, 0);

	//Field 1, First Vertical Blanking(Top), EAV1_FF0000F1
	// offset 103
	microcode_addr = microcode_addr_EAV1_FF0000F1_First;
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0xFF << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_HSYNC, 0, 0, 0);
	microcode_addr ++;

	// offset 104
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, 0x00, 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	//Store jump address 0, microcode_addr_SAV1_FF0000EC_First
	// offset 105
	_ipu_dc_write_tmpl(ipu, microcode_addr, HMA, microcode_addr_SAV1_FF0000EC_First, 0, 0, 0, 0, 0, 0, 0);
	microcode_addr ++;

	// offset 106
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, 0x00, 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	// offset 107
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0xF1 << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	//Store jump address 1, microcode_addr + 1 for blank video
	// offset 108
	_ipu_dc_write_tmpl(ipu, microcode_addr, HMA1, microcode_addr + 1, 0, 0, 0, 0, 0, 0, 0);
	microcode_addr ++;

	//Blank Video between EAV and SAV, UYVY 80 10 80 10
	// offset 109,110,111,112
	for (i=0; i<loop_N_mode; i++) {
		if(is_bt1120) {
			//Send 0x1080
			_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0x1080 << (di_msb - 15)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
			microcode_addr ++;
		} else {
			//Send 0x80
			_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0x80 << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
			microcode_addr ++;

			//Send 0x10
			_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0x10 << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
			microcode_addr ++;
		}
	}

	//LOOP:
	//Jump Store jump address 1 loop_blank_video_times, then jump Store jump address 0
	// offset 113
	_ipu_dc_write_tmpl(ipu, microcode_addr, BMA, loop_blank_video_times, 0, 0, 0, 0, 0, 0, 0);

	//Field 1, First Vertical Blanking(Top), SAV1_FF0000EC
	// offset 114
	microcode_addr = microcode_addr_SAV1_FF0000EC_First;
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0xFF << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	// offset 115
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, 0x00, 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	//Store jump address 0: microcode_addr_EAV1_FF0000F1_First
	// offset 116
	_ipu_dc_write_tmpl(ipu, microcode_addr, HMA, microcode_addr_EAV1_FF0000F1_First, 0, 0, 0, 0, 0, 0, 0);
	microcode_addr ++;

	// offset 117
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, 0x00, 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	// offset 118
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0xEC << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	//Store jump address 1, microcode_addr_BlankDone
	// offset 119
	_ipu_dc_write_tmpl(ipu, microcode_addr, HMA1, microcode_addr_BlankDone, 0, 0, 0, 0, 0, 0, 0);
	microcode_addr ++;

	// offset 120, Blank data in Active Video
	if(is_bt1120) {
		//Send 0x1080 and keep
		_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0x1080 << (di_msb - 15)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
		microcode_addr ++;
	} else {
		//Send 0x10 and keep
		_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0x10 << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
		microcode_addr ++;
	}

	//LOOP:
	//Jump Store jump address 0 loop_field1_start_blank_line_times, then jump Store jump address 1
	// offset 121
	_ipu_dc_write_tmpl(ipu, microcode_addr, BMA, loop_field1_start_blank_line_times, 0, 0, 0, 0, 0, 1, 1);

	//Store jump address 0, field 1 active video, microcode_addr_EAV1_FF0000DA
	// offset 122
	microcode_addr = microcode_addr_BlankDone;
	_ipu_dc_write_tmpl(ipu, microcode_addr, HMA, microcode_addr_EAV1_FF0000DA, 0, 0, 0, 0, 0, 0, 0);
	microcode_addr ++;

	// Stop
	// offset 123
	if(is_bt1120) {
		//Send 0x1080 and keep
		_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0x1080 << (di_msb - 15)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 1, 0, 0);
	} else {
		//Send 0x10 and keep
		_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0x10 << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 1, 0, 0);
	}

	//Field 1 Acvite Video, EAV = FF 00 00 DA
	// microcode_addr_EAV1_FF0000DA
	// offset 124
	microcode_addr = microcode_addr_EAV1_FF0000DA;
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0xFF << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_HSYNC, 0, 0, 0);
	microcode_addr ++;

	// offset 125
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, 0x00, 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	//Store jump address 1, microcode_addr_SAV1_FF0000C7
	// offset 126
	_ipu_dc_write_tmpl(ipu, microcode_addr, HMA, microcode_addr_SAV1_FF0000C7, 0, 0, 0, 0, 0, 0, 0);
	microcode_addr ++;

	// offset 127
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, 0x00, 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	// offset 128
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0xDA << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	//Store jump address 1, microcode_addr + 1
	// offset 129
	_ipu_dc_write_tmpl(ipu, microcode_addr, HMA1, microcode_addr + 1, 0, 0, 0, 0, 0, 0, 0);
	microcode_addr ++;

	//Blank Video between EAV and SAV, UYVY 80 10 80 10
	// offset 130,131,132,133
	for (i=0; i<loop_N_mode; i++) {
		if(is_bt1120) {
			//Send 0x1080
			_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0x1080 << (di_msb - 15)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
			microcode_addr ++;
		} else {
			//Send 0x80
			_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0x80 << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
			microcode_addr ++;

			//Send 0x10
			_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0x10 << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
			microcode_addr ++;
		}
	}

	//LOOP:
	//Jump Store jump address 1 loop_blank_video_times, else jump Store jump address 0
	// offset 134
	_ipu_dc_write_tmpl(ipu, microcode_addr, BMA, loop_blank_video_times, 0, 0, 0, 0, 0, 0, 0);

	//Field 1, SAV1_FF0000C7
	// offset 135
	microcode_addr = microcode_addr_SAV1_FF0000C7;
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0xFF << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	// offset 136
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, 0x00, 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	//Store jump address 1, microcode_addr_DataW
	// offset 137
	_ipu_dc_write_tmpl(ipu, microcode_addr, HMA1, microcode_addr_DataW, 0, 0, 0, 0, 0, 0, 0);
	microcode_addr ++;

	// offset 138
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, 0x00, 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	// offset 139
	_ipu_dc_write_tmpl(ipu, microcode_addr, WRG, (0xC7 << (di_msb - 7)), 0, 0, 0, DI_BT656_SYNC_BASECLK, 0, 0, 0);
	microcode_addr ++;

	//Store jump address 0, microcode_addr_EAV1_FF0000DA
	// offset 140
	_ipu_dc_write_tmpl(ipu, microcode_addr, HMA, microcode_addr_EAV1_FF0000DA, 0, 0, 0, 0, 0, 0, 0);
	microcode_addr ++;

	//Jump Store jump address 1
	// offset 141
	_ipu_dc_write_tmpl(ipu, microcode_addr, BMA, 0, 0, 0, 0, 0, 0, 1, 1);

	//Note, DC max word address is 255, so "last offset + microcode_start_addr" should be less than 255.
}

/*!
 * This function is called to adapt synchronous LCD panel to IPU restriction.
 *
 */
void adapt_panel_to_ipu_restricitions(struct ipu_soc *ipu, uint16_t *v_start_width,
					uint16_t *v_sync_width,
					uint16_t *v_end_width)
{
	if (*v_end_width < 2) {
		uint16_t diff = 2 - *v_end_width;
		if (*v_start_width >= diff) {
			*v_end_width = 2;
			*v_start_width = *v_start_width - diff;
		} else if (*v_sync_width > diff) {
			*v_end_width = 2;
			*v_sync_width = *v_sync_width - diff;
		} else
			dev_err(ipu->dev, "WARNING: try to adapt timming, but failed\n");
		dev_err(ipu->dev, "WARNING: adapt panel end blank lines\n");
	}
}

/*!
 * This function is called to initialize a synchronous LCD panel.
 *
 * @param	ipu		ipu handler
 * @param       disp            The DI the panel is attached to.
 *
 * @param       pixel_clk       Desired pixel clock frequency in Hz.
 *
 * @param       pixel_fmt       Input parameter for pixel format of buffer.
 *                              Pixel format is a FOURCC ASCII code.
 *
 * @param       width           The width of panel in pixels.
 *
 * @param       height          The height of panel in pixels.
 *
 * @param       hStartWidth     The number of pixel clocks between the HSYNC
 *                              signal pulse and the start of valid data.
 *
 * @param       hSyncWidth      The width of the HSYNC signal in units of pixel
 *                              clocks.
 *
 * @param       hEndWidth       The number of pixel clocks between the end of
 *                              valid data and the HSYNC signal for next line.
 *
 * @param       vStartWidth     The number of lines between the VSYNC
 *                              signal pulse and the start of valid data.
 *
 * @param       vSyncWidth      The width of the VSYNC signal in units of lines
 *
 * @param       vEndWidth       The number of lines between the end of valid
 *                              data and the VSYNC signal for next frame.
 *
 * @param       sig             Bitfield of signal polarities for LCD interface.
 *
 * @return      This function returns 0 on success or negative error code on
 *              fail.
 */
int32_t ipu_init_sync_panel(struct ipu_soc *ipu, int disp, uint32_t pixel_clk,
			    uint16_t width, uint16_t height,
			    uint32_t pixel_fmt,
			    uint16_t h_start_width, uint16_t h_sync_width,
			    uint16_t h_end_width, uint16_t v_start_width,
			    uint16_t v_sync_width, uint16_t v_end_width,
			    uint32_t v_to_h_sync, ipu_di_signal_cfg_t sig)
{
	uint32_t field0_offset = 0;
	uint32_t field1_offset;
	uint32_t reg;
	uint32_t di_gen, vsync_cnt;
	uint32_t div, rounded_pixel_clk;
	uint32_t h_total, v_total;
	int map;
	int ret;
	struct clk *ldb_di0_clk, *ldb_di1_clk;
	struct clk *di_parent;
	uint32_t bt656_h_start_width = 0;
	uint32_t bt656_v_start_width_field0 = 0, bt656_v_end_width_field0 = 0;
	uint32_t bt656_v_start_width_field1 = 0, bt656_v_end_width_field1 = 0;
	int u_map = 0, y_map = 0, v_map = 0;
	bool bt656_need_insert_line = 0;

	dev_dbg(ipu->dev, "panel size = %d x %d\n", width, height);

	if ((v_sync_width == 0) || (h_sync_width == 0))
		return -EINVAL;

	if((pixel_fmt == IPU_PIX_FMT_BT656) || (pixel_fmt == IPU_PIX_FMT_BT1120)) {
		bt656_h_start_width = h_sync_width;
		bt656_v_start_width_field0 = h_start_width;
		bt656_v_end_width_field0 = h_end_width;
		bt656_v_start_width_field1 =v_start_width ;
		bt656_v_end_width_field1 = v_end_width;

		if (sig.interlaced)
			v_total = height + bt656_v_start_width_field0 + bt656_v_end_width_field0 + bt656_v_start_width_field1 + bt656_v_end_width_field1;
		else
			v_total = height + bt656_v_start_width_field0 + bt656_v_end_width_field0;

		if(pixel_fmt == IPU_PIX_FMT_BT656) {
			/* BT656 */
			h_total = bt656_h_start_width + width * 2;
		} else {
			/* BT1120 */
			h_total = bt656_h_start_width + width;
		}

		// If the heigh is not aligned in 2 lines, it needs insert one active video line by microcode
		if ((height & 0x1) && sig.interlaced)
			bt656_need_insert_line = 1;
	} else {
		adapt_panel_to_ipu_restricitions(ipu, &v_start_width, &v_sync_width, &v_end_width);
		h_total = width + h_sync_width + h_start_width + h_end_width;
		v_total = height + v_sync_width + v_start_width + v_end_width;
	}

	/* Init clocking */
	dev_dbg(ipu->dev, "pixel clk = %d\n", pixel_clk);

	di_parent = clk_get_parent(ipu->di_clk_sel[disp]);
	if (!di_parent) {
		dev_err(ipu->dev, "get di clk parent fail\n");
		return -EINVAL;
	}
	ldb_di0_clk = clk_get(ipu->dev, "ldb_di0");
	if (IS_ERR(ldb_di0_clk)) {
		dev_err(ipu->dev, "clk_get di0 failed");
		return PTR_ERR(ldb_di0_clk);
	}
	ldb_di1_clk = clk_get(ipu->dev, "ldb_di1");
	if (IS_ERR(ldb_di1_clk)) {
		dev_err(ipu->dev, "clk_get di1 failed");
		return PTR_ERR(ldb_di1_clk);
	}
	if (!strcmp(__clk_get_name(di_parent), __clk_get_name(ldb_di0_clk)) ||
		!strcmp(__clk_get_name(di_parent), __clk_get_name(ldb_di1_clk))) {
		/* if di clk parent is tve/ldb, then keep it;*/
		dev_dbg(ipu->dev, "use special clk parent\n");
		ret = clk_set_parent(ipu->pixel_clk_sel[disp], ipu->di_clk[disp]);
		if (ret) {
			dev_err(ipu->dev, "set pixel clk error:%d\n", ret);
			return ret;
		}
		clk_put(ldb_di0_clk);
		clk_put(ldb_di1_clk);
	} else if ((pixel_fmt == IPU_PIX_FMT_BT656) || (pixel_fmt == IPU_PIX_FMT_BT1120)) {
		if (pixel_clk < 74000000)
			rounded_pixel_clk = 108000000;	/* For PAL and NTSC */
		else
			rounded_pixel_clk =
				clk_round_rate(ipu->di_clk[disp], pixel_clk);

		ret = clk_set_rate(ipu->di_clk[disp],
					rounded_pixel_clk);
		if (ret) {
			dev_err(ipu->dev,
				"set bt656 di clk rate error:%d\n", ret);
			return ret;
		}
		dev_dbg(ipu->dev, "bt656 di clk:%d\n", rounded_pixel_clk);
		ret = clk_set_parent(ipu->pixel_clk_sel[disp],
					ipu->di_clk[disp]);
		if (ret) {
			dev_err(ipu->dev,
				"set bt656 pixel clk parent error:%d\n", ret);
			return ret;
		}
	} else {
		/* try ipu clk first*/
		dev_dbg(ipu->dev, "try ipu internal clk\n");
		ret = clk_set_parent(ipu->pixel_clk_sel[disp], ipu->ipu_clk);
		if (ret) {
			dev_err(ipu->dev, "set pixel clk error:%d\n", ret);
			return ret;
		}
		rounded_pixel_clk = clk_round_rate(ipu->pixel_clk[disp], pixel_clk);
		dev_dbg(ipu->dev, "rounded pix clk:%d\n", rounded_pixel_clk);
		/*
		 * we will only use 1/2 fraction for ipu clk,
		 * so if the clk rate is not fit, try ext clk.
		 */
		if (!sig.int_clk &&
			((rounded_pixel_clk >= pixel_clk + pixel_clk/200) ||
			(rounded_pixel_clk <= pixel_clk - pixel_clk/200))) {
			dev_dbg(ipu->dev, "try ipu ext di clk\n");

			rounded_pixel_clk =
				clk_round_rate(ipu->di_clk[disp], pixel_clk);
			ret = clk_set_rate(ipu->di_clk[disp],
						rounded_pixel_clk);
			if (ret) {
				dev_err(ipu->dev,
					"set di clk rate error:%d\n", ret);
				return ret;
			}
			dev_dbg(ipu->dev, "di clk:%d\n", rounded_pixel_clk);
			ret = clk_set_parent(ipu->pixel_clk_sel[disp],
						ipu->di_clk[disp]);
			if (ret) {
				dev_err(ipu->dev,
					"set pixel clk parent error:%d\n", ret);
				return ret;
			}
		}
	}
	rounded_pixel_clk = clk_round_rate(ipu->pixel_clk[disp], pixel_clk);
	if (rounded_pixel_clk == 0) {
		dev_err(ipu->dev, "rounded pixel clock should not be zero\n");
		return -EINVAL;
	}
	dev_dbg(ipu->dev, "round pixel clk:%d\n", rounded_pixel_clk);
	ret = clk_set_rate(ipu->pixel_clk[disp], rounded_pixel_clk);
	if (ret) {
		dev_err(ipu->dev, "set pixel clk rate error:%d\n", ret);
		return ret;
	}
	msleep(5);
	/* Get integer portion of divider */
	div = clk_get_rate(clk_get_parent(ipu->pixel_clk_sel[disp])) / rounded_pixel_clk;
	dev_dbg(ipu->dev, "div:%d\n", div);
	if (!div) {
		dev_err(ipu->dev, "invalid pixel clk div = 0\n");
		return -EINVAL;
	}


	mutex_lock(&ipu->mutex_lock);

	_ipu_di_data_wave_config(ipu, disp, SYNC_WAVE, div - 1, div - 1);
	_ipu_di_data_pin_config(ipu, disp, SYNC_WAVE, DI_PIN15, 3, 0, div * 2);

	map = _ipu_pixfmt_to_map(pixel_fmt);
	if (map < 0) {
		dev_dbg(ipu->dev, "IPU_DISP: No MAP\n");
		mutex_unlock(&ipu->mutex_lock);
		return -EINVAL;
	}

	if(pixel_fmt == IPU_PIX_FMT_BT656) {
		u_map = map - 2;
		y_map = map - 1;
		v_map = map;
		h_total = bt656_h_start_width + width * 2;
	} else if(pixel_fmt == IPU_PIX_FMT_BT1120) {
		u_map = map - 1;
		v_map = map;
		h_total = bt656_h_start_width + width;
	}

	/*clear DI*/
	di_gen = ipu_di_read(ipu, disp, DI_GENERAL);
	di_gen &= (0x3 << 20);
	ipu_di_write(ipu, disp, di_gen, DI_GENERAL);

	if (sig.interlaced) {
		if (ipu->devtype >= IPUv3EX) {
			if((pixel_fmt == IPU_PIX_FMT_BT656) || (pixel_fmt == IPU_PIX_FMT_BT1120)) {
				/* COUNTER_1: basic clock */
				_ipu_di_sync_config(ipu,
						disp, 		/* display */
						DI_BT656_SYNC_BASECLK, 		/* counter */
						0, 	/* run count */
						DI_SYNC_CLK,	/* run_resolution */
						0, 		/* offset */
						DI_SYNC_NONE, 	/* offset resolution */
						0, 		/* repeat count */
						DI_SYNC_NONE, 	/* CNT_CLR_SEL */
						0, 		/* CNT_POLARITY_GEN_EN */
						DI_SYNC_NONE, 	/* CNT_POLARITY_CLR_SEL */
						DI_SYNC_NONE, 	/* CNT_POLARITY_TRIGGER_SEL */
						0, 		/* COUNT UP */
						0		/* COUNT DOWN */
						);

				/* COUNTER_2: HSYNC for each line */
				_ipu_di_sync_config(ipu,
						disp, 		/* display */
						DI_BT656_SYNC_HSYNC, 		/* counter */
						h_total - 1, 	/* run count */
						DI_SYNC_CLK,	/* run_resolution */
						0, 		/* offset */
						DI_SYNC_NONE, 	/* offset resolution */
						0, 		/* repeat count */
						DI_SYNC_NONE, 	/* CNT_CLR_SEL */
						0, 		/* CNT_POLARITY_GEN_EN */
						DI_SYNC_NONE, 	/* CNT_POLARITY_CLR_SEL */
						DI_SYNC_NONE, 	/* CNT_POLARITY_TRIGGER_SEL */
						0, 		/* COUNT UP */
						2*div		/* COUNT DOWN */
						);

				/* COUNTER_3: internal VSYNC for each frame */
				_ipu_di_sync_config(ipu,
						disp, 		/* display */
						DI_BT656_SYNC_IVSYNC, 		/* counter */
						v_total - 1, 	/* run count */
						DI_BT656_SYNC_HSYNC,	/* run_resolution */
						0, 			/* offset */
						DI_SYNC_NONE, 	/* offset resolution */
						0, 		/* repeat count */
						DI_SYNC_NONE, 	/* CNT_CLR_SEL */
						0, 		/* CNT_POLARITY_GEN_EN */
						DI_SYNC_NONE, 	/* CNT_POLARITY_CLR_SEL */
						DI_SYNC_NONE, 	/* CNT_POLARITY_TRIGGER_SEL */
						0, 		/* COUNT UP */
						0		/* COUNT DOWN */
						);

				vsync_cnt = DI_BT656_SYNC_VSYNC;

				/* COUNTER_4: VSYNC for field1 only */
				_ipu_di_sync_config(ipu,
						disp, 		/* display */
						DI_BT656_SYNC_VSYNC, 		/* counter */
						0, 	/* run count */
						DI_BT656_SYNC_HSYNC,	/* run_resolution */
						bt656_v_start_width_field0 + height / 2 + bt656_v_end_width_field0, 	/*  offset */
						DI_BT656_SYNC_HSYNC, 	/* offset resolution */
						1, 		/* repeat count */
						DI_BT656_SYNC_IVSYNC, 	/* CNT_CLR_SEL */
						0, 		/* CNT_POLARITY_GEN_EN */
						DI_SYNC_NONE, 	/* CNT_POLARITY_CLR_SEL */
						DI_SYNC_NONE, 	/* CNT_POLARITY_TRIGGER_SEL */
						0, 		/* COUNT UP */
						2*div		/* COUNT DOWN */
						);

				/* COUNTER_5: first active line for field0 */
				_ipu_di_sync_config(ipu,
						disp, 		/* display */
						DI_BT656_SYNC_AFIELD, 		/* counter */
						0, 		/* run count */
						DI_BT656_SYNC_HSYNC,	/* run_resolution */
						bt656_v_start_width_field0 + height / 2 + bt656_v_end_width_field0 + 2, 		/*  offset, +2 means at least 2 blank lines from VSYNC to data */
						DI_BT656_SYNC_HSYNC, 	/* offset resolution */
						1, 	/* repeat count */
						DI_BT656_SYNC_IVSYNC, 		/* CNT_CLR_SEL */
						0, 		/* CNT_POLARITY_GEN_EN */
						DI_SYNC_NONE, 	/* CNT_POLARITY_CLR_SEL */
						DI_SYNC_NONE, 	/* CNT_POLARITY_TRIGGER_SEL */
						0, 		/* COUNT UP */
						0		/* COUNT DOWN */
						);

				/* COUNTER_9: VSYNC for field0 only */
				_ipu_di_sync_config(ipu,
						disp, 		/* display */
						DI_BT656_SYNC_NVSYNC, 		/* counter */
						0, 	/* run count */
						DI_BT656_SYNC_HSYNC - 1,	/* run_resolution, the counter#9 setting is different with others , no necessary to +1! */
						0, 		/* offset  */
						DI_SYNC_NONE, 	/* offset resolution  */
						1, 		/* repeat count */
						DI_BT656_SYNC_IVSYNC - 1, 	/* CNT_CLR_SEL, the counter#9 setting is different with others , no necessary to +1! */
						0, 		/* CNT_POLARITY_GEN_EN  */
						DI_SYNC_NONE, 	/* CNT_POLARITY_CLR_SEL  */
						DI_SYNC_NONE, 	/* CNT_POLARITY_TRIGGER_SEL */
						0, 		/* COUNT UP */
						2*div		/* COUNT DOWN */
						);

				/* set gentime select and tag sel */
				reg = ipu_di_read(ipu, disp, DI_SW_GEN1(9));
				reg &= 0x1FFFFFFF;
				reg |= (DI_BT656_SYNC_VSYNC - 1) << 29;
				ipu_di_write(ipu, disp, reg, DI_SW_GEN1(9));

				ipu_di_write(ipu, disp, height / 2 + bt656_v_start_width_field0 + bt656_v_end_width_field0 - 1, DI_SCR_CONF);

				/* set y_sel = DI_BT656_SYNC_HSYNC - 1 */
				di_gen |= ((DI_BT656_SYNC_HSYNC - 1) << 28);
			} else {
				/* Setup internal HSYNC waveform */
				_ipu_di_sync_config(ipu,
						disp, 		/* display */
						DI_SYNC_INT_HSYNC, 		/* counter */
						h_total/2 - 1, 	/* run count */
						DI_SYNC_CLK,	/* run_resolution */
						0, 		/* offset */
						DI_SYNC_NONE, 	/* offset resolution */
						0, 		/* repeat count */
						DI_SYNC_NONE, 	/* CNT_CLR_SEL */
						0, 		/* CNT_POLARITY_GEN_EN */
						DI_SYNC_NONE, 	/* CNT_POLARITY_CLR_SEL */
						DI_SYNC_NONE, 	/* CNT_POLARITY_TRIGGER_SEL */
						0, 		/* COUNT UP */
						0		/* COUNT DOWN */
						);

				/* Field 1 VSYNC waveform */
				_ipu_di_sync_config(ipu,
						disp, 		/* display */
						DI_SYNC_HSYNC, 		/* counter */
						h_total - 1, 	/* run count */
						DI_SYNC_CLK,	/* run_resolution */
						0, 		/* offset */
						DI_SYNC_NONE, 	/* offset resolution */
						0, 		/* repeat count */
						DI_SYNC_NONE, 	/* CNT_CLR_SEL */
						0, 		/* CNT_POLARITY_GEN_EN */
						DI_SYNC_NONE, 	/* CNT_POLARITY_CLR_SEL */
						DI_SYNC_NONE, 	/* CNT_POLARITY_TRIGGER_SEL */
						0, 		/* COUNT UP */
						2*div		/* COUNT DOWN */
						);

				/* Setup internal HSYNC waveform */
				_ipu_di_sync_config(ipu,
						disp, 		/* display */
						DI_SYNC_VSYNC, 		/* counter */
						v_total*2 - 1, 	/* run count */
						DI_SYNC_INT_HSYNC,	/* run_resolution */
						1, 			/* offset */
						DI_SYNC_INT_HSYNC, 	/* offset resolution */
						0, 		/* repeat count */
						DI_SYNC_NONE, 	/* CNT_CLR_SEL */
						0, 		/* CNT_POLARITY_GEN_EN */
						DI_SYNC_NONE, 	/* CNT_POLARITY_CLR_SEL */
						DI_SYNC_NONE, 	/* CNT_POLARITY_TRIGGER_SEL */
						0, 		/* COUNT UP */
						2*div		/* COUNT DOWN */
						);

				/* Active Field ? */
				_ipu_di_sync_config(ipu,
						disp, 		/* display */
						DI_SYNC_AFIELD, 		/* counter */
						v_total/2 - 1, 	/* run count */
						DI_SYNC_HSYNC,	/* run_resolution */
						v_start_width, 	/*  offset */
						DI_SYNC_HSYNC, 	/* offset resolution */
						2, 		/* repeat count */
						DI_SYNC_VSYNC, 	/* CNT_CLR_SEL */
						0, 		/* CNT_POLARITY_GEN_EN */
						DI_SYNC_NONE, 	/* CNT_POLARITY_CLR_SEL */
						DI_SYNC_NONE, 	/* CNT_POLARITY_TRIGGER_SEL */
						0, 		/* COUNT UP */
						0		/* COUNT DOWN */
						);

				/* Active Line */
				_ipu_di_sync_config(ipu,
						disp, 		/* display */
						DI_SYNC_ALINE, 		/* counter */
						0, 		/* run count */
						DI_SYNC_HSYNC,	/* run_resolution */
						0, 		/*  offset */
						DI_SYNC_NONE, 	/* offset resolution */
						height/2, 	/* repeat count */
						DI_SYNC_AFIELD, 		/* CNT_CLR_SEL */
						0, 		/* CNT_POLARITY_GEN_EN */
						DI_SYNC_NONE, 	/* CNT_POLARITY_CLR_SEL */
						DI_SYNC_NONE, 	/* CNT_POLARITY_TRIGGER_SEL */
						0, 		/* COUNT UP */
						0		/* COUNT DOWN */
						);

				/* Field 0 VSYNC waveform */
				_ipu_di_sync_config(ipu,
						disp, 		/* display */
						DI_SYNC_COUNT_6, 		/* counter */
						v_total - 1, 	/* run count */
						DI_SYNC_HSYNC,	/* run_resolution */
						0, 		/* offset */
						DI_SYNC_NONE, 	/* offset resolution */
						0, 		/* repeat count */
						DI_SYNC_NONE, 	/* CNT_CLR_SEL  */
						0, 		/* CNT_POLARITY_GEN_EN */
						DI_SYNC_NONE, 	/* CNT_POLARITY_CLR_SEL */
						DI_SYNC_NONE, 	/* CNT_POLARITY_TRIGGER_SEL */
						0, 		/* COUNT UP */
						0		/* COUNT DOWN */
						);

				/* DC VSYNC waveform */
				vsync_cnt = DI_SYNC_COUNT_7;
				_ipu_di_sync_config(ipu,
						disp, 		/* display */
						DI_SYNC_COUNT_7, 		/* counter */
						v_total/2 - 1, 	/* run count */
						DI_SYNC_HSYNC,	/* run_resolution  */
						9, 		/* offset  */
						DI_SYNC_HSYNC, 	/* offset resolution */
						2, 		/* repeat count */
						DI_SYNC_VSYNC, 	/* CNT_CLR_SEL */
						0, 		/* CNT_POLARITY_GEN_EN */
						DI_SYNC_NONE, 	/* CNT_POLARITY_CLR_SEL */
						DI_SYNC_NONE, 	/* CNT_POLARITY_TRIGGER_SEL */
						0, 		/* COUNT UP */
						0		/* COUNT DOWN */
						);

				/* active pixel waveform */
				_ipu_di_sync_config(ipu,
						disp, 		/* display */
						DI_SYNC_COUNT_8, 		/* counter */
						0, 		/* run count  */
						DI_SYNC_CLK,	/* run_resolution */
						h_start_width, 	/* offset  */
						DI_SYNC_CLK, 	/* offset resolution */
						width, 		/* repeat count  */
						DI_SYNC_ALINE, 		/* CNT_CLR_SEL  */
						0, 		/* CNT_POLARITY_GEN_EN  */
						DI_SYNC_NONE, 	/* CNT_POLARITY_CLR_SEL */
						DI_SYNC_NONE, 	/* CNT_POLARITY_TRIGGER_SEL  */
						0, 		/* COUNT UP  */
						0		/* COUNT DOWN */
						);

				/* Second VSYNC */
				_ipu_di_sync_config(ipu,
						disp, 		/* display */
						DI_SYNC_COUNT_9, 		/* counter */
						v_total - 1, 	/* run count */
						DI_SYNC_INT_HSYNC,	/* run_resolution */
						v_total/2, 		/* offset  */
						DI_SYNC_INT_HSYNC, 	/* offset resolution  */
						0, 		/* repeat count */
						DI_SYNC_HSYNC, 	/* CNT_CLR_SEL */
						0, 		/* CNT_POLARITY_GEN_EN  */
						DI_SYNC_NONE, 	/* CNT_POLARITY_CLR_SEL  */
						DI_SYNC_NONE, 	/* CNT_POLARITY_TRIGGER_SEL */
						0, 		/* COUNT UP */
						2*div		/* COUNT DOWN */
						);

				/* set gentime select and tag sel */
				reg = ipu_di_read(ipu, disp, DI_SW_GEN1(9));
				reg &= 0x1FFFFFFF;
				reg |= (DI_SYNC_VSYNC-1)<<29 | 0x00008000;
				ipu_di_write(ipu, disp, reg, DI_SW_GEN1(9));

				ipu_di_write(ipu, disp, v_total / 2 - 1, DI_SCR_CONF);

				/* set y_sel = 1 */
				di_gen |= 0x10000000;
				di_gen |= DI_GEN_POLARITY_5;
				di_gen |= DI_GEN_POLARITY_8;
			}
		} else {
			/* Setup internal HSYNC waveform */
			_ipu_di_sync_config(ipu, disp, DI_SYNC_INT_HSYNC, h_total - 1, DI_SYNC_CLK,
					0, DI_SYNC_NONE, 0, DI_SYNC_NONE, 0, DI_SYNC_NONE,
					DI_SYNC_NONE, 0, 0);

			field1_offset = v_sync_width + v_start_width + height / 2 +
				v_end_width;
			if (sig.odd_field_first) {
				field0_offset = field1_offset - 1;
				field1_offset = 0;
			}
			v_total += v_start_width + v_end_width;

			/* HSYNC waveform */
			_ipu_di_sync_config(ipu, disp, DI_SYNC_HSYNC, h_total - 1, DI_SYNC_CLK,
					0, DI_SYNC_NONE, 0, DI_SYNC_NONE, 0,
					DI_SYNC_NONE, DI_SYNC_NONE, 0, 4);

			/* Field 1 VSYNC waveform */
			_ipu_di_sync_config(ipu, disp, DI_SYNC_VSYNC, v_total - 1, DI_SYNC_INT_HSYNC,
					field0_offset,
					field0_offset ? DI_SYNC_INT_HSYNC : DI_SYNC_NONE,
					0, DI_SYNC_NONE, 0,
					DI_SYNC_NONE, DI_SYNC_NONE, 0, 4);

			/* Active Field ? */
			_ipu_di_sync_config(ipu, disp, DI_SYNC_AFIELD,
					field0_offset ?
					field0_offset : field1_offset - 2,
					DI_SYNC_INT_HSYNC, v_start_width + v_sync_width, 1, DI_SYNC_HSYNC, 2,
					0, DI_SYNC_NONE, DI_SYNC_NONE, 0, 0);

			/* Active Line */
			_ipu_di_sync_config(ipu, disp, DI_SYNC_ALINE, 0, DI_SYNC_INT_HSYNC,
					0, DI_SYNC_NONE,
					height / 2, DI_SYNC_AFIELD, 0, DI_SYNC_NONE,
					DI_SYNC_NONE, 0, 0);

			/* active pixel waveform */
			_ipu_di_sync_config(ipu, disp, DI_SYNC_APIXEL, 0, DI_SYNC_CLK,
					h_sync_width + h_start_width, DI_SYNC_CLK,
					width, DI_SYNC_ALINE, 0, DI_SYNC_NONE, DI_SYNC_NONE,
					0, 0);

			/* DC VSYNC waveform */
			vsync_cnt = DI_SYNC_COUNT_7;
			_ipu_di_sync_config(ipu, disp, DI_SYNC_COUNT_7, 0, DI_SYNC_INT_HSYNC,
					field1_offset,
					field1_offset ? DI_SYNC_INT_HSYNC : DI_SYNC_NONE,
					1, DI_SYNC_VSYNC, 0, DI_SYNC_NONE, DI_SYNC_NONE, 0, 0);

			/* Field 0 VSYNC waveform */
			_ipu_di_sync_config(ipu, disp, DI_SYNC_COUNT_8, v_total - 1, DI_SYNC_INT_HSYNC,
					0, DI_SYNC_NONE,
					0, DI_SYNC_NONE, 0, DI_SYNC_NONE,
					DI_SYNC_NONE, 0, 0);

			/* ??? */
			_ipu_di_sync_config(ipu, disp, DI_SYNC_COUNT_9, v_total - 1, (DI_SYNC_HSYNC - 1),
					0, DI_SYNC_NONE,
					0, DI_SYNC_NONE, 6, DI_SYNC_NONE,
					DI_SYNC_NONE, 0, 0);

			reg = ipu_di_read(ipu, disp, DI_SW_GEN1(9));
			reg |= 0x8000;
			ipu_di_write(ipu, disp, reg, DI_SW_GEN1(9));

			ipu_di_write(ipu, disp, v_sync_width + v_start_width +
					v_end_width + height / 2 - 1, DI_SCR_CONF);
		}

		if((pixel_fmt == IPU_PIX_FMT_BT656) || (pixel_fmt == IPU_PIX_FMT_BT1120)) {
			/* Init template microcode */
#ifdef BT656_IF_DI_MSB
			if(pixel_fmt == IPU_PIX_FMT_BT656) {
				_ipu_dc_setup_bt656_interlaced(ipu, u_map, y_map, v_map, 0, BT656_IF_DI_MSB,
						bt656_h_start_width,
						bt656_v_start_width_field0, bt656_v_end_width_field0,
						bt656_v_start_width_field1, bt656_v_end_width_field1,
						bt656_need_insert_line);
			} else {
				_ipu_dc_setup_bt656_interlaced(ipu, u_map, y_map, v_map, 1, BT656_IF_DI_MSB,
						bt656_h_start_width,
						bt656_v_start_width_field0, bt656_v_end_width_field0,
						bt656_v_start_width_field1, bt656_v_end_width_field1,
						bt656_need_insert_line);
			}
#else
			if(pixel_fmt == IPU_PIX_FMT_BT656) {
				_ipu_dc_setup_bt656_interlaced(ipu, u_map, y_map, v_map, 0, 23,
						bt656_h_start_width,
						bt656_v_start_width_field0, bt656_v_end_width_field0,
						bt656_v_start_width_field1, bt656_v_end_width_field1,
						bt656_need_insert_line);
			} else {
				_ipu_dc_setup_bt656_interlaced(ipu, u_map, y_map, v_map, 1, 23,
						bt656_h_start_width,
						bt656_v_start_width_field0, bt656_v_end_width_field0,
						bt656_v_start_width_field1, bt656_v_end_width_field1,
						bt656_need_insert_line);
			}
#endif
			ipu_dc_write(ipu, (width - 1), DC_UGDE_3(disp));

			if (sig.Hsync_pol)
				di_gen |= DI_GEN_POLARITY_2;
			if (sig.Vsync_pol)
				di_gen |= DI_GEN_POLARITY_3;
		} else {
			/* Init template microcode */
			if (disp) {
				_ipu_dc_write_tmpl(ipu, 1, WROD, 0, map, SYNC_WAVE, 0, DI_SYNC_APIXEL, 1, 0, 0);
				if ((pixel_fmt == IPU_PIX_FMT_YUYV) ||
					(pixel_fmt == IPU_PIX_FMT_UYVY) ||
					(pixel_fmt == IPU_PIX_FMT_YVYU) ||
					(pixel_fmt == IPU_PIX_FMT_VYUY)) {
					_ipu_dc_write_tmpl(ipu, 8, WROD, 0, (map - 1), SYNC_WAVE, 0, DI_SYNC_APIXEL, 1, 0, 0);
					_ipu_dc_write_tmpl(ipu, 9, WROD, 0, map, SYNC_WAVE, 0, DI_SYNC_APIXEL, 1, 0, 0);
					/* configure user events according to DISP NUM */
					ipu_dc_write(ipu, (width - 1), DC_UGDE_3(disp));
				}
			} else {
				_ipu_dc_write_tmpl(ipu, 0, WROD, 0, map, SYNC_WAVE, 0, DI_SYNC_APIXEL, 1, 0, 0);
				if ((pixel_fmt == IPU_PIX_FMT_YUYV) ||
					(pixel_fmt == IPU_PIX_FMT_UYVY) ||
					(pixel_fmt == IPU_PIX_FMT_YVYU) ||
					(pixel_fmt == IPU_PIX_FMT_VYUY)) {
					_ipu_dc_write_tmpl(ipu, 10, WROD, 0, (map - 1), SYNC_WAVE, 0, DI_SYNC_APIXEL, 1, 0, 0);
					_ipu_dc_write_tmpl(ipu, 11, WROD, 0, map, SYNC_WAVE, 0, DI_SYNC_APIXEL, 1, 0, 0);
					/* configure user events according to DISP NUM */
					ipu_dc_write(ipu, width - 1, DC_UGDE_3(disp));
				}
			}

			if (sig.Hsync_pol)
				di_gen |= DI_GEN_POLARITY_2;
			if (sig.Vsync_pol)
				di_gen |= DI_GEN_POLARITY_3;
		}
	} else {
		if((pixel_fmt == IPU_PIX_FMT_BT656) || (pixel_fmt == IPU_PIX_FMT_BT1120)) {
			/* COUNTER_1: basic clock */
			_ipu_di_sync_config(ipu,
					disp,		/* display */
					DI_BT656_SYNC_BASECLK,		/* counter */
					0,	/* run count */
					DI_SYNC_CLK,	/* run_resolution */
					0,		/* offset */
					DI_SYNC_NONE,	/* offset resolution */
					0,		/* repeat count */
					DI_SYNC_NONE,	/* CNT_CLR_SEL */
					0,		/* CNT_POLARITY_GEN_EN */
					DI_SYNC_NONE,	/* CNT_POLARITY_CLR_SEL */
					DI_SYNC_NONE,	/* CNT_POLARITY_TRIGGER_SEL */
					0,		/* COUNT UP */
					0		/* COUNT DOWN */
					);

			/* COUNTER_2: HSYNC for each line */
			_ipu_di_sync_config(ipu,
					disp,		/* display */
					DI_BT656_SYNC_HSYNC,		/* counter */
					h_total - 1,	/* run count */
					DI_SYNC_CLK,	/* run_resolution */
					0,		/* offset */
					DI_SYNC_NONE,	/* offset resolution */
					0,		/* repeat count */
					DI_SYNC_NONE,	/* CNT_CLR_SEL */
					0,		/* CNT_POLARITY_GEN_EN */
					DI_SYNC_NONE,	/* CNT_POLARITY_CLR_SEL */
					DI_SYNC_NONE,	/* CNT_POLARITY_TRIGGER_SEL */
					0,		/* COUNT UP */
					2*div		/* COUNT DOWN */
					);

			vsync_cnt = DI_BT656_SYNC_IVSYNC;

			/* COUNTER_3: VSYNC for each frame */
			_ipu_di_sync_config(ipu,
					disp,		/* display */
					DI_BT656_SYNC_IVSYNC,		/* counter */
					v_total - 1,	/* run count */
					DI_BT656_SYNC_HSYNC,	/* run_resolution */
					0,			/* offset */
					DI_SYNC_NONE,	/* offset resolution */
					0,		/* repeat count */
					DI_SYNC_NONE,	/* CNT_CLR_SEL */
					0,		/* CNT_POLARITY_GEN_EN */
					DI_SYNC_NONE,	/* CNT_POLARITY_CLR_SEL */
					DI_SYNC_NONE,	/* CNT_POLARITY_TRIGGER_SEL */
					0,		/* COUNT UP */
					0		/* COUNT DOWN */
					);

			/* COUNTER_5: first active line */
			_ipu_di_sync_config(ipu,
					disp,		/* display */
					DI_BT656_SYNC_AFIELD,		/* counter */
					0,		/* run count */
					DI_BT656_SYNC_HSYNC,	/* run_resolution */
					2,		/*	offset, 2 means at least 2 blank lines from VSYNC to data */
					DI_BT656_SYNC_HSYNC,	/* offset resolution */
					1,	/* repeat count */
					DI_BT656_SYNC_IVSYNC,		/* CNT_CLR_SEL */
					0,		/* CNT_POLARITY_GEN_EN */
					DI_SYNC_NONE,	/* CNT_POLARITY_CLR_SEL */
					DI_SYNC_NONE,	/* CNT_POLARITY_TRIGGER_SEL */
					0,		/* COUNT UP */
					0		/* COUNT DOWN */
					);

			ipu_di_write(ipu, disp, height + bt656_v_start_width_field0 + bt656_v_end_width_field0 - 1, DI_SCR_CONF);

			/* set y_sel = DI_BT656_SYNC_HSYNC - 1 */
			di_gen |= ((DI_BT656_SYNC_HSYNC - 1) << 28);

			/* Init template microcode */
#ifdef BT656_IF_DI_MSB
			if(pixel_fmt == IPU_PIX_FMT_BT656) {
				_ipu_dc_setup_bt656_progressive(ipu, u_map, y_map, v_map, 0, BT656_IF_DI_MSB,
						bt656_h_start_width,
						bt656_v_start_width_field0, bt656_v_end_width_field0);
			} else {
				_ipu_dc_setup_bt656_progressive(ipu, u_map, y_map, v_map, 1, BT656_IF_DI_MSB,
						bt656_h_start_width,
						bt656_v_start_width_field0, bt656_v_end_width_field0);
			}
#else
			if(pixel_fmt == IPU_PIX_FMT_BT656) {
				_ipu_dc_setup_bt656_progressive(ipu, u_map, y_map, v_map, 0, 23,
						bt656_h_start_width,
						bt656_v_start_width_field0, bt656_v_end_width_field0);
			} else {
				_ipu_dc_setup_bt656_progressive(ipu, u_map, y_map, v_map, 1, 23,
						bt656_h_start_width,
						bt656_v_start_width_field0, bt656_v_end_width_field0);
			}
#endif
			ipu_dc_write(ipu, (width - 1), DC_UGDE_3(disp));

			if (sig.Hsync_pol)
				di_gen |= DI_GEN_POLARITY_2;
			if (sig.Vsync_pol)
				di_gen |= DI_GEN_POLARITY_3;
		} else {
			/* Setup internal HSYNC waveform */
			_ipu_di_sync_config(ipu, disp, DI_SYNC_INT_HSYNC, h_total - 1, DI_SYNC_CLK,
						0, DI_SYNC_NONE, 0, DI_SYNC_NONE, 0, DI_SYNC_NONE,
						DI_SYNC_NONE, 0, 0);

			/* Setup external (delayed) HSYNC waveform */
			_ipu_di_sync_config(ipu, disp, DI_SYNC_HSYNC, h_total - 1,
					    DI_SYNC_CLK, div * v_to_h_sync, DI_SYNC_CLK,
					    0, DI_SYNC_NONE, 1, DI_SYNC_NONE,
					    DI_SYNC_CLK, 0, h_sync_width * 2);
			/* Setup VSYNC waveform */
			vsync_cnt = DI_SYNC_VSYNC;
			_ipu_di_sync_config(ipu, disp, DI_SYNC_VSYNC, v_total - 1,
					    DI_SYNC_INT_HSYNC, 0, DI_SYNC_NONE, 0,
					    DI_SYNC_NONE, 1, DI_SYNC_NONE,
					    DI_SYNC_INT_HSYNC, 0, v_sync_width * 2);
			ipu_di_write(ipu, disp, v_total - 1, DI_SCR_CONF);

			/* Setup active data waveform to sync with DC */
			_ipu_di_sync_config(ipu, disp, DI_SYNC_ALINE, 0, DI_SYNC_HSYNC,
					    v_sync_width + v_start_width, DI_SYNC_HSYNC, height,
					    DI_SYNC_VSYNC, 0, DI_SYNC_NONE,
					    DI_SYNC_NONE, 0, 0);
			_ipu_di_sync_config(ipu, disp, DI_SYNC_APIXEL, 0, DI_SYNC_CLK,
					    h_sync_width + h_start_width, DI_SYNC_CLK,
					    width, DI_SYNC_ALINE, 0, DI_SYNC_NONE, DI_SYNC_NONE, 0,
					    0);

			/* set VGA delayed hsync/vsync no matter VGA enabled */
			if (disp) {
				/* couter 7 for VGA delay HSYNC */
				_ipu_di_sync_config(ipu, disp, DI_SYNC_COUNT_7,
						h_total - 1, DI_SYNC_CLK,
						18, DI_SYNC_CLK,
						0, DI_SYNC_NONE,
						1, DI_SYNC_NONE, DI_SYNC_CLK,
						0, h_sync_width * 2);

				/* couter 8 for VGA delay VSYNC */
				_ipu_di_sync_config(ipu, disp, DI_SYNC_COUNT_8,
						v_total - 1, DI_SYNC_INT_HSYNC,
						1, DI_SYNC_INT_HSYNC,
						0, DI_SYNC_NONE,
						1, DI_SYNC_NONE, DI_SYNC_INT_HSYNC,
						0, v_sync_width * 2);
			}

			/* reset all unused counters */
			if (!disp) {
				ipu_di_write(ipu, disp, 0, DI_SW_GEN0(7));
				ipu_di_write(ipu, disp, 0, DI_SW_GEN1(7));
				ipu_di_write(ipu, disp, 0, DI_STP_REP(7));
				ipu_di_write(ipu, disp, 0, DI_SW_GEN0(8));
				ipu_di_write(ipu, disp, 0, DI_SW_GEN1(8));
				ipu_di_write(ipu, disp, 0, DI_STP_REP(8));
			}
			ipu_di_write(ipu, disp, 0, DI_SW_GEN0(9));
			ipu_di_write(ipu, disp, 0, DI_SW_GEN1(9));
			ipu_di_write(ipu, disp, 0, DI_STP_REP(9));

			/* Init template microcode */
			if (disp) {
				if ((pixel_fmt == IPU_PIX_FMT_YUYV) ||
					(pixel_fmt == IPU_PIX_FMT_UYVY) ||
					(pixel_fmt == IPU_PIX_FMT_YVYU) ||
					(pixel_fmt == IPU_PIX_FMT_VYUY)) {
					_ipu_dc_write_tmpl(ipu, 8, WROD, 0, (map - 1), SYNC_WAVE, 0, DI_SYNC_APIXEL, 1, 0, 0);
					_ipu_dc_write_tmpl(ipu, 9, WROD, 0, map, SYNC_WAVE, 0, DI_SYNC_APIXEL, 1, 0, 0);
					/* configure user events according to DISP NUM */
					ipu_dc_write(ipu, (width - 1), DC_UGDE_3(disp));
				}
				_ipu_dc_write_tmpl(ipu, 2, WROD, 0, map, SYNC_WAVE, 8, DI_SYNC_APIXEL, 1, 0, 0);
				_ipu_dc_write_tmpl(ipu, 3, WROD, 0, map, SYNC_WAVE, 4, DI_SYNC_APIXEL, 0, 0, 0);
				_ipu_dc_write_tmpl(ipu, 4, WRG, 0, map, NULL_WAVE, 0, DI_SYNC_CLK, 1, 0, 0);
				_ipu_dc_write_tmpl(ipu, 1, WROD, 0, map, SYNC_WAVE, 0, DI_SYNC_APIXEL, 1, 0, 0);

			} else {
				if ((pixel_fmt == IPU_PIX_FMT_YUYV) ||
					(pixel_fmt == IPU_PIX_FMT_UYVY) ||
					(pixel_fmt == IPU_PIX_FMT_YVYU) ||
					(pixel_fmt == IPU_PIX_FMT_VYUY)) {
					_ipu_dc_write_tmpl(ipu, 10, WROD, 0, (map - 1), SYNC_WAVE, 0, DI_SYNC_APIXEL, 1, 0, 0);
					_ipu_dc_write_tmpl(ipu, 11, WROD, 0, map, SYNC_WAVE, 0, DI_SYNC_APIXEL, 1, 0, 0);
					/* configure user events according to DISP NUM */
					ipu_dc_write(ipu, width - 1, DC_UGDE_3(disp));
				}
				_ipu_dc_write_tmpl(ipu, 5, WROD, 0, map, SYNC_WAVE, 8, DI_SYNC_APIXEL, 1, 0, 0);
				_ipu_dc_write_tmpl(ipu, 6, WROD, 0, map, SYNC_WAVE, 4, DI_SYNC_APIXEL, 0, 0, 0);
				_ipu_dc_write_tmpl(ipu, 7, WRG, 0, map, NULL_WAVE, 0, DI_SYNC_CLK, 1, 0, 0);
				_ipu_dc_write_tmpl(ipu, 12, WROD, 0, map, SYNC_WAVE, 0, DI_SYNC_APIXEL, 1, 0, 0);
			}

			if (sig.Hsync_pol) {
				di_gen |= DI_GEN_POLARITY_2;
				if (disp)
					di_gen |= DI_GEN_POLARITY_7;
			}
			if (sig.Vsync_pol) {
				di_gen |= DI_GEN_POLARITY_3;
				if (disp)
					di_gen |= DI_GEN_POLARITY_8;
			}
		}
	}

	if (!sig.clk_pol)
		di_gen |= DI_GEN_POLARITY_DISP_CLK;
	else
		di_gen &= ~DI_GEN_POLARITY_DISP_CLK;

	if((pixel_fmt == IPU_PIX_FMT_BT656) || (pixel_fmt == IPU_PIX_FMT_BT1120)) {
		/* select external VSYNC for DI error recovery */
		di_gen |= (1 << 10);
	}

	ipu_di_write(ipu, disp, di_gen, DI_GENERAL);

	if((pixel_fmt == IPU_PIX_FMT_BT656) || (pixel_fmt == IPU_PIX_FMT_BT1120))
		ipu_di_write(ipu, disp, (--vsync_cnt << DI_VSYNC_SEL_OFFSET), DI_SYNC_AS_GEN);
	else
		ipu_di_write(ipu, disp, (--vsync_cnt << DI_VSYNC_SEL_OFFSET) |
				0x00000002, DI_SYNC_AS_GEN);

	reg = ipu_di_read(ipu, disp, DI_POL);
	reg &= ~(DI_POL_DRDY_DATA_POLARITY | DI_POL_DRDY_POLARITY_15);
	if (sig.enable_pol)
		reg |= DI_POL_DRDY_POLARITY_15;
	if (sig.data_pol)
		reg |= DI_POL_DRDY_DATA_POLARITY;
	ipu_di_write(ipu, disp, reg, DI_POL);

	ipu_dc_write(ipu, width, DC_DISP_CONF2(DC_DISP_ID_SYNC(disp)));

	mutex_unlock(&ipu->mutex_lock);

	return 0;
}
EXPORT_SYMBOL(ipu_init_sync_panel);

void ipu_uninit_sync_panel(struct ipu_soc *ipu, int disp)
{
	uint32_t reg;
	uint32_t di_gen;

	if (disp != 0 && disp != 1)
		return;

	mutex_lock(&ipu->mutex_lock);

	di_gen = ipu_di_read(ipu, disp, DI_GENERAL);
	di_gen |= 0x3ff | DI_GEN_POLARITY_DISP_CLK;
	ipu_di_write(ipu, disp, di_gen, DI_GENERAL);

	reg = ipu_di_read(ipu, disp, DI_POL);
	reg |= 0x3ffffff;
	ipu_di_write(ipu, disp, reg, DI_POL);

	mutex_unlock(&ipu->mutex_lock);
}
EXPORT_SYMBOL(ipu_uninit_sync_panel);

int ipu_init_async_panel(struct ipu_soc *ipu, int disp, int type, uint32_t cycle_time,
			 uint32_t pixel_fmt, ipu_adc_sig_cfg_t sig)
{
	int map;
	u32 ser_conf = 0;
	u32 div;
	u32 di_clk = clk_get_rate(ipu->ipu_clk);

	if (di_clk == 0) {
		dev_err(ipu->dev, "di clock rate should not be zero\n");
		return -EINVAL;
	}

	/* round up cycle_time, then calcalate the divider using scaled math */
	cycle_time += (1000000000UL / di_clk) - 1;
	div = (cycle_time * (di_clk / 256UL)) / (1000000000UL / 256UL);

	map = _ipu_pixfmt_to_map(pixel_fmt);
	if (map < 0)
		return -EINVAL;

	mutex_lock(&ipu->mutex_lock);

	if (type == IPU_PANEL_SERIAL) {
		ipu_di_write(ipu, disp, (div << 24) | ((sig.ifc_width - 1) << 4),
			     DI_DW_GEN(ASYNC_SER_WAVE));

		_ipu_di_data_pin_config(ipu, disp, ASYNC_SER_WAVE, DI_PIN_CS,
					0, 0, (div * 2) + 1);
		_ipu_di_data_pin_config(ipu, disp, ASYNC_SER_WAVE, DI_PIN_SER_CLK,
					1, div, div * 2);
		_ipu_di_data_pin_config(ipu, disp, ASYNC_SER_WAVE, DI_PIN_SER_RS,
					2, 0, 0);

		_ipu_dc_write_tmpl(ipu, 0x64, WROD, 0, map, ASYNC_SER_WAVE, 0, 0, 1, 0, 0);

		/* Configure DC for serial panel */
		ipu_dc_write(ipu, 0x14, DC_DISP_CONF1(DC_DISP_ID_SERIAL));

		if (sig.clk_pol)
			ser_conf |= DI_SER_CONF_SERIAL_CLK_POL;
		if (sig.data_pol)
			ser_conf |= DI_SER_CONF_SERIAL_DATA_POL;
		if (sig.rs_pol)
			ser_conf |= DI_SER_CONF_SERIAL_RS_POL;
		if (sig.cs_pol)
			ser_conf |= DI_SER_CONF_SERIAL_CS_POL;
		ipu_di_write(ipu, disp, ser_conf, DI_SER_CONF);
	}

	mutex_unlock(&ipu->mutex_lock);
	return 0;
}
EXPORT_SYMBOL(ipu_init_async_panel);

/*!
 * This function sets the foreground and background plane global alpha blending
 * modes. This function also sets the DP graphic plane according to the
 * parameter of IPUv3 DP channel.
 *
 * @param	ipu		ipu handler
 * @param	channel		IPUv3 DP channel
 *
 * @param       enable          Boolean to enable or disable global alpha
 *                              blending. If disabled, local blending is used.
 *
 * @param       alpha           Global alpha value.
 *
 * @return      Returns 0 on success or negative error code on fail
 */
int32_t ipu_disp_set_global_alpha(struct ipu_soc *ipu, ipu_channel_t channel,
				bool enable, uint8_t alpha)
{
	uint32_t reg;
	uint32_t flow;
	bool bg_chan;

	if (channel == MEM_BG_SYNC || channel == MEM_FG_SYNC)
		flow = DP_SYNC;
	else if (channel == MEM_BG_ASYNC0 || channel == MEM_FG_ASYNC0)
		flow = DP_ASYNC0;
	else if (channel == MEM_BG_ASYNC1 || channel == MEM_FG_ASYNC1)
		flow = DP_ASYNC1;
	else
		return -EINVAL;

	if (channel == MEM_BG_SYNC || channel == MEM_BG_ASYNC0 ||
	    channel == MEM_BG_ASYNC1)
		bg_chan = true;
	else
		bg_chan = false;

	_ipu_get(ipu);

	mutex_lock(&ipu->mutex_lock);

	if (bg_chan) {
		reg = ipu_dp_read(ipu, DP_COM_CONF(flow));
		ipu_dp_write(ipu, reg & ~DP_COM_CONF_GWSEL, DP_COM_CONF(flow));
	} else {
		reg = ipu_dp_read(ipu, DP_COM_CONF(flow));
		ipu_dp_write(ipu, reg | DP_COM_CONF_GWSEL, DP_COM_CONF(flow));
	}

	if (enable) {
		reg = ipu_dp_read(ipu, DP_GRAPH_WIND_CTRL(flow)) & 0x00FFFFFFL;
		ipu_dp_write(ipu, reg | ((uint32_t) alpha << 24),
			     DP_GRAPH_WIND_CTRL(flow));

		reg = ipu_dp_read(ipu, DP_COM_CONF(flow));
		ipu_dp_write(ipu, reg | DP_COM_CONF_GWAM, DP_COM_CONF(flow));
	} else {
		reg = ipu_dp_read(ipu, DP_COM_CONF(flow));
		ipu_dp_write(ipu, reg & ~DP_COM_CONF_GWAM, DP_COM_CONF(flow));
	}

	reg = ipu_cm_read(ipu, IPU_SRM_PRI2) | 0x8;
	ipu_cm_write(ipu, reg, IPU_SRM_PRI2);

	mutex_unlock(&ipu->mutex_lock);

	_ipu_put(ipu);

	return 0;
}
EXPORT_SYMBOL(ipu_disp_set_global_alpha);

/*!
 * This function sets the transparent color key for SDC graphic plane.
 *
 * @param	ipu		ipu handler
 * @param       channel         Input parameter for the logical channel ID.
 *
 * @param       enable          Boolean to enable or disable color key
 *
 * @param       colorKey        24-bit RGB color for transparent color key.
 *
 * @return      Returns 0 on success or negative error code on fail
 */
int32_t ipu_disp_set_color_key(struct ipu_soc *ipu, ipu_channel_t channel,
				bool enable, uint32_t color_key)
{
	uint32_t reg, flow;
	int y, u, v;
	int red, green, blue;

	if (channel == MEM_BG_SYNC || channel == MEM_FG_SYNC)
		flow = DP_SYNC;
	else if (channel == MEM_BG_ASYNC0 || channel == MEM_FG_ASYNC0)
		flow = DP_ASYNC0;
	else if (channel == MEM_BG_ASYNC1 || channel == MEM_FG_ASYNC1)
		flow = DP_ASYNC1;
	else
		return -EINVAL;

	_ipu_get(ipu);

	mutex_lock(&ipu->mutex_lock);

	ipu->color_key_4rgb = true;
	/* Transform color key from rgb to yuv if CSC is enabled */
	if (((ipu->fg_csc_type == RGB2YUV) && (ipu->bg_csc_type == YUV2YUV)) ||
			((ipu->fg_csc_type == YUV2YUV) && (ipu->bg_csc_type == RGB2YUV)) ||
			((ipu->fg_csc_type == YUV2YUV) && (ipu->bg_csc_type == YUV2YUV)) ||
			((ipu->fg_csc_type == YUV2RGB) && (ipu->bg_csc_type == YUV2RGB))) {

		dev_dbg(ipu->dev, "color key 0x%x need change to yuv fmt\n", color_key);

		red = (color_key >> 16) & 0xFF;
		green = (color_key >> 8) & 0xFF;
		blue = color_key & 0xFF;

		y = _rgb_to_yuv(0, red, green, blue);
		u = _rgb_to_yuv(1, red, green, blue);
		v = _rgb_to_yuv(2, red, green, blue);
		color_key = (y << 16) | (u << 8) | v;

		ipu->color_key_4rgb = false;

		dev_dbg(ipu->dev, "color key change to yuv fmt 0x%x\n", color_key);
	}

	if (enable) {
		reg = ipu_dp_read(ipu, DP_GRAPH_WIND_CTRL(flow)) & 0xFF000000L;
		ipu_dp_write(ipu, reg | color_key, DP_GRAPH_WIND_CTRL(flow));

		reg = ipu_dp_read(ipu, DP_COM_CONF(flow));
		ipu_dp_write(ipu, reg | DP_COM_CONF_GWCKE, DP_COM_CONF(flow));
	} else {
		reg = ipu_dp_read(ipu, DP_COM_CONF(flow));
		ipu_dp_write(ipu, reg & ~DP_COM_CONF_GWCKE, DP_COM_CONF(flow));
	}

	reg = ipu_cm_read(ipu, IPU_SRM_PRI2) | 0x8;
	ipu_cm_write(ipu, reg, IPU_SRM_PRI2);

	mutex_unlock(&ipu->mutex_lock);

	_ipu_put(ipu);

	return 0;
}
EXPORT_SYMBOL(ipu_disp_set_color_key);

/*!
 * This function sets the gamma correction for DP output.
 *
 * @param	ipu		ipu handler
 * @param       channel         Input parameter for the logical channel ID.
 *
 * @param       enable          Boolean to enable or disable gamma correction.
 *
 * @param       constk        	Gamma piecewise linear approximation constk coeff.
 *
 * @param       slopek        	Gamma piecewise linear approximation slopek coeff.
 *
 * @return      Returns 0 on success or negative error code on fail
 */
int32_t ipu_disp_set_gamma_correction(struct ipu_soc *ipu, ipu_channel_t channel, bool enable, int constk[], int slopek[])
{
	uint32_t reg, flow, i;

	if (channel == MEM_BG_SYNC || channel == MEM_FG_SYNC)
		flow = DP_SYNC;
	else if (channel == MEM_BG_ASYNC0 || channel == MEM_FG_ASYNC0)
		flow = DP_ASYNC0;
	else if (channel == MEM_BG_ASYNC1 || channel == MEM_FG_ASYNC1)
		flow = DP_ASYNC1;
	else
		return -EINVAL;

	_ipu_get(ipu);

	mutex_lock(&ipu->mutex_lock);

	for (i = 0; i < 8; i++)
		ipu_dp_write(ipu, (constk[2*i] & 0x1ff) | ((constk[2*i+1] & 0x1ff) << 16), DP_GAMMA_C(flow, i));
	for (i = 0; i < 4; i++)
		ipu_dp_write(ipu, (slopek[4*i] & 0xff) | ((slopek[4*i+1] & 0xff) << 8) |
			((slopek[4*i+2] & 0xff) << 16) | ((slopek[4*i+3] & 0xff) << 24), DP_GAMMA_S(flow, i));

	reg = ipu_dp_read(ipu, DP_COM_CONF(flow));
	if (enable) {
		if ((ipu->bg_csc_type == RGB2YUV) || (ipu->bg_csc_type == YUV2YUV))
			reg |= DP_COM_CONF_GAMMA_YUV_EN;
		else
			reg &= ~DP_COM_CONF_GAMMA_YUV_EN;
		ipu_dp_write(ipu, reg | DP_COM_CONF_GAMMA_EN, DP_COM_CONF(flow));
	} else
		ipu_dp_write(ipu, reg & ~DP_COM_CONF_GAMMA_EN, DP_COM_CONF(flow));

	reg = ipu_cm_read(ipu, IPU_SRM_PRI2) | 0x8;
	ipu_cm_write(ipu, reg, IPU_SRM_PRI2);

	mutex_unlock(&ipu->mutex_lock);

	_ipu_put(ipu);

	return 0;
}
EXPORT_SYMBOL(ipu_disp_set_gamma_correction);

/*!
 * This function sets the window position of the foreground or background plane.
 * modes.
 *
 * @param	ipu		ipu handler
 * @param       channel         Input parameter for the logical channel ID.
 *
 * @param       x_pos           The X coordinate position to place window at.
 *                              The position is relative to the top left corner.
 *
 * @param       y_pos           The Y coordinate position to place window at.
 *                              The position is relative to the top left corner.
 *
 * @return      Returns 0 on success or negative error code on fail
 */
int32_t _ipu_disp_set_window_pos(struct ipu_soc *ipu, ipu_channel_t channel,
				int16_t x_pos, int16_t y_pos)
{
	u32 reg;
	uint32_t flow = 0;
	uint32_t dp_srm_shift;

	if ((channel == MEM_FG_SYNC) || (channel == MEM_BG_SYNC)) {
		flow = DP_SYNC;
		dp_srm_shift = 3;
	} else if (channel == MEM_FG_ASYNC0) {
		flow = DP_ASYNC0;
		dp_srm_shift = 5;
	} else if (channel == MEM_FG_ASYNC1) {
		flow = DP_ASYNC1;
		dp_srm_shift = 7;
	} else
		return -EINVAL;

	ipu_dp_write(ipu, (x_pos << 16) | y_pos, DP_FG_POS(flow));

	if (ipu_is_channel_busy(ipu, channel)) {
		/* controled by FSU if channel enabled */
		reg = ipu_cm_read(ipu, IPU_SRM_PRI2) & (~(0x3 << dp_srm_shift));
		reg |= (0x1 << dp_srm_shift);
		ipu_cm_write(ipu, reg, IPU_SRM_PRI2);
	} else {
		/* disable auto swap, controled by MCU if channel disabled */
		reg = ipu_cm_read(ipu, IPU_SRM_PRI2) & (~(0x3 << dp_srm_shift));
		ipu_cm_write(ipu, reg, IPU_SRM_PRI2);
	}

	return 0;
}

int32_t ipu_disp_set_window_pos(struct ipu_soc *ipu, ipu_channel_t channel,
				int16_t x_pos, int16_t y_pos)
{
	int ret;

	_ipu_get(ipu);
	mutex_lock(&ipu->mutex_lock);
	ret = _ipu_disp_set_window_pos(ipu, channel, x_pos, y_pos);
	mutex_unlock(&ipu->mutex_lock);
	_ipu_put(ipu);
	return ret;
}
EXPORT_SYMBOL(ipu_disp_set_window_pos);

int32_t _ipu_disp_get_window_pos(struct ipu_soc *ipu, ipu_channel_t channel,
				int16_t *x_pos, int16_t *y_pos)
{
	u32 reg;
	uint32_t flow = 0;

	if (channel == MEM_FG_SYNC)
		flow = DP_SYNC;
	else if (channel == MEM_FG_ASYNC0)
		flow = DP_ASYNC0;
	else if (channel == MEM_FG_ASYNC1)
		flow = DP_ASYNC1;
	else
		return -EINVAL;

	reg = ipu_dp_read(ipu, DP_FG_POS(flow));

	*x_pos = (reg >> 16) & 0x7FF;
	*y_pos = reg & 0x7FF;

	return 0;
}
int32_t ipu_disp_get_window_pos(struct ipu_soc *ipu, ipu_channel_t channel,
				int16_t *x_pos, int16_t *y_pos)
{
	int ret;

	_ipu_get(ipu);
	mutex_lock(&ipu->mutex_lock);
	ret = _ipu_disp_get_window_pos(ipu, channel, x_pos, y_pos);
	mutex_unlock(&ipu->mutex_lock);
	_ipu_put(ipu);
	return ret;
}
EXPORT_SYMBOL(ipu_disp_get_window_pos);

void ipu_reset_disp_panel(struct ipu_soc *ipu)
{
	uint32_t tmp;

	tmp = ipu_di_read(ipu, 1, DI_GENERAL);
	ipu_di_write(ipu, 1, tmp | 0x08, DI_GENERAL);
	msleep(10); /* tRES >= 100us */
	tmp = ipu_di_read(ipu, 1, DI_GENERAL);
	ipu_di_write(ipu, 1, tmp & ~0x08, DI_GENERAL);
	msleep(60);

	return;
}
EXPORT_SYMBOL(ipu_reset_disp_panel);

void ipu_disp_init(struct ipu_soc *ipu)
{
	ipu->fg_csc_type = ipu->bg_csc_type = CSC_NONE;
	ipu->color_key_4rgb = true;
	_ipu_init_dc_mappings(ipu);
	_ipu_dmfc_init(ipu, DMFC_NORMAL, 1);
}
