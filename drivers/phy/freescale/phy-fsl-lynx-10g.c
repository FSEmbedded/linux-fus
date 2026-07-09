// SPDX-License-Identifier: GPL-2.0+
/* Copyright 2021-2023 NXP */

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/phy.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/fsl/guts.h>

#include "phy-fsl-lynx-core.h"

/* SoC IP wrapper for protocol converters */
#define PCCR8				0x220
#define PCCR8_SGMIIa_KX			BIT(3)
#define PCCR8_SGMIIa_CFG		BIT(0)

#define PCCR9				0x224
#define PCCR9_QSGMIIa_CFG		BIT(0)
#define PCCR9_QXGMIIa_CFG		BIT(0)

#define PCCRB				0x22c
#define PCCRB_XFIa_CFG			BIT(0)
#define PCCRB_SXGMIIa_CFG		BIT(0)

#define SGMII_CFG(id)			(28 - (id) * 4)
#define QSGMII_CFG(id)			(28 - (id) * 4)
#define SXGMII_CFG(id)			(28 - (id) * 4)
#define QXGMII_CFG(id)			(12 - (id) * 4)
#define XFI_CFG(id)			(28 - (id) * 4)

#define CR(x)				((x) * 4)

#define A				0
#define B				1
#define C				2
#define D				3
#define E				4
#define F				5
#define G				6
#define H				7

#define SGMIIaCR0(id)			(0x1800 + (id) * 0x10)
#define QSGMIIaCR0(id)			(0x1880 + (id) * 0x10)
#define XAUIaCR0(id)			(0x1900 + (id) * 0x10)
#define XFIaCR0(id)			(0x1980 + (id) * 0x10)
#define SXGMIIaCR0(id)			(0x1a80 + (id) * 0x10)
#define QXGMIIaCR0(id)			(0x1b00 + (id) * 0x20)

#define SGMIIaCR0_RST_SGM		BIT(31)
#define SGMIIaCR0_RST_SGM_OFF		SGMIIaCR0_RST_SGM
#define SGMIIaCR0_RST_SGM_ON		0
#define SGMIIaCR0_PD_SGM		BIT(30)
#define SGMIIaCR1_SGPCS_EN		BIT(11)
#define SGMIIaCR1_SGPCS_DIS		0x0

#define QSGMIIaCR0_RST_QSGM		BIT(31)
#define QSGMIIaCR0_RST_QSGM_OFF		QSGMIIaCR0_RST_QSGM
#define QSGMIIaCR0_RST_QSGM_ON		0
#define QSGMIIaCR0_PD_QSGM		BIT(30)

/* MDEV_PORT is at the same bitfield address for all protocol converters */
#define MDEV_PORT_MSK			GENMASK(31, 27)
#define MDEV_PORT_X(x)			(((x) & MDEV_PORT_MSK) >> 27)

/* Per PLL registers */
#define PLLnCR0(pll)			((pll) * 0x20 + 0x4)

#define PLLnCR0_POFF			BIT(31)

#define PLLnCR0_REFCLK_SEL		GENMASK(30, 28)
#define PLLnCR0_REFCLK_SEL_100MHZ	0x0
#define PLLnCR0_REFCLK_SEL_125MHZ	0x1
#define PLLnCR0_REFCLK_SEL_156MHZ	0x2
#define PLLnCR0_REFCLK_SEL_150MHZ	0x3
#define PLLnCR0_REFCLK_SEL_161MHZ	0x4
#define PLLnCR0_PLL_LCK			BIT(23)
#define PLLnCR0_FRATE_SEL		GENMASK(19, 16)
#define PLLnCR0_FRATE_5G		0x0
#define PLLnCR0_FRATE_5_15625G		0x6
#define PLLnCR0_FRATE_4G		0x7
#define PLLnCR0_FRATE_3_125G		0x9
#define PLLnCR0_FRATE_3G		0xa

#define PLLnCR0_DLYDIV_SEL		GENMASK(1, 0)
#define PLLnCR0_DLYDIV_SEL_312_5_MHZ	1

/* Per SerDes lane registers */

/* Lane a Protocol Select status register */
#define LNaPSSR0(lane)			(0x100 + (lane) * 0x20)
#define LNaPSSR0_TYPE_X(x)		(((x) & GENMASK(30, 26)) >> 26)
#define LNaPSSR0_IS_QUAD_X(x)		(((x) & GENMASK(25, 24)) >> 24)
#define LNaPSSR0_MAC_X(x)		(((x) & GENMASK(19, 16)) >> 16)
#define LNaPSSR0_PCS_X(x)		(((x) & GENMASK(10, 8)) >> 8)
#define LNaPSSR0_LANE(x)		((x) & GENMASK(2, 0))

/* Lane a General Control Register */
#define LNaGCR0(lane)			(0x800 + (lane) * 0x40 + 0x0)
#define LNaGCR0_RPLL_PLLF		BIT(31)
#define LNaGCR0_RPLL_PLLS		0x0
#define LNaGCR0_RPLL_MSK		BIT(31)
#define LNaGCR0_RRAT_SEL_MSK		GENMASK(29, 28)
#define LNaGCR0_RRAT_SEL_X(x)		(((x) & LNaGCR0_RRAT_SEL_MSK) >> 28)
#define LNaGCR0_RRAT_SEL(x)		(((x) << 28) & LNaGCR0_RRAT_SEL_MSK)
#define LNaGCR0_TRAT_SEL_MSK		GENMASK(25, 24)
#define LNaGCR0_TRAT_SEL_X(x)		(((x) & LNaGCR0_TRAT_SEL_MSK) >> 24)
#define LNaGCR0_TRAT_SEL(x)		(((x) << 24) & LNaGCR0_TRAT_SEL_MSK)
#define LNaGCR0_TPLL_PLLF		BIT(27)
#define LNaGCR0_TPLL_PLLS		0x0
#define LNaGCR0_TPLL_MSK		BIT(27)
#define LNaGCR0_RRST_OFF		LNaGCR0_RRST
#define LNaGCR0_TRST_OFF		LNaGCR0_TRST
#define LNaGCR0_RRST_ON			0x0
#define LNaGCR0_TRST_ON			0x0
#define LNaGCR0_RRST			BIT(22)
#define LNaGCR0_TRST			BIT(21)
#define LNaGCR0_RX_PD			BIT(20)
#define LNaGCR0_TX_PD			BIT(19)
#define LNaGCR0_IF20BIT_EN		BIT(18)
#define LNaGCR0_PROTS_MSK		GENMASK(11, 7)
#define LNaGCR0_PROTS(x)		(((x) << 7) & LNaGCR0_PROTS_MSK)

#define LNaGCR1(lane)			(0x800 + (lane) * 0x40 + 0x4)
#define LNaGCR1_RDAT_INV		BIT(31)
#define LNaGCR1_TDAT_INV		BIT(30)
#define LNaGCR1_OPAD_CTL		BIT(26)
#define LNaGCR1_REIDL_TH_MSK		GENMASK(22, 20)
#define LNaGCR1_REIDL_TH(x)		(((x) << 20) & LNaGCR1_REIDL_TH_MSK)
#define LNaGCR1_REIDL_TH_X(x)		(((x) & LNaGCR1_REIDL_TH_MSK) >> 20)
#define LNaGCR1_REIDL_EX_SEL_MSK	GENMASK(19, 18)
#define LNaGCR1_REIDL_EX_SEL(x)		(((x) << 18) & LNaGCR1_REIDL_EX_SEL_MSK)
#define LNaGCR1_REIDL_ET_SEL_MSK	GENMASK(17, 16)
#define LNaGCR1_REIDL_ET_SEL(x)		(((x) << 16) & LNaGCR1_REIDL_ET_SEL_MSK)
#define LNaGCR1_REIDL_EX_MSB		BIT(15)
#define LNaGCR1_REIDL_ET_MSB		BIT(14)
#define LNaGCR1_REQ_CTL_SNP		BIT(13)
#define LNaGCR1_REQ_CDR_SNP		BIT(12)
#define LNaGCR1_TRSTDIR			BIT(7)
#define LNaGCR1_REQ_BIN_SNP		BIT(6)
#define LNaGCR1_ISLEW_RCTL_MSK		GENMASK(5, 4)
#define LNaGCR1_ISLEW_RCTL(x)		(((x) << 4) & LNaGCR1_ISLEW_RCTL_MSK)
#define LNaGCR1_OSLEW_RCTL_MSK		GENMASK(1, 0)
#define LNaGCR1_OSLEW_RCTL(x)		((x) & LNaGCR1_OSLEW_RCTL_MSK)

#define LNaRECR0(lane)			(0x800 + (lane) * 0x40 + 0x10)
#define LNaRECR0_RXEQ_BST		BIT(28)
#define LNaRECR0_GK2OVD_MSK		GENMASK(27, 24)
#define LNaRECR0_GK2OVD(x)		(((x) << 24) & LNaRECR0_GK2OVD_MSK)
#define LNaRECR0_GK3OVD_MSK		GENMASK(19, 16)
#define LNaRECR0_GK3OVD(x)		(((x) << 16) & LNaRECR0_GK3OVD_MSK)
#define LNaRECR0_GK2OVD_EN		BIT(15)
#define LNaRECR0_GK3OVD_EN		BIT(14)
#define LNaRECR0_OSETOVD_EN		BIT(13)
#define LNaRECR0_BASE_WAND_MSK		GENMASK(11, 10)
#define LNaRECR0_BASE_WAND(x)		(((x) << 10) & LNaRECR0_BASE_WAND_MSK)
#define LNaRECR0_OSETOVD_MSK		((x) & GENMASK(6, 0))
#define LNaRECR0_OSETOVD(x)		((x) & LNaRECR0_OSETOVD_MSK)

#define LNaRECR1(lane)			(0x800 + (lane) * 0x40 + 0x14)
#define LNaRECR1_GK2STAT_X(x)		(((x) & GENMASK(27, 24)) >> 24)
#define LNaRECR1_GK3STAT_X(x)		(((x) & GENMASK(19, 16)) >> 16)
#define LNaRECR1_OSETSTAT_X(x)		(((x) & GENMASK(13, 7)) >> 7)
#define LNaRECR1_BIN_SNP_DONE		BIT(2)
#define LNaRECR1_CTL_SNP_DONE		BIT(1)

#define LNaTECR0(lane)			(0x800 + (lane) * 0x40 + 0x18)
#define LNaTECR0_TEQ_TYPE_MSK		GENMASK(29, 28)
#define LNaTECR0_TEQ_TYPE(x)		(((x) << 28) & LNaTECR0_TEQ_TYPE_MSK)
#define LNaTECR0_TEQ_TYPE_X(x)		(((x) & LNaTECR0_TEQ_TYPE_MSK) >> 28)
#define LNaTECR0_SGN_PREQ		BIT(26)
#define LNaTECR0_RATIO_PREQ_MSK		GENMASK(25, 22)
#define LNaTECR0_RATIO_PREQ(x)		(((x) << 22) & LNaTECR0_RATIO_PREQ_MSK)
#define LNaTECR0_RATIO_PREQ_X(x)	(((x) & LNaTECR0_RATIO_PREQ_MSK) >> 22)
#define LNaTECR0_SGN_POST1Q		BIT(21)
#define LNaTECR0_RATIO_PST1Q_MSK	GENMASK(20, 16)
#define LNaTECR0_RATIO_PST1Q(x)		(((x) << 16) & LNaTECR0_RATIO_PST1Q_MSK)
#define LNaTECR0_RATIO_PST1Q_X(x)	(((x) & LNaTECR0_RATIO_PST1Q_MSK) >> 16)
#define LNaTECR0_ADPT_EQ_MSK		GENMASK(13, 8)
#define LNaTECR0_ADPT_EQ(x)		(((x) << 8) & LNaTECR0_ADPT_EQ_MSK)
#define LNaTECR0_ADPT_EQ_X(x)		(((x) & LNaTECR0_ADPT_EQ_MSK) >> 8)
#define LNaTECR0_AMP_RED_MSK		GENMASK(5, 0)
#define LNaTECR0_AMP_RED(x)		((x) & LNaTECR0_AMP_RED_MSK)

#define LNaTTLCR0(lane)			(0x800 + (lane) * 0x40 + 0x20)
#define LNaTTLCR1(lane)			(0x800 + (lane) * 0x40 + 0x24)
#define LNaTTLCR2(lane)			(0x800 + (lane) * 0x40 + 0x28)

#define LNaTCSR0(lane)			(0x800 + (lane) * 0x40 + 0x30)

#define LNaTCSR1(lane)			(0x800 + (lane) * 0x40 + 0x34)
#define LNaTCSR1_CDR_SEL_MSK		GENMASK(18, 16)
#define LNaTCSR1_CDR_SEL(x)		(((x) << 16) & LNaTCSR1_CDR_SEL_MSK)
#define LNaTCSR1_EQ_SNPBIN_DATA_X(x)	(((x) & GENMASK(15, 6)) >> 6)
#define LNaTCSR1_EQ_SNPBIN_DATA_SGN	BIT(8)
#define LNaTCSR1_CDR_STAT_X(x)		(((x) & GENMASK(5, 1)) >> 1)

#define LNaTCSR2(lane)			(0x800 + (lane) * 0x40 + 0x38)
#define LNaTCSR3(lane)			(0x800 + (lane) * 0x40 + 0x3C)
#define LNaTCSR3_CDR_LCK		BIT(27)

#define CDR_SLEEP_US			50
#define CDR_TIMEOUT_US			500

#define SNAPSHOT_SLEEP_US		1
#define SNAPSHOT_TIMEOUT_US		1000

enum lynx_10g_rat_sel {
	RAT_SEL_FULL = 0x0,
	RAT_SEL_HALF = 0x1,
	RAT_SEL_QUARTER = 0x2,
	RAT_SEL_DOUBLE = 0x3,
};

enum lynx_10g_eq_bin_data_type {
	EQ_BIN_DATA_SEL_BIN_1 = 0,
	EQ_BIN_DATA_SEL_BIN_2 = 1,
	EQ_BIN_DATA_SEL_BIN_3 = 2,
	EQ_BIN_DATA_SEL_OFFSET = 3,
	EQ_BIN_DATA_SEL_BIN_BLW = 4,
	EQ_BIN_DATA_SEL_BIN_DATA_AVG = 5,
	EQ_BIN_DATA_SEL_BIN_M1 = 6,
	EQ_BIN_DATA_SEL_BIN_LONG = 7,
};

enum lynx_10g_eq_type {
	EQ_TYPE_NO_EQ = 0,
	EQ_TYPE_2TAP = 1,
	EQ_TYPE_3TAP = 2,
};

enum lynx_10g_proto_sel {
	PROTO_SEL_PCIE = 0,
	PROTO_SEL_SGMII_BASEX_KX_QSGMII = 1,
	PROTO_SEL_SATA = 2,
	PROTO_SEL_XAUI = 4,
	PROTO_SEL_XFI_10GBASER_KR_SXGMII = 0xa,
};

struct lynx_10g_proto_conf {
	int proto_sel;
	int if20bit_en;
	int reidl_th;
	int reidl_et_msb;
	int reidl_et_sel;
	int reidl_ex_msb;
	int reidl_ex_sel;
	int islew_rctl;
	int oslew_rctl;
	int rxeq_bst;
	int gk2ovd;
	int gk3ovd;
	int gk2ovd_en;
	int gk3ovd_en;
	int base_wand;
	int teq_type;
	int sgn_preq;
	int ratio_preq;
	int sgn_post1q;
	int ratio_post1q;
	int adpt_eq;
	int amp_red;
	int ttlcr0;
};

static const struct lynx_10g_proto_conf lynx_10g_proto_conf[LANE_MODE_MAX] = {
	[LANE_MODE_1000BASEX_SGMII] = {
		.proto_sel = PROTO_SEL_SGMII_BASEX_KX_QSGMII,
		.if20bit_en = 0,
		.reidl_th = 1,
		.reidl_ex_msb = 0,
		.reidl_ex_sel = 3,
		.reidl_et_msb = 1,
		.reidl_et_sel = 0,
		.islew_rctl = 1,
		.oslew_rctl = 1,
		.rxeq_bst = 0,
		.gk2ovd = 15,
		.gk3ovd = 15,
		.gk2ovd_en = 1,
		.gk3ovd_en = 1,
		.base_wand = 0,
		.teq_type = EQ_TYPE_NO_EQ,
		.sgn_preq = 0,
		.ratio_preq = 0,
		.sgn_post1q = 0,
		.ratio_post1q = 0,
		.adpt_eq = 48,
		.amp_red = 6,
		.ttlcr0 = 0x39000400,
	},
	[LANE_MODE_1000BASEKX] = {
		.proto_sel = PROTO_SEL_SGMII_BASEX_KX_QSGMII,
		.if20bit_en = 0,
		.reidl_th = 0,
		.reidl_ex_msb = 0,
		.reidl_ex_sel = 0,
		.reidl_et_msb = 0,
		.reidl_et_sel = 0,
		.islew_rctl = 1,
		.oslew_rctl = 1,
		.rxeq_bst = 0,
		.gk2ovd = 15,
		.gk3ovd = 15,
		.gk2ovd_en = 1,
		.gk3ovd_en = 1,
		.base_wand = 0,
		.teq_type = EQ_TYPE_NO_EQ,
		.sgn_preq = 0,
		.ratio_preq = 0,
		.sgn_post1q = 0,
		.ratio_post1q = 0,
		.adpt_eq = 48,
		.amp_red = 0,
		.ttlcr0 = 0x39000400,
	},
	[LANE_MODE_2500BASEX] = {
		.proto_sel = PROTO_SEL_SGMII_BASEX_KX_QSGMII,
		.if20bit_en = 0,
		.reidl_th = 0,
		.reidl_ex_msb = 0,
		.reidl_ex_sel = 0,
		.reidl_et_msb = 0,
		.reidl_et_sel = 0,
		.islew_rctl = 2,
		.oslew_rctl = 2,
		.rxeq_bst = 0,
		.gk2ovd = 0,
		.gk3ovd = 0,
		.gk2ovd_en = 0,
		.gk3ovd_en = 0,
		.base_wand = 0,
		.teq_type = EQ_TYPE_2TAP,
		.sgn_preq = 0,
		.ratio_preq = 0,
		.sgn_post1q = 1,
		.ratio_post1q = 6,
		.adpt_eq = 48,
		.amp_red = 0,
		.ttlcr0 = 0x00000400,
	},
	[LANE_MODE_QSGMII] = {
		.proto_sel = PROTO_SEL_SGMII_BASEX_KX_QSGMII,
		.if20bit_en = 0,
		.reidl_th = 0,
		.reidl_ex_msb = 0,
		.reidl_ex_sel = 0,
		.reidl_et_msb = 0,
		.reidl_et_sel = 0,
		.islew_rctl = 1,
		.oslew_rctl = 1,
		.rxeq_bst = 0,
		.gk2ovd = 0,
		.gk3ovd = 0,
		.gk2ovd_en = 0,
		.gk3ovd_en = 0,
		.base_wand = 0,
		.teq_type = EQ_TYPE_2TAP,
		.sgn_preq = 0,
		.ratio_preq = 0,
		.sgn_post1q = 1,
		.ratio_post1q = 6,
		.adpt_eq = 48,
		.amp_red = 2,
		.ttlcr0 = 0x00000400,
	},
	[LANE_MODE_10G_QXGMII] = {
		.proto_sel = PROTO_SEL_XFI_10GBASER_KR_SXGMII,
		.if20bit_en = 1,
		.reidl_th = 0,
		.reidl_ex_msb = 0,
		.reidl_ex_sel = 0,
		.reidl_et_msb = 0,
		.reidl_et_sel = 0,
		.islew_rctl = 1,
		.oslew_rctl = 1,
		.rxeq_bst = 0,
		.gk2ovd = 0,
		.gk3ovd = 0,
		.gk2ovd_en = 0,
		.gk3ovd_en = 0,
		.base_wand = 1,
		.teq_type = EQ_TYPE_NO_EQ,
		.sgn_preq = 0,
		.ratio_preq = 0,
		.sgn_post1q = 0,
		.ratio_post1q = 0,
		.adpt_eq = 48,
		.amp_red = 0,
		.ttlcr0 = 0x00000400,
	},
	[LANE_MODE_USXGMII] = {
		.proto_sel = PROTO_SEL_XFI_10GBASER_KR_SXGMII,
		.if20bit_en = 1,
		.reidl_th = 0,
		.reidl_ex_msb = 0,
		.reidl_ex_sel = 0,
		.reidl_et_msb = 0,
		.reidl_et_sel = 0,
		.islew_rctl = 1,
		.oslew_rctl = 1,
		.rxeq_bst = 0,
		.gk2ovd = 0,
		.gk3ovd = 0,
		.gk2ovd_en = 0,
		.gk3ovd_en = 0,
		.base_wand = 1,
		.teq_type = EQ_TYPE_NO_EQ,
		.sgn_preq = 0,
		.ratio_preq = 0,
		.sgn_post1q = 1,
		.ratio_post1q = 0,
		.adpt_eq = 48,
		.amp_red = 0,
		.ttlcr0 = 0x00000400,
	},
	[LANE_MODE_10GBASER] = {
		.proto_sel = PROTO_SEL_XFI_10GBASER_KR_SXGMII,
		.if20bit_en = 1,
		.reidl_th = 0,
		.reidl_ex_msb = 0,
		.reidl_ex_sel = 0,
		.reidl_et_msb = 0,
		.reidl_et_sel = 0,
		.islew_rctl = 2,
		.oslew_rctl = 2,
		.rxeq_bst = 1,
		.gk2ovd = 0,
		.gk3ovd = 0,
		.gk2ovd_en = 0,
		.gk3ovd_en = 0,
		.base_wand = 1,
		.teq_type = EQ_TYPE_2TAP,
		.sgn_preq = 0,
		.ratio_preq = 0,
		.sgn_post1q = 1,
		.ratio_post1q = 3,
		.adpt_eq = 48,
		.amp_red = 7,
		.ttlcr0 = 0x00000400,
	},
	[LANE_MODE_10GBASEKR] = {
		.proto_sel = PROTO_SEL_XFI_10GBASER_KR_SXGMII,
		.if20bit_en = 1,
		.reidl_th = 0,
		.reidl_ex_msb = 0,
		.reidl_ex_sel = 0,
		.reidl_et_msb = 0,
		.reidl_et_sel = 0,
		.islew_rctl = 2,
		.oslew_rctl = 2,
		.rxeq_bst = 1,
		.gk2ovd = 0,
		.gk3ovd = 0,
		.gk2ovd_en = 0,
		.gk3ovd_en = 0,
		.base_wand = 1,
		.teq_type = EQ_TYPE_3TAP,
		.sgn_preq = 1,
		.ratio_preq = 2,
		.sgn_post1q = 1,
		.ratio_post1q = 5,
		.adpt_eq = 41,
		.amp_red = 7,
		.ttlcr0 = 0x00000400,
	},
};

static const int lynx_10g_bin_type_to_bin_sel[] = {
	[BIN_1] = EQ_BIN_DATA_SEL_BIN_1,
	[BIN_2] = EQ_BIN_DATA_SEL_BIN_2,
	[BIN_3] = EQ_BIN_DATA_SEL_BIN_3,
	[BIN_4] = -EOPNOTSUPP,
	[BIN_OFFSET] = EQ_BIN_DATA_SEL_OFFSET,
	[BIN_BLW] = -EOPNOTSUPP,
	[BIN_DATA_AVG] = -EOPNOTSUPP,
	[BIN_M1] = EQ_BIN_DATA_SEL_BIN_M1,
	[BIN_LONG] = EQ_BIN_DATA_SEL_BIN_LONG,
};

static int ls1028a_get_pccr(enum lynx_lane_mode lane_mode, int lane,
			    struct lynx_pccr *pccr)
{
	switch (lane_mode) {
	case LANE_MODE_1000BASEX_SGMII:
	case LANE_MODE_1000BASEKX:
	case LANE_MODE_2500BASEX:
		pccr->offset = PCCR8;
		pccr->width = 4;
		pccr->shift = SGMII_CFG(lane);
		break;
	case LANE_MODE_QSGMII:
		if (lane != 1)
			return -EOPNOTSUPP;

		pccr->offset = PCCR9;
		pccr->width = 3;
		pccr->shift = QSGMII_CFG(A);
		break;
	case LANE_MODE_10G_QXGMII:
		if (lane != 1)
			return -EOPNOTSUPP;

		pccr->offset = PCCR9;
		pccr->width = 3;
		pccr->shift = QXGMII_CFG(A);
		break;
	case LANE_MODE_USXGMII:
		if (lane != 0)
			return -EOPNOTSUPP;

		pccr->offset = PCCRB;
		pccr->width = 3;
		pccr->shift = SXGMII_CFG(A);
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static int ls1028a_get_pcvt_offset(int lane, enum lynx_lane_mode mode)
{
	switch (mode) {
	case LANE_MODE_1000BASEX_SGMII:
	case LANE_MODE_1000BASEKX:
	case LANE_MODE_2500BASEX:
		return SGMIIaCR0(lane);
	case LANE_MODE_QSGMII:
		return lane == 1 ? QSGMIIaCR0(A) : -EOPNOTSUPP;
	case LANE_MODE_USXGMII:
		return lane == 0 ? SXGMIIaCR0(A) : -EOPNOTSUPP;
	case LANE_MODE_10G_QXGMII:
		return lane == 1 ? QXGMIIaCR0(A) : -EOPNOTSUPP;
	default:
		return -EOPNOTSUPP;
	}
}

static bool ls1028a_lane_supports_mode(int lane, enum lynx_lane_mode mode)
{
	switch (mode) {
	case LANE_MODE_1000BASEX_SGMII:
	case LANE_MODE_1000BASEKX:
	case LANE_MODE_2500BASEX:
		return true;
	case LANE_MODE_QSGMII:
		return lane == 1;
	case LANE_MODE_USXGMII:
		return lane == 0;
	case LANE_MODE_10G_QXGMII:
		return lane == 1;
	default:
		return false;
	}
}

static const struct lynx_info lynx_info_ls1028a = {
	.get_pccr = ls1028a_get_pccr,
	.get_pcvt_offset = ls1028a_get_pcvt_offset,
	.lane_supports_mode = ls1028a_lane_supports_mode,
	.num_lanes = 4,
	.has_hardcoded_usxgmii = true,
	.index = 1,
};

static int ls1046a_serdes1_get_pccr(enum lynx_lane_mode lane_mode, int lane,
				    struct lynx_pccr *pccr)
{
	switch (lane_mode) {
	case LANE_MODE_1000BASEX_SGMII:
	case LANE_MODE_1000BASEKX:
	case LANE_MODE_2500BASEX:
		pccr->offset = PCCR8;
		pccr->width = 4;
		pccr->shift = SGMII_CFG(lane);
		break;
	case LANE_MODE_QSGMII:
		if (lane != 1)
			return -EOPNOTSUPP;

		pccr->offset = PCCR9;
		pccr->width = 3;
		pccr->shift = QSGMII_CFG(B);
		break;
	case LANE_MODE_10GBASER:
	case LANE_MODE_10GBASEKR:
		switch (lane) {
		case 2:
			pccr->shift = XFI_CFG(A);
			break;
		case 3:
			pccr->shift = XFI_CFG(B);
			break;
		default:
			return -EOPNOTSUPP;
		}

		pccr->offset = PCCRB;
		pccr->width = 3;
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static int ls1046a_serdes1_get_pcvt_offset(int lane, enum lynx_lane_mode mode)
{
	switch (mode) {
	case LANE_MODE_1000BASEX_SGMII:
	case LANE_MODE_1000BASEKX:
	case LANE_MODE_2500BASEX:
		return SGMIIaCR0(lane);
	case LANE_MODE_QSGMII:
		if (lane != 1)
			return -EOPNOTSUPP;

		return QSGMIIaCR0(B);
	case LANE_MODE_10GBASER:
	case LANE_MODE_10GBASEKR:
		switch (lane) {
		case 2:
			return XFIaCR0(A);
		case 3:
			return XFIaCR0(B);
		default:
			return -EOPNOTSUPP;
		}
	default:
		return -EOPNOTSUPP;
	}
}

static bool ls1046a_serdes1_lane_supports_mode(int lane,
					       enum lynx_lane_mode mode)
{
	switch (mode) {
	case LANE_MODE_1000BASEX_SGMII:
	case LANE_MODE_1000BASEKX:
	case LANE_MODE_2500BASEX:
		return true;
	case LANE_MODE_QSGMII:
		return lane == 1;
	case LANE_MODE_10GBASER:
	case LANE_MODE_10GBASEKR:
		return lane == 2 || lane == 3;
	default:
		return false;
	}
}

static const struct lynx_info lynx_info_ls1046a_serdes1 = {
	.get_pccr = ls1046a_serdes1_get_pccr,
	.get_pcvt_offset = ls1046a_serdes1_get_pcvt_offset,
	.lane_supports_mode = ls1046a_serdes1_lane_supports_mode,
	.num_lanes = 4,
	.index = 1,
};

static int ls1046a_serdes2_get_pccr(enum lynx_lane_mode lane_mode, int lane,
				    struct lynx_pccr *pccr)
{
	switch (lane_mode) {
	case LANE_MODE_1000BASEX_SGMII:
	case LANE_MODE_1000BASEKX:
	case LANE_MODE_2500BASEX:
		if (lane != 1)
			return -EOPNOTSUPP;

		pccr->offset = PCCR8;
		pccr->width = 4;
		pccr->shift = SGMII_CFG(B);
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static int ls1046a_serdes2_get_pcvt_offset(int lane, enum lynx_lane_mode mode)
{
	switch (mode) {
	case LANE_MODE_1000BASEX_SGMII:
	case LANE_MODE_1000BASEKX:
	case LANE_MODE_2500BASEX:
		if (lane != 1)
			return -EOPNOTSUPP;

		return SGMIIaCR0(B);
	default:
		return -EOPNOTSUPP;
	}
}

static bool ls1046a_serdes2_lane_supports_mode(int lane,
					       enum lynx_lane_mode mode)
{
	switch (mode) {
	case LANE_MODE_1000BASEX_SGMII:
	case LANE_MODE_1000BASEKX:
	case LANE_MODE_2500BASEX:
		return lane == 1;
	default:
		return false;
	}
}

static const struct lynx_info lynx_info_ls1046a_serdes2 = {
	.get_pccr = ls1046a_serdes2_get_pccr,
	.get_pcvt_offset = ls1046a_serdes2_get_pcvt_offset,
	.lane_supports_mode = ls1046a_serdes2_lane_supports_mode,
	.num_lanes = 4,
	.index = 2,
};

static int ls1088a_serdes1_get_pccr(enum lynx_lane_mode lane_mode, int lane,
				    struct lynx_pccr *pccr)
{
	switch (lane_mode) {
	case LANE_MODE_1000BASEX_SGMII:
	case LANE_MODE_1000BASEKX:
		pccr->offset = PCCR8;
		pccr->width = 4;
		pccr->shift = SGMII_CFG(lane);
		break;
	case LANE_MODE_QSGMII:
		switch (lane) {
		case 0:
			pccr->shift = QSGMII_CFG(A);
			break;
		case 1:
		case 3:
			pccr->shift = QSGMII_CFG(B);
			break;
		default:
			return -EOPNOTSUPP;
		}

		pccr->offset = PCCR9;
		pccr->width = 3;
		break;
	case LANE_MODE_10GBASER:
	case LANE_MODE_10GBASEKR:
		switch (lane) {
		case 2:
			pccr->shift = XFI_CFG(A);
			break;
		case 3:
			pccr->shift = XFI_CFG(B);
			break;
		default:
			return -EOPNOTSUPP;
		}

		pccr->offset = PCCRB;
		pccr->width = 3;
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static int ls1088a_serdes1_get_pcvt_offset(int lane, enum lynx_lane_mode mode)
{
	switch (mode) {
	case LANE_MODE_1000BASEX_SGMII:
	case LANE_MODE_1000BASEKX:
		return SGMIIaCR0(lane);
	case LANE_MODE_QSGMII:
		switch (lane) {
		case 0:
			return QSGMIIaCR0(A);
		case 1:
		case 3:
			return QSGMIIaCR0(B);
		default:
			return -EOPNOTSUPP;
		}
	case LANE_MODE_10GBASER:
	case LANE_MODE_10GBASEKR:
		switch (lane) {
		case 2:
			return XFIaCR0(A);
		case 3:
			return XFIaCR0(B);
		default:
			return -EOPNOTSUPP;
		}
	default:
		return -EOPNOTSUPP;
	}
}

static bool ls1088a_serdes1_lane_supports_mode(int lane, enum lynx_lane_mode mode)
{
	switch (mode) {
	case LANE_MODE_1000BASEX_SGMII:
	case LANE_MODE_1000BASEKX:
		return true;
	case LANE_MODE_QSGMII:
		switch (lane) {
		case 0:
		case 1:
		case 3:
			return true;
		default:
			return false;
		}
	case LANE_MODE_10GBASER:
	case LANE_MODE_10GBASEKR:
		switch (lane) {
		case 2:
		case 3:
			return true;
		default:
			return false;
		}
		return true;
	default:
		return false;
	}
}

static const struct lynx_info lynx_info_ls1088a_serdes1 = {
	.get_pccr = ls1088a_serdes1_get_pccr,
	.get_pcvt_offset = ls1088a_serdes1_get_pcvt_offset,
	.lane_supports_mode = ls1088a_serdes1_lane_supports_mode,
	.num_lanes = 4,
	.index = 1,
};

static int ls2088a_serdes1_get_pccr(enum lynx_lane_mode lane_mode, int lane,
				    struct lynx_pccr *pccr)
{
	switch (lane_mode) {
	case LANE_MODE_1000BASEX_SGMII:
	case LANE_MODE_1000BASEKX:
	case LANE_MODE_2500BASEX:
		pccr->offset = PCCR8;
		pccr->width = 4;
		pccr->shift = SGMII_CFG(lane);
		break;
	case LANE_MODE_QSGMII:
		switch (lane) {
		case 2:
		case 6:
			pccr->shift = QSGMII_CFG(A);
			break;
		case 7:
			pccr->shift = QSGMII_CFG(B);
			break;
		case 0:
		case 4:
			pccr->shift = QSGMII_CFG(C);
			break;
		case 1:
		case 5:
			pccr->shift = QSGMII_CFG(D);
			break;
		default:
			return -EOPNOTSUPP;
		}

		pccr->offset = PCCR9;
		pccr->width = 3;
		break;
	case LANE_MODE_10GBASER:
	case LANE_MODE_10GBASEKR:
		pccr->offset = PCCRB;
		pccr->width = 3;
		pccr->shift = XFI_CFG(lane);
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static int ls2088a_serdes1_get_pcvt_offset(int lane, enum lynx_lane_mode mode)
{
	switch (mode) {
	case LANE_MODE_1000BASEX_SGMII:
	case LANE_MODE_1000BASEKX:
	case LANE_MODE_2500BASEX:
		return SGMIIaCR0(lane);
	case LANE_MODE_QSGMII:
		switch (lane) {
		case 2:
		case 6:
			return QSGMIIaCR0(A);
		case 7:
			return QSGMIIaCR0(B);
		case 0:
		case 4:
			return QSGMIIaCR0(C);
		case 1:
		case 5:
			return QSGMIIaCR0(D);
			break;
		default:
			return -EOPNOTSUPP;
		}
	case LANE_MODE_10GBASER:
	case LANE_MODE_10GBASEKR:
		return XFIaCR0(lane);
	default:
		return -EOPNOTSUPP;
	}
}

static bool ls2088a_serdes1_lane_supports_mode(int lane, enum lynx_lane_mode mode)
{
	switch (mode) {
	case LANE_MODE_1000BASEX_SGMII:
	case LANE_MODE_1000BASEKX:
	case LANE_MODE_2500BASEX:
		return true;
	case LANE_MODE_QSGMII:
		switch (lane) {
		case 0:
		case 1:
		case 2:
		case 4:
		case 5:
		case 6:
		case 7:
			return true;
		default:
			return false;
		}
	case LANE_MODE_10GBASER:
	case LANE_MODE_10GBASEKR:
		return true;
	default:
		return false;
	}
}

static const struct lynx_info lynx_info_ls2088a_serdes1 = {
	.get_pccr = ls2088a_serdes1_get_pccr,
	.get_pcvt_offset = ls2088a_serdes1_get_pcvt_offset,
	.lane_supports_mode = ls2088a_serdes1_lane_supports_mode,
	.num_lanes = 8,
	.index = 1,
};

static int ls2088a_serdes2_get_pccr(enum lynx_lane_mode lane_mode, int lane,
				    struct lynx_pccr *pccr)
{
	switch (lane_mode) {
	case LANE_MODE_1000BASEX_SGMII:
	case LANE_MODE_1000BASEKX:
	case LANE_MODE_2500BASEX:
		pccr->offset = PCCR8;
		pccr->width = 4;
		pccr->shift = SGMII_CFG(lane);
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static int ls2088a_serdes2_get_pcvt_offset(int lane, enum lynx_lane_mode mode)
{
	switch (mode) {
	case LANE_MODE_1000BASEX_SGMII:
	case LANE_MODE_1000BASEKX:
	case LANE_MODE_2500BASEX:
		return SGMIIaCR0(lane);
	default:
		return -EOPNOTSUPP;
	}
}

static bool ls2088a_serdes2_lane_supports_mode(int lane, enum lynx_lane_mode mode)
{
	switch (mode) {
	case LANE_MODE_1000BASEX_SGMII:
	case LANE_MODE_1000BASEKX:
	case LANE_MODE_2500BASEX:
		return true;
	default:
		return false;
	}
}

static const struct lynx_info lynx_info_ls2088a_serdes2 = {
	.get_pccr = ls2088a_serdes2_get_pccr,
	.get_pcvt_offset = ls2088a_serdes2_get_pcvt_offset,
	.lane_supports_mode = ls2088a_serdes2_lane_supports_mode,
	.num_lanes = 8,
	.index = 2,
};

static bool lynx_10g_cdr_lock_check(struct lynx_lane *lane)
{
	u32 tcsr3 = lynx_lane_read(lane, LNaTCSR3);

	if (tcsr3 & LNaTCSR3_CDR_LCK)
		return true;

	dev_dbg(&lane->phy->dev,
		"Lane %c CDR unlocked, resetting receiver...\n",
		'A' + lane->id);

	lynx_lane_rmw(lane, LNaGCR0, LNaGCR0_RRST_ON, LNaGCR0_RRST);
	usleep_range(1, 2);
	lynx_lane_rmw(lane, LNaGCR0, LNaGCR0_RRST_OFF, LNaGCR0_RRST);

	usleep_range(1, 2);
	tcsr3 = lynx_lane_read(lane, LNaTCSR3);

	return !!(tcsr3 & LNaTCSR3_CDR_LCK);
}

#define work_to_lynx(w) container_of((w), struct lynx_priv, cdr_check.work)

static void lynx_10g_cdr_lock_check_work(struct work_struct *work)
{
	struct lynx_priv *priv = work_to_lynx(work);
	struct lynx_lane *lane;
	int i;

	for (i = 0; i < priv->info->num_lanes; i++) {
		lane = &priv->lane[i];
		if (!lane->phy)
			continue;

		mutex_lock(&lane->phy->mutex);

		if (!lane->init || !lane->powered_up) {
			mutex_unlock(&lane->phy->mutex);
			continue;
		}

		lynx_10g_cdr_lock_check(lane);

		mutex_unlock(&lane->phy->mutex);
	}

	queue_delayed_work(system_power_efficient_wq, &priv->cdr_check,
			   msecs_to_jiffies(1000));
}

/* Halting puts the lane in a mode in which it can be reconfigured */
static void lynx_10g_lane_halt(struct phy *phy)
{
	struct lynx_lane *lane = phy_get_drvdata(phy);

	if (!lane->powered_up)
		return;

	/* Issue a reset request */
	lynx_lane_rmw(lane, LNaGCR0,
		      LNaGCR0_RRST_ON | LNaGCR0_TRST_ON,
		      LNaGCR0_RRST | LNaGCR0_TRST);

	/* The RM says to wait for at least 50ns */
	usleep_range(1, 2);
}

static void lynx_10g_lane_reset(struct phy *phy)
{
	struct lynx_lane *lane = phy_get_drvdata(phy);

	if (!lane->powered_up)
		return;

	/* Finalize the reset request */
	lynx_lane_rmw(lane, LNaGCR0,
		      LNaGCR0_RRST_OFF | LNaGCR0_TRST_OFF,
		      LNaGCR0_RRST | LNaGCR0_TRST);

	/* The RM says to wait for at least 50ns */
	usleep_range(1, 2);
}

static int lynx_10g_power_off(struct phy *phy)
{
	struct lynx_lane *lane = phy_get_drvdata(phy);

	if (!lane->powered_up)
		return 0;

	/* Issue a reset request with the power down bits set */
	lynx_lane_rmw(lane, LNaGCR0,
		      LNaGCR0_RRST_ON | LNaGCR0_TRST_ON |
		      LNaGCR0_RX_PD | LNaGCR0_TX_PD,
		      LNaGCR0_RRST | LNaGCR0_TRST |
		      LNaGCR0_RX_PD | LNaGCR0_TX_PD);

	/* The RM says to wait for at least 50ns */
	usleep_range(1, 2);

	lane->powered_up = false;

	return 0;
}

static int lynx_10g_power_on(struct phy *phy)
{
	struct lynx_lane *lane = phy_get_drvdata(phy);

	if (lane->powered_up)
		return 0;

	/* The RM says to wait for at least 120ns between per lane setting have
	 * been changed and the lane is taken out of reset
	 */
	usleep_range(1, 2);

	lynx_lane_rmw(lane, LNaGCR0, LNaGCR0_RRST_OFF | LNaGCR0_TRST_OFF,
		      LNaGCR0_RRST | LNaGCR0_TRST |
		      LNaGCR0_RX_PD | LNaGCR0_TX_PD);

	lane->powered_up = true;

	return 0;
}

static void lynx_10g_lane_set_nrate(struct lynx_lane *lane,
				    struct lynx_pll *pll,
				    enum lynx_lane_mode mode)
{
	switch (pll->frate_sel) {
	case PLLnCR0_FRATE_5G:
		switch (mode) {
		case LANE_MODE_1000BASEX_SGMII:
		case LANE_MODE_1000BASEKX:
			lynx_lane_rmw(lane, LNaGCR0,
				      LNaGCR0_TRAT_SEL(RAT_SEL_QUARTER) |
				      LNaGCR0_RRAT_SEL(RAT_SEL_QUARTER),
				      LNaGCR0_RRAT_SEL_MSK |
				      LNaGCR0_TRAT_SEL_MSK);
			break;
		case LANE_MODE_QSGMII:
			lynx_lane_rmw(lane, LNaGCR0,
				      LNaGCR0_TRAT_SEL(RAT_SEL_FULL) |
				      LNaGCR0_RRAT_SEL(RAT_SEL_FULL),
				      LNaGCR0_RRAT_SEL_MSK |
				      LNaGCR0_TRAT_SEL_MSK);
			break;
		default:
			break;
		}
		break;
	case PLLnCR0_FRATE_3_125G:
		switch (mode) {
		case LANE_MODE_2500BASEX:
			lynx_lane_rmw(lane, LNaGCR0,
				      LNaGCR0_TRAT_SEL(RAT_SEL_FULL) |
				      LNaGCR0_RRAT_SEL(RAT_SEL_FULL),
				      LNaGCR0_RRAT_SEL_MSK |
				      LNaGCR0_TRAT_SEL_MSK);
			break;
		default:
			break;
		}
		break;
	case PLLnCR0_FRATE_5_15625G:
		switch (mode) {
		case LANE_MODE_10GBASER:
		case LANE_MODE_10GBASEKR:
		case LANE_MODE_USXGMII:
		case LANE_MODE_10G_QXGMII:
			lynx_lane_rmw(lane, LNaGCR0,
				      LNaGCR0_TRAT_SEL(RAT_SEL_DOUBLE) |
				      LNaGCR0_RRAT_SEL(RAT_SEL_DOUBLE),
				      LNaGCR0_RRAT_SEL_MSK |
				      LNaGCR0_TRAT_SEL_MSK);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void lynx_10g_lane_set_pll(struct lynx_lane *lane,
				  struct lynx_pll *pll)
{
	if (pll->id == 0) {
		lynx_lane_rmw(lane, LNaGCR0,
			      LNaGCR0_RPLL_PLLF | LNaGCR0_TPLL_PLLF,
			      LNaGCR0_RPLL_MSK | LNaGCR0_TPLL_MSK);
	} else {
		lynx_lane_rmw(lane, LNaGCR0,
			      LNaGCR0_RPLL_PLLS | LNaGCR0_TPLL_PLLS,
			      LNaGCR0_RPLL_MSK | LNaGCR0_TPLL_MSK);
	}
}

/* Enabling ex_dly_clk does not require turning the PLL off, and does not
 * affect the state of the lanes mapped to it. It is one of the few safe things
 * that can be done with it at runtime.
 */
static void lynx_10g_pll_ex_dly_clk_enable(struct lynx_pll *pll,
					   bool enable)
{
	u32 val = 0;

	dev_err(pll->priv->dev, "Turning %s EX_DLY_CLK on PLL%c\n",
		enable ? "on" : "off", pll->id == 0 ? 'F' : 'S');

	if (enable)
		val = PLLnCR0_DLYDIV_SEL_312_5_MHZ;

	lynx_pll_rmw(pll, PLLnCR0, val, PLLnCR0_DLYDIV_SEL);
}

static void lynx_10g_pll_get_ex_dly_clk(struct lynx_pll *pll)
{
	spin_lock(&pll->lock);

	if (++pll->ex_dly_clk_use_count > 1) {
		spin_unlock(&pll->lock);
		return;
	}

	lynx_10g_pll_ex_dly_clk_enable(pll, true);

	spin_unlock(&pll->lock);
}

static void lynx_10g_pll_put_ex_dly_clk(struct lynx_pll *pll)
{
	spin_lock(&pll->lock);

	if (--pll->ex_dly_clk_use_count != 0) {
		spin_unlock(&pll->lock);
		return;
	}

	lynx_10g_pll_ex_dly_clk_enable(pll, false);

	spin_unlock(&pll->lock);
}

static void lynx_10g_lane_remap_pll(struct lynx_lane *lane,
				    enum lynx_lane_mode lane_mode)
{
	struct lynx_priv *priv = lane->priv;
	struct lynx_pll *pll;

	/* Switch to the PLL that works with this interface type */
	pll = lynx_pll_get(priv, lane_mode);
	if (unlikely(pll == NULL))
		return;

	lynx_10g_lane_set_pll(lane, pll);

	/* Choose the portion of clock net to be used on this lane */
	lynx_10g_lane_set_nrate(lane, pll, lane_mode);
}

static void lynx_10g_lane_change_proto_conf(struct lynx_lane *lane,
					    enum lynx_lane_mode mode)
{
	const struct lynx_10g_proto_conf *conf = &lynx_10g_proto_conf[mode];

	lynx_lane_rmw(lane, LNaGCR0, LNaGCR0_PROTS(conf->proto_sel) |
		      (conf->if20bit_en ? LNaGCR0_IF20BIT_EN : 0),
		      LNaGCR0_PROTS_MSK | LNaGCR0_IF20BIT_EN);
	lynx_lane_rmw(lane, LNaGCR1,
		      LNaGCR1_REIDL_TH(conf->reidl_th) |
		      (conf->reidl_et_msb ? LNaGCR1_REIDL_ET_MSB : 0) |
		      LNaGCR1_REIDL_ET_SEL(conf->reidl_et_sel) |
		      (conf->reidl_ex_msb ? LNaGCR1_REIDL_EX_MSB : 0) |
		      LNaGCR1_REIDL_EX_SEL(conf->reidl_ex_sel) |
		      LNaGCR1_ISLEW_RCTL(conf->islew_rctl) |
		      LNaGCR1_OSLEW_RCTL(conf->oslew_rctl),
		      LNaGCR1_REIDL_TH_MSK |
		      LNaGCR1_REIDL_ET_MSB | LNaGCR1_REIDL_ET_SEL_MSK |
		      LNaGCR1_REIDL_EX_MSB | LNaGCR1_REIDL_EX_SEL_MSK |
		      LNaGCR1_ISLEW_RCTL_MSK | LNaGCR1_OSLEW_RCTL_MSK);
	lynx_lane_rmw(lane, LNaRECR0,
		      (conf->rxeq_bst ? LNaRECR0_RXEQ_BST : 0) |
		      LNaRECR0_GK2OVD(conf->gk2ovd) |
		      LNaRECR0_GK3OVD(conf->gk3ovd) |
		      (conf->gk2ovd_en ? LNaRECR0_GK2OVD_EN : 0) |
		      (conf->gk3ovd_en ? LNaRECR0_GK3OVD_EN : 0) |
		      LNaRECR0_BASE_WAND(conf->base_wand),
		      LNaRECR0_RXEQ_BST |
		      LNaRECR0_GK2OVD_MSK | LNaRECR0_GK3OVD_MSK |
		      LNaRECR0_GK2OVD_EN | LNaRECR0_GK3OVD_EN |
		      LNaRECR0_BASE_WAND_MSK);
	lynx_lane_rmw(lane, LNaTECR0,
		      LNaTECR0_TEQ_TYPE(conf->teq_type) |
		      (conf->sgn_preq ? LNaTECR0_SGN_PREQ : 0) |
		      LNaTECR0_RATIO_PREQ(conf->ratio_preq) |
		      (conf->sgn_post1q ? LNaTECR0_SGN_POST1Q : 0) |
		      LNaTECR0_RATIO_PST1Q(conf->ratio_post1q) |
		      LNaTECR0_ADPT_EQ(conf->adpt_eq) |
		      LNaTECR0_AMP_RED(conf->amp_red),
		      LNaTECR0_TEQ_TYPE_MSK | LNaTECR0_SGN_PREQ |
		      LNaTECR0_RATIO_PREQ_MSK | LNaTECR0_SGN_POST1Q |
		      LNaTECR0_RATIO_PST1Q_MSK | LNaTECR0_ADPT_EQ_MSK |
		      LNaTECR0_AMP_RED_MSK);
	lynx_lane_write(lane, LNaTTLCR0, conf->ttlcr0);
}

static int lynx_10g_lane_disable_pcvt(struct lynx_lane *lane,
				      enum lynx_lane_mode mode)
{
	struct lynx_priv *priv = lane->priv;
	int err;

	spin_lock(&priv->pcc_lock);

	err = lynx_pccr_write(lane, mode, 0);
	if (err)
		goto out;

	switch (mode) {
	case LANE_MODE_1000BASEX_SGMII:
	case LANE_MODE_1000BASEKX:
	case LANE_MODE_2500BASEX:
		err = lynx_pcvt_rmw(lane, mode, CR(1), SGMIIaCR1_SGPCS_DIS,
				    SGMIIaCR1_SGPCS_EN);
		if (err)
			goto out;

		lynx_pcvt_rmw(lane, mode, CR(0),
			      SGMIIaCR0_RST_SGM_ON | SGMIIaCR0_PD_SGM,
			      SGMIIaCR0_RST_SGM | SGMIIaCR0_PD_SGM);
		break;
	case LANE_MODE_QSGMII:
		err = lynx_pcvt_rmw(lane, mode, CR(0),
				    QSGMIIaCR0_RST_QSGM_ON | QSGMIIaCR0_PD_QSGM,
				    QSGMIIaCR0_RST_QSGM | QSGMIIaCR0_PD_QSGM);
		if (err)
			goto out;
		break;
	default:
		err = 0;
	}

out:
	spin_unlock(&priv->pcc_lock);

	return err;
}

static int lynx_10g_lane_enable_pcvt(struct lynx_lane *lane,
				     enum lynx_lane_mode mode)
{
	struct lynx_priv *priv = lane->priv;
	u32 val;
	int err;

	spin_lock(&priv->pcc_lock);

	switch (mode) {
	case LANE_MODE_1000BASEX_SGMII:
	case LANE_MODE_1000BASEKX:
	case LANE_MODE_2500BASEX:
		err = lynx_pcvt_rmw(lane, mode, CR(1), SGMIIaCR1_SGPCS_EN,
				    SGMIIaCR1_SGPCS_EN);
		if (err)
			goto out;

		lynx_pcvt_rmw(lane, mode, CR(0), SGMIIaCR0_RST_SGM_OFF,
			      SGMIIaCR0_RST_SGM | SGMIIaCR0_PD_SGM);
		break;
	case LANE_MODE_QSGMII:
		err = lynx_pcvt_rmw(lane, mode, CR(0), QSGMIIaCR0_RST_QSGM_OFF,
				    QSGMIIaCR0_RST_QSGM | QSGMIIaCR0_PD_QSGM);
		if (err)
			goto out;
		break;
	default:
		err = 0;
	}

	if (lane->default_pccr[mode]) {
		err = lynx_pccr_write(lane, mode, lane->default_pccr[mode]);
		goto out;
	}

	val = 0;

	switch (mode) {
	case LANE_MODE_1000BASEKX:
		val |= PCCR8_SGMIIa_KX;
		fallthrough;
	case LANE_MODE_1000BASEX_SGMII:
	case LANE_MODE_2500BASEX:
		val |= PCCR8_SGMIIa_CFG;
		break;
	case LANE_MODE_QSGMII:
		val |= PCCR9_QSGMIIa_CFG;
		break;
	case LANE_MODE_10G_QXGMII:
		val |= PCCR9_QXGMIIa_CFG;
		break;
	case LANE_MODE_10GBASER:
	case LANE_MODE_10GBASEKR:
		val |= PCCRB_XFIa_CFG;
		break;
	case LANE_MODE_USXGMII:
		val |= PCCRB_SXGMIIa_CFG;
		break;
	default:
		err = 0;
		goto out;
	}

	err = lynx_pccr_write(lane, mode, val);
out:
	spin_unlock(&priv->pcc_lock);

	return err;
}

static bool lynx_10g_switch_needs_rcw_override(enum lynx_lane_mode crr,
					       enum lynx_lane_mode new)
{
	if ((crr == LANE_MODE_1000BASEKX ||
	     crr == LANE_MODE_1000BASEX_SGMII ||
	     crr == LANE_MODE_2500BASEX) &&
	    (new == LANE_MODE_1000BASEKX ||
	     new == LANE_MODE_1000BASEX_SGMII ||
	     new == LANE_MODE_2500BASEX))
		return false;

	return true;
}

static void lynx_10g_tune_tx_eq(struct phy *phy,
				const struct lynx_xgkr_tx_eq *tx_eq)
{
	struct lynx_lane *lane = phy_get_drvdata(phy);

	lynx_lane_rmw(lane, LNaTECR0,
		      LNaTECR0_RATIO_PREQ(tx_eq->ratio_preq) |
		      LNaTECR0_RATIO_PST1Q(tx_eq->ratio_post1q) |
		      LNaTECR0_ADPT_EQ(tx_eq->adapt_eq) |
		      LNaTECR0_AMP_RED(tx_eq->amp_reduction),
		      LNaTECR0_RATIO_PREQ_MSK |
		      LNaTECR0_RATIO_PST1Q_MSK |
		      LNaTECR0_ADPT_EQ_MSK |
		      LNaTECR0_AMP_RED_MSK);
}

static void lynx_10g_read_tx_eq(struct phy *phy, struct lynx_xgkr_tx_eq *tx_eq)
{
	struct lynx_lane *lane = phy_get_drvdata(phy);
	int val = lynx_lane_read(lane, LNaTECR0);

	tx_eq->ratio_preq = LNaTECR0_RATIO_PREQ_X(val);
	tx_eq->ratio_post1q = LNaTECR0_RATIO_PST1Q_X(val);
	tx_eq->amp_reduction = LNaTECR0_AMP_RED(val);
	tx_eq->adapt_eq = LNaTECR0_ADPT_EQ_X(val);
}

static int lynx_10g_snapshot_rx_eq_gains(struct phy *phy, u8 *gaink2,
					 u8 *gaink3, u8 *eq_offset)
{
	struct lynx_lane *lane = phy_get_drvdata(phy);
	bool cdr_locked;
	int err, val;

	err = read_poll_timeout(lynx_10g_cdr_lock_check, cdr_locked,
				cdr_locked, CDR_SLEEP_US, CDR_TIMEOUT_US,
				false, lane);
	if (err) {
		dev_err(&phy->dev, "CDR not locked, cannot collect RX EQ snapshots\n");
		return err;
	}

	/* wait until a previous snapshot has cleared */
	err = read_poll_timeout(lynx_lane_read, val,
				!(val & LNaRECR1_CTL_SNP_DONE),
				SNAPSHOT_SLEEP_US, SNAPSHOT_TIMEOUT_US,
				false, lane, LNaRECR1);
	if (err)
		return err;

	/* start snapshot of RX Equalization Control Gaink2, Gaink3
	 * and Offset Registers
	 */
	lynx_lane_rmw(lane, LNaGCR1, LNaGCR1_REQ_CTL_SNP, LNaGCR1_REQ_CTL_SNP);

	/* wait for the snapshot to finish */
	err = read_poll_timeout(lynx_lane_read, val,
				!!(val & LNaRECR1_CTL_SNP_DONE),
				SNAPSHOT_SLEEP_US, SNAPSHOT_TIMEOUT_US,
				false, lane, LNaRECR1);
	if (err) {
		dev_err(&phy->dev,
			"Failed to snapshot RX EQ: undetected loss of CDR lock?\n");
		lynx_lane_rmw(lane, LNaGCR1, 0, LNaGCR1_REQ_CTL_SNP);
		return err;
	}

	val = lynx_lane_read(lane, LNaRECR1);
	*gaink2 = LNaRECR1_GK2STAT_X(val);
	*gaink3 = LNaRECR1_GK3STAT_X(val);
	/* The algorithm should only be looking at offset_stat[5:0].
	 * Bit 6 is only used at higher bit rates to adjust the overall range
	 * of the internal offset DAC
	 */
	*eq_offset = LNaRECR1_OSETSTAT_X(val) & GENMASK(5, 0);

	/* terminate the snapshot */
	lynx_lane_rmw(lane, LNaGCR1, 0, LNaGCR1_REQ_CTL_SNP);

	return 0;
}

static int lynx_10g_snapshot_rx_eq_bin(struct phy *phy, enum lynx_bin_type bin_type,
				       s16 *bin)
{
	int bin_sel = lynx_10g_bin_type_to_bin_sel[bin_type];
	struct lynx_lane *lane = phy_get_drvdata(phy);
	bool cdr_locked;
	int err, val;

	if (WARN_ON(bin_sel < 0))
		return bin_sel;

	err = read_poll_timeout(lynx_10g_cdr_lock_check, cdr_locked,
				cdr_locked, CDR_SLEEP_US, CDR_TIMEOUT_US,
				false, lane);
	if (err) {
		dev_err(&phy->dev, "CDR not locked, cannot collect RX EQ snapshots\n");
		return err;
	}

	/* wait until a previous snapshot has cleared */
	err = read_poll_timeout(lynx_lane_read, val,
				!(val & LNaRECR1_BIN_SNP_DONE),
				SNAPSHOT_SLEEP_US, SNAPSHOT_TIMEOUT_US,
				false, lane, LNaRECR1);
	if (err)
		return err;

	/* select the binning register we would like to snapshot */
	lynx_lane_rmw(lane, LNaTCSR1, LNaTCSR1_CDR_SEL(bin_sel),
		      LNaTCSR1_CDR_SEL_MSK);

	/* start snapshot */
	lynx_lane_rmw(lane, LNaGCR1, LNaGCR1_REQ_BIN_SNP,
		      LNaGCR1_REQ_BIN_SNP);

	/* wait for the snapshot to finish */
	err = read_poll_timeout(lynx_lane_read, val,
				!!(val & LNaRECR1_BIN_SNP_DONE),
				SNAPSHOT_SLEEP_US, SNAPSHOT_TIMEOUT_US,
				false, lane, LNaRECR1);
	if (err) {
		dev_err(&phy->dev,
			"Failed to snapshot RX EQ: undetected loss of CDR lock?\n");
		lynx_lane_rmw(lane, LNaGCR1, 0, LNaGCR1_REQ_BIN_SNP);
		return err;
	}

	val = lynx_lane_read(lane, LNaTCSR1);
	val = LNaTCSR1_EQ_SNPBIN_DATA_X(val);

	/* The snapshot is a 2's complement value stored on 9 bits
	 * (-256 to 255)
	 */
	if (val & LNaTCSR1_EQ_SNPBIN_DATA_SGN) {
		val &= ~LNaTCSR1_EQ_SNPBIN_DATA_SGN;
		val -= 256;
	}

	*bin = (s16)val;

	/* terminate the snapshot */
	lynx_lane_rmw(lane, LNaGCR1, 0, LNaGCR1_REQ_BIN_SNP);

	return 0;
}

static const struct lynx_xgkr_algorithm_ops lynx_10g_xgkr_ops = {
	.tune_tx_eq = lynx_10g_tune_tx_eq,
	.read_tx_eq = lynx_10g_read_tx_eq,
	.snapshot_rx_eq_gains = lynx_10g_snapshot_rx_eq_gains,
	.snapshot_rx_eq_bin = lynx_10g_snapshot_rx_eq_bin,
};

static int lynx_10g_set_mode(struct phy *phy, enum phy_mode mode, int submode)
{
	struct lynx_lane *lane = phy_get_drvdata(phy);
	struct lynx_xgkr_algorithm *algorithm = NULL;
	struct lynx_priv *priv = lane->priv;
	bool powered_up = lane->powered_up;
	enum lynx_lane_mode lane_mode;
	int err;

	if (mode != PHY_MODE_ETHERNET)
		return -EOPNOTSUPP;

	if (lane->mode == LANE_MODE_UNKNOWN)
		return -EOPNOTSUPP;

	lane_mode = phy_interface_to_lane_mode(submode);
	if (!lynx_lane_supports_mode(lane, lane_mode))
		return -EOPNOTSUPP;

	if (lane_mode == lane->mode)
		return 0;

	if (lynx_lane_mode_needs_link_training(lane_mode)) {
		algorithm = lynx_xgkr_algorithm_create(phy, &lynx_10g_xgkr_ops);
		if (!algorithm)
			return -ENOMEM;
	}

	/* If the lane is powered up, put the lane into the halt state while
	 * the reconfiguration is being done.
	 */
	if (powered_up)
		lynx_10g_lane_halt(phy);

	if (lynx_10g_switch_needs_rcw_override(lane->mode, lane_mode)) {
		err = fsl_guts_lane_set_mode(priv->info->index, lane->id, lane_mode);
		if (err)
			goto out;
	}

	err = lynx_10g_lane_disable_pcvt(lane, lane->mode);
	if (err)
		goto out;

	lynx_10g_lane_change_proto_conf(lane, lane_mode);
	lynx_10g_lane_remap_pll(lane, lane_mode);
	WARN_ON(lynx_10g_lane_enable_pcvt(lane, lane_mode));

	/* 1000Base-KX lanes need their PLL to generate a 312.5 MHz frequency
	 * through EX_DLY_CLK.
	 */
	if (lane_mode == LANE_MODE_1000BASEKX)
		lynx_10g_pll_get_ex_dly_clk(lynx_pll_get(priv, lane_mode));
	else if (lane->mode == LANE_MODE_1000BASEKX)
		lynx_10g_pll_put_ex_dly_clk(lynx_pll_get(priv, lane->mode));

	if (lane->algorithm)
		lynx_xgkr_algorithm_destroy(lane->algorithm);

	lane->algorithm = algorithm;
	lane->mode = lane_mode;

out:
	if (powered_up)
		lynx_10g_lane_reset(phy);

	return err;
}

static int lynx_10g_validate(struct phy *phy, enum phy_mode mode, int submode,
			     union phy_configure_opts *opts __always_unused)
{
	struct lynx_lane *lane = phy_get_drvdata(phy);
	struct lynx_priv *priv = lane->priv;
	enum lynx_lane_mode lane_mode;

	if (mode != PHY_MODE_ETHERNET)
		return -EOPNOTSUPP;

	lane_mode = phy_interface_to_lane_mode(submode);
	if (!lynx_lane_supports_mode(lane, lane_mode))
		return -EOPNOTSUPP;

	if (lynx_10g_switch_needs_rcw_override(lane->mode, lane_mode))
		return fsl_guts_lane_validate(priv->info->index, lane->id, lane_mode);

	return 0;
}

static int lynx_10g_init(struct phy *phy)
{
	struct lynx_lane *lane = phy_get_drvdata(phy);

	/* Mark the fact that the lane was init */
	lane->init = true;

	/* SerDes lanes are powered on at boot time. Any lane that is
	 * managed by this driver will get powered off when its consumer
	 * calls phy_init().
	 */
	lane->powered_up = true;
	lynx_10g_power_off(phy);

	return 0;
}

static int lynx_10g_exit(struct phy *phy)
{
	struct lynx_lane *lane = phy_get_drvdata(phy);

	/* The lane returns to the state where it isn't managed by the
	 * consumer, so we must treat is as if it isn't initialized, and always
	 * powered on.
	 */
	lane->init = false;
	lane->powered_up = false;
	lynx_10g_power_on(phy);

	return 0;
}

static void lynx_10g_check_cdr_lock(struct phy *phy,
				    struct phy_status_opts_cdr *cdr)
{
	struct lynx_lane *lane = phy_get_drvdata(phy);

	cdr->cdr_locked = lynx_10g_cdr_lock_check(lane);
}

static void lynx_10g_get_pcvt_count(struct phy *phy,
				    struct phy_status_opts_pcvt_count *opts)
{
	struct lynx_lane *lane = phy_get_drvdata(phy);
	enum lynx_lane_mode lane_mode = lane->mode;

	switch (opts->type) {
	case PHY_PCVT_ETHERNET_PCS:
		switch (lane_mode) {
		case LANE_MODE_1000BASEX_SGMII:
		case LANE_MODE_1000BASEKX:
		case LANE_MODE_2500BASEX:
		case LANE_MODE_USXGMII:
		case LANE_MODE_10GBASER:
		case LANE_MODE_10GBASEKR:
			opts->num_pcvt = 1;
			break;
		case LANE_MODE_QSGMII:
		case LANE_MODE_10G_QXGMII:
			opts->num_pcvt = 4;
			break;
		default:
			break;
		}
		break;
	case PHY_PCVT_ETHERNET_ANLT:
		switch (lane_mode) {
		case LANE_MODE_1000BASEKX:
		case LANE_MODE_10GBASEKR:
			opts->num_pcvt = 1;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void lynx_10g_get_pcvt_addr(struct phy *phy,
				   struct phy_status_opts_pcvt *pcvt)
{
	struct lynx_lane *lane = phy_get_drvdata(phy);
	u32 cr1;

	switch (pcvt->type) {
	case PHY_PCVT_ETHERNET_PCS:
	case PHY_PCVT_ETHERNET_ANLT:
		WARN_ON(lynx_pcvt_read(lane, lane->mode, CR(1), &cr1));
		pcvt->addr.mdio = MDEV_PORT_X(cr1) + pcvt->index;
		break;
	default:
		break;
	}
}

static int lynx_10g_get_status(struct phy *phy, enum phy_status_type type,
			       union phy_status_opts *opts)
{
	switch (type) {
	case PHY_STATUS_CDR_LOCK:
		lynx_10g_check_cdr_lock(phy, &opts->cdr);
		break;
	case PHY_STATUS_PCVT_COUNT:
		lynx_10g_get_pcvt_count(phy, &opts->pcvt_count);
		break;
	case PHY_STATUS_PCVT_ADDR:
		lynx_10g_get_pcvt_addr(phy, &opts->pcvt);
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static int lynx_10g_configure(struct phy *phy, union phy_configure_opts *opts)
{
	struct lynx_lane *lane = phy_get_drvdata(phy);

	return lynx_xgkr_algorithm_configure(lane->algorithm, &opts->ethernet);
}

static const struct phy_ops lynx_10g_ops = {
	.init		= lynx_10g_init,
	.exit		= lynx_10g_exit,
	.power_on	= lynx_10g_power_on,
	.power_off	= lynx_10g_power_off,
	.set_mode	= lynx_10g_set_mode,
	.validate	= lynx_10g_validate,
	.get_status	= lynx_10g_get_status,
	.configure	= lynx_10g_configure,
	.owner		= THIS_MODULE,
};

static const char *lynx_10g_refclk_str(int refclk)
{
	switch (refclk) {
	case PLLnCR0_REFCLK_SEL_100MHZ:
		return "100MHz";
	case PLLnCR0_REFCLK_SEL_125MHZ:
		return "125MHz";
	case PLLnCR0_REFCLK_SEL_156MHZ:
		return "156.25MHz";
	case PLLnCR0_REFCLK_SEL_150MHZ:
		return "150MHz";
	case PLLnCR0_REFCLK_SEL_161MHZ:
		return "161.1328125MHz";
	default:
		return "unknown";
	}
}

static const char *lynx_10g_clock_net_str(int frate)
{
	switch (frate) {
	case PLLnCR0_FRATE_5G:
		return "5GHz";
	case PLLnCR0_FRATE_3_125G:
		return "3.125GHz";
	case PLLnCR0_FRATE_5_15625G:
		return "5.15625GHz";
	default:
		return "unknown";
	}
}

#define LYNX_SUPPORT_BUF_LEN		128

static void lynx_10g_pll_dump(struct lynx_pll *pll)
{
	struct lynx_priv *priv = pll->priv;
	char buf[LYNX_SUPPORT_BUF_LEN];
	struct device *dev = priv->dev;
	enum lynx_lane_mode mode;
	int total_len = 0, len;
	bool truncated = false;
	int i;

	dev_info(dev, "PLL%c: %s, %s, reference clock %s, clock net %s\n",
		 pll->id == 0 ? 'F' : 'S',
		 str_enabled_disabled(pll->enabled),
		 pll->locked ? "locked" : "unlocked",
		 lynx_10g_refclk_str(pll->refclk_sel),
		 lynx_10g_clock_net_str(pll->frate_sel));

	if (!pll->enabled)
		return;

	for (mode = LANE_MODE_UNKNOWN; mode < LANE_MODE_MAX; mode++) {
		if (!test_bit(mode, pll->supported))
			continue;

		for (i = priv->info->first_lane; i < priv->info->num_lanes; i++) {
			if (!priv->info->lane_supports_mode(i, mode))
				continue;

			len = snprintf(&buf[total_len],
				       LYNX_SUPPORT_BUF_LEN - total_len,
				       " %s", lynx_lane_mode_str(mode));
			if (len >= LYNX_SUPPORT_BUF_LEN - total_len)
				truncated = true;
			total_len += len;

			break;
		}

		if (truncated)
			break;
	}

	dev_info(dev, "\tSupported lane modes:%s%s\n", buf,
		 truncated ? " (truncated)" : "");
}

static void lynx_10g_pll_read_configuration(struct lynx_priv *priv)
{
	struct lynx_pll *pll;
	int dlydiv_sel;
	u32 val;
	int i;

	for (i = 0; i < LYNX_NUM_PLL; i++) {
		pll = &priv->pll[i];
		pll->priv = priv;
		pll->id = i;
		spin_lock_init(&pll->lock);

		val = lynx_pll_read(pll, PLLnCR0);
		pll->frate_sel = FIELD_GET(PLLnCR0_FRATE_SEL, val);
		pll->refclk_sel = FIELD_GET(PLLnCR0_REFCLK_SEL, val);
		pll->enabled = !(val & PLLnCR0_POFF);
		pll->locked = !!(val & PLLnCR0_PLL_LCK);

		if (!pll->enabled)
			continue;

		dlydiv_sel = FIELD_GET(PLLnCR0_DLYDIV_SEL, val);
		if (dlydiv_sel) {
			dev_dbg(priv->dev, "PLL%cCR0[DLYDIV_SEL] found set\n",
				pll->id == 0 ? 'F' : 'S');
			pll->ex_dly_clk_use_count = 1;
		}

		switch (pll->frate_sel) {
		case PLLnCR0_FRATE_5G:
			/* 5GHz clock net */
			__set_bit(LANE_MODE_1000BASEX_SGMII, pll->supported);
			__set_bit(LANE_MODE_1000BASEKX, pll->supported);
			__set_bit(LANE_MODE_QSGMII, pll->supported);
			if (dlydiv_sel && dlydiv_sel != PLLnCR0_DLYDIV_SEL_312_5_MHZ) {
				dev_dbg(priv->dev,
					"PLL%c has ex_dly_clk provisioned for a frequency incompatible with 1000Base-KX\n",
					pll->id == 0 ? 'F' : 'S');
			} else {
				__set_bit(LANE_MODE_1000BASEKX, pll->supported);
			}
			break;
		case PLLnCR0_FRATE_3_125G:
			__set_bit(LANE_MODE_2500BASEX, pll->supported);
			break;
		case PLLnCR0_FRATE_5_15625G:
			/* 10.3125GHz clock net */
			__set_bit(LANE_MODE_10GBASER, pll->supported);
			__set_bit(LANE_MODE_10GBASEKR, pll->supported);
			__set_bit(LANE_MODE_USXGMII, pll->supported);
			__set_bit(LANE_MODE_10G_QXGMII, pll->supported);
			break;
		default:
			break;
		}
	}

	for (i = 0; i < LYNX_NUM_PLL; i++)
		lynx_10g_pll_dump(&priv->pll[i]);
}

/* On LS1028A, SGMIIA_CFG, SGMIIB_CFG, and SGMIIC_CFG from PCCR8 have the
 * ability to map either an ENETC PCS or a Felix switch PCS to the same lane.
 * The PHY API lacks the capability to distinguish between one consumer and
 * another, so we don't support changing the initial muxing done by the RCW.
 * However, when disabling a PCS through PCCR8, we need to properly restore
 * the original value to keep the same muxing, and for that we need to back
 * it up (here).
 */
static void lynx_10g_backup_pccr_val(struct lynx_lane *lane)
{
	u32 val;
	int err;

	if (lane->mode == LANE_MODE_UNKNOWN)
		return;

	err = lynx_pccr_read(lane, lane->mode, &val);
	if (err) {
		dev_warn(&lane->phy->dev,
			 "The driver doesn't know how to access the PCCR for lane mode %d\n",
			 lane->mode);
		lane->mode = LANE_MODE_UNKNOWN;
		return;
	}

	lane->default_pccr[lane->mode] = val;

	switch (lane->mode) {
	case LANE_MODE_1000BASEKX:
	case LANE_MODE_1000BASEX_SGMII:
	case LANE_MODE_2500BASEX:
		lane->default_pccr[LANE_MODE_1000BASEKX] = val | PCCR8_SGMIIa_KX;
		lane->default_pccr[LANE_MODE_1000BASEX_SGMII] = val & ~PCCR8_SGMIIa_KX;
		lane->default_pccr[LANE_MODE_2500BASEX] = val & ~PCCR8_SGMIIa_KX;
		break;
	default:
		break;
	}
}

static bool lynx_10g_lane_is_3_125g(struct lynx_lane *lane)
{
	struct lynx_priv *priv = lane->priv;
	struct lynx_pll *pll;
	u32 gcr0;

	gcr0 = lynx_lane_read(lane, LNaGCR0);

	if (gcr0 & LNaGCR0_TPLL_PLLF)
		pll = &priv->pll[0];
	else
		pll = &priv->pll[1];

	if (pll->frate_sel != PLLnCR0_FRATE_3_125G)
		return false;

	if (LNaGCR0_TRAT_SEL_X(gcr0) != RAT_SEL_FULL ||
	    LNaGCR0_RRAT_SEL_X(gcr0) != RAT_SEL_FULL)
		return false;

	return true;
}

static void lynx_10g_lane_read_configuration(struct lynx_lane *lane)
{
	u32 pssr0 = lynx_lane_read(lane, LNaPSSR0);
	struct lynx_priv *priv = lane->priv;
	int proto = LNaPSSR0_TYPE_X(pssr0);

	switch (proto) {
	case PROTO_SEL_SGMII_BASEX_KX_QSGMII:
		if (lynx_10g_lane_is_3_125g(lane))
			lane->mode = LANE_MODE_2500BASEX;
		else if (LNaPSSR0_IS_QUAD_X(pssr0))
			lane->mode = LANE_MODE_QSGMII;
		else
			lane->mode = LANE_MODE_1000BASEX_SGMII;
		break;
	case PROTO_SEL_XFI_10GBASER_KR_SXGMII:
		if (LNaPSSR0_IS_QUAD_X(pssr0))
			lane->mode = LANE_MODE_10G_QXGMII;
		else if (priv->info->has_hardcoded_usxgmii)
			lane->mode = LANE_MODE_USXGMII;
		else
			lane->mode = LANE_MODE_10GBASER;
		break;
	case PROTO_SEL_PCIE:
	case PROTO_SEL_SATA:
	case PROTO_SEL_XAUI:
		break;
	default:
		dev_warn(&lane->phy->dev, "Unknown lane protocol 0x%x\n", proto);
	}

	lynx_10g_backup_pccr_val(lane);
}

static struct phy *lynx_10g_xlate(struct device *dev,
				  const struct of_phandle_args *args)
{
	struct lynx_priv *priv = dev_get_drvdata(dev);
	int idx = args->args[0];

	if (idx >= priv->info->num_lanes)
		return ERR_PTR(-EINVAL);

	return priv->lane[idx].phy;
}

static int lynx_10g_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct phy_provider *provider;
	struct lynx_priv *priv;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	priv->dev = &pdev->dev;
	priv->info = of_device_get_match_data(dev);
	priv->big_endian = of_property_read_bool(dev->of_node, "big-endian");

	priv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	dev_set_drvdata(dev, priv);
	spin_lock_init(&priv->pcc_lock);
	INIT_DELAYED_WORK(&priv->cdr_check, lynx_10g_cdr_lock_check_work);

	priv->lane = devm_kcalloc(dev, priv->info->num_lanes,
				  sizeof(*priv->lane), GFP_KERNEL);
	if (!priv->lane)
		return -ENOMEM;

	lynx_10g_pll_read_configuration(priv);

	for (int reg = 0; reg < priv->info->num_lanes; reg++) {
		struct lynx_lane *lane;
		struct phy *phy;

		lane = &priv->lane[reg];
		phy = devm_phy_create(&pdev->dev, NULL, &lynx_10g_ops);
		if (IS_ERR(phy))
			return PTR_ERR(phy);

		lane->priv = priv;
		lane->phy = phy;
		lane->id = reg;
		phy_set_drvdata(phy, lane);
		lynx_10g_lane_read_configuration(lane);
		fsl_guts_lane_init(priv->info->index, lane->id, lane->mode);
	}

	provider = devm_of_phy_provider_register(&pdev->dev, lynx_10g_xlate);
	if (IS_ERR(provider))
		return PTR_ERR(provider);

	queue_delayed_work(system_power_efficient_wq, &priv->cdr_check,
			   msecs_to_jiffies(1000));

	return 0;
}

static void lynx_10g_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct lynx_priv *priv = dev_get_drvdata(dev);

	cancel_delayed_work_sync(&priv->cdr_check);
}

static const struct of_device_id lynx_10g_of_match_table[] = {
	{ .compatible = "fsl,ls1028a-serdes", .data = &lynx_info_ls1028a },
	{ .compatible = "fsl,ls1046a-serdes1", .data = &lynx_info_ls1046a_serdes1 },
	{ .compatible = "fsl,ls1046a-serdes2", .data = &lynx_info_ls1046a_serdes2 },
	{ .compatible = "fsl,ls1088a-serdes1", .data = &lynx_info_ls1088a_serdes1 },
	{ .compatible = "fsl,ls2088a-serdes1", .data = &lynx_info_ls2088a_serdes1 },
	{ .compatible = "fsl,ls2088a-serdes2", .data = &lynx_info_ls2088a_serdes2 },
	{ },
};
MODULE_DEVICE_TABLE(of, lynx_10g_of_match_table);

static struct platform_driver lynx_10g_driver = {
	.probe	= lynx_10g_probe,
	.remove	= lynx_10g_remove,
	.driver	= {
		.name = "lynx-10g",
		.of_match_table = lynx_10g_of_match_table,
	},
};
module_platform_driver(lynx_10g_driver);

MODULE_AUTHOR("Ioana Ciornei <ioana.ciornei@nxp.com>");
MODULE_AUTHOR("Vladimir Oltean <vladimir.oltean@nxp.com>");
MODULE_DESCRIPTION("Lynx 10G SerDes PHY driver for Layerscape SoCs");
MODULE_LICENSE("GPL v2");
