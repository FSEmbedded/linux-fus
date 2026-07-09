// SPDX-License-Identifier: GPL-2.0+
/* Copyright (c) 2021-2022 NXP. */

#include <linux/bitfield.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>

#include "phy-fsl-lynx-core.h"

#define LYNX_28G_NUM_LANE			8
#define LYNX_28G_NUM_PLL			LYNX_NUM_PLL

/* SoC IP wrapper for protocol converters */
#define PCC8					0x10a0
#define PCC8_SGMIIa_KX				BIT(3)
#define PCC8_SGMIIa_CFG				BIT(0)

#define PCCC					0x10b0
#define PCCC_SXGMIIn_XFI			BIT(3)
#define PCCC_SXGMIIn_CFG			BIT(0)

#define PCCD					0x10b4
#define PCCD_E25Gn_CFG				BIT(0)

#define PCCE					0x10b8
#define PCCE_E40Gn_LRV				BIT(3)
#define PCCE_E40Gn_CFG				BIT(0)
#define PCCE_E50Gn_LRV				BIT(3)
#define PCCE_E50GnCFG				BIT(0)
#define PCCE_E100Gn_LRV				BIT(3)
#define PCCE_E100Gn_CFG				BIT(0)

#define SGMII_CFG(id)				(28 - (id) * 4) /* Offset into PCC8 */
#define SXGMII_CFG(id)				(28 - (id) * 4) /* Offset into PCCC */
#define E25G_CFG(id)				(28 - (id) * 4) /* Offset into PCCD */
#define E40G_CFG(id)				(28 - (id) * 4) /* Offset into PCCE */
#define E50G_CFG(id)				(20 - (id) * 4) /* Offset into PCCE */
#define E100G_CFG(id)				(12 - (id) * 4) /* Offset into PCCE */

/* Per PLL registers */
#define PLLnRSTCTL(pll)				(0x400 + (pll) * 0x100 + 0x0)
#define PLLnRSTCTL_DIS				BIT(24)
#define PLLnRSTCTL_LOCK				BIT(23)

#define PLLnCR0(pll)				(0x400 + (pll) * 0x100 + 0x4)
#define PLLnCR0_REFCLK_SEL			GENMASK(20, 16)
#define PLLnCR0_REFCLK_SEL_100MHZ		0x0
#define PLLnCR0_REFCLK_SEL_125MHZ		0x1
#define PLLnCR0_REFCLK_SEL_156MHZ		0x2
#define PLLnCR0_REFCLK_SEL_150MHZ		0x3
#define PLLnCR0_REFCLK_SEL_161MHZ		0x4

#define PLLnCR1(pll)				(0x400 + (pll) * 0x100 + 0x8)
#define PLLnCR1_FRATE_SEL			GENMASK(28, 24)
#define PLLnCR1_FRATE_5G_10GVCO			0x0
#define PLLnCR1_FRATE_5G_25GVCO			0x10
#define PLLnCR1_FRATE_10G_20GVCO		0x6
#define PLLnCR1_FRATE_12G_25GVCO		0x16
#define PLLnCR1_EX_DLY_SEL			GENMASK(1, 0)
#define PLLnCR1_EX_DLY_SEL_312_5_MHZ		2

/* Per SerDes lane registers */
/* Lane a General Control Register */
#define LNaGCR0(lane)				(0x800 + (lane) * 0x100 + 0x0)
#define LNaGCR0_PROTO_SEL			GENMASK(7, 3)
#define LNaGCR0_PROTO_SEL_SGMII			0x1
#define LNaGCR0_PROTO_SEL_XFI			0xa
#define LNaGCR0_PROTO_SEL_25G			0x1a
#define LNaGCR0_IF_WIDTH			GENMASK(2, 0)
#define LNaGCR0_IF_WIDTH_10_BIT			0x0
#define LNaGCR0_IF_WIDTH_20_BIT			0x2
#define LNaGCR0_IF_WIDTH_40_BIT			0x4

/* Lane a Tx Reset Control Register */
#define LNaTRSTCTL(lane)			(0x800 + (lane) * 0x100 + 0x20)
#define LNaTRSTCTL_RST_REQ			BIT(31)
#define LNaTRSTCTL_RST_DONE			BIT(30)
#define LNaTRSTCTL_HLT_REQ			BIT(27)
#define LNaTRSTCTL_STP_REQ			BIT(26)
#define LNaTRSTCTL_DIS				BIT(24)

/* Lane a Tx General Control Register */
#define LNaTGCR0(lane)				(0x800 + (lane) * 0x100 + 0x24)
#define LNaTGCR0_USE_PLL			BIT(28)
#define LNaTGCR0_USE_PLLF			0x0
#define LNaTGCR0_USE_PLLS			0x1
#define LNaTGCR0_N_RATE				GENMASK(26, 24)
#define LNaTGCR0_N_RATE_FULL			0x0
#define LNaTGCR0_N_RATE_HALF			0x1
#define LNaTGCR0_N_RATE_QUARTER			0x2
#define LNaTGCR0_N_RATE_DOUBLE			0x3

#define LNaTGCR1(lane)				(0x800 + (lane) * 0x100 + 0x28)

#define LNaTECR0(lane)				(0x800 + (lane) * 0x100 + 0x30)
#define LNaTECR0_EQ_TYPE			GENMASK(30, 28)
#define LNaTECR0_EQ_SGN_PREQ			BIT(23)
#define LNaTECR0_EQ_PREQ			GENMASK(19, 16)
#define LNaTECR0_EQ_SGN_POST1Q			BIT(15)
#define LNaTECR0_EQ_POST1Q			GENMASK(12, 8)
#define LNaTECR0_EQ_AMP_RED			GENMASK(5, 0)

#define LNaTECR1(lane)				(0x800 + (lane) * 0x100 + 0x34)
#define LNaTECR1_EQ_ADPT_EQ_DRVR_DIS		BIT(31)
#define LNaTECR1_EQ_ADPT_EQ			GENMASK(29, 24)

/* Lane a Rx Reset Control Register */
#define LNaRRSTCTL(lane)			(0x800 + (lane) * 0x100 + 0x40)
#define LNaRRSTCTL_RST_REQ			BIT(31)
#define LNaRRSTCTL_RST_DONE			BIT(30)
#define LNaRRSTCTL_HLT_REQ			BIT(27)
#define LNaRRSTCTL_STP_REQ			BIT(26)
#define LNaRRSTCTL_DIS				BIT(24)
#define LNaRRSTCTL_CDR_LOCK			BIT(12)

/* Lane a Rx General Control Register */
#define LNaRGCR0(lane)				(0x800 + (lane) * 0x100 + 0x44)
#define LNaRGCR0_USE_PLL			BIT(28)
#define LNaRGCR0_USE_PLLF			0x0
#define LNaRGCR0_USE_PLLS			0x1
#define LNaRGCR0_N_RATE				GENMASK(26, 24)
#define LNaRGCR0_N_RATE_FULL			0x0
#define LNaRGCR0_N_RATE_HALF			0x1
#define LNaRGCR0_N_RATE_QUARTER			0x2
#define LNaRGCR0_N_RATE_DOUBLE			0x3
#define LNaRGCR0_INTACCPL_DIS			BIT(5)
#define LNaRGCR0_CMADJ_DIS			BIT(4)

#define LNaRGCR1(lane)				(0x800 + (lane) * 0x100 + 0x48)
#define LNaRGCR1_RX_ORD_ELECIDLE		BIT(31)
#define LNaRGCR1_DATA_LOST_FLT			BIT(30)
#define LNaRGCR1_DATA_LOST			BIT(29)
#define LNaRGCR1_IDLE_CONFIG			BIT(28)
#define LNaRGCR1_ENTER_IDLE_FLT_SEL		GENMASK(26, 24)
#define LNaRGCR1_EXIT_IDLE_FLT_SEL		GENMASK(22, 20)
#define LNaRGCR1_DATA_LOST_TH_SEL		GENMASK(18, 16)
#define LNaRGCR1_EXT_REC_CLK_SEL		GENMASK(10, 8)
#define LNaRGCR1_WAKE_TX_DIS			BIT(5)
#define LNaRGCR1_PHY_RDY			BIT(4)
#define LNaRGCR1_CHANGE_RX_CLK			BIT(3)
#define LNaRGCR1_PWR_MGT			GENMASK(2, 0)

#define LNaRECR0(lane)				(0x800 + (lane) * 0x100 + 0x50)
#define LNaRECR0_EQ_GAINK2_HF_OV_EN		BIT(31)
#define LNaRECR0_EQ_GAINK2_HF_OV		GENMASK(28, 24)
#define LNaRECR0_EQ_GAINK3_MF_OV_EN		BIT(23)
#define LNaRECR0_EQ_GAINK3_MF_OV		GENMASK(20, 16)
#define LNaRECR0_EQ_GAINK4_LF_OV_EN		BIT(7)
#define LNaRECR0_EQ_GAINK4_LF_DIS		BIT(6)
#define LNaRECR0_EQ_GAINK4_LF_OV		GENMASK(4, 0)

#define LNaRECR1(lane)				(0x800 + (lane) * 0x100 + 0x54)
#define LNaRECR1_EQ_BLW_OV_EN			BIT(31)
#define LNaRECR1_EQ_BLW_OV			GENMASK(28, 24)
#define LNaRECR1_EQ_OFFSET_OV_EN		BIT(23)
#define LNaRECR1_EQ_OFFSET_OV			GENMASK(21, 16)

#define LNaRECR2(lane)				(0x800 + (lane) * 0x100 + 0x58)
#define LNaRECR2_EQ_OFFSET_RNG_DBL		BIT(31)
#define LNaRECR2_EQ_BOOST			GENMASK(29, 28)
#define LNaRECR2_EQ_BLW_SEL			GENMASK(25, 24)
#define LNaRECR2_EQ_ZERO			GENMASK(17, 16)
#define LNaRECR2_EQ_IND				GENMASK(13, 12)
#define LNaRECR2_EQ_BIN_DATA_AVG_TC		GENMASK(5, 4)
#define LNaRECR2_SPARE_IN			GENMASK(1, 0)

#define LNaRECR3(lane)				(0x800 + (lane) * 0x100 + 0x5c)
#define LNaRECR3_EQ_SNAP_START			BIT(31)
#define LNaRECR3_EQ_SNAP_DONE			BIT(30)
#define LNaRECR3_EQ_GAINK2_HF_STAT		GENMASK(28, 24)
#define LNaRECR3_EQ_GAINK3_MF_STAT		GENMASK(20, 16)
#define LNaRECR3_SPARE_OUT			GENMASK(13, 12)
#define LNaRECR3_EQ_GAINK4_LF_STAT		GENMASK(4, 0)

#define LNaRECR4(lane)				(0x800 + (lane) * 0x100 + 0x60)
#define LNaRECR4_BLW_STAT			GENMASK(28, 24)
#define LNaRECR4_EQ_OFFSET_STAT			GENMASK(21, 16)
#define LNaRECR4_EQ_BIN_DATA_SEL		GENMASK(15, 12)
#define LNaRECR4_EQ_BIN_DATA			GENMASK(8, 0) /* bit 9 is reserved */
#define LNaRECR4_EQ_BIN_DATA_SGN		BIT(8)

#define LNaRCCR0(lane)				(0x800 + (lane) * 0x100 + 0x68)
#define LNaRCCR0_CAL_EN				BIT(31)
#define LNaRCCR0_MEAS_EN			BIT(30)
#define LNaRCCR0_CAL_BIN_SEL			BIT(28)
#define LNaRCCR0_CAL_DC3_DIS			BIT(27)
#define LNaRCCR0_CAL_DC2_DIS			BIT(26)
#define LNaRCCR0_CAL_DC1_DIS			BIT(25)
#define LNaRCCR0_CAL_DC0_DIS			BIT(24)
#define LNaRCCR0_CAL_AC3_OV_EN			BIT(15)
#define LNaRCCR0_CAL_AC3_OV			GENMASK(11, 8)
#define LNaRCCR0_CAL_AC2_OV_EN			BIT(7)

#define LNaRSCCR0(lane)				(0x800 + (lane) * 0x100 + 0x74)
#define LNaRSCCR0_SMP_OFF_EN			BIT(31)
#define LNaRSCCR0_SMP_OFF_OV_EN			BIT(30)
#define LNaRSCCR0_SMP_MAN_OFF_EN		BIT(29)
#define LNaRSCCR0_SMP_OFF_RNG_OV_EN		BIT(27)
#define LNaRSCCR0_SMP_OFF_RNG_4X_OV		BIT(25)
#define LNaRSCCR0_SMP_OFF_RNG_2X_OV		BIT(24)
#define LNaRSCCR0_SMP_AUTOZ_PD			BIT(23)
#define LNaRSCCR0_SMP_AUTOZ_CTRL		GENMASK(19, 16)
#define LNaRSCCR0_SMP_AUTOZ_D1R			GENMASK(13, 12)
#define LNaRSCCR0_SMP_AUTOZ_D1F			GENMASK(9, 8)
#define LNaRSCCR0_SMP_AUTOZ_EG1R		GENMASK(5, 4)
#define LNaRSCCR0_SMP_AUTOZ_EG1F		GENMASK(1, 0)

#define LNaTTLCR0(lane)				(0x800 + (lane) * 0x100 + 0x80)
#define LNaTTLCR0_TTL_FLT_SEL			GENMASK(29, 24)
#define LNaTTLCR0_TTL_SLO_PM_BYP		BIT(22)
#define LNaTTLCR0_STALL_DET_DIS			BIT(21)
#define LNaTTLCR0_INACT_MON_DIS			BIT(20)
#define LNaTTLCR0_CDR_OV			GENMASK(18, 16)
#define LNaTTLCR0_DATA_IN_SSC			BIT(15)
#define LNaTTLCR0_CDR_MIN_SMP_ON		GENMASK(1, 0)

#define LNaTCSR0(lane)				(0x800 + (lane) * 0x100 + 0xa0)
#define LNaTCSR0_SD_STAT_OBS_EN			BIT(31)
#define LNaTCSR0_SD_LPBK_SEL			GENMASK(29, 28)

#define LNaPSS(lane)				(0x1000 + (lane) * 0x4)
#define LNaPSS_TYPE				GENMASK(30, 24)
#define LNaPSS_TYPE_SGMII			(PROTO_SEL_SGMII_BASEX_KX << 2)
#define LNaPSS_TYPE_XFI				(PROTO_SEL_XFI_10GBASER_KR_SXGMII << 2)
#define LNaPSS_TYPE_40G				((PROTO_SEL_XFI_10GBASER_KR_SXGMII << 2) | 3)
#define LNaPSS_TYPE_25G				(PROTO_SEL_25G_50G_100G << 2)
#define LNaPSS_TYPE_100G			((PROTO_SEL_25G_50G_100G << 2) | 2)

/* MDEV_PORT is at the same bitfield address for all protocol converters */
#define MDEV_PORT				GENMASK(31, 27)

#define SGMIIaCR0(lane)				(0x1800 + (lane) * 0x10)
#define SGMIIaCR1(lane)				(0x1804 + (lane) * 0x10)
#define SGMIIaCR1_SGPCS_EN			BIT(11)

#define ANLTaCR0(lane)				(0x1a00 + (lane) * 0x10)
#define ANLTaCR1(lane)				(0x1a04 + (lane) * 0x10)

#define SXGMIIaCR0(lane)			(0x1a80 + (lane) * 0x10)
#define SXGMIIaCR0_RST				BIT(31)
#define SXGMIIaCR0_PD				BIT(30)

#define SXGMIIaCR1(lane)			(0x1a84 + (lane) * 0x10)

#define E25GaCR0(lane)				(0x1b00 + (lane) * 0x10)
#define E25GaCR0_RST				BIT(31)
#define E25GaCR0_PD				BIT(30)

#define E25GaCR1(lane)				(0x1b04 + (lane) * 0x10)

#define E25GaCR2(lane)				(0x1b08 + (lane) * 0x10)
#define E25GaCR2_FEC_ENA			BIT(23)
#define E25GaCR2_FEC_ERR_ENA			BIT(22)
#define E25GaCR2_FEC91_ENA			BIT(20)

#define E40GaCR0(pcvt)				(0x1b40 + (pcvt) * 0x20)
#define E40GaCR1(pcvt)				(0x1b44 + (pcvt) * 0x20)

#define E50GaCR1(pcvt)				(0x1b84 + (pcvt) * 0x10)

#define E100GaCR1(pcvt)				(0x1c04 + (pcvt) * 0x20)

#define CR(x)					((x) * 4)

#define LYNX_28G_LANE_HALT_SLEEP_US		100
#define LYNX_28G_LANE_HALT_TIMEOUT_US		1000000

#define LYNX_28G_LANE_RESET_SLEEP_US		100
#define LYNX_28G_LANE_RESET_TIMEOUT_US		1000000

#define LYNX_28G_LANE_STOP_SLEEP_US		100
#define LYNX_28G_LANE_STOP_TIMEOUT_US		1000000

#define LYNX_28G_CDR_SLEEP_US			50
#define LYNX_28G_CDR_TIMEOUT_US			500

#define LYNX_28G_SNAPSHOT_SLEEP_US		1
#define LYNX_28G_SNAPSHOT_TIMEOUT_US		1000

#define lynx_28g_read				lynx_read
#define lynx_28g_write				lynx_write
#define lynx_28g_lane_rmw			lynx_lane_rmw
#define lynx_28g_lane_read			lynx_lane_read
#define lynx_28g_lane_write			lynx_lane_write
#define lynx_28g_pll_read			lynx_pll_read

#define lynx_28g_priv				lynx_priv
#define lynx_28g_lane				lynx_lane
#define lynx_28g_pll				lynx_pll

enum lynx_28g_eq_bin_data_type {
	EQ_BIN_DATA_SEL_BIN_1 = 0,
	EQ_BIN_DATA_SEL_BIN_2 = 1,
	EQ_BIN_DATA_SEL_BIN_3 = 2,
	EQ_BIN_DATA_SEL_BIN_4 = 3,
	EQ_BIN_DATA_SEL_OFFSET = 4,
	EQ_BIN_DATA_SEL_BIN_BLW = 8,
	EQ_BIN_DATA_SEL_BIN_DATA_AVG = 9,
	EQ_BIN_DATA_SEL_BIN_M1 = 0xc,
	EQ_BIN_DATA_SEL_BIN_LONG = 0xd,
};

enum lynx_28g_eq_type {
	EQ_TYPE_NO_EQ = 0,
	EQ_TYPE_2TAP = 1,
	EQ_TYPE_3TAP = 2,
};

enum lynx_28g_proto_sel {
	PROTO_SEL_PCIE = 0,
	PROTO_SEL_SGMII_BASEX_KX = 1,
	PROTO_SEL_SATA = 2,
	PROTO_SEL_XAUI = 4,
	PROTO_SEL_XFI_10GBASER_KR_SXGMII = 0xa,
	PROTO_SEL_25G_50G_100G = 0x1a,
};

struct lynx_28g_proto_conf {
	/* LNaGCR0 */
	int proto_sel;
	int if_width;
	/* LNaTECR0 */
	int teq_type;
	int sgn_preq;
	int ratio_preq;
	int sgn_post1q;
	int ratio_post1q;
	int amp_red;
	/* LNaTECR1 */
	int adpt_eq;
	/* LNaRGCR1 */
	int enter_idle_flt_sel;
	int exit_idle_flt_sel;
	int data_lost_th_sel;
	/* LNaRECR0 */
	int gk2ovd;
	int gk3ovd;
	int gk4ovd;
	int gk2ovd_en;
	int gk3ovd_en;
	int gk4ovd_en;
	/* LNaRECR1 ? */
	int eq_offset_ovd;
	int eq_offset_ovd_en;
	/* LNaRECR2 */
	int eq_offset_rng_dbl;
	int eq_blw_sel;
	int eq_boost;
	int spare_in;
	/* LNaRSCCR0 */
	int smp_autoz_d1r;
	int smp_autoz_eg1r;
	/* LNaRCCR0 */
	int rccr0;
	/* LNaTTLCR0 */
	int ttlcr0;
};

static const struct lynx_28g_proto_conf lynx_28g_proto_conf[LANE_MODE_MAX] = {
	[LANE_MODE_1000BASEX_SGMII] = {
		.proto_sel = LNaGCR0_PROTO_SEL_SGMII,
		.if_width = LNaGCR0_IF_WIDTH_10_BIT,
		.teq_type = EQ_TYPE_NO_EQ,
		.sgn_preq = 1,
		.ratio_preq = 0,
		.sgn_post1q = 1,
		.ratio_post1q = 0,
		.amp_red = 6,
		.adpt_eq = 48,
		.enter_idle_flt_sel = 4,
		.exit_idle_flt_sel = 3,
		.data_lost_th_sel = 1,
		.gk2ovd = 0x1f,
		.gk3ovd = 0,
		.gk4ovd = 0,
		.gk2ovd_en = 1,
		.gk3ovd_en = 1,
		.gk4ovd_en = 0,
		.eq_offset_ovd = 0x1f,
		.eq_offset_ovd_en = 0,
		.eq_offset_rng_dbl = 0,
		.eq_blw_sel = 0,
		.eq_boost = 0,
		.spare_in = 0,
		.smp_autoz_d1r = 0,
		.smp_autoz_eg1r = 0,
		.rccr0 = LNaRCCR0_CAL_EN,
		.ttlcr0 = LNaTTLCR0_TTL_SLO_PM_BYP |
			  LNaTTLCR0_DATA_IN_SSC,
	},
	[LANE_MODE_1000BASEKX] = {
		.proto_sel = LNaGCR0_PROTO_SEL_SGMII,
		.if_width = LNaGCR0_IF_WIDTH_10_BIT,
		.teq_type = EQ_TYPE_NO_EQ,
		.sgn_preq = 1,
		.ratio_preq = 0,
		.sgn_post1q = 1,
		.ratio_post1q = 0,
		.amp_red = 0,
		.adpt_eq = 48,
		.enter_idle_flt_sel = 0,
		.exit_idle_flt_sel = 0,
		.data_lost_th_sel = 0,
		.gk2ovd = 0x1f,
		.gk3ovd = 0,
		.gk4ovd = 0,
		.gk2ovd_en = 1,
		.gk3ovd_en = 1,
		.gk4ovd_en = 0,
		.eq_offset_ovd = 0x1f,
		.eq_offset_ovd_en = 0,
		.eq_offset_rng_dbl = 0,
		.eq_blw_sel = 0,
		.eq_boost = 0,
		.spare_in = 0,
		.smp_autoz_d1r = 0,
		.smp_autoz_eg1r = 0,
		.rccr0 = LNaRCCR0_CAL_EN,
		.ttlcr0 = LNaTTLCR0_TTL_SLO_PM_BYP |
			  LNaTTLCR0_DATA_IN_SSC,
	},
	[LANE_MODE_USXGMII] = {
		.proto_sel = LNaGCR0_PROTO_SEL_XFI,
		.if_width = LNaGCR0_IF_WIDTH_20_BIT,
		.teq_type = EQ_TYPE_2TAP,
		.sgn_preq = 1,
		.ratio_preq = 0,
		.sgn_post1q = 1,
		.ratio_post1q = 3,
		.amp_red = 7,
		.adpt_eq = 48,
		.enter_idle_flt_sel = 0,
		.exit_idle_flt_sel = 0,
		.data_lost_th_sel = 0,
		.gk2ovd = 0,
		.gk3ovd = 0,
		.gk4ovd = 0,
		.gk2ovd_en = 0,
		.gk3ovd_en = 0,
		.gk4ovd_en = 0,
		.eq_offset_ovd = 0x1f,
		.eq_offset_ovd_en = 0,
		.eq_offset_rng_dbl = 1,
		.eq_blw_sel = 1,
		.eq_boost = 0,
		.spare_in = 0,
		.smp_autoz_d1r = 2,
		.smp_autoz_eg1r = 0,
		.rccr0 = LNaRCCR0_CAL_EN,
		.ttlcr0 = LNaTTLCR0_TTL_SLO_PM_BYP |
			  LNaTTLCR0_DATA_IN_SSC,
	},
	[LANE_MODE_10GBASER] = {
		.proto_sel = LNaGCR0_PROTO_SEL_XFI,
		.if_width = LNaGCR0_IF_WIDTH_20_BIT,
		.teq_type = EQ_TYPE_2TAP,
		.sgn_preq = 1,
		.ratio_preq = 0,
		.sgn_post1q = 1,
		.ratio_post1q = 3,
		.amp_red = 7,
		.adpt_eq = 48,
		.enter_idle_flt_sel = 0,
		.exit_idle_flt_sel = 0,
		.data_lost_th_sel = 0,
		.gk2ovd = 0,
		.gk3ovd = 0,
		.gk4ovd = 0,
		.gk2ovd_en = 0,
		.gk3ovd_en = 0,
		.gk4ovd_en = 0,
		.eq_offset_ovd = 0x1f,
		.eq_offset_ovd_en = 0,
		.eq_offset_rng_dbl = 1,
		.eq_blw_sel = 1,
		.eq_boost = 0,
		.spare_in = 0,
		.smp_autoz_d1r = 2,
		.smp_autoz_eg1r = 0,
		.rccr0 = LNaRCCR0_CAL_EN,
		.ttlcr0 = LNaTTLCR0_TTL_SLO_PM_BYP |
			  LNaTTLCR0_DATA_IN_SSC,
	},
	[LANE_MODE_10GBASEKR] = {
		.proto_sel = LNaGCR0_PROTO_SEL_XFI,
		.if_width = LNaGCR0_IF_WIDTH_20_BIT,
		.teq_type = EQ_TYPE_3TAP,
		.sgn_preq = 1,
		.ratio_preq = 2,
		.sgn_post1q = 1,
		.ratio_post1q = 5,
		.amp_red = 0,
		.adpt_eq = 41,
		.enter_idle_flt_sel = 0,
		.exit_idle_flt_sel = 0,
		.data_lost_th_sel = 0,
		.gk2ovd = 0,
		.gk3ovd = 0,
		.gk4ovd = 0,
		.gk2ovd_en = 0,
		.gk3ovd_en = 0,
		.gk4ovd_en = 0,
		.eq_offset_ovd = 0x1f,
		.eq_offset_ovd_en = 0,
		.eq_offset_rng_dbl = 1,
		.eq_blw_sel = 1,
		.eq_boost = 0,
		.spare_in = 0,
		.smp_autoz_d1r = 2,
		.smp_autoz_eg1r = 0,
		.rccr0 = LNaRCCR0_CAL_EN,
		.ttlcr0 = LNaTTLCR0_TTL_SLO_PM_BYP |
			  LNaTTLCR0_DATA_IN_SSC,
	},
	[LANE_MODE_25GBASER] = {
		.proto_sel = LNaGCR0_PROTO_SEL_25G,
		.if_width = LNaGCR0_IF_WIDTH_40_BIT,
		.teq_type = EQ_TYPE_3TAP,
		.sgn_preq = 1,
		.ratio_preq = 2,
		.sgn_post1q = 1,
		.ratio_post1q = 7,
		.amp_red = 0,
		.adpt_eq = 48,
		.enter_idle_flt_sel = 0,
		.exit_idle_flt_sel = 0,
		.data_lost_th_sel = 0,
		.gk2ovd = 0,
		.gk3ovd = 0,
		.gk4ovd = 5,
		.gk2ovd_en = 0,
		.gk3ovd_en = 0,
		.gk4ovd_en = 1,
		.eq_offset_ovd = 0x1f,
		.eq_offset_ovd_en = 0,
		.eq_offset_rng_dbl = 1,
		.eq_blw_sel = 1,
		.eq_boost = 2,
		.spare_in = 3,
		.smp_autoz_d1r = 2,
		.smp_autoz_eg1r = 2,
		.rccr0 = LNaRCCR0_CAL_EN |
			 LNaRCCR0_CAL_DC3_DIS |
			 LNaRCCR0_CAL_DC2_DIS |
			 LNaRCCR0_CAL_DC1_DIS |
			 LNaRCCR0_CAL_DC0_DIS,
		.ttlcr0 = LNaTTLCR0_DATA_IN_SSC |
			  FIELD_PREP_CONST(LNaTTLCR0_CDR_MIN_SMP_ON, 1),
	},
	[LANE_MODE_25GBASEKR] = {
		.proto_sel = LNaGCR0_PROTO_SEL_25G,
		.if_width = LNaGCR0_IF_WIDTH_40_BIT,
		.teq_type = EQ_TYPE_3TAP,
		.sgn_preq = 1,
		.ratio_preq = 2,
		.sgn_post1q = 1,
		.ratio_post1q = 7,
		.amp_red = 0, // FIXME 32 for C2C?
		.adpt_eq = 38,
		.enter_idle_flt_sel = 0,
		.exit_idle_flt_sel = 0,
		.data_lost_th_sel = 0,
		.gk2ovd = 0,
		.gk3ovd = 0,
		.gk4ovd = 5,
		.gk2ovd_en = 0,
		.gk3ovd_en = 0,
		.gk4ovd_en = 1,
		.eq_offset_ovd = 0x1f,
		.eq_offset_ovd_en = 0,
		.eq_offset_rng_dbl = 1,
		.eq_blw_sel = 1,
		.eq_boost = 2,
		.spare_in = 3,
		.smp_autoz_d1r = 2,
		.smp_autoz_eg1r = 2,
		.rccr0 = LNaRCCR0_CAL_EN |
			 LNaRCCR0_CAL_DC3_DIS |
			 LNaRCCR0_CAL_DC2_DIS |
			 LNaRCCR0_CAL_DC1_DIS |
			 LNaRCCR0_CAL_DC0_DIS,
		.ttlcr0 = LNaTTLCR0_DATA_IN_SSC |
			  FIELD_PREP_CONST(LNaTTLCR0_CDR_MIN_SMP_ON, 1),
	},
	[LANE_MODE_40GBASER_XLAUI] = {
		.proto_sel = LNaGCR0_PROTO_SEL_XFI,
		.if_width = LNaGCR0_IF_WIDTH_20_BIT,
		.teq_type = EQ_TYPE_3TAP,
		.sgn_preq = 1,
		.ratio_preq = 2,
		.sgn_post1q = 1,
		.ratio_post1q = 5,
		.amp_red = 0,
		.adpt_eq = 41,
		.enter_idle_flt_sel = 0,
		.exit_idle_flt_sel = 0,
		.data_lost_th_sel = 0,
		.gk2ovd = 0,
		.gk3ovd = 0,
		.gk4ovd = 0,
		.gk2ovd_en = 0,
		.gk3ovd_en = 0,
		.gk4ovd_en = 0,
		.eq_offset_ovd = 0x1f,
		.eq_offset_ovd_en = 0,
		.eq_offset_rng_dbl = 1,
		.eq_blw_sel = 1,
		.eq_boost = 0,
		.spare_in = 0,
		.smp_autoz_d1r = 2,
		.smp_autoz_eg1r = 0,
		.rccr0 = LNaRCCR0_CAL_EN,
		.ttlcr0 = LNaTTLCR0_TTL_SLO_PM_BYP |
			  LNaTTLCR0_DATA_IN_SSC,
	},
	[LANE_MODE_40GBASEKR4] = {
		.proto_sel = LNaGCR0_PROTO_SEL_XFI,
		.if_width = LNaGCR0_IF_WIDTH_20_BIT,
		.teq_type = EQ_TYPE_3TAP,
		.sgn_preq = 1,
		.ratio_preq = 2,
		.sgn_post1q = 1,
		.ratio_post1q = 5,
		.amp_red = 0,
		.adpt_eq = 41,
		.enter_idle_flt_sel = 0,
		.exit_idle_flt_sel = 0,
		.data_lost_th_sel = 0,
		.gk2ovd = 0,
		.gk3ovd = 0,
		.gk4ovd = 0,
		.gk2ovd_en = 0,
		.gk3ovd_en = 0,
		.gk4ovd_en = 0,
		.eq_offset_ovd = 0x1f,
		.eq_offset_ovd_en = 0,
		.eq_offset_rng_dbl = 1,
		.eq_blw_sel = 1,
		.eq_boost = 0,
		.spare_in = 0,
		.smp_autoz_d1r = 2,
		.smp_autoz_eg1r = 0,
		.rccr0 = LNaRCCR0_CAL_EN,
		.ttlcr0 = LNaTTLCR0_TTL_SLO_PM_BYP |
			  LNaTTLCR0_DATA_IN_SSC,
	},
};

static const int lynx_28g_bin_type_to_bin_sel[] = {
	[BIN_1] = EQ_BIN_DATA_SEL_BIN_1,
	[BIN_2] = EQ_BIN_DATA_SEL_BIN_2,
	[BIN_3] = EQ_BIN_DATA_SEL_BIN_3,
	[BIN_4] = EQ_BIN_DATA_SEL_BIN_4,
	[BIN_OFFSET] = EQ_BIN_DATA_SEL_OFFSET,
	[BIN_M1] = EQ_BIN_DATA_SEL_BIN_M1,
	[BIN_LONG] = EQ_BIN_DATA_SEL_BIN_LONG,
};

static void lynx_28g_lane_set_nrate(struct lynx_28g_lane *lane,
				    struct lynx_28g_pll *pll,
				    enum lynx_lane_mode lane_mode)
{
	switch (pll->frate_sel) {
	case PLLnCR1_FRATE_5G_10GVCO:
	case PLLnCR1_FRATE_5G_25GVCO:
		switch (lane_mode) {
		case LANE_MODE_1000BASEX_SGMII:
		case LANE_MODE_1000BASEKX:
			lynx_28g_lane_rmw(lane, LNaTGCR0,
					  FIELD_PREP(LNaTGCR0_N_RATE, LNaTGCR0_N_RATE_QUARTER),
					  LNaTGCR0_N_RATE);
			lynx_28g_lane_rmw(lane, LNaRGCR0,
					  FIELD_PREP(LNaRGCR0_N_RATE, LNaRGCR0_N_RATE_QUARTER),
					  LNaRGCR0_N_RATE);
			break;
		default:
			break;
		}
		break;
	case PLLnCR1_FRATE_10G_20GVCO:
		switch (lane_mode) {
		case LANE_MODE_10GBASER:
		case LANE_MODE_USXGMII:
		case LANE_MODE_10GBASEKR:
		case LANE_MODE_40GBASER_XLAUI:
		case LANE_MODE_40GBASEKR4:
			lynx_28g_lane_rmw(lane, LNaTGCR0,
					  FIELD_PREP(LNaTGCR0_N_RATE, LNaTGCR0_N_RATE_FULL),
					  LNaTGCR0_N_RATE);
			lynx_28g_lane_rmw(lane, LNaRGCR0,
					  FIELD_PREP(LNaRGCR0_N_RATE, LNaRGCR0_N_RATE_FULL),
					  LNaRGCR0_N_RATE);
			break;
		default:
			break;
		}
		break;
	case PLLnCR1_FRATE_12G_25GVCO:
		switch (lane_mode) {
		case LANE_MODE_25GBASER:
		case LANE_MODE_25GBASEKR:
			lynx_28g_lane_rmw(lane, LNaTGCR0,
					  FIELD_PREP(LNaTGCR0_N_RATE, LNaTGCR0_N_RATE_DOUBLE),
					  LNaTGCR0_N_RATE);
			lynx_28g_lane_rmw(lane, LNaRGCR0,
					  FIELD_PREP(LNaRGCR0_N_RATE, LNaRGCR0_N_RATE_DOUBLE),
					  LNaRGCR0_N_RATE);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void lynx_28g_lane_set_pll(struct lynx_28g_lane *lane,
				  struct lynx_28g_pll *pll)
{
	if (pll->id == 0) {
		lynx_28g_lane_rmw(lane, LNaTGCR0,
				  FIELD_PREP(LNaTGCR0_USE_PLL, LNaTGCR0_USE_PLLF),
				  LNaTGCR0_USE_PLL);
		lynx_28g_lane_rmw(lane, LNaRGCR0,
				  FIELD_PREP(LNaRGCR0_USE_PLL, LNaRGCR0_USE_PLLF),
				  LNaRGCR0_USE_PLL);
	} else {
		lynx_28g_lane_rmw(lane, LNaTGCR0,
				  FIELD_PREP(LNaTGCR0_USE_PLL, LNaTGCR0_USE_PLLS),
				  LNaTGCR0_USE_PLL);
		lynx_28g_lane_rmw(lane, LNaRGCR0,
				  FIELD_PREP(LNaRGCR0_USE_PLL, LNaRGCR0_USE_PLLS),
				  LNaRGCR0_USE_PLL);
	}
}

static bool lynx_28g_lane_halt_done(struct lynx_28g_lane *lane)
{
	u32 trstctl = lynx_28g_lane_read(lane, LNaTRSTCTL);
	u32 rrstctl = lynx_28g_lane_read(lane, LNaRRSTCTL);

	return !(trstctl & LNaTRSTCTL_HLT_REQ) &&
	       !(rrstctl & LNaRRSTCTL_HLT_REQ);
}

static bool lynx_28g_lane_stop_done(struct lynx_28g_lane *lane)
{
	u32 trstctl = lynx_28g_lane_read(lane, LNaTRSTCTL);
	u32 rrstctl = lynx_28g_lane_read(lane, LNaRRSTCTL);

	return !(trstctl & LNaTRSTCTL_STP_REQ) &&
	       !(rrstctl & LNaRRSTCTL_STP_REQ);
}

static bool lynx_28g_lane_reset_done(struct lynx_28g_lane *lane)
{
	u32 trstctl = lynx_28g_lane_read(lane, LNaTRSTCTL);
	u32 rrstctl = lynx_28g_lane_read(lane, LNaRRSTCTL);

	return (trstctl & LNaTRSTCTL_RST_DONE) &&
	       (rrstctl & LNaRRSTCTL_RST_DONE);
}

/* Halting puts the lane in a mode in which it can be reconfigured */
static int lynx_28g_lane_halt(struct phy *phy)
{
	struct lynx_28g_lane *lane = phy_get_drvdata(phy);
	bool done;
	int err;

	/* Issue a halt request */
	lynx_28g_lane_rmw(lane, LNaTRSTCTL, LNaTRSTCTL_HLT_REQ,
			  LNaTRSTCTL_HLT_REQ);
	lynx_28g_lane_rmw(lane, LNaRRSTCTL, LNaRRSTCTL_HLT_REQ,
			  LNaRRSTCTL_HLT_REQ);

	/* Wait until the halting process is complete */
	err = read_poll_timeout(lynx_28g_lane_halt_done, done, done,
				LYNX_28G_LANE_HALT_SLEEP_US,
				LYNX_28G_LANE_HALT_TIMEOUT_US,
				false, lane);
	if (err) {
		dev_err(&phy->dev, "Lane %c halt failed: %pe\n",
			'A' + lane->id, ERR_PTR(err));
		return err;
	}

	lane->powered_up = false;

	return 0;
}

static int lynx_28g_lane_reset(struct phy *phy)
{
	struct lynx_28g_lane *lane = phy_get_drvdata(phy);
	bool done;
	int err;

	/* Issue a reset request on the lane */
	lynx_28g_lane_rmw(lane, LNaTRSTCTL, LNaTRSTCTL_RST_REQ,
			  LNaTRSTCTL_RST_REQ);
	lynx_28g_lane_rmw(lane, LNaRRSTCTL, LNaRRSTCTL_RST_REQ,
			  LNaRRSTCTL_RST_REQ);

	/* Wait until the reset sequence is completed */
	err = read_poll_timeout(lynx_28g_lane_reset_done, done, done,
				LYNX_28G_LANE_RESET_SLEEP_US,
				LYNX_28G_LANE_RESET_TIMEOUT_US,
				false, lane);
	if (err) {
		dev_err(&phy->dev, "Lane %c reset failed: %pe\n",
			'A' + lane->id, ERR_PTR(err));
	}

	return err;
}

static int lynx_28g_power_off(struct phy *phy)
{
	struct lynx_28g_lane *lane = phy_get_drvdata(phy);
	bool done;
	int err;

	if (!lane->powered_up)
		return 0;

	/* Issue a stop request */
	lynx_28g_lane_rmw(lane, LNaTRSTCTL, LNaTRSTCTL_STP_REQ,
			  LNaTRSTCTL_STP_REQ);
	lynx_28g_lane_rmw(lane, LNaRRSTCTL, LNaRRSTCTL_STP_REQ,
			  LNaRRSTCTL_STP_REQ);

	/* Wait until the stop process is complete */
	err = read_poll_timeout(lynx_28g_lane_stop_done, done, done,
				LYNX_28G_LANE_STOP_SLEEP_US,
				LYNX_28G_LANE_STOP_TIMEOUT_US,
				false, lane);
	if (err) {
		dev_err(&phy->dev, "Lane %c stop failed: %pe\n",
			'A' + lane->id, ERR_PTR(err));
		return err;
	}

	/* Power down the RX and TX portions of the lane */
	lynx_28g_lane_rmw(lane, LNaRRSTCTL, LNaRRSTCTL_DIS,
			  LNaRRSTCTL_DIS);
	lynx_28g_lane_rmw(lane, LNaTRSTCTL, LNaTRSTCTL_DIS,
			  LNaTRSTCTL_DIS);

	lane->powered_up = false;

	return 0;
}

static int lynx_28g_power_on(struct phy *phy)
{
	struct lynx_28g_lane *lane = phy_get_drvdata(phy);
	int err;

	if (lane->powered_up)
		return 0;

	/* Power up the RX and TX portions of the lane */
	lynx_28g_lane_rmw(lane, LNaRRSTCTL, 0, LNaRRSTCTL_DIS);
	lynx_28g_lane_rmw(lane, LNaTRSTCTL, 0, LNaTRSTCTL_DIS);

	err = lynx_28g_lane_reset(phy);
	if (err)
		return err;

	lane->powered_up = true;

	return 0;
}

static bool lynx_28g_cdr_lock_check(struct lynx_28g_lane *lane)
{
	u32 rrstctl = lynx_28g_lane_read(lane, LNaRRSTCTL);
	int err;

	if (rrstctl & LNaRRSTCTL_CDR_LOCK)
		return true;

	/* Omit resetting the receiver unless the lane is up. Otherwise,
	 * if powered down, it won't complete the operation.
	 */
	if (!lane->init || !lane->powered_up)
		return false;

	dev_dbg(&lane->phy->dev,
		"Lane %c CDR unlocked, resetting receiver...\n",
		'A' + lane->id);

	lynx_28g_lane_rmw(lane, LNaRRSTCTL, LNaRRSTCTL_RST_REQ,
			  LNaRRSTCTL_RST_REQ);

	err = read_poll_timeout(lynx_28g_lane_read, rrstctl,
				!!(rrstctl & LNaRRSTCTL_RST_DONE),
				LYNX_28G_LANE_RESET_SLEEP_US,
				LYNX_28G_LANE_RESET_TIMEOUT_US,
				false, lane, LNaRRSTCTL);
	if (err) {
		dev_warn_once(&lane->phy->dev,
			      "Lane %c receiver reset failed: %pe\n",
			      'A' + lane->id, ERR_PTR(err));
		return false;
	}

	return !!(rrstctl & LNaRRSTCTL_CDR_LOCK);
}

static int lynx_28g_e25g_pcvt(int lane)
{
	return 7 - lane;
}

static int lynx_28g_e40g_pcvt(int lane)
{
	return lane < 4 ? 1 : 0;
}

static int lynx_28g_get_pccr(enum lynx_lane_mode lane_mode, int lane,
			     struct lynx_pccr *pccr)
{
	switch (lane_mode) {
	case LANE_MODE_1000BASEX_SGMII:
	case LANE_MODE_1000BASEKX:
		pccr->offset = PCC8;
		pccr->width = 4;
		pccr->shift = SGMII_CFG(lane);
		break;
	case LANE_MODE_USXGMII:
	case LANE_MODE_10GBASER:
	case LANE_MODE_10GBASEKR:
		pccr->offset = PCCC;
		pccr->width = 4;
		pccr->shift = SXGMII_CFG(lane);
		break;
	case LANE_MODE_25GBASER:
	case LANE_MODE_25GBASEKR:
		pccr->offset = PCCD;
		pccr->width = 4;
		pccr->shift = E25G_CFG(lynx_28g_e25g_pcvt(lane));
		break;
	case LANE_MODE_40GBASER_XLAUI:
	case LANE_MODE_40GBASEKR4:
		pccr->offset = PCCE;
		pccr->width = 4;
		pccr->shift = E40G_CFG(lynx_28g_e40g_pcvt(lane));
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static int lynx_28g_get_pcvt_offset(int lane, enum lynx_lane_mode lane_mode)
{
	switch (lane_mode) {
	case LANE_MODE_1000BASEX_SGMII:
	case LANE_MODE_1000BASEKX:
		return SGMIIaCR0(lane);
	case LANE_MODE_USXGMII:
	case LANE_MODE_10GBASER:
	case LANE_MODE_10GBASEKR:
		return SXGMIIaCR0(lane);
	case LANE_MODE_25GBASER:
	case LANE_MODE_25GBASEKR:
		return E25GaCR0(lynx_28g_e25g_pcvt(lane));
	case LANE_MODE_40GBASER_XLAUI:
	case LANE_MODE_40GBASEKR4:
		return E40GaCR0(lynx_28g_e40g_pcvt(lane));
	default:
		return -EOPNOTSUPP;
	}
}

static int lynx_28g_get_anlt_offset(int lane, enum lynx_lane_mode mode)
{
	switch (mode) {
	case LANE_MODE_25GBASEKR:
	case LANE_MODE_40GBASEKR4:
		return ANLTaCR0(lane);
	default:
		return -EOPNOTSUPP;
	}
}

static bool lx2160a_serdes1_lane_supports_mode(int lane,
					       enum lynx_lane_mode mode)
{
	switch (mode) {
	case LANE_MODE_25GBASER:
	case LANE_MODE_25GBASEKR:
		return lane != 2 && lane != 3;
	default:
		return true;
	}
}

static bool lx2160a_serdes2_lane_supports_mode(int lane,
					       enum lynx_lane_mode mode)
{
	switch (mode) {
	case LANE_MODE_1000BASEX_SGMII:
	case LANE_MODE_1000BASEKX:
		return true;
	case LANE_MODE_USXGMII:
	case LANE_MODE_10GBASER:
	case LANE_MODE_10GBASEKR:
		return lane == 6 || lane == 7;
	default:
		return false;
	}
}

static bool lx2160a_serdes3_lane_supports_mode(int lane,
					       enum lynx_lane_mode mode)
{
	/*
	 * Non-networking SerDes, and this driver supports only
	 * networking protocols
	 */
	return false;
}

static bool lx2162a_serdes1_lane_supports_mode(int lane,
					       enum lynx_lane_mode mode)
{
	return true;
}

static bool lx2162a_serdes2_lane_supports_mode(int lane,
					       enum lynx_lane_mode mode)
{
	return lx2160a_serdes2_lane_supports_mode(lane, mode);
}

static bool lynx_28g_compat_lane_supports_mode(int lane,
					       enum lynx_lane_mode mode)
{
	switch (mode) {
	case LANE_MODE_1000BASEX_SGMII:
	case LANE_MODE_USXGMII:
	case LANE_MODE_10GBASER:
		return true;
	default:
		return false;
	}
}

static const struct lynx_info lynx_info_compat = {
	.get_pccr = lynx_28g_get_pccr,
	.get_pcvt_offset = lynx_28g_get_pcvt_offset,
	.lane_supports_mode = lynx_28g_compat_lane_supports_mode,
	.num_lanes = LYNX_28G_NUM_LANE,
};

static const struct lynx_info lynx_info_lx2160a_serdes1 = {
	.get_pccr = lynx_28g_get_pccr,
	.get_pcvt_offset = lynx_28g_get_pcvt_offset,
	.lane_supports_mode = lx2160a_serdes1_lane_supports_mode,
	.num_lanes = LYNX_28G_NUM_LANE,
};

static const struct lynx_info lynx_info_lx2160a_serdes2 = {
	.get_pccr = lynx_28g_get_pccr,
	.get_pcvt_offset = lynx_28g_get_pcvt_offset,
	.lane_supports_mode = lx2160a_serdes2_lane_supports_mode,
	.num_lanes = LYNX_28G_NUM_LANE,
};

static const struct lynx_info lynx_info_lx2160a_serdes3 = {
	.get_pccr = lynx_28g_get_pccr,
	.get_pcvt_offset = lynx_28g_get_pcvt_offset,
	.lane_supports_mode = lx2160a_serdes3_lane_supports_mode,
	.num_lanes = LYNX_28G_NUM_LANE,
};

static const struct lynx_info lynx_info_lx2162a_serdes1 = {
	.get_pccr = lynx_28g_get_pccr,
	.get_pcvt_offset = lynx_28g_get_pcvt_offset,
	.lane_supports_mode = lx2162a_serdes1_lane_supports_mode,
	.first_lane = 4,
	.num_lanes = LYNX_28G_NUM_LANE,
};

static const struct lynx_info lynx_info_lx2162a_serdes2 = {
	.get_pccr = lynx_28g_get_pccr,
	.get_pcvt_offset = lynx_28g_get_pcvt_offset,
	.lane_supports_mode = lx2162a_serdes2_lane_supports_mode,
	.num_lanes = LYNX_28G_NUM_LANE,
};

static int lynx_anlt_read(struct lynx_28g_lane *lane, enum lynx_lane_mode mode,
			  int cr, u32 *val)
{
	struct lynx_28g_priv *priv = lane->priv;
	int offset;

	switch (mode) {
	case LANE_MODE_1000BASEKX:
	case LANE_MODE_10GBASEKR:
		/* For 1G and 10G, AN/LT registers are merged with the PCS */
		return lynx_pcvt_read(lane, mode, cr, val);
	case LANE_MODE_25GBASEKR:
	case LANE_MODE_40GBASEKR4:
		offset = lynx_28g_get_anlt_offset(lane->id, mode);
		if (offset < 0)
			return offset;

		*val = lynx_28g_read(priv, offset + cr);

		return 0;
	default:
		break;
	}

	return -EOPNOTSUPP;
}

/* Enabling ex_dly_clk does not require turning the PLL off, and does not
 * affect the state of the lanes mapped to it. It is one of the few safe things
 * that can be done with it at runtime.
 */
static void lynx_28g_pll_ex_dly_clk_enable(struct lynx_28g_pll *pll,
					   bool enable)
{
	u32 val = 0;

	if (enable)
		val = FIELD_PREP(PLLnCR1_EX_DLY_SEL, PLLnCR1_EX_DLY_SEL_312_5_MHZ);

	dev_dbg(pll->priv->dev, "Turning %s EX_DLY_CLK on PLL%c\n",
		str_on_off(enable), pll->id == 0 ? 'F' : 'S');

	lynx_pll_rmw(pll, PLLnCR1, val, PLLnCR1_EX_DLY_SEL);
}

static void lynx_28g_pll_get_ex_dly_clk(struct lynx_28g_pll *pll)
{
	spin_lock(&pll->lock);

	if (++pll->ex_dly_clk_use_count > 1) {
		spin_unlock(&pll->lock);
		return;
	}

	lynx_28g_pll_ex_dly_clk_enable(pll, true);

	spin_unlock(&pll->lock);
}

static void lynx_28g_pll_put_ex_dly_clk(struct lynx_28g_pll *pll)
{
	spin_lock(&pll->lock);

	if (--pll->ex_dly_clk_use_count != 0) {
		spin_unlock(&pll->lock);
		return;
	}

	lynx_28g_pll_ex_dly_clk_enable(pll, false);

	spin_unlock(&pll->lock);
}

static void lynx_28g_lane_remap_pll(struct lynx_28g_lane *lane,
				    enum lynx_lane_mode lane_mode)
{
	struct lynx_28g_priv *priv = lane->priv;
	struct lynx_28g_pll *pll;

	/* Switch to the PLL that works with this interface type */
	pll = lynx_pll_get(priv, lane_mode);
	if (unlikely(pll == NULL))
		return;

	lynx_28g_lane_set_pll(lane, pll);

	/* Choose the portion of clock net to be used on this lane */
	lynx_28g_lane_set_nrate(lane, pll, lane_mode);
}

static void lynx_28g_lane_change_proto_conf(struct lynx_28g_lane *lane,
					    enum lynx_lane_mode lane_mode)
{
	const struct lynx_28g_proto_conf *conf = &lynx_28g_proto_conf[lane_mode];

	lynx_28g_lane_rmw(lane, LNaGCR0,
			  FIELD_PREP(LNaGCR0_PROTO_SEL, conf->proto_sel) |
			  FIELD_PREP(LNaGCR0_IF_WIDTH, conf->if_width),
			  LNaGCR0_PROTO_SEL | LNaGCR0_IF_WIDTH);

	lynx_28g_lane_rmw(lane, LNaTECR0,
			  FIELD_PREP(LNaTECR0_EQ_TYPE, conf->teq_type) |
			  FIELD_PREP(LNaTECR0_EQ_SGN_PREQ, conf->sgn_preq) |
			  FIELD_PREP(LNaTECR0_EQ_PREQ, conf->ratio_preq) |
			  FIELD_PREP(LNaTECR0_EQ_SGN_POST1Q, conf->sgn_post1q) |
			  FIELD_PREP(LNaTECR0_EQ_POST1Q, conf->ratio_post1q) |
			  FIELD_PREP(LNaTECR0_EQ_AMP_RED, conf->amp_red),
			  LNaTECR0_EQ_TYPE |
			  LNaTECR0_EQ_SGN_PREQ |
			  LNaTECR0_EQ_PREQ |
			  LNaTECR0_EQ_SGN_POST1Q |
			  LNaTECR0_EQ_POST1Q |
			  LNaTECR0_EQ_AMP_RED);

	lynx_28g_lane_rmw(lane, LNaTECR1,
			  FIELD_PREP(LNaTECR1_EQ_ADPT_EQ, conf->adpt_eq),
			  LNaTECR1_EQ_ADPT_EQ);

	lynx_28g_lane_rmw(lane, LNaRGCR1,
			  FIELD_PREP(LNaRGCR1_ENTER_IDLE_FLT_SEL, conf->enter_idle_flt_sel) |
			  FIELD_PREP(LNaRGCR1_EXIT_IDLE_FLT_SEL, conf->exit_idle_flt_sel) |
			  FIELD_PREP(LNaRGCR1_DATA_LOST_TH_SEL, conf->data_lost_th_sel),
			  LNaRGCR1_ENTER_IDLE_FLT_SEL |
			  LNaRGCR1_EXIT_IDLE_FLT_SEL |
			  LNaRGCR1_DATA_LOST_TH_SEL);

	lynx_28g_lane_rmw(lane, LNaRECR0,
			  FIELD_PREP(LNaRECR0_EQ_GAINK2_HF_OV_EN, conf->gk2ovd_en) |
			  FIELD_PREP(LNaRECR0_EQ_GAINK3_MF_OV_EN, conf->gk3ovd_en) |
			  FIELD_PREP(LNaRECR0_EQ_GAINK4_LF_OV_EN, conf->gk4ovd_en) |
			  FIELD_PREP(LNaRECR0_EQ_GAINK2_HF_OV, conf->gk2ovd) |
			  FIELD_PREP(LNaRECR0_EQ_GAINK3_MF_OV, conf->gk3ovd) |
			  FIELD_PREP(LNaRECR0_EQ_GAINK4_LF_OV, conf->gk4ovd),
			  LNaRECR0_EQ_GAINK2_HF_OV |
			  LNaRECR0_EQ_GAINK3_MF_OV |
			  LNaRECR0_EQ_GAINK4_LF_OV |
			  LNaRECR0_EQ_GAINK2_HF_OV_EN |
			  LNaRECR0_EQ_GAINK3_MF_OV_EN |
			  LNaRECR0_EQ_GAINK4_LF_OV_EN);

	lynx_28g_lane_rmw(lane, LNaRECR1,
			  FIELD_PREP(LNaRECR1_EQ_OFFSET_OV, conf->eq_offset_ovd) |
			  FIELD_PREP(LNaRECR1_EQ_OFFSET_OV_EN, conf->eq_offset_ovd_en),
			  LNaRECR1_EQ_OFFSET_OV |
			  LNaRECR1_EQ_OFFSET_OV_EN);

	lynx_28g_lane_rmw(lane, LNaRECR2,
			  FIELD_PREP(LNaRECR2_EQ_OFFSET_RNG_DBL, conf->eq_offset_rng_dbl) |
			  FIELD_PREP(LNaRECR2_EQ_BLW_SEL, conf->eq_blw_sel) |
			  FIELD_PREP(LNaRECR2_EQ_BOOST, conf->eq_boost) |
			  FIELD_PREP(LNaRECR2_SPARE_IN, conf->spare_in),
			  LNaRECR2_EQ_OFFSET_RNG_DBL |
			  LNaRECR2_EQ_BLW_SEL |
			  LNaRECR2_EQ_BOOST |
			  LNaRECR2_SPARE_IN);

	lynx_28g_lane_rmw(lane, LNaRSCCR0,
			  FIELD_PREP(LNaRSCCR0_SMP_AUTOZ_D1R, conf->smp_autoz_d1r) |
			  FIELD_PREP(LNaRSCCR0_SMP_AUTOZ_EG1R, conf->smp_autoz_eg1r),
			  LNaRSCCR0_SMP_AUTOZ_D1R |
			  LNaRSCCR0_SMP_AUTOZ_EG1R);

	lynx_28g_lane_write(lane, LNaRCCR0, conf->rccr0);
	lynx_28g_lane_write(lane, LNaTTLCR0, conf->ttlcr0);
}

static int lynx_28g_lane_disable_pcvt(struct lynx_28g_lane *lane,
				      enum lynx_lane_mode lane_mode)
{
	struct lynx_28g_priv *priv = lane->priv;
	int err;

	spin_lock(&priv->pcc_lock);

	err = lynx_pccr_write(lane, lane_mode, 0);
	if (err)
		goto out;

	switch (lane_mode) {
	case LANE_MODE_1000BASEX_SGMII:
	case LANE_MODE_1000BASEKX:
		err = lynx_pcvt_rmw(lane, lane_mode, CR(1), 0,
				    SGMIIaCR1_SGPCS_EN);
		break;
	default:
		err = 0;
	}

out:
	spin_unlock(&priv->pcc_lock);

	return err;
}

static int lynx_28g_lane_enable_pcvt(struct lynx_28g_lane *lane,
				     enum lynx_lane_mode lane_mode)
{
	struct lynx_28g_priv *priv = lane->priv;
	u32 val;
	int err;

	spin_lock(&priv->pcc_lock);

	switch (lane_mode) {
	case LANE_MODE_1000BASEX_SGMII:
	case LANE_MODE_1000BASEKX:
		err = lynx_pcvt_rmw(lane, lane_mode, CR(1), SGMIIaCR1_SGPCS_EN,
				    SGMIIaCR1_SGPCS_EN);
		break;
	default:
		err = 0;
	}

	val = 0;

	switch (lane_mode) {
	case LANE_MODE_1000BASEKX:
		val |= PCC8_SGMIIa_KX;
		fallthrough;
	case LANE_MODE_1000BASEX_SGMII:
		val |= PCC8_SGMIIa_CFG;
		break;
	case LANE_MODE_10GBASER:
	case LANE_MODE_10GBASEKR:
		val |= PCCC_SXGMIIn_XFI;
		fallthrough;
	case LANE_MODE_USXGMII:
		val |= PCCC_SXGMIIn_CFG;
		break;
	case LANE_MODE_25GBASER:
	case LANE_MODE_25GBASEKR:
		val |= PCCD_E25Gn_CFG;
		break;
	case LANE_MODE_40GBASER_XLAUI:
	case LANE_MODE_40GBASEKR4:
		val |= PCCE_E40Gn_CFG;
		break;
	default:
		break;
	}

	err = lynx_pccr_write(lane, lane_mode, val);

	spin_unlock(&priv->pcc_lock);

	return err;
}

static void lynx_28g_tune_tx_eq(struct phy *phy,
				const struct lynx_xgkr_tx_eq *tx_eq)
{
	struct lynx_28g_lane *lane = phy_get_drvdata(phy);

	lynx_28g_lane_rmw(lane, LNaTECR0,
			  FIELD_PREP(LNaTECR0_EQ_PREQ, tx_eq->ratio_preq) |
			  FIELD_PREP(LNaTECR0_EQ_POST1Q, tx_eq->ratio_post1q) |
			  FIELD_PREP(LNaTECR0_EQ_AMP_RED, tx_eq->amp_reduction),
			  LNaTECR0_EQ_PREQ |
			  LNaTECR0_EQ_POST1Q |
			  LNaTECR0_EQ_AMP_RED);

	lynx_28g_lane_rmw(lane, LNaTECR1,
			  FIELD_PREP(LNaTECR1_EQ_ADPT_EQ, tx_eq->adapt_eq),
			  LNaTECR1_EQ_ADPT_EQ);

	udelay(1);
}

static void lynx_28g_read_tx_eq(struct phy *phy, struct lynx_xgkr_tx_eq *tx_eq)
{
	struct lynx_28g_lane *lane = phy_get_drvdata(phy);
	int val;

	val = lynx_28g_lane_read(lane, LNaTECR0);
	tx_eq->ratio_preq = FIELD_GET(LNaTECR0_EQ_PREQ, val);
	tx_eq->ratio_post1q = FIELD_GET(LNaTECR0_EQ_POST1Q, val);
	tx_eq->amp_reduction = FIELD_GET(LNaTECR0_EQ_AMP_RED, val);

	val = lynx_28g_lane_read(lane, LNaTECR1);
	tx_eq->adapt_eq = FIELD_GET(LNaTECR1_EQ_ADPT_EQ, val);
}

static int lynx_28g_snapshot_rx_eq(struct phy *phy, int bin_sel, void *ctx,
				   void (*cb)(struct phy *phy, void *ctx))
{
	struct lynx_28g_lane *lane = phy_get_drvdata(phy);
	bool cdr_locked;
	int err, val;

	err = read_poll_timeout(lynx_28g_cdr_lock_check, cdr_locked,
				cdr_locked, LYNX_28G_CDR_SLEEP_US,
				LYNX_28G_CDR_TIMEOUT_US, false, lane);
	if (err) {
		dev_err(&phy->dev, "CDR not locked, cannot collect RX EQ snapshots\n");
		return err;
	}

	/* wait until a previous snapshot has cleared */
	err = read_poll_timeout(lynx_28g_lane_read, val,
				!(val & LNaRECR3_EQ_SNAP_DONE),
				LYNX_28G_SNAPSHOT_SLEEP_US,
				LYNX_28G_SNAPSHOT_TIMEOUT_US,
				false, lane, LNaRECR3);
	if (err)
		return err;

	/* select the binning register we would like to snapshot */
	lynx_28g_lane_rmw(lane, LNaRECR4,
			  FIELD_PREP(LNaRECR4_EQ_BIN_DATA_SEL, bin_sel),
			  LNaRECR4_EQ_BIN_DATA_SEL);

	/* start snapshot */
	lynx_28g_lane_rmw(lane, LNaRECR3, LNaRECR3_EQ_SNAP_START,
			  LNaRECR3_EQ_SNAP_START);

	/* wait for the snapshot to finish */
	err = read_poll_timeout(lynx_28g_lane_read, val,
				!!(val & LNaRECR3_EQ_SNAP_DONE),
				LYNX_28G_SNAPSHOT_SLEEP_US,
				LYNX_28G_SNAPSHOT_TIMEOUT_US,
				false, lane, LNaRECR3);
	if (err) {
		dev_err(&phy->dev,
			"Failed to snapshot RX EQ: undetected loss of CDR lock?\n");
		lynx_28g_lane_rmw(lane, LNaRECR3, 0, LNaRECR3_EQ_SNAP_START);
		return err;
	}

	cb(phy, ctx);

	/* terminate the snapshot */
	lynx_28g_lane_rmw(lane, LNaRECR3, 0, LNaRECR3_EQ_SNAP_START);

	return 0;
}

struct lynx_28g_snapshot_gains_ctx {
	u8 *gaink2;
	u8 *gaink3;
	u8 *eq_offset;
};

static void lynx_28g_snapshot_gains_cb(struct phy *phy, void *priv)
{
	struct lynx_28g_lane *lane = phy_get_drvdata(phy);
	struct lynx_28g_snapshot_gains_ctx *ctx = priv;
	int recr3, recr4;

	recr3 = lynx_28g_lane_read(lane, LNaRECR3);
	recr4 = lynx_28g_lane_read(lane, LNaRECR4);

	*(ctx->gaink2) = FIELD_GET(LNaRECR3_EQ_GAINK2_HF_STAT, recr3);
	*(ctx->gaink3) = FIELD_GET(LNaRECR3_EQ_GAINK3_MF_STAT, recr3);
	*(ctx->eq_offset) = FIELD_GET(LNaRECR4_EQ_OFFSET_STAT, recr4);
}

static int lynx_28g_snapshot_rx_eq_gains(struct phy *phy, u8 *gaink2,
					 u8 *gaink3, u8 *eq_offset)
{
	struct lynx_28g_snapshot_gains_ctx ctx = {
		.gaink2 = gaink2,
		.gaink3 = gaink3,
		.eq_offset = eq_offset,
	};

	return lynx_28g_snapshot_rx_eq(phy, EQ_BIN_DATA_SEL_BIN_1, &ctx,
				       lynx_28g_snapshot_gains_cb);
}

static void lynx_28g_snapshot_bin_cb(struct phy *phy, void *priv)
{
	struct lynx_28g_lane *lane = phy_get_drvdata(phy);
	s16 *bin = priv;
	int val;

	/* The snapshot is a 2's complement 9 bit long value (-256 to 255) */
	val = FIELD_GET(LNaRECR4_EQ_BIN_DATA,
			lynx_28g_lane_read(lane, LNaRECR4));
	if (val & LNaRECR4_EQ_BIN_DATA_SGN) {
		val &= ~LNaRECR4_EQ_BIN_DATA_SGN;
		val -= 256;
	}

	*bin = (s16)val;
}

static int lynx_28g_snapshot_rx_eq_bin(struct phy *phy, enum lynx_bin_type bin_type,
				       s16 *bin)
{
	return lynx_28g_snapshot_rx_eq(phy, lynx_28g_bin_type_to_bin_sel[bin_type],
				       bin, lynx_28g_snapshot_bin_cb);
}

static const struct lynx_xgkr_algorithm_ops lynx_28g_xgkr_ops = {
	.tune_tx_eq = lynx_28g_tune_tx_eq,
	.read_tx_eq = lynx_28g_read_tx_eq,
	.snapshot_rx_eq_gains = lynx_28g_snapshot_rx_eq_gains,
	.snapshot_rx_eq_bin = lynx_28g_snapshot_rx_eq_bin,
};

static int lynx_28g_set_mode(struct phy *phy, enum phy_mode mode, int submode)
{
	struct lynx_28g_lane *lane = phy_get_drvdata(phy);
	struct lynx_xgkr_algorithm *algorithm = NULL;
	struct lynx_priv *priv = lane->priv;
	int powered_up = lane->powered_up;
	enum lynx_lane_mode lane_mode;
	bool needs_link_training;
	int err = 0;

	if (mode != PHY_MODE_ETHERNET)
		return -EOPNOTSUPP;

	if (lane->mode == LANE_MODE_UNKNOWN)
		return -EOPNOTSUPP;

	lane_mode = phy_interface_to_lane_mode(submode);
	if (!lynx_lane_supports_mode(lane, lane_mode))
		return -EOPNOTSUPP;

	if (lane_mode == lane->mode)
		return 0;

	needs_link_training = lynx_lane_mode_needs_link_training(lane_mode);
	if (needs_link_training) {
		algorithm = lynx_xgkr_algorithm_create(phy, &lynx_28g_xgkr_ops);
		if (!algorithm)
			return -ENOMEM;
	}

	/* If the lane is powered up, put the lane into the halt state while
	 * the reconfiguration is being done.
	 */
	if (powered_up) {
		err = lynx_28g_lane_halt(phy);
		if (err)
			return err;
	}

	err = lynx_28g_lane_disable_pcvt(lane, lane->mode);
	if (err)
		goto out;

	lynx_28g_lane_change_proto_conf(lane, lane_mode);
	lynx_28g_lane_remap_pll(lane, lane_mode);
	WARN_ON(lynx_28g_lane_enable_pcvt(lane, lane_mode));

	/* 1000Base-KX lanes need their PLL to generate a 312.5 MHz frequency
	 * through EX_DLY_CLK.
	 */
	if (lane_mode == LANE_MODE_1000BASEKX)
		lynx_28g_pll_get_ex_dly_clk(lynx_pll_get(priv, lane_mode));
	else if (lane->mode == LANE_MODE_1000BASEKX)
		lynx_28g_pll_put_ex_dly_clk(lynx_pll_get(priv, lane->mode));

	if (algorithm) {
		/* Plug in the TX equalization settings done by
		 * lynx_28g_lane_change_proto_conf() into the link training
		 * algorithm's defaults. These defaults are protocol-dependent,
		 * so we can't do any better for now, like read them from
		 * hardware as set by a previous boot stage, because we don't
		 * know what protocol those were for.
		 */
		lynx_xgkr_read_default_tx_eq(algorithm);
	}

	if (lane->algorithm)
		lynx_xgkr_algorithm_destroy(lane->algorithm);

	lane->algorithm = algorithm;

	/* Enable observation of SerDes status on all status registers */
	lynx_28g_lane_rmw(lane, LNaTCSR0,
			  FIELD_PREP(LNaTCSR0_SD_STAT_OBS_EN, needs_link_training),
			  LNaTCSR0_SD_STAT_OBS_EN);

	lane->mode = lane_mode;

out:
	/* Reset the lane if necessary */
	if (powered_up) {
		int err2 = lynx_28g_lane_reset(phy);
		/*
		 * Don't overwrite a failed protocol converter disable error
		 * code with a successful lane reset error code, but propagate
		 * a failed lane reset error.
		 */
		if (!err)
			err = err2;
	}

	return err;
}

static int lynx_28g_validate(struct phy *phy, enum phy_mode mode, int submode,
			     union phy_configure_opts *opts __always_unused)
{
	struct lynx_28g_lane *lane = phy_get_drvdata(phy);
	enum lynx_lane_mode lane_mode;

	if (mode != PHY_MODE_ETHERNET)
		return -EOPNOTSUPP;

	lane_mode = phy_interface_to_lane_mode(submode);
	if (!lynx_lane_supports_mode(lane, lane_mode))
		return -EOPNOTSUPP;

	if (lynx_lane_mode_num_lanes(lane_mode) !=
	    lynx_lane_mode_num_lanes(lane->mode))
		return -EOPNOTSUPP;

	return 0;
}

static int lynx_28g_init(struct phy *phy)
{
	struct lynx_28g_lane *lane = phy_get_drvdata(phy);

	/* Mark the fact that the lane was init */
	lane->init = true;

	/* SerDes lanes are powered on at boot time.  Any lane that is managed
	 * by this driver will get powered down at init time aka at dpaa2-eth
	 * probe time.
	 */
	lane->powered_up = true;

	return lynx_28g_power_off(phy);
}

static int lynx_28g_exit(struct phy *phy)
{
	struct lynx_28g_lane *lane = phy_get_drvdata(phy);

	/* The lane returns to the state where it isn't managed by the
	 * consumer, so we must treat is as if it isn't initialized, and always
	 * powered on.
	 */
	lane->init = false;
	lane->powered_up = false;

	return lynx_28g_power_on(phy);
}

static void lynx_28g_check_cdr_lock(struct phy *phy,
				    struct phy_status_opts_cdr *cdr)
{
	struct lynx_28g_lane *lane = phy_get_drvdata(phy);

	cdr->cdr_locked = lynx_28g_cdr_lock_check(lane);
}

static void lynx_28g_get_pcvt_count(struct phy *phy,
				    struct phy_status_opts_pcvt_count *opts)
{
	struct lynx_28g_lane *lane = phy_get_drvdata(phy);
	enum lynx_lane_mode lane_mode = lane->mode;

	switch (opts->type) {
	case PHY_PCVT_ETHERNET_PCS:
		switch (lane_mode) {
		case LANE_MODE_1000BASEX_SGMII:
		case LANE_MODE_1000BASEKX:
		case LANE_MODE_10GBASER:
		case LANE_MODE_USXGMII:
		case LANE_MODE_10GBASEKR:
		case LANE_MODE_25GBASER:
		case LANE_MODE_25GBASEKR:
		case LANE_MODE_40GBASER_XLAUI:
		case LANE_MODE_40GBASEKR4:
			opts->num_pcvt = 1;
			break;
		default:
			break;
		}
		break;
	case PHY_PCVT_ETHERNET_ANLT:
		switch (lane_mode) {
		case LANE_MODE_1000BASEKX:
		case LANE_MODE_10GBASEKR:
		case LANE_MODE_25GBASEKR:
		case LANE_MODE_40GBASEKR4:
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

static void lynx_28g_get_pcvt_addr(struct phy *phy,
				   struct phy_status_opts_pcvt *pcvt)
{
	struct lynx_28g_lane *lane = phy_get_drvdata(phy);
	enum lynx_lane_mode lane_mode = lane->mode;
	u32 cr1;

	switch (pcvt->type) {
	case PHY_PCVT_ETHERNET_PCS:
		WARN_ON(lynx_pcvt_read(lane, lane_mode, CR(1), &cr1));
		break;
	case PHY_PCVT_ETHERNET_ANLT:
		WARN_ON(lynx_anlt_read(lane, lane_mode, CR(1), &cr1));
		break;
	default:
		return;
	}

	pcvt->addr.mdio = FIELD_GET(MDEV_PORT, cr1);
}

static int lynx_28g_get_status(struct phy *phy, enum phy_status_type type,
			       union phy_status_opts *opts)
{
	switch (type) {
	case PHY_STATUS_CDR_LOCK:
		lynx_28g_check_cdr_lock(phy, &opts->cdr);
		break;
	case PHY_STATUS_PCVT_COUNT:
		lynx_28g_get_pcvt_count(phy, &opts->pcvt_count);
		break;
	case PHY_STATUS_PCVT_ADDR:
		lynx_28g_get_pcvt_addr(phy, &opts->pcvt);
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static int lynx_28g_configure(struct phy *phy, union phy_configure_opts *opts)
{
	struct lynx_28g_lane *lane = phy_get_drvdata(phy);

	return lynx_xgkr_algorithm_configure(lane->algorithm, &opts->ethernet);
}

static const struct phy_ops lynx_28g_ops = {
	.init		= lynx_28g_init,
	.exit		= lynx_28g_exit,
	.power_on	= lynx_28g_power_on,
	.power_off	= lynx_28g_power_off,
	.set_mode	= lynx_28g_set_mode,
	.validate	= lynx_28g_validate,
	.get_status	= lynx_28g_get_status,
	.configure	= lynx_28g_configure,
	.owner		= THIS_MODULE,
};

static const char *lynx_refclk_str(int refclk)
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

static const char *lynx_28g_clock_net_str(int frate)
{
	switch (frate) {
	case PLLnCR1_FRATE_5G_10GVCO:
		return "5 GHz on 10 GHz VCO";
	case PLLnCR1_FRATE_5G_25GVCO:
		return "5 GHz on 25 GHz VCO";
	case PLLnCR1_FRATE_10G_20GVCO:
		return "10.3125 GHz on 20.625 GHz VCO";
	case PLLnCR1_FRATE_12G_25GVCO:
		return "12.890625 GHz on 25.78125 GHz VCO";
	default:
		return "unknown";
	}
}

#define LYNX_28G_SUPPORT_BUF_LEN	128

static void lynx_28g_pll_dump(struct lynx_28g_pll *pll)
{
	struct lynx_28g_priv *priv = pll->priv;
	char buf[LYNX_28G_SUPPORT_BUF_LEN];
	struct device *dev = priv->dev;
	enum lynx_lane_mode mode;
	int total_len = 0, len;
	bool truncated = false;
	int i;

	dev_info(dev, "PLL%c: %s, %s, reference clock %s, clock net %s\n",
		 pll->id == 0 ? 'F' : 'S',
		 str_enabled_disabled(pll->enabled),
		 pll->locked ? "locked" : "unlocked",
		 lynx_refclk_str(pll->refclk_sel),
		 lynx_28g_clock_net_str(pll->frate_sel));

	if (!pll->enabled)
		return;

	for (mode = LANE_MODE_UNKNOWN; mode < LANE_MODE_MAX; mode++) {
		if (!test_bit(mode, pll->supported))
			continue;

		for (i = priv->info->first_lane; i < LYNX_28G_NUM_LANE; i++) {
			if (!priv->info->lane_supports_mode(i, mode))
				continue;

			len = snprintf(&buf[total_len],
				       LYNX_28G_SUPPORT_BUF_LEN - total_len,
				       " %s", lynx_lane_mode_str(mode));
			if (len >= LYNX_28G_SUPPORT_BUF_LEN - total_len)
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

static void lynx_28g_pll_read_configuration(struct lynx_28g_priv *priv)
{
	struct lynx_28g_pll *pll;
	int i, ex_dly_sel;
	u32 val;

	for (i = 0; i < LYNX_28G_NUM_PLL; i++) {
		pll = &priv->pll[i];
		pll->priv = priv;
		pll->id = i;
		spin_lock_init(&pll->lock);

		val = lynx_28g_pll_read(pll, PLLnRSTCTL);
		pll->enabled = !(val & PLLnRSTCTL_DIS);
		pll->locked = !!(val & PLLnRSTCTL_LOCK);

		val = lynx_28g_pll_read(pll, PLLnCR0);
		pll->refclk_sel = FIELD_GET(PLLnCR0_REFCLK_SEL, val);

		val = lynx_28g_pll_read(pll, PLLnCR1);
		pll->frate_sel = FIELD_GET(PLLnCR1_FRATE_SEL, val);

		if (!pll->enabled)
			continue;

		ex_dly_sel = FIELD_GET(PLLnCR1_EX_DLY_SEL, val);
		if (ex_dly_sel) {
			dev_dbg(priv->dev, "PLL%cCR1[EX_DLY_SEL] found set\n",
				pll->id == 0 ? 'F' : 'S');
			pll->ex_dly_clk_use_count = 1;
		}

		switch (pll->frate_sel) {
		case PLLnCR1_FRATE_5G_10GVCO:
		case PLLnCR1_FRATE_5G_25GVCO:
			/* 5GHz clock net */
			__set_bit(LANE_MODE_1000BASEX_SGMII, pll->supported);
			if (ex_dly_sel && ex_dly_sel != PLLnCR1_EX_DLY_SEL_312_5_MHZ) {
				dev_dbg(priv->dev,
					"PLL%c has ex_dly_clk provisioned for a frequency incompatible with 1000Base-KX\n",
					pll->id == 0 ? 'F' : 'S');
			} else {
				__set_bit(LANE_MODE_1000BASEKX, pll->supported);
			}
			break;
		case PLLnCR1_FRATE_10G_20GVCO:
			/* 10.3125GHz clock net */
			__set_bit(LANE_MODE_10GBASER, pll->supported);
			__set_bit(LANE_MODE_USXGMII, pll->supported);
			__set_bit(LANE_MODE_10GBASEKR, pll->supported);
			__set_bit(LANE_MODE_40GBASER_XLAUI, pll->supported);
			__set_bit(LANE_MODE_40GBASEKR4, pll->supported);
			break;
		case PLLnCR1_FRATE_12G_25GVCO:
			/* 12.890625GHz clock net */
			__set_bit(LANE_MODE_25GBASER, pll->supported);
			__set_bit(LANE_MODE_25GBASEKR, pll->supported);
			break;
		default:
			/* 6GHz, 8GHz */
			break;
		}
	}

	for (i = 0; i < LYNX_28G_NUM_PLL; i++)
		lynx_28g_pll_dump(&priv->pll[i]);
}

#define work_to_lynx(w) container_of((w), struct lynx_28g_priv, cdr_check.work)

static void lynx_28g_cdr_lock_check_work(struct work_struct *work)
{
	struct lynx_28g_priv *priv = work_to_lynx(work);
	struct lynx_28g_lane *lane;
	int i;

	for (i = priv->info->first_lane; i < LYNX_28G_NUM_LANE; i++) {
		lane = &priv->lane[i];
		if (!lane->phy)
			continue;

		mutex_lock(&lane->phy->mutex);

		if (!lane->init || !lane->powered_up) {
			mutex_unlock(&lane->phy->mutex);
			continue;
		}

		lynx_28g_cdr_lock_check(lane);

		mutex_unlock(&lane->phy->mutex);
	}
	queue_delayed_work(system_power_efficient_wq, &priv->cdr_check,
			   msecs_to_jiffies(1000));
}

static void lynx_28g_lane_read_configuration(struct lynx_28g_lane *lane)
{
	u32 pccr, pss, protocol;

	pss = lynx_28g_lane_read(lane, LNaPSS);
	protocol = FIELD_GET(LNaPSS_TYPE, pss);
	switch (protocol) {
	case LNaPSS_TYPE_SGMII:
		lynx_pccr_read(lane, LANE_MODE_1000BASEX_SGMII, &pccr);
		if (pccr & PCC8_SGMIIa_KX)
			lane->mode = LANE_MODE_1000BASEKX;
		else
			lane->mode = LANE_MODE_1000BASEX_SGMII;
		break;
	case LNaPSS_TYPE_XFI:
		lynx_pccr_read(lane, LANE_MODE_10GBASER, &pccr);
		if (pccr & PCCC_SXGMIIn_XFI)
			lane->mode = LANE_MODE_10GBASER;
		else
			lane->mode = LANE_MODE_USXGMII;
		break;
	case LNaPSS_TYPE_25G:
		lane->mode = LANE_MODE_25GBASER;
		break;
	case LNaPSS_TYPE_40G:
		lane->mode = LANE_MODE_40GBASER_XLAUI;
		break;
	default:
		lane->mode = LANE_MODE_UNKNOWN;
	}
}

static struct phy *lynx_28g_xlate(struct device *dev,
				  const struct of_phandle_args *args)
{
	struct lynx_28g_priv *priv = dev_get_drvdata(dev);
	int idx = args->args[0];

	if (WARN_ON(idx >= LYNX_28G_NUM_LANE ||
		    idx < priv->info->first_lane))
		return ERR_PTR(-EINVAL);

	return priv->lane[idx].phy;
}

static int lynx_28g_probe_lane(struct lynx_28g_priv *priv, int id,
			       struct device_node *dn)
{
	struct lynx_28g_lane *lane = &priv->lane[id];
	struct phy *phy;

	memset(lane, 0, sizeof(*lane));

	phy = devm_phy_create(priv->dev, dn, &lynx_28g_ops);
	if (IS_ERR(phy))
		return PTR_ERR(phy);

	lane->priv = priv;
	lane->phy = phy;
	lane->id = id;
	phy_set_drvdata(phy, lane);
	lynx_28g_lane_read_configuration(lane);

	return 0;
}

static int lynx_28g_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	bool lane_phy_providers = true;
	struct phy_provider *provider;
	struct lynx_28g_priv *priv;
	int err;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	priv->info = of_device_get_match_data(dev);
	dev_set_drvdata(dev, priv);
	spin_lock_init(&priv->pcc_lock);
	INIT_DELAYED_WORK(&priv->cdr_check, lynx_28g_cdr_lock_check_work);

	priv->lane = devm_kcalloc(dev, priv->info->num_lanes,
				  sizeof(*priv->lane), GFP_KERNEL);
	if (!priv->lane)
		return -ENOMEM;

	priv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	if (priv->info == &lynx_info_compat) {
		/*
		 * If we get here it means we probed on a device tree where
		 * "fsl,lynx-28g" wasn't the fallback, but the sole compatible
		 * string.
		 */
		dev_warn(dev, "Please update device tree to use per-device compatible strings\n");
		lane_phy_providers = false;
	}

	lynx_28g_pll_read_configuration(priv);

	if (lane_phy_providers) {
		struct device_node *dn = dev_of_node(dev), *child;

		for_each_available_child_of_node(dn, child) {
			u32 reg;

			/* PHY subnode name must be 'phy'. */
			if (!(of_node_name_eq(child, "phy")))
				continue;

			if (of_property_read_u32(child, "reg", &reg)) {
				dev_err(dev, "No \"reg\" property for %pOF\n", child);
				of_node_put(child);
				return -EINVAL;
			}

			if (reg < priv->info->first_lane || reg >= LYNX_28G_NUM_LANE) {
				dev_err(dev, "\"reg\" property out of range for %pOF\n", child);
				of_node_put(child);
				return -EINVAL;
			}

			err = lynx_28g_probe_lane(priv, reg, child);
			if (err) {
				of_node_put(child);
				return err;
			}
		}

		provider = devm_of_phy_provider_register(&pdev->dev,
							 of_phy_simple_xlate);
	} else {
		for (int i = priv->info->first_lane; i < LYNX_28G_NUM_LANE; i++) {
			err = lynx_28g_probe_lane(priv, i, NULL);
			if (err)
				return err;
		}

		provider = devm_of_phy_provider_register(&pdev->dev,
							 lynx_28g_xlate);
	}

	if (IS_ERR(provider))
		return PTR_ERR(provider);

	queue_delayed_work(system_power_efficient_wq, &priv->cdr_check,
			   msecs_to_jiffies(1000));

	return 0;
}

static void lynx_28g_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct lynx_28g_priv *priv = dev_get_drvdata(dev);

	cancel_delayed_work_sync(&priv->cdr_check);
}

static const struct of_device_id lynx_28g_of_match_table[] = {
	{ .compatible = "fsl,lx2160a-serdes1", .data = &lynx_info_lx2160a_serdes1 },
	{ .compatible = "fsl,lx2160a-serdes2", .data = &lynx_info_lx2160a_serdes2 },
	{ .compatible = "fsl,lx2160a-serdes3", .data = &lynx_info_lx2160a_serdes3 },
	{ .compatible = "fsl,lx2162a-serdes1", .data = &lynx_info_lx2162a_serdes1 },
	{ .compatible = "fsl,lx2162a-serdes2", .data = &lynx_info_lx2162a_serdes2 },
	{ .compatible = "fsl,lynx-28g", .data = &lynx_info_compat }, /* fallback, keep last */
	{ },
};
MODULE_DEVICE_TABLE(of, lynx_28g_of_match_table);

static struct platform_driver lynx_28g_driver = {
	.probe = lynx_28g_probe,
	.remove = lynx_28g_remove,
	.driver = {
		.name = "lynx-28g",
		.of_match_table = lynx_28g_of_match_table,
	},
};
module_platform_driver(lynx_28g_driver);

MODULE_AUTHOR("Ioana Ciornei <ioana.ciornei@nxp.com>");
MODULE_DESCRIPTION("Lynx 28G SerDes PHY driver for Layerscape SoCs");
MODULE_LICENSE("GPL v2");
