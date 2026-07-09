/* SPDX-License-Identifier: GPL-2.0+ */
/* Copyright 2025 NXP */

#ifndef _PHY_FSL_LYNX_CORE_H
#define _PHY_FSL_LYNX_CORE_H

#include <linux/phy/phy-fsl-lynx.h>
#include <linux/phy/phy.h>
#include <linux/phy.h>
#include <linux/types.h>

struct lynx_xgkr_algorithm;

struct lynx_xgkr_tx_eq {
	u32 ratio_preq;
	u32 ratio_post1q;
	u32 adapt_eq;
	u32 amp_reduction;
};

enum lynx_bin_type {
	BIN_1,
	BIN_2,
	BIN_3,
	BIN_4,
	BIN_OFFSET,
	BIN_BLW,
	BIN_DATA_AVG,
	BIN_M1,
	BIN_LONG,
};

#define LYNX_NUM_PLL				2

struct lynx_pccr {
	int offset;
	int width;
	int shift;
};

struct lynx_info {
	int (*get_pccr)(enum lynx_lane_mode lane_mode, int lane,
		        struct lynx_pccr *pccr);
	int (*get_pcvt_offset)(int lane, enum lynx_lane_mode mode);
	bool (*lane_supports_mode)(int lane, enum lynx_lane_mode mode);
	int first_lane;
	int num_lanes;
	bool has_hardcoded_usxgmii;
	int index;
};

struct lynx_priv;

struct lynx_pll {
	struct lynx_priv *priv;
	int id;
	int refclk_sel;
	int frate_sel;
	int ex_dly_clk_use_count;
	bool enabled;
	bool locked;
	/*
	 * There are fewer PLLs than lanes. This serializes calls to
	 * lynx_10g_pll_get_ex_dly_clk() and lynx_10g_pll_put_ex_dly_clk().
	 */
	spinlock_t lock;
	DECLARE_BITMAP(supported, LANE_MODE_MAX);
};

struct lynx_lane {
	struct lynx_priv *priv;
	struct phy *phy;
	struct lynx_xgkr_algorithm *algorithm;
	bool powered_up;
	bool init;
	unsigned int id;
	enum lynx_lane_mode mode;
	u32 default_pccr[LANE_MODE_MAX];
};

struct lynx_priv {
	void __iomem *base;
	struct device *dev;
	const struct lynx_info *info;
	/* Serialize concurrent access to registers shared between lanes,
	 * like PCCn
	 */
	spinlock_t pcc_lock;
	bool big_endian;
	struct lynx_pll pll[LYNX_NUM_PLL];
	struct lynx_lane *lane;

	struct delayed_work cdr_check;
};

static inline u32 lynx_read(struct lynx_priv *priv, unsigned long off)
{
	void __iomem *reg = priv->base + off;

	if (priv->big_endian)
		return ioread32be(reg);

	return ioread32(reg);
}

static inline void lynx_write(struct lynx_priv *priv, unsigned long off, u32 val)
{
	void __iomem *reg = priv->base + off;

	if (priv->big_endian)
		return iowrite32be(val, reg);

	return iowrite32(val, reg);
}

static inline void lynx_rmw(struct lynx_priv *priv, unsigned long off, u32 val,
			    u32 mask)
{
	u32 orig, tmp;

	orig = lynx_read(priv, off);
	tmp = orig & ~mask;
	tmp |= val;
	if (orig != tmp)
		lynx_write(priv, off, tmp);
}

#define lynx_lane_rmw(lane, reg, val, mask)	\
	lynx_rmw((lane)->priv, reg(lane->id), val, mask)
#define lynx_lane_read(lane, reg)			\
	lynx_read((lane)->priv, reg((lane)->id))
#define lynx_lane_write(lane, reg, val)		\
	lynx_write((lane)->priv, reg((lane)->id), val)
#define lynx_pll_read(pll, reg)			\
	lynx_read((pll)->priv, reg((pll)->id))
#define lynx_pll_write(pll, reg, val)		\
	lynx_write((pll)->priv, reg((pll)->id), val)
#define lynx_pll_rmw(pll, reg, val, mask)	\
	lynx_rmw((pll)->priv, reg((pll)->id), val, mask)

const char *lynx_lane_mode_str(enum lynx_lane_mode lane_mode);
enum lynx_lane_mode phy_interface_to_lane_mode(phy_interface_t intf);
int lynx_lane_mode_num_lanes(enum lynx_lane_mode lane_mode);
bool lynx_lane_mode_needs_link_training(enum lynx_lane_mode mode);
bool lynx_lane_supports_mode(struct lynx_lane *lane, enum lynx_lane_mode mode);

struct lynx_pll *lynx_pll_get(struct lynx_priv *priv, enum lynx_lane_mode mode);

int lynx_pccr_read(struct lynx_lane *lane, enum lynx_lane_mode mode, u32 *val);
int lynx_pccr_write(struct lynx_lane *lane, enum lynx_lane_mode mode, u32 val);
int lynx_pcvt_read(struct lynx_lane *lane, enum lynx_lane_mode mode, int cr,
		   u32 *val);
int lynx_pcvt_write(struct lynx_lane *lane, enum lynx_lane_mode mode, int cr,
		    u32 val);
int lynx_pcvt_rmw(struct lynx_lane *lane, enum lynx_lane_mode mode, int cr,
		  u32 val, u32 mask);

struct lynx_xgkr_algorithm_ops {
	void (*read_tx_eq)(struct phy *phy, struct lynx_xgkr_tx_eq *tx_eq);
	void (*tune_tx_eq)(struct phy *phy, const struct lynx_xgkr_tx_eq *tx_eq);
	int (*snapshot_rx_eq_gains)(struct phy *phy, u8 *gaink2, u8 *gaink3,
				    u8 *eq_offset);
	int (*snapshot_rx_eq_bin)(struct phy *phy, enum lynx_bin_type bin_type,
				  s16 *snapshot);
};

struct lynx_xgkr_algorithm *
lynx_xgkr_algorithm_create(struct phy *phy,
			   const struct lynx_xgkr_algorithm_ops *ops);
void lynx_xgkr_algorithm_destroy(struct lynx_xgkr_algorithm *algorithm);
void lynx_xgkr_read_default_tx_eq(struct lynx_xgkr_algorithm *algorithm);
int lynx_xgkr_algorithm_configure(struct lynx_xgkr_algorithm *algorithm,
				  struct phy_configure_opts_ethernet *opts);

#endif
