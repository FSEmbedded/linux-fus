// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for the Texas Instruments DP83848 PHY
 *
 * Copyright (C) 2015-2016 Texas Instruments Incorporated - http://www.ti.com/
 */

#include <linux/module.h>
#include <linux/phy.h>
#include <linux/of.h>

#define TI_DP83848C_PHY_ID		0x20005ca0
#define TI_DP83620_PHY_ID		0x20005ce0
#define NS_DP83848C_PHY_ID		0x20005c90
#define TLK10X_PHY_ID			0x2000a210

/* Registers */
#define DP83848_MICR			0x11 /* MII Interrupt Control Register */
#define DP83848_MISR			0x12 /* MII Interrupt Status Register */
#define DP83848_PHYCR			0x19 /* PHY Control Register */

/* MICR Register Fields */
#define DP83848_MICR_INT_OE		BIT(0) /* Interrupt Output Enable */
#define DP83848_MICR_INTEN		BIT(1) /* Interrupt Enable */

/* MISR Register Fields */
#define DP83848_MISR_RHF_INT_EN		BIT(0) /* Receive Error Counter */
#define DP83848_MISR_FHF_INT_EN		BIT(1) /* False Carrier Counter */
#define DP83848_MISR_ANC_INT_EN		BIT(2) /* Auto-negotiation complete */
#define DP83848_MISR_DUP_INT_EN		BIT(3) /* Duplex Status */
#define DP83848_MISR_SPD_INT_EN		BIT(4) /* Speed status */
#define DP83848_MISR_LINK_INT_EN	BIT(5) /* Link status */
#define DP83848_MISR_ED_INT_EN		BIT(6) /* Energy detect */
#define DP83848_MISR_LQM_INT_EN		BIT(7) /* Link Quality Monitor */

/* PHYCR Register Fields */
#define DP83848_PHYCR_LED_CNFG		BIT(5) /* LED Configuration (0: Link+Activity, 1: Link) */
#define DP83848_PHYCR_MDIX_EN		BIT(15) /* Auto-MDIX (0: Disabled, 1: Enabled) */

#define DP83848_INT_EN_MASK		\
	(DP83848_MISR_ANC_INT_EN |	\
	 DP83848_MISR_DUP_INT_EN |	\
	 DP83848_MISR_SPD_INT_EN |	\
	 DP83848_MISR_LINK_INT_EN)

#define DP83848_MISR_RHF_INT		BIT(8)
#define DP83848_MISR_FHF_INT		BIT(9)
#define DP83848_MISR_ANC_INT		BIT(10)
#define DP83848_MISR_DUP_INT		BIT(11)
#define DP83848_MISR_SPD_INT		BIT(12)
#define DP83848_MISR_LINK_INT		BIT(13)
#define DP83848_MISR_ED_INT		BIT(14)

#define DP83848_INT_MASK		\
	(DP83848_MISR_ANC_INT |	\
	 DP83848_MISR_DUP_INT |	\
	 DP83848_MISR_SPD_INT |	\
	 DP83848_MISR_LINK_INT)

struct dp83848_priv {
	u32 quirks;
};

static int dp83848_probe(struct phy_device *phydev)
{
	struct device *dev = &phydev->mdio.dev;
	struct dp83848_priv *priv;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	if (of_property_read_bool(dev->of_node, "dp83848,no-led-activity"))
		priv->quirks |= DP83848_PHYCR_LED_CNFG;

	if (of_property_read_bool(dev->of_node, "dp83848,auto-mdix"))
		priv->quirks |= DP83848_PHYCR_MDIX_EN;

	phydev->priv = priv;

	return 0;
}

static int dp83848_ack_interrupt(struct phy_device *phydev)
{
	int err = phy_read(phydev, DP83848_MISR);

	return err < 0 ? err : 0;
}

static int dp83848_config_intr(struct phy_device *phydev)
{
	int control, ret;

	control = phy_read(phydev, DP83848_MICR);
	if (control < 0)
		return control;

	if (phydev->interrupts == PHY_INTERRUPT_ENABLED) {
		ret = dp83848_ack_interrupt(phydev);
		if (ret)
			return ret;

		control |= DP83848_MICR_INT_OE;
		control |= DP83848_MICR_INTEN;

		ret = phy_write(phydev, DP83848_MISR, DP83848_INT_EN_MASK);
		if (ret < 0)
			return ret;

		ret = phy_write(phydev, DP83848_MICR, control);
	} else {
		control &= ~DP83848_MICR_INTEN;
		ret = phy_write(phydev, DP83848_MICR, control);
		if (ret)
			return ret;

		ret = dp83848_ack_interrupt(phydev);
	}

	return ret;
}

static irqreturn_t dp83848_handle_interrupt(struct phy_device *phydev)
{
	int irq_status;

	irq_status = phy_read(phydev, DP83848_MISR);
	if (irq_status < 0) {
		phy_error(phydev);
		return IRQ_NONE;
	}

	if (!(irq_status & DP83848_INT_MASK))
		return IRQ_NONE;

	phy_trigger_machine(phydev);

	return IRQ_HANDLED;
}

static int dp83848_config_init(struct phy_device *phydev)
{
	u16 reg;
	int val;

	struct dp83848_priv *priv = phydev->priv;

	reg = phy_read(phydev, DP83848_PHYCR);

	/* If not set by the Device-Tree, disable LED Activity */
	if ((priv->quirks & DP83848_PHYCR_LED_CNFG))
		reg |= DP83848_PHYCR_LED_CNFG;
	else
		reg &= ~DP83848_PHYCR_LED_CNFG;

	/* If enabled by HW, do not disable Auto-MDIX */
	if ((priv->quirks & DP83848_PHYCR_MDIX_EN))
		reg |= DP83848_PHYCR_MDIX_EN;

	phy_write(phydev,DP83848_PHYCR, reg);

	return 0;
}

static int dp83620_config_init(struct phy_device *phydev)
{
	int val;

	/* DP83620 always reports Auto Negotiation Ability on BMSR. Instead,
	 * we check initial value of BMCR Auto negotiation enable bit
	 */
	val = phy_read(phydev, MII_BMCR);
	if (!(val & BMCR_ANENABLE))
		phydev->autoneg = AUTONEG_DISABLE;

	return 0;
}

static struct mdio_device_id __maybe_unused dp83848_tbl[] = {
	{ TI_DP83848C_PHY_ID, 0xfffffff0 },
	{ NS_DP83848C_PHY_ID, 0xfffffff0 },
	{ TI_DP83620_PHY_ID, 0xfffffff0 },
	{ TLK10X_PHY_ID, 0xfffffff0 },
	{ }
};
MODULE_DEVICE_TABLE(mdio, dp83848_tbl);

#define DP83848_PHY_DRIVER(_id, _name, _config_init, _probe)	\
	{							\
		.phy_id		= _id,				\
		.phy_id_mask	= 0xfffffff0,			\
		.name		= _name,			\
		/* PHY_BASIC_FEATURES */			\
								\
		.probe		= _probe,			\
		.soft_reset	= genphy_soft_reset,		\
		.config_init	= _config_init,			\
		.suspend	= genphy_suspend,		\
		.resume		= genphy_resume,		\
								\
		/* IRQ related */				\
		.config_intr	= dp83848_config_intr,		\
		.handle_interrupt = dp83848_handle_interrupt,	\
	}

static struct phy_driver dp83848_driver[] = {
	DP83848_PHY_DRIVER(TI_DP83848C_PHY_ID, "TI DP83848C 10/100 Mbps PHY",
			   NULL, NULL),
	DP83848_PHY_DRIVER(NS_DP83848C_PHY_ID, "NS DP83848C 10/100 Mbps PHY",
			   dp83848_config_init, dp83848_probe),
	DP83848_PHY_DRIVER(TI_DP83620_PHY_ID, "TI DP83620 10/100 Mbps PHY",
			   dp83620_config_init, NULL),
	DP83848_PHY_DRIVER(TLK10X_PHY_ID, "TI TLK10X 10/100 Mbps PHY",
			   NULL, NULL),
};
module_phy_driver(dp83848_driver);

MODULE_DESCRIPTION("Texas Instruments DP83848 PHY driver");
MODULE_AUTHOR("Andrew F. Davis <afd@ti.com>");
MODULE_LICENSE("GPL v2");
