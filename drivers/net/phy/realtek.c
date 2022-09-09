/*
 * drivers/net/phy/realtek.c
 *
 * Driver for Realtek PHYs
 *
 * Author: Johnson Leung <r58129@freescale.com>
 *
 * Copyright (c) 2004 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */
#include <linux/of.h>
#include <linux/phy.h>
#include <linux/module.h>

#define RTL821x_PHYSR		0x11
#define RTL821x_PHYSR_DUPLEX	0x2000
#define RTL821x_PHYSR_SPEED	0xc000
#define RTL821x_INER		0x12
#define RTL821x_INER_INIT	0x6400
#define RTL821x_INSR		0x13
#define RTL8211E_INER_LINK_STATUS 0x400

#define RTL8211F_INER_LINK_STATUS 0x0010
#define RTL8211F_LCR		0x10
#define RTL8211F_PHYCR1		0x18
#define RTL8211F_PHYCR2		0x19
#define RTL8211F_INSR		0x1d
#define RTL8211F_PAGE_SELECT	0x1f
#define RTL8211F_TX_DELAY	0x100
#define RTL8211F_RX_DELAY	0x8

#define RTL8211F_SSC_CLKOUT 0x3080
#define RTL8211F_SSC_SYSCLK 0x0008

#define RTL8211F_ALDPS_PLL_OFF			(1 << 1)
#define RTL8211F_ALDPS_ENABLE			(1 << 2)
#define RTL8211F_ALDPS_XTAL_OFF			(1 << 12)

#define RTL8211F_CLKOUT_EN			(1 << 0)

#define RTL821X_CLKOUT_DISABLE		(1 << 0)
#define RTL821X_ALDPS_ENABLE			(1 << 1)
#define RTL821X_SSC_RXC_EN_FEATURE		(1 << 2)
#define RTL821X_SSC_SYSCLK_EN_FEATURE		(1 << 3)
#define RTL821X_SSC_CLKOUT_EN_FEATURE		(1 << 4)

#define LED_MODE_B (1 << 15)
#define LED_LINK(X) (0x0b << (5*X))
#define LED_ACT(X) (0x10 << (5*X))
#define LED_LINK_MASK (LED_LINK(2)|LED_LINK(1)|LED_LINK(0))
#define LED_ACT_MASK (LED_ACT(2)|LED_ACT(1)|LED_ACT(0))

MODULE_DESCRIPTION("Realtek PHY driver");
MODULE_AUTHOR("Johnson Leung");
MODULE_LICENSE("GPL");

struct rtl821x_priv {
	u32 quirks;
	u32 led_link;
	u32 led_act;
	bool set_link;
	bool set_act;
};

static int rtl821x_probe(struct phy_device *phydev)
{
	struct device *dev = &phydev->mdio.dev;
	struct rtl821x_priv *priv;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	/* Set default values */
	priv->set_link = false;
	priv->set_act = false;

	if (of_property_read_bool(dev->of_node, "rtl821x,clkout-disable"))
		priv->quirks |= RTL821X_CLKOUT_DISABLE;

	if (of_property_read_bool(dev->of_node, "rtl821x,aldps-enable"))
		priv->quirks |= RTL821X_ALDPS_ENABLE;

	if (of_property_read_bool(dev->of_node, "rtl821x,ssc-rxc-enable"))
		priv->quirks |= RTL821X_SSC_RXC_EN_FEATURE;

	if (of_property_read_bool(dev->of_node, "rtl821x,ssc-sysclk-enable"))
		priv->quirks |= RTL821X_SSC_SYSCLK_EN_FEATURE;

	if (of_property_read_bool(dev->of_node, "rtl821x,ssc-clkout-enable"))
		priv->quirks |= RTL821X_SSC_CLKOUT_EN_FEATURE;

	if (!of_property_read_u32(dev->of_node, "rtl821x,led-link", &priv->led_link))
		priv->set_link = true;

	if (!of_property_read_u32(dev->of_node, "rtl821x,led-act", &priv->led_act))
		priv->set_act = true;

	phydev->priv = priv;

	return 0;
}

static int rtl821x_ack_interrupt(struct phy_device *phydev)
{
	int err;

	err = phy_read(phydev, RTL821x_INSR);

	return (err < 0) ? err : 0;
}

static int rtl8211f_ack_interrupt(struct phy_device *phydev)
{
	int err;

	phy_write(phydev, RTL8211F_PAGE_SELECT, 0xa43);
	err = phy_read(phydev, RTL8211F_INSR);
	/* restore to default page 0 */
	phy_write(phydev, RTL8211F_PAGE_SELECT, 0x0);

	return (err < 0) ? err : 0;
}

static int rtl8211b_config_intr(struct phy_device *phydev)
{
	int err;

	if (phydev->interrupts == PHY_INTERRUPT_ENABLED)
		err = phy_write(phydev, RTL821x_INER,
				RTL821x_INER_INIT);
	else
		err = phy_write(phydev, RTL821x_INER, 0);

	return err;
}

static int rtl8211e_config_intr(struct phy_device *phydev)
{
	int err;

	if (phydev->interrupts == PHY_INTERRUPT_ENABLED)
		err = phy_write(phydev, RTL821x_INER,
				RTL8211E_INER_LINK_STATUS);
	else
		err = phy_write(phydev, RTL821x_INER, 0);

	return err;
}

static int rtl8211f_config_intr(struct phy_device *phydev)
{
	int err;

	if (phydev->interrupts == PHY_INTERRUPT_ENABLED)
		err = phy_write(phydev, RTL821x_INER,
				RTL8211F_INER_LINK_STATUS);
	else
		err = phy_write(phydev, RTL821x_INER, 0);

	return err;
}

static int rtl8211f_config_init(struct phy_device *phydev)
{
	int ret;
	u16 reg;
	struct rtl821x_priv *priv = phydev->priv;

	ret = genphy_config_init(phydev);
	if (ret < 0)
		return ret;

	phy_write(phydev, RTL8211F_PAGE_SELECT, 0xd08);
	reg = phy_read(phydev, 0x11);

	/* enable TX-delay for rgmii-id and rgmii-txid, otherwise disable it */
	if (phydev->interface == PHY_INTERFACE_MODE_RGMII_ID ||
	    phydev->interface == PHY_INTERFACE_MODE_RGMII_TXID)
		reg |= RTL8211F_TX_DELAY;
	else
		reg &= ~RTL8211F_TX_DELAY;

	phy_write(phydev, 0x11, reg);
	reg = phy_read(phydev, 0x15);

	/* enable RX-delay for rgmii-id and rgmii-rxid, otherwise disable it */
	if (phydev->interface == PHY_INTERFACE_MODE_RGMII_ID ||
	    phydev->interface == PHY_INTERFACE_MODE_RGMII_RXID)
		reg |= RTL8211F_RX_DELAY;
	else
		reg &= ~RTL8211F_RX_DELAY;

	phy_write(phydev, 0x15, reg);

	phy_write(phydev, RTL8211F_PAGE_SELECT, 0xa43);
	reg = phy_read(phydev, RTL8211F_PHYCR1);

	if ((priv->quirks & RTL821X_ALDPS_ENABLE))
		reg |= RTL8211F_ALDPS_PLL_OFF | RTL8211F_ALDPS_ENABLE | RTL8211F_ALDPS_XTAL_OFF;

	phy_write(phydev, RTL8211F_PHYCR1, reg);
	reg = phy_read(phydev, RTL8211F_PHYCR2);

	if ((priv->quirks & RTL821X_CLKOUT_DISABLE))
		reg &= ~RTL8211F_CLKOUT_EN;

	if ((priv->quirks & RTL821X_SSC_RXC_EN_FEATURE)) {
		phy_write(phydev, RTL8211F_PAGE_SELECT, 0x0C44);
		phy_write(phydev, 0x13, 0x5F00); // RXC SSC initialization & enable RXC SSC
	}

	if ((priv->quirks & RTL821X_SSC_SYSCLK_EN_FEATURE)) {
		phy_write(phydev, RTL8211F_PAGE_SELECT, 0x0C44);
		phy_write(phydev, 0x17, 0x5F00); // System Clock SSC initialization
		reg |= RTL8211F_SSC_SYSCLK;
	}

	if ((priv->quirks & RTL821X_SSC_CLKOUT_EN_FEATURE)) {
		phy_write(phydev, RTL8211F_PAGE_SELECT, 0x0D09);
		phy_write(phydev, 0x10, 0xCF00); // CLK_OUT SSC initialization
		reg |= RTL8211F_SSC_CLKOUT;
	}

	phy_write(phydev, RTL8211F_PAGE_SELECT, 0xa43);
	phy_write(phydev, RTL8211F_PHYCR2, reg);

	/* Set LED Configuration Register */
	phy_write(phydev, RTL8211F_PAGE_SELECT, 0xd04);
	reg = phy_read(phydev, RTL8211F_LCR);
	/* Default to LED Mode B */
	reg |= LED_MODE_B;
	/* Set LED for link indication if specified */
	if (priv->set_link) {
		reg &= ~LED_LINK_MASK;
		reg |= LED_LINK(priv->led_link);
	}
	/* Set LED for activity if specified */
	if (priv->set_act) {
		reg &= ~LED_ACT_MASK;
		reg |= LED_ACT(priv->led_act);
	}
	/* Write the actual register */
	phy_write(phydev, RTL8211F_LCR, reg);

	/* restore to default page 0 */
	phy_write(phydev, RTL8211F_PAGE_SELECT, 0x0);

	return genphy_soft_reset(phydev);
}

static struct phy_driver realtek_drvs[] = {
	{
		.phy_id         = 0x00008201,
		.name           = "RTL8201CP Ethernet",
		.phy_id_mask    = 0x0000ffff,
		.features       = PHY_BASIC_FEATURES,
		.flags          = PHY_HAS_INTERRUPT,
		.config_aneg    = &genphy_config_aneg,
		.read_status    = &genphy_read_status,
	}, {
		.phy_id		= 0x001cc912,
		.name		= "RTL8211B Gigabit Ethernet",
		.phy_id_mask	= 0x001fffff,
		.features	= PHY_GBIT_FEATURES,
		.flags		= PHY_HAS_INTERRUPT,
		.config_aneg	= &genphy_config_aneg,
		.read_status	= &genphy_read_status,
		.ack_interrupt	= &rtl821x_ack_interrupt,
		.config_intr	= &rtl8211b_config_intr,
	}, {
		.phy_id		= 0x001cc914,
		.name		= "RTL8211DN Gigabit Ethernet",
		.phy_id_mask	= 0x001fffff,
		.features	= PHY_GBIT_FEATURES,
		.flags		= PHY_HAS_INTERRUPT,
		.config_aneg	= genphy_config_aneg,
		.read_status	= genphy_read_status,
		.ack_interrupt	= rtl821x_ack_interrupt,
		.config_intr	= rtl8211e_config_intr,
		.suspend	= genphy_suspend,
		.resume		= genphy_resume,
	}, {
		.phy_id		= 0x001cc915,
		.name		= "RTL8211E Gigabit Ethernet",
		.phy_id_mask	= 0x001fffff,
		.features	= PHY_GBIT_FEATURES,
		.flags		= PHY_HAS_INTERRUPT,
		.config_aneg	= &genphy_config_aneg,
		.read_status	= &genphy_read_status,
		.ack_interrupt	= &rtl821x_ack_interrupt,
		.config_intr	= &rtl8211e_config_intr,
		.suspend	= genphy_suspend,
		.resume		= genphy_resume,
	}, {
		.phy_id		= 0x001cc916,
		.name		= "RTL8211F Gigabit Ethernet",
		.phy_id_mask	= 0x001fffff,
		.features	= PHY_GBIT_FEATURES,
		.flags		= PHY_HAS_INTERRUPT,
		.probe		= rtl821x_probe,
		.config_aneg	= &genphy_config_aneg,
		.config_init	= &rtl8211f_config_init,
		.read_status	= &genphy_read_status,
		.ack_interrupt	= &rtl8211f_ack_interrupt,
		.config_intr	= &rtl8211f_config_intr,
		.suspend	= genphy_suspend,
		.resume		= genphy_resume,
	},
};

module_phy_driver(realtek_drvs);

static struct mdio_device_id __maybe_unused realtek_tbl[] = {
	{ 0x001cc912, 0x001fffff },
	{ 0x001cc914, 0x001fffff },
	{ 0x001cc915, 0x001fffff },
	{ 0x001cc916, 0x001fffff },
	{ }
};

MODULE_DEVICE_TABLE(mdio, realtek_tbl);
