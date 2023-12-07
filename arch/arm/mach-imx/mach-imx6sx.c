// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2014 Freescale Semiconductor, Inc.
 */

#include <linux/irqchip.h>
#include <linux/of_platform.h>
#include <linux/phy.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/imx6q-iomuxc-gpr.h>
#include <linux/micrel_phy.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include "common.h"
#include "cpuidle.h"

#if defined(CONFIG_FEC) || defined(CONFIG_FEC_MODULE)

static int dp83848_phy_fixup(struct phy_device *dev)
{
	u16 val;

	val = phy_read(dev, 0x19);
	val &= ~(0x1 << 5);
	phy_write(dev, 0x19, val);
	return 0;
}

static int ksz8081_phy_fixup(struct phy_device *dev)
{
	/* Do not use PHY address 0 for broadcast, switch LED to show link and
	   activity and activate correct clock speed */
	if (dev && dev->interface == PHY_INTERFACE_MODE_MII) {
		phy_write(dev, 0x1f, 0x8100);
		phy_write(dev, 0x16, 0x201);
	} else if (dev && dev->interface == PHY_INTERFACE_MODE_RMII) {
		phy_write(dev, 0x1f, 0x8180);
		phy_write(dev, 0x16, 0x202);
	}

	return 0;
}

static int ar8031_phy_fixup(struct phy_device *dev)
{
	u16 val;

	/* Set RGMII IO voltage to 1.8V */
	phy_write(dev, 0x1d, 0x1f);
	phy_write(dev, 0x1e, 0x8);

	/* disable phy AR8031 SmartEEE function. */
	phy_write(dev, 0xd, 0x3);
	phy_write(dev, 0xe, 0x805d);
	phy_write(dev, 0xd, 0x4003);
	val = phy_read(dev, 0xe);
	val &= ~(0x1 << 8);
	phy_write(dev, 0xe, val);

	/* introduce tx clock delay */
	phy_write(dev, 0x1d, 0x5);
	val = phy_read(dev, 0x1e);
	val |= 0x0100;
	phy_write(dev, 0x1e, val);

	return 0;
}

#define PHY_ID_DP83848 	0x20005c90
#define PHY_ID_AR8031   0x004dd074
static void __init imx6sx_enet_phy_init(void)
{
	if (IS_BUILTIN(CONFIG_PHYLIB)) {
		phy_register_fixup_for_uid(PHY_ID_DP83848, 0xfffffff0,
					   dp83848_phy_fixup);
		phy_register_fixup_for_uid(PHY_ID_KSZ8081, MICREL_PHY_ID_MASK,
				   ksz8081_phy_fixup);
		phy_register_fixup_for_uid(PHY_ID_AR8031, 0xffffffff,
					   ar8031_phy_fixup);
	}
}

static void __init imx6sx_enet_clk_sel(void)
{
	struct regmap *gpr;
	struct device_node *np;

	gpr = syscon_regmap_lookup_by_compatible("fsl,imx6sx-iomuxc-gpr");
	if (IS_ERR(gpr)) {
		pr_err("failed to find fsl,imx6sx-iomux-gpr regmap\n");
		return;
	}

	np = of_find_node_by_path("/soc/bus@2100000/ethernet@2188000");
	if (np && of_get_property(np, "fsl,ref-clock-out", NULL))
		regmap_update_bits(gpr, IOMUXC_GPR1,
				   IMX6SX_GPR1_ENET1_CLOCK_MASK,
				   IMX6SX_GPR1_ENET1_CLOCK_MASK);
	else
		regmap_update_bits(gpr, IOMUXC_GPR1,
				   IMX6SX_GPR1_ENET1_CLOCK_MASK, 0);

	np = of_find_node_by_path("/soc/bus@2100000/ethernet@21b4000");
	if (np && of_get_property(np, "fsl,ref-clock-out", NULL))
		regmap_update_bits(gpr, IOMUXC_GPR1,
				   IMX6SX_GPR1_ENET2_CLOCK_MASK,
				   IMX6SX_GPR1_ENET2_CLOCK_MASK);
	else
		regmap_update_bits(gpr, IOMUXC_GPR1,
				   IMX6SX_GPR1_ENET2_CLOCK_MASK, 0);
}

static inline void imx6sx_enet_init(void)
{
	imx6_enet_mac_init("fsl,imx6sx-fec", "fsl,imx6sx-ocotp");
	imx6sx_enet_phy_init();
	imx6sx_enet_clk_sel();
}
#endif /* CONFIG_FEC || CONFIG_FEC_MODULE */

static void __init imx6sx_init_machine(void)
{
	of_platform_default_populate(NULL, NULL, NULL);

#if defined(CONFIG_FEC) || defined(CONFIG_FEC_MODULE)
	imx6sx_enet_init();
#endif
	imx_anatop_init();
	imx6sx_pm_init();
}

static void __init imx6sx_init_irq(void)
{
	imx_gpc_check_dt();
	imx_init_revision_from_anatop();
	imx_init_l2cache();
	imx_src_init();
	irqchip_init();
	imx6_pm_ccm_init("fsl,imx6sx-ccm");
}

static void __init imx6sx_map_io(void)
{
	debug_ll_io_init();
	imx6_pm_map_io();
	imx_busfreq_map_io();
}

static void __init imx6sx_init_late(void)
{
	imx6sx_cpuidle_init();

	if (IS_ENABLED(CONFIG_ARM_IMX6Q_CPUFREQ))
		platform_device_register_simple("imx6q-cpufreq", -1, NULL, 0);
}

static const char * const imx6sx_dt_compat[] __initconst = {
	"fsl,imx6sx",
	NULL,
};

DT_MACHINE_START(IMX6SX, "Freescale i.MX6 SoloX (Device Tree)")
	.l2c_aux_val 	= 0,
	.l2c_aux_mask	= ~0,
	.map_io		= imx6sx_map_io,
	.init_irq	= imx6sx_init_irq,
	.init_machine	= imx6sx_init_machine,
	.dt_compat	= imx6sx_dt_compat,
	.init_late	= imx6sx_init_late,
MACHINE_END
