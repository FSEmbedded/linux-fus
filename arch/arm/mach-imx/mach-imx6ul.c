// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2015 Freescale Semiconductor, Inc.
 */
#include <linux/irqchip.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/imx6q-iomuxc-gpr.h>
#include <linux/micrel_phy.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/phy.h>
#include <linux/pm_opp.h>
#include <linux/regmap.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include "common.h"
#include "cpuidle.h"
#include "hardware.h"

#if defined(CONFIG_FEC) || defined(CONFIG_FEC_MODULE)
static void __init imx6ul_enet_clk_init(void)
{
	struct regmap *gpr;
	struct device_node *np;

	gpr = syscon_regmap_lookup_by_compatible("fsl,imx6ul-iomuxc-gpr");
	if (IS_ERR(gpr)) {
		pr_err("failed to find fsl,imx6ul-iomux-gpr regmap\n");
		return;
	}

	np = of_find_node_by_path("/soc/aips-bus@2100000/ethernet@2188000");
	if (np && of_get_property(np, "fsl,ref-clock-out", NULL))
		regmap_update_bits(gpr, IOMUXC_GPR1,
				   IMX6UL_GPR1_ENET1_CLK_OUTPUT,
				   IMX6UL_GPR1_ENET1_CLK_OUTPUT);
	else
		regmap_update_bits(gpr, IOMUXC_GPR1,
				   IMX6UL_GPR1_ENET1_CLK_OUTPUT, 0);

	np = of_find_node_by_path("/soc/aips-bus@2000000/ethernet@20b4000");
	if (np && of_get_property(np, "fsl,ref-clock-out", NULL))
		regmap_update_bits(gpr, IOMUXC_GPR1,
				   IMX6UL_GPR1_ENET2_CLK_OUTPUT,
				   IMX6UL_GPR1_ENET2_CLK_OUTPUT);
	else
		regmap_update_bits(gpr, IOMUXC_GPR1,
				   IMX6UL_GPR1_ENET2_CLK_OUTPUT, 0);
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

static void __init imx6ul_enet_phy_init(void)
{
	phy_register_fixup_for_uid(PHY_ID_KSZ8081, MICREL_PHY_ID_MASK,
				   ksz8081_phy_fixup);
}

static inline void imx6ul_enet_init(void)
{
	imx6ul_enet_clk_init();
	imx6ul_enet_phy_init();
	if (cpu_is_imx6ul())
		imx6_enet_mac_init("fsl,imx6ul-fec", "fsl,imx6ul-ocotp");
	else
		imx6_enet_mac_init("fsl,imx6ul-fec", "fsl,imx6ull-ocotp");
}
#endif /* CONFIG_FEC || CONFIG_FEC_MODULE */

#if defined(CONFIG_SND_SOC_FSL_SAI) || defined(CONFIG_SND_SOC_FSL_SAI_MODULE)
static void imx6ul_sai_init(void)
{
	struct regmap *gpr;
	struct device_node *np;

	gpr = syscon_regmap_lookup_by_compatible("fsl,imx6ul-iomuxc-gpr");
	if (IS_ERR(gpr)) {
		pr_err("failed to find fsl,imx6ul-iomux-gpr regmap\n");
		return;
	}

	/* Set MCLK direction depending on fsl,mclk-out property */
	np = of_find_node_by_path("/soc/aips-bus@2000000/spba-bus@2000000/sai@2028000");
	if (np && of_get_property(np, "fsl,mclk-out", NULL))
		regmap_update_bits(gpr, IOMUXC_GPR1,
				IMX6UL_GPR1_SAI1_MCLK_DIR, IMX6UL_GPR1_SAI1_MCLK_DIR);
	else
		regmap_update_bits(gpr, IOMUXC_GPR1,
				IMX6UL_GPR1_SAI1_MCLK_DIR, 0);

	np = of_find_node_by_path("/soc/aips-bus@2000000/spba-bus@2000000/sai@202c000");
	if (np && of_get_property(np, "fsl,mclk-out", NULL))
		regmap_update_bits(gpr, IOMUXC_GPR1,
				IMX6UL_GPR1_SAI2_MCLK_DIR, IMX6UL_GPR1_SAI2_MCLK_DIR);
	else
		regmap_update_bits(gpr, IOMUXC_GPR1,
				IMX6UL_GPR1_SAI2_MCLK_DIR, 0);

	np = of_find_node_by_path("/soc/aips-bus@2000000/spba-bus@2000000/sai@2030000");
	if (np && of_get_property(np, "fsl,mclk-out", NULL))
		regmap_update_bits(gpr, IOMUXC_GPR1,
				IMX6UL_GPR1_SAI3_MCLK_DIR, IMX6UL_GPR1_SAI3_MCLK_DIR);
	else
		regmap_update_bits(gpr, IOMUXC_GPR1,
				IMX6UL_GPR1_SAI3_MCLK_DIR, 0);
}
#endif

static void __init imx6ul_init_machine(void)
{
	imx_print_silicon_rev(cpu_is_imx6ull() ? "i.MX6ULL" : "i.MX6UL",
		imx_get_soc_revision());

	of_platform_default_populate(NULL, NULL, NULL);
#if defined(CONFIG_FEC) || defined(CONFIG_FEC_MODULE)
	imx6ul_enet_init();
#endif
#if defined(CONFIG_SND_SOC_FSL_SAI) || defined(CONFIG_SND_SOC_FSL_SAI_MODULE)
	imx6ul_sai_init();
#endif
	imx_anatop_init();
	imx6ul_pm_init();
}

static void __init imx6ul_init_irq(void)
{
	imx_init_revision_from_anatop();
	imx_src_init();
	irqchip_init();
	imx6_pm_ccm_init("fsl,imx6ul-ccm");
}

static void __init imx6ul_init_late(void)
{
	imx6ul_cpuidle_init();

	if (IS_ENABLED(CONFIG_ARM_IMX6Q_CPUFREQ))
		platform_device_register_simple("imx6q-cpufreq", -1, NULL, 0);
}

static void __init imx6ul_map_io(void)
{
	imx6_pm_map_io();
	imx_busfreq_map_io();
}

static const char * const imx6ul_dt_compat[] __initconst = {
	"fsl,imx6ul",
	"fsl,imx6ull",
	NULL,
};

DT_MACHINE_START(IMX6UL, "Freescale i.MX6 Ultralite (Device Tree)")
	.map_io		= imx6ul_map_io,
	.init_irq	= imx6ul_init_irq,
	.init_machine	= imx6ul_init_machine,
	.init_late	= imx6ul_init_late,
	.dt_compat	= imx6ul_dt_compat,
MACHINE_END
