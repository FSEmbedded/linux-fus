// SPDX-License-Identifier: GPL-2.0
/*
 * ARM PL353 SMC driver
 *
 * Copyright (C) 2012 - 2018 Xilinx, Inc
 * Author: Punnaiah Choudary Kalluri <punnaiah@xilinx.com>
 * Author: Naga Sureshkumar Relli <nagasure@xilinx.com>
 */

#include <linux/clk.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/amba/bus.h>

/* Register definitions */
#define PL353_SMC_MEMC_STATUS_OFFS	0	/* Controller status reg, RO */
#define PL353_SMC_CFG_CLR_OFFS		0xC	/* Clear config reg, WO */
#define PL353_SMC_DIRECT_CMD_OFFS	0x10	/* Direct command reg, WO */
#define PL353_SMC_SET_CYCLES_OFFS	0x14	/* Set cycles register, WO */
#define PL353_SMC_SET_OPMODE_OFFS	0x18	/* Set opmode register, WO */
#define PL353_SMC_ECC_STATUS_OFFS	0x400	/* ECC status register */
#define PL353_SMC_ECC_MEMCFG_OFFS	0x404	/* ECC mem config reg */
#define PL353_SMC_ECC_MEMCMD1_OFFS	0x408	/* ECC mem cmd1 reg */
#define PL353_SMC_ECC_MEMCMD2_OFFS	0x40C	/* ECC mem cmd2 reg */
#define PL353_SMC_ECC_VALUE0_OFFS	0x418	/* ECC value 0 reg */

/* Controller status register specific constants */
#define PL353_SMC_MEMC_STATUS_RAW_INT_1_SHIFT	6

/* Clear configuration register specific constants */
#define PL353_SMC_CFG_CLR_INT_CLR_1	0x10
#define PL353_SMC_CFG_CLR_ECC_INT_DIS_1	0x40
#define PL353_SMC_CFG_CLR_INT_DIS_1	0x2
#define PL353_SMC_CFG_CLR_DEFAULT_MASK	(PL353_SMC_CFG_CLR_INT_CLR_1 | \
					 PL353_SMC_CFG_CLR_ECC_INT_DIS_1 | \
					 PL353_SMC_CFG_CLR_INT_DIS_1)

/* Set cycles register specific constants */
#define PL353_SMC_SET_CYCLES_T0_MASK	0xF
#define PL353_SMC_SET_CYCLES_T0_SHIFT	0
#define PL353_SMC_SET_CYCLES_T1_MASK	0xF
#define PL353_SMC_SET_CYCLES_T1_SHIFT	4
#define PL353_SMC_SET_CYCLES_T2_MASK	0x7
#define PL353_SMC_SET_CYCLES_T2_SHIFT	8
#define PL353_SMC_SET_CYCLES_T3_MASK	0x7
#define PL353_SMC_SET_CYCLES_T3_SHIFT	11
#define PL353_SMC_SET_CYCLES_T4_MASK	0x7
#define PL353_SMC_SET_CYCLES_T4_SHIFT	14
#define PL353_SMC_SET_CYCLES_T5_MASK	0x7
#define PL353_SMC_SET_CYCLES_T5_SHIFT	17
#define PL353_SMC_SET_CYCLES_T6_MASK	0xF
#define PL353_SMC_SET_CYCLES_T6_SHIFT	20

/* ECC status register specific constants */
#define PL353_SMC_ECC_STATUS_BUSY	BIT(6)
#define PL353_SMC_ECC_REG_SIZE_OFFS	4

/* ECC memory config register specific constants */
#define PL353_SMC_ECC_MEMCFG_MODE_MASK	0xC
#define PL353_SMC_ECC_MEMCFG_MODE_SHIFT	2
#define PL353_SMC_ECC_MEMCFG_PGSIZE_MASK	0x3

#define PL353_SMC_DC_UPT_NAND_REGS	((4 << 23) |	/* CS: NAND chip */ \
				 (2 << 21))	/* UpdateRegs operation */

#define PL353_NAND_ECC_CMD1	((0x80)       |	/* Write command */ \
				 (0 << 8)     |	/* Read command */ \
				 (0x30 << 16) |	/* Read End command */ \
				 (1 << 24))	/* Read End command calid */

#define PL353_NAND_ECC_CMD2	((0x85)	      |	/* Write col change cmd */ \
				 (5 << 8)     |	/* Read col change cmd */ \
				 (0xE0 << 16) |	/* Read col change end cmd */ \
				 (1 << 24)) /* Read col change end cmd valid */
#define PL353_NAND_ECC_BUSY_TIMEOUT	(1 * HZ)
/**
 * struct pl353_smc_data - Private smc driver structure
 * @memclk:		Pointer to the peripheral clock
 * @aclk:		Pointer to the AXI peripheral clock
 */
struct pl353_smc_data {
	struct clk		*memclk;
	struct clk		*aclk;
};

static int __maybe_unused pl353_smc_suspend(struct device *dev)
{
	struct pl353_smc_data *pl353_smc = dev_get_drvdata(dev);

	clk_disable(pl353_smc->memclk);
	clk_disable(pl353_smc->aclk);

	return 0;
}

static int __maybe_unused pl353_smc_resume(struct device *dev)
{
	struct pl353_smc_data *pl353_smc = dev_get_drvdata(dev);
	int ret;

	ret = clk_enable(pl353_smc->aclk);
	if (ret) {
		dev_err(dev, "Cannot enable axi domain clock.\n");
		return ret;
	}

	ret = clk_enable(pl353_smc->memclk);
	if (ret) {
		dev_err(dev, "Cannot enable memory clock.\n");
		clk_disable(pl353_smc->aclk);
		return ret;
	}

	return ret;
}

static SIMPLE_DEV_PM_OPS(pl353_smc_dev_pm_ops, pl353_smc_suspend,
			 pl353_smc_resume);

static const struct of_device_id pl353_smc_supported_children[] = {
	{
		.compatible = "cfi-flash"
	},
	{
		.compatible = "arm,pl353-nand-r2p1",
	},
	{}
};

static int pl353_smc_probe(struct amba_device *adev, const struct amba_id *id)
{
	struct device_node *of_node = adev->dev.of_node;
	const struct of_device_id *match = NULL;
	struct pl353_smc_data *pl353_smc;
	struct device_node *child;
	int err;

	pl353_smc = devm_kzalloc(&adev->dev, sizeof(*pl353_smc), GFP_KERNEL);
	if (!pl353_smc)
		return -ENOMEM;

	pl353_smc->aclk = devm_clk_get(&adev->dev, "apb_pclk");
	if (IS_ERR(pl353_smc->aclk)) {
		dev_err(&adev->dev, "aclk clock not found.\n");
		return PTR_ERR(pl353_smc->aclk);
	}

	pl353_smc->memclk = devm_clk_get(&adev->dev, "memclk");
	if (IS_ERR(pl353_smc->memclk)) {
		dev_err(&adev->dev, "memclk clock not found.\n");
		return PTR_ERR(pl353_smc->memclk);
	}

	err = clk_prepare_enable(pl353_smc->aclk);
	if (err) {
		dev_err(&adev->dev, "Unable to enable AXI clock.\n");
		return err;
	}

	err = clk_prepare_enable(pl353_smc->memclk);
	if (err) {
		dev_err(&adev->dev, "Unable to enable memory clock.\n");
		goto disable_axi_clk;
	}

	amba_set_drvdata(adev, pl353_smc);

	/* Find compatible children. Only a single child is supported */
	for_each_available_child_of_node(of_node, child) {
		match = of_match_node(pl353_smc_supported_children, child);
		if (!match) {
			dev_warn(&adev->dev, "unsupported child node\n");
			continue;
		}
		break;
	}
	if (!match) {
		err = -ENODEV;
		dev_err(&adev->dev, "no matching children\n");
		goto disable_mem_clk;
	}

	of_platform_device_create(child, NULL, &adev->dev);

	return 0;

disable_mem_clk:
	clk_disable_unprepare(pl353_smc->memclk);
disable_axi_clk:
	clk_disable_unprepare(pl353_smc->aclk);

	return err;
}

static void pl353_smc_remove(struct amba_device *adev)
{
	struct pl353_smc_data *pl353_smc = amba_get_drvdata(adev);

	clk_disable_unprepare(pl353_smc->memclk);
	clk_disable_unprepare(pl353_smc->aclk);
}

static const struct amba_id pl353_ids[] = {
	{
		.id = 0x00041353,
		.mask = 0x000fffff,
	},
	{ 0, 0 },
};
MODULE_DEVICE_TABLE(amba, pl353_ids);

static struct amba_driver pl353_smc_driver = {
	.drv = {
		.owner = THIS_MODULE,
		.name = "pl353-smc",
		.pm = &pl353_smc_dev_pm_ops,
	},
	.id_table = pl353_ids,
	.probe = pl353_smc_probe,
	.remove = pl353_smc_remove,
};

module_amba_driver(pl353_smc_driver);

MODULE_AUTHOR("Xilinx, Inc.");
MODULE_DESCRIPTION("ARM PL353 SMC Driver");
MODULE_LICENSE("GPL");
