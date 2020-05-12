/* * CAAM control-plane driver backend
 * Controller-level driver, kernel property detection, initialization
 *
 * Copyright 2008-2016 Freescale Semiconductor, Inc.
 * Copyright 2017-2019 NXP
 */

#include <linux/device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/sys_soc.h>

#include "compat.h"
#include "regs.h"
#include "intern.h"
#include "jr.h"
#include "desc_constr.h"
#include "ctrl.h"
#include "sm.h"

bool caam_little_end;
EXPORT_SYMBOL(caam_little_end);
bool caam_dpaa2;
EXPORT_SYMBOL(caam_dpaa2);
bool caam_imx;
EXPORT_SYMBOL(caam_imx);

#ifdef CONFIG_CAAM_QI
#include "qi.h"
#endif

/* Forward declarations of the functions in order of appearance */
static inline struct clk *caam_drv_identify_clk(struct device *dev,
						char *clk_name);
static int caam_remove(struct platform_device *pdev);
static void detect_era(struct caam_drv_private *ctrlpriv);
static void handle_imx6_err005766(struct caam_drv_private *ctrlpriv);
static int init_clocks(struct caam_drv_private *ctrlpriv);
static int caam_probe(struct platform_device *pdev);
static void check_virt(struct caam_drv_private *ctrlpriv, u32 comp_params);
static int enable_jobrings(struct caam_drv_private *ctrlpriv, int block_offset);
static void enable_qi(struct caam_drv_private *ctrlpriv, int block_offset);
static int probe_w_seco(struct caam_drv_private *ctrlpriv);
static void init_debugfs(struct caam_drv_private *ctrlpriv);
static void caam_ctrl_hw_configuration(struct caam_drv_private *ctrlpriv);
static void enable_virt(struct caam_drv_private *ctrlpriv);

#ifdef CONFIG_PM_SLEEP
static int caam_off_during_pm(void);
#endif

/*
 * i.MX targets tend to have clock control subsystems that can
 * enable/disable clocking to our device.
 */
static inline struct clk *caam_drv_identify_clk(struct device *dev,
						char *clk_name)
{
	return caam_imx ? devm_clk_get(dev, clk_name) : NULL;
}

static int caam_remove(struct platform_device *pdev)
{
	struct device *ctrldev;
	struct caam_drv_private *ctrlpriv;
	struct caam_ctrl __iomem *ctrl;

	ctrldev = &pdev->dev;
	ctrlpriv = dev_get_drvdata(ctrldev);
	ctrl = (struct caam_ctrl __iomem *)ctrlpriv->ctrl;

	/* Remove platform devices under the crypto node */
	of_platform_depopulate(ctrldev);

#ifdef CONFIG_CAAM_QI
	if (ctrlpriv->qidev)
		caam_qi_shutdown(ctrlpriv->qidev);
#endif

	/* Shut down debug views */
#ifdef CONFIG_DEBUG_FS
	debugfs_remove_recursive(ctrlpriv->dfs_root);
#endif

	/* Unmap controller region */
	iounmap(ctrl);

	/* shut clocks off before finalizing shutdown */
	if (!of_machine_is_compatible("fsl,imx8mn") &&
	    !of_machine_is_compatible("fsl,imx8mm") &&
	    !of_machine_is_compatible("fsl,imx8mq") &&
	    !of_machine_is_compatible("fsl,imx8qm") &&
	    !of_machine_is_compatible("fsl,imx8qxp")) {
		clk_disable_unprepare(ctrlpriv->caam_ipg);
		clk_disable_unprepare(ctrlpriv->caam_aclk);
		if (ctrlpriv->caam_mem)
			clk_disable_unprepare(ctrlpriv->caam_mem);
		if (ctrlpriv->caam_emi_slow)
			clk_disable_unprepare(ctrlpriv->caam_emi_slow);
	}

	return 0;
}

static void detect_era(struct caam_drv_private *ctrlpriv)
{
	int ret, i;
	u32 caam_era;
	u32 caam_id_ms;
	char *era_source;
	struct device_node *caam_node;
	struct sec_vid sec_vid;
	struct device *dev = ctrlpriv->dev;
	static const struct {
		u16 ip_id;
		u8 maj_rev;
		u8 era;
	} caam_eras[] = {
		{0x0A10, 1, 1},
		{0x0A10, 2, 2},
		{0x0A12, 1, 3},
		{0x0A14, 1, 3},
		{0x0A10, 3, 4},
		{0x0A11, 1, 4},
		{0x0A14, 2, 4},
		{0x0A16, 1, 4},
		{0x0A18, 1, 4},
		{0x0A11, 2, 5},
		{0x0A12, 2, 5},
		{0x0A13, 1, 5},
		{0x0A1C, 1, 5},
		{0x0A12, 4, 6},
		{0x0A13, 2, 6},
		{0x0A16, 2, 6},
		{0x0A17, 1, 6},
		{0x0A18, 2, 6},
		{0x0A1A, 1, 6},
		{0x0A1C, 2, 6},
		{0x0A14, 3, 7},
		{0x0A10, 4, 8},
		{0x0A11, 3, 8},
		{0x0A11, 4, 8},
		{0x0A12, 5, 8},
		{0x0A16, 3, 8},
	};

	/* If the user or bootloader has set the property we'll use that */
	caam_node = of_find_compatible_node(NULL, NULL, "fsl,sec-v4.0");
	ret = of_property_read_u32(caam_node, "fsl,sec-era", &caam_era);
	of_node_put(caam_node);

	if (!ret) {
		era_source = "device tree";
		goto era_found;
	}

	if (ctrlpriv->has_seco)
		caam_era = rd_reg32(&ctrlpriv->jr[0]->perfmon.ccb_id);
	else
		caam_era = rd_reg32(&ctrlpriv->ctrl->perfmon.ccb_id);

	caam_era = caam_era >> CCB_VID_ERA_SHIFT & CCB_VID_ERA_MASK;
	if (caam_era) {
		era_source = "CCBVID";
		goto era_found;
	}

	/* If we can match caamvid to known versions, use that */
	if (ctrlpriv->has_seco)
		caam_id_ms = rd_reg32(&ctrlpriv->jr[0]->perfmon.caam_id_ms);
	else
		caam_id_ms = rd_reg32(&ctrlpriv->ctrl->perfmon.caam_id_ms);
	sec_vid.ip_id = caam_id_ms >> SEC_VID_IPID_SHIFT;
	sec_vid.maj_rev = (caam_id_ms & SEC_VID_MAJ_MASK) >> SEC_VID_MAJ_SHIFT;

	for (i = 0; i < ARRAY_SIZE(caam_eras); i++)
		if (caam_eras[i].ip_id == sec_vid.ip_id &&
		    caam_eras[i].maj_rev == sec_vid.maj_rev) {
			caam_era = caam_eras[i].era;
			era_source = "CAAMVID";
			goto era_found;
		}

	ctrlpriv->era = -ENOTSUPP;
	dev_info(dev, "ERA undetermined!.\n");
	return;

era_found:
	ctrlpriv->era = caam_era;
	dev_info(dev, "ERA source: %s.\n", era_source);
}

static void handle_imx6_err005766(struct caam_drv_private *ctrlpriv)
{
	/*
	 * ERRATA:  mx6 devices have an issue wherein AXI bus transactions
	 * may not occur in the correct order. This isn't a problem running
	 * single descriptors, but can be if running multiple concurrent
	 * descriptors. Reworking the driver to throttle to single requests
	 * is impractical, thus the workaround is to limit the AXI pipeline
	 * to a depth of 1 (from it's default of 4) to preclude this situation
	 * from occurring.
	 */

	u32 mcr_val;

	if (ctrlpriv->era != IMX_ERR005766_ERA)
		return;

	if (of_machine_is_compatible("fsl,imx6q") ||
	    of_machine_is_compatible("fsl,imx6dl") ||
	    of_machine_is_compatible("fsl,imx6qp")) {
		dev_info(&ctrlpriv->pdev->dev,
			 "AXI pipeline throttling enabled.\n");
		mcr_val = rd_reg32(&ctrlpriv->ctrl->mcr);
		wr_reg32(&ctrlpriv->ctrl->mcr,
			 (mcr_val & ~(MCFGR_AXIPIPE_MASK)) |
			 ((1 << MCFGR_AXIPIPE_SHIFT) & MCFGR_AXIPIPE_MASK));
	}
}

static int init_clocks(struct caam_drv_private *ctrlpriv)
{
	struct clk *clk;
	struct device *dev = ctrlpriv->dev;
	int ret = 0;

	for (sh_idx = 0; sh_idx < RNG4_MAX_HANDLES; sh_idx++) {
		/*
		 * If the corresponding bit is set, this state handle
		 * was initialized by somebody else, so it's left alone.
		 */
		if ((1 << sh_idx) & state_handle_mask)
			continue;

		/* Create the descriptor for instantiating RNG State Handle */
		build_instantiation_desc(desc, sh_idx, gen_sk);

		/* Try to run it through DECO0 */
		ret = run_descriptor_deco0(ctrldev, desc, &status);

		/*
		 * If ret is not 0, or descriptor status is not 0, then
		 * something went wrong. No need to try the next state
		 * handle (if available), bail out here.
		 * Also, if for some reason, the State Handle didn't get
		 * instantiated although the descriptor has finished
		 * without any error (HW optimizations for later
		 * CAAM eras), then try again.
		 */
		if (ret)
			break;

		rdsta_val = rd_reg32(&ctrl->r4tst[0].rdsta) & RDSTA_IFMASK;
		if ((status && status != JRSTA_SSRC_JUMP_HALT_CC) ||
		    !(rdsta_val & (1 << sh_idx))) {
			ret = -EAGAIN;
			break;
		}

		dev_info(ctrldev, "Instantiated RNG4 SH%d\n", sh_idx);
		/* Clear the contents before recreating the descriptor */
		memset(desc, 0x00, CAAM_CMD_SZ * 7);
	}
	ctrlpriv->caam_ipg = clk;

	ret = clk_prepare_enable(ctrlpriv->caam_ipg);
	if (ret < 0) {
		dev_err(dev, "can't enable CAAM ipg clock: %d\n", ret);
		goto exit;
	}

	clk = caam_drv_identify_clk(dev, "aclk");
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		dev_err(dev, "can't identify CAAM aclk clk: %d\n", ret);
		goto disable_caam_ipg;
	}
	ctrlpriv->caam_aclk = clk;

	ret = clk_prepare_enable(ctrlpriv->caam_aclk);
	if (ret < 0) {
		dev_err(dev, "can't enable CAAM aclk clock: %d\n", ret);
		goto disable_caam_ipg;
	}

	if (!(of_find_compatible_node(NULL, NULL, "fsl,imx7d-caam"))) {
		clk = caam_drv_identify_clk(dev, "mem");
		if (IS_ERR(clk)) {
			ret = PTR_ERR(clk);
			dev_err(dev, "can't identify CAAM mem clk: %d\n", ret);
			goto disable_caam_aclk;
		}
		ctrlpriv->caam_mem = clk;

		ret = clk_prepare_enable(ctrlpriv->caam_mem);
		if (ret < 0) {
			dev_err(dev, "can't enable CAAM secure mem clock: %d\n",
				ret);
			goto disable_caam_aclk;
		}

		if (!(of_find_compatible_node(NULL, NULL, "fsl,imx6ul-caam"))) {
			clk = caam_drv_identify_clk(dev, "emi_slow");
			if (IS_ERR(clk)) {
				ret = PTR_ERR(clk);
				dev_err(dev,
					"can't identify CAAM emi_slow clk: %d\n",
					ret);
				goto disable_caam_mem;
			}
			ctrlpriv->caam_emi_slow = clk;

			ret = clk_prepare_enable(ctrlpriv->caam_emi_slow);
			if (ret < 0) {
				dev_err(dev,
					"can't enable CAAM emi slow clock: %d\n",
					ret);
				goto disable_caam_mem;
			}
		}
	}

	goto exit;

	ctrldev = &pdev->dev;
	ctrlpriv = dev_get_drvdata(ctrldev);
	ctrl = (struct caam_ctrl __iomem *)ctrlpriv->ctrl;

	/* Remove platform devices under the crypto node */
	of_platform_depopulate(ctrldev);

#ifdef CONFIG_CAAM_QI
	if (ctrlpriv->qidev)
		caam_qi_shutdown(ctrlpriv->qidev);
#endif

	/*
	 * De-initialize RNG state handles initialized by this driver.
	 * In case of SoCs with Management Complex, RNG is managed by MC f/w.
	 */
	if (!ctrlpriv->mc_en && ctrlpriv->rng4_sh_init)
		deinstantiate_rng(ctrldev, ctrlpriv->rng4_sh_init);

	/* Shut down debug views */
#ifdef CONFIG_DEBUG_FS
	debugfs_remove_recursive(ctrlpriv->dfs_root);
#endif

	/* Unmap controller region */
	iounmap(ctrl);

	/* shut clocks off before finalizing shutdown */
	clk_disable_unprepare(ctrlpriv->caam_ipg);
	if (ctrlpriv->caam_mem)
		clk_disable_unprepare(ctrlpriv->caam_mem);
	clk_disable_unprepare(ctrlpriv->caam_aclk);
disable_caam_ipg:
	clk_disable_unprepare(ctrlpriv->caam_ipg);
exit:
	return ret;
}

static void caam_ctrl_hw_configuration(struct caam_drv_private *ctrlpriv)
{
	/*
	 * Enable DECO watchdogs and, if this is a PHYS_ADDR_T_64BIT kernel,
	 * long pointers in master configuration register.
	 * In case of DPAA 2.x, Management Complex firmware performs
	 * the configuration.
	 */
	clrsetbits_32(&r4tst->rtmctl, RTMCTL_PRGM, RTMCTL_SAMP_MODE_RAW_ES_SC);
}

static int caam_get_era_from_hw(struct caam_ctrl __iomem *ctrl)
{
	static const struct {
		u16 ip_id;
		u8 maj_rev;
		u8 era;
	} id[] = {
		{0x0A10, 1, 1},
		{0x0A10, 2, 2},
		{0x0A12, 1, 3},
		{0x0A14, 1, 3},
		{0x0A14, 2, 4},
		{0x0A16, 1, 4},
		{0x0A10, 3, 4},
		{0x0A11, 1, 4},
		{0x0A18, 1, 4},
		{0x0A11, 2, 5},
		{0x0A12, 2, 5},
		{0x0A13, 1, 5},
		{0x0A1C, 1, 5}
	};
	u32 ccbvid, id_ms;
	u8 maj_rev, era;
	u16 ip_id;
	int i;

	ccbvid = rd_reg32(&ctrl->perfmon.ccb_id);
	era = (ccbvid & CCBVID_ERA_MASK) >> CCBVID_ERA_SHIFT;
	if (era)	/* This is '0' prior to CAAM ERA-6 */
		return era;

	id_ms = rd_reg32(&ctrl->perfmon.caam_id_ms);
	ip_id = (id_ms & SECVID_MS_IPID_MASK) >> SECVID_MS_IPID_SHIFT;
	maj_rev = (id_ms & SECVID_MS_MAJ_REV_MASK) >> SECVID_MS_MAJ_REV_SHIFT;

	for (i = 0; i < ARRAY_SIZE(id); i++)
		if (id[i].ip_id == ip_id && id[i].maj_rev == maj_rev)
			return id[i].era;

	return -ENOTSUPP;
}

/**
 * caam_get_era() - Return the ERA of the SEC on SoC, based
 * on "sec-era" optional property in the DTS. This property is updated
 * by u-boot.
 * In case this property is not passed an attempt to retrieve the CAAM
 * era via register reads will be made.
 **/
static int caam_get_era(struct caam_ctrl __iomem *ctrl)
{
	struct device_node *caam_node;
	int ret;
	u32 prop;

	handle_imx6_err005766(ctrlpriv);

	if (!ret)
		return prop;
	else
		return caam_get_era_from_hw(ctrl);
}

static const struct of_device_id caam_match[] = {
	{
		.compatible = "fsl,sec-v4.0",
	},
	{
		.compatible = "fsl,sec4.0",
	},
	{},
};
MODULE_DEVICE_TABLE(of, caam_match);

/* Probe routine for CAAM top (controller) level */
static int caam_probe(struct platform_device *pdev)
{
	int ret;
	u64 caam_id;
	static const struct soc_device_attribute imx_soc[] = {
		{.family = "Freescale i.MX"},
		{},
	};
	struct device *dev;
	struct device_node *nprop, *np;
	struct resource res_regs;
	struct caam_ctrl __iomem *ctrl;
	struct caam_drv_private *ctrlpriv;
	u32 comp_params;
	int pg_size;
	int block_offset = 0;

	ctrlpriv = devm_kzalloc(&pdev->dev, sizeof(*ctrlpriv), GFP_KERNEL);
	if (!ctrlpriv) {
		ret = -ENOMEM;
		goto exit;
	}

	dev = &pdev->dev;
	dev_set_drvdata(dev, ctrlpriv);
	ctrlpriv->dev = dev;
	ctrlpriv->pdev = pdev;
	nprop = pdev->dev.of_node;

	caam_imx = (bool)soc_device_match(imx_soc);

	/* Enable clocking */
	clk = caam_drv_identify_clk(&pdev->dev, "ipg");
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		dev_err(&pdev->dev,
			"can't identify CAAM ipg clk: %d\n", ret);
		return ret;
	}
	ctrlpriv->caam_ipg = clk;

	if (!of_machine_is_compatible("fsl,imx7d") &&
	    !of_machine_is_compatible("fsl,imx7s")) {
		clk = caam_drv_identify_clk(&pdev->dev, "mem");
		if (IS_ERR(clk)) {
			ret = PTR_ERR(clk);
			dev_err(&pdev->dev,
				"can't identify CAAM mem clk: %d\n", ret);
			return ret;
		}
		ctrlpriv->caam_mem = clk;
	}

	clk = caam_drv_identify_clk(&pdev->dev, "aclk");
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		dev_err(&pdev->dev,
			"can't identify CAAM aclk clk: %d\n", ret);
		return ret;
	}
	ctrlpriv->caam_aclk = clk;

	if (!of_machine_is_compatible("fsl,imx6ul") &&
	    !of_machine_is_compatible("fsl,imx7d") &&
	    !of_machine_is_compatible("fsl,imx7s")) {
		clk = caam_drv_identify_clk(&pdev->dev, "emi_slow");
		if (IS_ERR(clk)) {
			ret = PTR_ERR(clk);
			dev_err(&pdev->dev,
				"can't identify CAAM emi_slow clk: %d\n", ret);
			return ret;
		}
		ctrlpriv->caam_emi_slow = clk;
	}

	ret = clk_prepare_enable(ctrlpriv->caam_ipg);
	if (ret < 0) {
		dev_err(&pdev->dev, "can't enable CAAM ipg clock: %d\n", ret);
		return ret;
	}

	if (ctrlpriv->caam_mem) {
		ret = clk_prepare_enable(ctrlpriv->caam_mem);
		if (ret < 0) {
			dev_err(&pdev->dev, "can't enable CAAM secure mem clock: %d\n",
				ret);
			goto disable_caam_ipg;
		}
	}

	ret = clk_prepare_enable(ctrlpriv->caam_aclk);
	if (ret < 0) {
		dev_err(&pdev->dev, "can't enable CAAM aclk clock: %d\n", ret);
		goto disable_caam_mem;
	}

	if (ctrlpriv->caam_emi_slow) {
		ret = clk_prepare_enable(ctrlpriv->caam_emi_slow);
		if (ret < 0) {
			dev_err(&pdev->dev, "can't enable CAAM emi slow clock: %d\n",
				ret);
			goto disable_caam_aclk;
		}
	}
	/* Get configuration properties from device tree */
	/* First, get register page */
	ctrl = of_iomap(nprop, 0);
	if (ctrl == NULL) {
		dev_err(dev, "caam: of_iomap() failed\n");
		ret = -ENOMEM;
		goto disable_clocks;
	}
	ctrlpriv->ctrl = (struct caam_ctrl __force *)ctrl;

	if (of_find_compatible_node(NULL, NULL, "linaro,optee-tz"))
		ctrlpriv->has_optee = 1;

	if (of_machine_is_compatible("fsl,imx8qm") ||
	    of_machine_is_compatible("fsl,imx8qxp"))
		ctrlpriv->has_seco = 1;

	/*
	 * The driver does not have access to Page 0 of the CAAM if there
	 * is a secure component managing the CAAM as optee or SECO.
	 */
	ctrlpriv->has_access_p0 = !(ctrlpriv->has_optee || ctrlpriv->has_seco);

#ifdef CONFIG_PM_SLEEP
	ctrlpriv->caam_off_during_pm = caam_off_during_pm();
#endif

	if (ctrlpriv->has_seco) {
		ret = probe_w_seco(ctrlpriv);
		if (ret)
			goto iounmap_ctrl;
		return ret;
	}

	if (caam_imx)
		caam_little_end = true;
	else
		caam_little_end = !(bool)(rd_reg32(&ctrl->perfmon.status) &
				  (CSTA_PLEND | CSTA_ALT_PLEND));

	/* Finding the page size for using the CTPR_MS register */
	comp_params = rd_reg32(&ctrl->perfmon.comp_parms_ms);
	pg_size = (comp_params & CTPR_MS_PG_SZ_MASK) >> CTPR_MS_PG_SZ_SHIFT;

	/* Allocating the block_offset based on the supported page size on
	 * the platform
	 */
	if (pg_size == 0)
		block_offset = PG_SIZE_4K;
	else
		block_offset = PG_SIZE_64K;

	ctrlpriv->assure = (struct caam_assurance __iomem __force *)
			   ((__force uint8_t *)ctrl +
			    block_offset * ASSURE_BLOCK_NUMBER);
	ctrlpriv->deco = (struct caam_deco __iomem __force *)
			 ((__force uint8_t *)ctrl +
			 block_offset * DECO_BLOCK_NUMBER);

	detect_era(ctrlpriv);

	/* Get CAAM-SM node and of_iomap() and save */
	np = of_find_compatible_node(NULL, NULL, "fsl,imx6q-caam-sm");
	if (!np) {
		ret = -ENODEV;
		goto iounmap_ctrl;
	}

	/* Get CAAM SM registers base address from device tree */
	ret = of_address_to_resource(np, 0, &res_regs);
	if (ret) {
		dev_err(dev, "failed to retrieve registers base from device tree\n");
		ret = -ENODEV;
		goto iounmap_ctrl;
	}

	ctrlpriv->sm_phy = res_regs.start;
	ctrlpriv->sm_base = devm_ioremap_resource(dev, &res_regs);
	if (IS_ERR(ctrlpriv->sm_base)) {
		ret = PTR_ERR(ctrlpriv->sm_base);
		goto iounmap_ctrl;
	}

	if (!of_machine_is_compatible("fsl,imx8mn") &&
	    !of_machine_is_compatible("fsl,imx8mm") &&
	    !of_machine_is_compatible("fsl,imx8mq") &&
	    !of_machine_is_compatible("fsl,imx8qm") &&
	    !of_machine_is_compatible("fsl,imx8qxp")) {
		ctrlpriv->sm_size = resource_size(&res_regs);
	} else {
		ctrlpriv->sm_size = PG_SIZE_64K;
	}

	/*
	 * Enable DECO watchdogs and, if this is a PHYS_ADDR_T_64BIT kernel,
	 * long pointers in master configuration register.
	 * In case of SoCs with Management Complex, MC f/w performs
	 * the configuration.
	 */
	caam_dpaa2 = !!(comp_params & CTPR_MS_DPAA2);
	np = of_find_compatible_node(NULL, NULL, "fsl,qoriq-mc");
	ctrlpriv->mc_en = !!np;
	of_node_put(np);

	if (!ctrlpriv->mc_en)
		clrsetbits_32(&ctrl->mcr, MCFGR_AWCACHE_MASK | MCFGR_LONG_PTR,
			      MCFGR_AWCACHE_CACH | MCFGR_AWCACHE_BUFF |
			      MCFGR_WDENABLE | MCFGR_LARGE_BURST |
			      (sizeof(dma_addr_t) == sizeof(u64) ?
			       MCFGR_LONG_PTR : 0));

	caam_id = (u64)rd_reg32(&ctrl->perfmon.caam_id_ms) << 32 |
		  (u64)rd_reg32(&ctrl->perfmon.caam_id_ls);

	dev_info(dev, "device ID = 0x%016llx (Era %d)\n", caam_id,
		 ctrlpriv->era);
	dev_info(dev, "job rings = %d, qi = %d, dpaa2 = %s\n",
		 ctrlpriv->total_jobrs, ctrlpriv->qi_present,
		 caam_dpaa2 ? "yes" : "no");

	init_debugfs(ctrlpriv);

	return 0;

caam_remove:
	caam_remove(pdev);
	return ret;

iounmap_ctrl:
	iounmap(ctrl);
disable_clocks:
	if (!of_machine_is_compatible("fsl,imx8mn") &&
	    !of_machine_is_compatible("fsl,imx8mm") &&
	    !of_machine_is_compatible("fsl,imx8mq") &&
	    !of_machine_is_compatible("fsl,imx8qm") &&
	    !of_machine_is_compatible("fsl,imx8qxp")) {
		clk_disable_unprepare(ctrlpriv->caam_emi_slow);
		clk_disable_unprepare(ctrlpriv->caam_aclk);
		clk_disable_unprepare(ctrlpriv->caam_mem);
		clk_disable_unprepare(ctrlpriv->caam_ipg);
	}

exit:
	return ret;
}

static void enable_virt(struct caam_drv_private *ctrlpriv)
{
	if (ctrlpriv->virt_en == 1)
		clrsetbits_32(&ctrlpriv->ctrl->jrstart, 0, JRSTART_JR0_START |
			      JRSTART_JR1_START | JRSTART_JR2_START |
			      JRSTART_JR3_START);
}

static void check_virt(struct caam_drv_private *ctrlpriv, u32 comp_params)
{
	/*
	 *  Read the Compile Time parameters and SCFGR to determine
	 * if Virtualization is enabled for this platform
	 */
	u32 scfgr;

	scfgr = rd_reg32(&ctrlpriv->ctrl->scfgr);

	ctrlpriv->virt_en = 0;
	if (comp_params & CTPR_MS_VIRT_EN_INCL) {
		/* VIRT_EN_INCL = 1 & VIRT_EN_POR = 1 or
		 * VIRT_EN_INCL = 1 & VIRT_EN_POR = 0 & SCFGR_VIRT_EN = 1
		 */
		if ((comp_params & CTPR_MS_VIRT_EN_POR) ||
		    (!(comp_params & CTPR_MS_VIRT_EN_POR) &&
		       (scfgr & SCFGR_VIRT_EN)))
			ctrlpriv->virt_en = 1;
	} else {
		/* VIRT_EN_INCL = 0 && VIRT_EN_POR_VALUE = 1 */
		if (comp_params & CTPR_MS_VIRT_EN_POR)
			ctrlpriv->virt_en = 1;
	}
}

static int enable_jobrings(struct caam_drv_private *ctrlpriv, int block_offset)
{
	int ring = 0;
	int ret;
	struct device_node *nprop, *np;
	struct device *dev = ctrlpriv->dev;

	ctrlpriv->era = caam_get_era(ctrl);

	ret = of_platform_populate(nprop, caam_match, NULL, dev);
	if (ret) {
		dev_err(dev, "JR platform devices creation error\n");
		return -ENOMEM;
	}

	/* Loop over the child node of the CAAM */
	for_each_available_child_of_node(nprop, np)
		if (of_device_is_compatible(np, "fsl,sec-v4.0-job-ring") ||
		    of_device_is_compatible(np, "fsl,sec4.0-job-ring")) {
			u32 reg;

			/* Read the reg property of the JR */
			if (of_property_read_u32_index(np, "reg", 0, &reg)) {
				dev_err(dev, "%s read reg property error.",
					np->full_name);
				continue;
			}

			/*
			 * Set the address of the JR regs which is caam
			 * address + jr reg offset
			 */
			ctrlpriv->jr[ring] = (struct caam_job_ring __force *)
					     ((uint8_t *)ctrlpriv->ctrl + reg);

			/* Update counters */
			ctrlpriv->total_jobrs++;
			ring++;
		}

	return 0;
}

static void enable_qi(struct caam_drv_private *ctrlpriv, int block_offset)
{
	u32 parms_ms = rd_reg32(&ctrlpriv->ctrl->perfmon.comp_parms_ms);

	/* Check to see if (DPAA 1.x) QI present. If so, enable */
	ctrlpriv->qi_present = !!(parms_ms & CTPR_MS_QI_MASK);
	if (ctrlpriv->qi_present && !caam_dpaa2) {
		ctrlpriv->qi = (struct caam_queue_if __iomem __force *)
			       ((__force uint8_t *)ctrlpriv->ctrl +
				 block_offset * QI_BLOCK_NUMBER
			       );

		/* This is all that's required to physically enable QI */
		wr_reg32(&ctrlpriv->qi->qi_control_lo, QICTL_DQEN);

		/* If QMAN driver is present, init CAAM-QI backend */
#ifdef CONFIG_CAAM_QI
		ret = caam_qi_init(pdev);
		if (ret)
			dev_err(dev, "caam qi i/f init failed: %d\n", ret);
#endif
	}
}

static int probe_w_seco(struct caam_drv_private *ctrlpriv)
{
	int ret = 0;
	struct device_node *np;

	ctrlpriv->has_seco = true;
	/*
	 * For imx8 page size is 64k, we can't access ctrl regs to dynamically
	 * obtain this info.
	 */
	ret = enable_jobrings(ctrlpriv, PG_SIZE_64K);
	if (ret)
		return ret;
	if (!ctrlpriv->total_jobrs) {
		dev_err(ctrlpriv->dev, "no job rings configured!\n");
		return -ENODEV;
	}

	caam_little_end = true;
	ctrlpriv->assure = ((struct caam_assurance __force *)
			    ((uint8_t *)ctrlpriv->ctrl +
			     PG_SIZE_64K * ASSURE_BLOCK_NUMBER));
	ctrlpriv->deco = ((struct caam_deco __force *)
			  ((uint8_t *)ctrlpriv->ctrl +
			   PG_SIZE_64K * DECO_BLOCK_NUMBER));

	detect_era(ctrlpriv);

	/* Get CAAM-SM node and of_iomap() and save */
	np = of_find_compatible_node(NULL, NULL, "fsl,imx6q-caam-sm");
	if (!np) {
		dev_warn(ctrlpriv->dev, "No CAAM-SM node found!\n");
		return -ENODEV;
	}

	ctrlpriv->sm_base = of_iomap(np, 0);
	ctrlpriv->sm_size = 0x3fff;

	/* Can't enable DECO WD and LPs those are in MCR */

	/*
	 * If SEC has RNG version >= 4 and RNG state handle has not been
	 * already instantiated, do RNG instantiation
	 * In case of SoCs with Management Complex, RNG is managed by MC f/w.
	 */
	if (!ctrlpriv->mc_en &&
	    (cha_vid_ls & CHA_ID_LS_RNG_MASK) >> CHA_ID_LS_RNG_SHIFT >= 4) {
		ctrlpriv->rng4_sh_init =
			rd_reg32(&ctrl->r4tst[0].rdsta);
		/*
		 * If the secure keys (TDKEK, JDKEK, TDSK), were already
		 * generated, signal this to the function that is instantiating
		 * the state handles. An error would occur if RNG4 attempts
		 * to regenerate these keys before the next POR.
		 */
		gen_sk = ctrlpriv->rng4_sh_init & RDSTA_SKVN ? 0 : 1;
		ctrlpriv->rng4_sh_init &= RDSTA_IFMASK;
		do {
			int inst_handles =
				rd_reg32(&ctrl->r4tst[0].rdsta) &
								RDSTA_IFMASK;
			/*
			 * If either SH were instantiated by somebody else
			 * (e.g. u-boot) then it is assumed that the entropy
			 * parameters are properly set and thus the function
			 * setting these (kick_trng(...)) is skipped.
			 * Also, if a handle was instantiated, do not change
			 * the TRNG parameters.
			 */
			if (!(ctrlpriv->rng4_sh_init || inst_handles)) {
				dev_info(dev,
					 "Entropy delay = %u\n",
					 ent_delay);
				kick_trng(pdev, ent_delay);
				ent_delay += 400;
			}
			/*
			 * if instantiate_rng(...) fails, the loop will rerun
			 * and the kick_trng(...) function will modfiy the
			 * upper and lower limits of the entropy sampling
			 * interval, leading to a sucessful initialization of
			 * the RNG.
			 */
			ret = instantiate_rng(dev, inst_handles,
					      gen_sk);
			if (ret == -EAGAIN)
				/*
				 * if here, the loop will rerun,
				 * so don't hog the CPU
				 */
				cpu_relax();
		} while ((ret == -EAGAIN) && (ent_delay < RTSDCTL_ENT_DLY_MAX));
		if (ret) {
			dev_err(dev, "failed to instantiate RNG");
			goto caam_remove;
		}
		/*
		 * Set handles init'ed by this module as the complement of the
		 * already initialized ones
		 */
		ctrlpriv->rng4_sh_init = ~ctrlpriv->rng4_sh_init & RDSTA_IFMASK;

	/* Set DMA masks according to platform ranging */
	if (of_machine_is_compatible("fsl,imx8mn") ||
	    of_machine_is_compatible("fsl,imx8mm") ||
	    of_machine_is_compatible("fsl,imx8qm") ||
	    of_machine_is_compatible("fsl,imx8qxp") ||
	    of_machine_is_compatible("fsl,imx8mq")) {
		ret = dma_set_mask_and_coherent(ctrlpriv->dev,
			DMA_BIT_MASK(32));
	} else if (sizeof(dma_addr_t) == sizeof(u64))
		if (of_device_is_compatible(ctrlpriv->pdev->dev.of_node,
					    "fsl,sec-v5.0"))
			ret = dma_set_mask_and_coherent(ctrlpriv->dev,
						  DMA_BIT_MASK(40));
		else
			ret = dma_set_mask_and_coherent(ctrlpriv->dev,
						  DMA_BIT_MASK(36));
	else
		ret = dma_set_mask_and_coherent(ctrlpriv->dev,
			DMA_BIT_MASK(32));

	if (ret) {
		dev_err(ctrlpriv->dev, "dma_set_mask_and_coherent failed (%d)\n",
			ret);
		return ret;
	}

	/*
	 * this is where we should run the descriptor for DRNG init
	 * TRNG must be initialized by SECO
	 */
	return ret;
}

static void init_debugfs(struct caam_drv_private *ctrlpriv)
{
#ifdef CONFIG_DEBUG_FS
	struct caam_perfmon *perfmon;
	/* Read permission of the file created:
	 *  - S_IRUSR (user): 0x400
	 *  - S_IRGRP (group): 0x040
	 *  - S_IROTH (other): 0x004
	 */
	umode_t perm = 0x400 | 0x040 | 0x004;

	/* Report "alive" for developer to see */
	dev_info(dev, "device ID = 0x%016llx (Era %d)\n", caam_id,
		 ctrlpriv->era);
	dev_info(dev, "job rings = %d, qi = %d\n",
		 ctrlpriv->total_jobrs, ctrlpriv->qi_present);

	ctrlpriv->dfs_root = debugfs_create_dir(dev_name(ctrlpriv->dev), NULL);
	ctrlpriv->ctl = debugfs_create_dir("ctl", ctrlpriv->dfs_root);

	/* Controller-level - performance monitor counters */

	debugfs_create_file("rq_dequeued", perm,
			    ctrlpriv->ctl, &perfmon->req_dequeued,
			    &caam_fops_u64_ro);
	debugfs_create_file("ob_rq_encrypted", perm,
			    ctrlpriv->ctl, &perfmon->ob_enc_req,
			    &caam_fops_u64_ro);
	debugfs_create_file("ib_rq_decrypted", perm,
			    ctrlpriv->ctl, &perfmon->ib_dec_req,
			    &caam_fops_u64_ro);
	debugfs_create_file("ob_bytes_encrypted", perm,
			    ctrlpriv->ctl, &perfmon->ob_enc_bytes,
			    &caam_fops_u64_ro);
	debugfs_create_file("ob_bytes_protected", perm,
			    ctrlpriv->ctl, &perfmon->ob_prot_bytes,
			    &caam_fops_u64_ro);
	debugfs_create_file("ib_bytes_decrypted", perm,
			    ctrlpriv->ctl, &perfmon->ib_dec_bytes,
			    &caam_fops_u64_ro);
	debugfs_create_file("ib_bytes_validated", perm,
			    ctrlpriv->ctl, &perfmon->ib_valid_bytes,
			    &caam_fops_u64_ro);

	/* Controller level - global status values */
	debugfs_create_file("fault_addr", perm,
			    ctrlpriv->ctl, &perfmon->faultaddr,
			    &caam_fops_u32_ro);
	debugfs_create_file("fault_detail", perm,
			    ctrlpriv->ctl, &perfmon->faultdetail,
			    &caam_fops_u32_ro);
	debugfs_create_file("fault_status", perm,
			    ctrlpriv->ctl, &perfmon->status,
			    &caam_fops_u32_ro);

	/* Internal covering keys (useful in non-secure mode only) */
	ctrlpriv->ctl_kek_wrap.data = (__force void *)&ctrlpriv->ctrl->kek[0];
	ctrlpriv->ctl_kek_wrap.size = KEK_KEY_SIZE * sizeof(u32);
	ctrlpriv->ctl_kek = debugfs_create_blob("kek",
						perm,
						ctrlpriv->ctl,
						&ctrlpriv->ctl_kek_wrap);

	ctrlpriv->ctl_tkek_wrap.data = (__force void *)&ctrlpriv->ctrl->tkek[0];
	ctrlpriv->ctl_tkek_wrap.size = KEK_KEY_SIZE * sizeof(u32);
	ctrlpriv->ctl_tkek = debugfs_create_blob("tkek",
						 perm,
						 ctrlpriv->ctl,
						 &ctrlpriv->ctl_tkek_wrap);

	ctrlpriv->ctl_tdsk_wrap.data = (__force void *)&ctrlpriv->ctrl->tdsk[0];
	ctrlpriv->ctl_tdsk_wrap.size = KEK_KEY_SIZE * sizeof(u32);
	ctrlpriv->ctl_tdsk = debugfs_create_blob("tdsk",
						 perm,
						 ctrlpriv->ctl,
						 &ctrlpriv->ctl_tdsk_wrap);
#endif
}

static const struct of_device_id caam_match[] = {
	{
		.compatible = "fsl,sec-v4.0",
	},
	{
		.compatible = "fsl,sec4.0",
	},
	{},
};
MODULE_DEVICE_TABLE(of, caam_match);

#ifdef CONFIG_PM_SLEEP

/*
 * Indicate if the internal state of the CAAM is lost during PM
 */
static int caam_off_during_pm(void)
{
	if (IS_ENABLED(CONFIG_ARM64))
		return 1;

	if (of_machine_is_compatible("fsl,imx6sx") ||
	    of_machine_is_compatible("fsl,imx6ul") ||
	    of_machine_is_compatible("fsl,imx7ulp") ||
	    of_machine_is_compatible("fsl,imx7d") ||
	    of_machine_is_compatible("fsl,imx7s"))
		return 1;

	return 0;
}

caam_remove:
	caam_remove(pdev);
	return ret;

iounmap_ctrl:
	iounmap(ctrl);
disable_caam_emi_slow:
	if (ctrlpriv->caam_emi_slow)
		clk_disable_unprepare(ctrlpriv->caam_emi_slow);
disable_caam_aclk:
	clk_disable_unprepare(ctrlpriv->caam_aclk);
disable_caam_mem:
	if (ctrlpriv->caam_mem)
		clk_disable_unprepare(ctrlpriv->caam_mem);
disable_caam_ipg:
	clk_disable_unprepare(ctrlpriv->caam_ipg);
	return ret;
}

static int caam_ctrl_suspend(struct device *dev)
{
	const struct caam_drv_private *ctrlpriv = dev_get_drvdata(dev);

	if (ctrlpriv->caam_off_during_pm && ctrlpriv->has_access_p0)
		caam_state_save(dev);

	return 0;
}

static int caam_ctrl_resume(struct device *dev)
{
	struct caam_drv_private *ctrlpriv = dev_get_drvdata(dev);

	if (ctrlpriv->caam_off_during_pm && ctrlpriv->has_access_p0) {
		caam_state_restore(dev);
		caam_ctrl_hw_configuration(ctrlpriv);
	}

	return 0;
}

SIMPLE_DEV_PM_OPS(caam_ctrl_pm_ops, caam_ctrl_suspend, caam_ctrl_resume);

#endif /* CONFIG_PM_SLEEP */

static struct platform_driver caam_driver = {
	.driver = {
		.name = "caam",
		.of_match_table = caam_match,
#ifdef CONFIG_PM_SLEEP
		.pm = &caam_ctrl_pm_ops,
#endif
	},
	.probe       = caam_probe,
	.remove      = caam_remove,
};

module_platform_driver(caam_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("FSL CAAM request backend");
MODULE_AUTHOR("Freescale Semiconductor - NMG/STC");
