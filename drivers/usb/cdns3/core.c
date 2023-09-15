/**
 * core.c - Cadence USB3 DRD Controller Core file
 *
 * Copyright 2017-2019 NXP
 *
 * Authors: Peter Chen <peter.chen@nxp.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2  of
 * the License as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/usb/of.h>
#include <linux/usb/phy.h>
#include <linux/extcon.h>
#include <linux/pm_runtime.h>

#include "cdns3-nxp-reg-def.h"
#include "core.h"
#include "host-export.h"
#include "gadget-export.h"

static int cdns3_idle_init(struct cdns3 *cdns);

static int cdns3_role_start(struct cdns3 *cdns, enum usb_role role)
{
	u32 value;

	pr_debug("begin of %s\n", __func__);

	writel(0x0830, regs + PHY_PMA_CMN_CTRL1);
	writel(0x10, regs + TB_ADDR_CMN_DIAG_HSCLK_SEL);
	writel(0x00F0, regs + TB_ADDR_CMN_PLL0_VCOCAL_INIT_TMR);
	writel(0x0018, regs + TB_ADDR_CMN_PLL0_VCOCAL_ITER_TMR);
	writel(0x00D0, regs + TB_ADDR_CMN_PLL0_INTDIV);
	writel(0x4aaa, regs + TB_ADDR_CMN_PLL0_FRACDIV);
	writel(0x0034, regs + TB_ADDR_CMN_PLL0_HIGH_THR);
	writel(0x1ee, regs + TB_ADDR_CMN_PLL0_SS_CTRL1);
	writel(0x7F03, regs + TB_ADDR_CMN_PLL0_SS_CTRL2);
	writel(0x0020, regs + TB_ADDR_CMN_PLL0_DSM_DIAG);
	writel(0x0000, regs + TB_ADDR_CMN_DIAG_PLL0_OVRD);
	writel(0x0000, regs + TB_ADDR_CMN_DIAG_PLL0_FBH_OVRD);
	writel(0x0000, regs + TB_ADDR_CMN_DIAG_PLL0_FBL_OVRD);
	writel(0x0007, regs + TB_ADDR_CMN_DIAG_PLL0_V2I_TUNE);
	writel(0x0027, regs + TB_ADDR_CMN_DIAG_PLL0_CP_TUNE);
	writel(0x0008, regs + TB_ADDR_CMN_DIAG_PLL0_LF_PROG);
	writel(0x0022, regs + TB_ADDR_CMN_DIAG_PLL0_TEST_MODE);
	writel(0x000a, regs + TB_ADDR_CMN_PSM_CLK_CTRL);
	writel(0x139, regs + TB_ADDR_XCVR_DIAG_RX_LANE_CAL_RST_TMR);
	writel(0xbefc, regs + TB_ADDR_XCVR_PSM_RCTRL);

	writel(0x7799, regs + TB_ADDR_TX_PSC_A0);
	writel(0x7798, regs + TB_ADDR_TX_PSC_A1);
	writel(0x509b, regs + TB_ADDR_TX_PSC_A2);
	writel(0x3, regs + TB_ADDR_TX_DIAG_ECTRL_OVRD);
	writel(0x509b, regs + TB_ADDR_TX_PSC_A3);
	writel(0x2090, regs + TB_ADDR_TX_PSC_CAL);
	writel(0x2090, regs + TB_ADDR_TX_PSC_RDY);

	writel(0xA6FD, regs + TB_ADDR_RX_PSC_A0);
	writel(0xA6FD, regs + TB_ADDR_RX_PSC_A1);
	writel(0xA410, regs + TB_ADDR_RX_PSC_A2);
	writel(0x2410, regs + TB_ADDR_RX_PSC_A3);

	writel(0x23FF, regs + TB_ADDR_RX_PSC_CAL);
	writel(0x2010, regs + TB_ADDR_RX_PSC_RDY);

	writel(0x0020, regs + TB_ADDR_TX_TXCC_MGNLS_MULT_000);
	writel(0x00ff, regs + TB_ADDR_TX_DIAG_BGREF_PREDRV_DELAY);
	writel(0x0002, regs + TB_ADDR_RX_SLC_CU_ITER_TMR);
	writel(0x0013, regs + TB_ADDR_RX_SIGDET_HL_FILT_TMR);
	writel(0x0000, regs + TB_ADDR_RX_SAMP_DAC_CTRL);
	writel(0x1004, regs + TB_ADDR_RX_DIAG_SIGDET_TUNE);
	writel(0x4041, regs + TB_ADDR_RX_DIAG_LFPSDET_TUNE2);
	writel(0x0480, regs + TB_ADDR_RX_DIAG_BS_TM);
	writel(0x8006, regs + TB_ADDR_RX_DIAG_DFE_CTRL1);
	writel(0x003f, regs + TB_ADDR_RX_DIAG_ILL_IQE_TRIM4);
	writel(0x543f, regs + TB_ADDR_RX_DIAG_ILL_E_TRIM0);
	writel(0x543f, regs + TB_ADDR_RX_DIAG_ILL_IQ_TRIM0);
	writel(0x0000, regs + TB_ADDR_RX_DIAG_ILL_IQE_TRIM6);
	writel(0x8000, regs + TB_ADDR_RX_DIAG_RXFE_TM3);
	writel(0x0003, regs + TB_ADDR_RX_DIAG_RXFE_TM4);
	writel(0x2408, regs + TB_ADDR_RX_DIAG_LFPSDET_TUNE);
	writel(0x05ca, regs + TB_ADDR_RX_DIAG_DFE_CTRL3);
	writel(0x0258, regs + TB_ADDR_RX_DIAG_SC2C_DELAY);
	writel(0x1fff, regs + TB_ADDR_RX_REE_VGA_GAIN_NODFE);

	writel(0x02c6, regs + TB_ADDR_XCVR_PSM_CAL_TMR);
	writel(0x0002, regs + TB_ADDR_XCVR_PSM_A0BYP_TMR);
	writel(0x02c6, regs + TB_ADDR_XCVR_PSM_A0IN_TMR);
	writel(0x0010, regs + TB_ADDR_XCVR_PSM_A1IN_TMR);
	writel(0x0010, regs + TB_ADDR_XCVR_PSM_A2IN_TMR);
	writel(0x0010, regs + TB_ADDR_XCVR_PSM_A3IN_TMR);
	writel(0x0010, regs + TB_ADDR_XCVR_PSM_A4IN_TMR);
	writel(0x0010, regs + TB_ADDR_XCVR_PSM_A5IN_TMR);

	writel(0x0002, regs + TB_ADDR_XCVR_PSM_A0OUT_TMR);
	writel(0x0002, regs + TB_ADDR_XCVR_PSM_A1OUT_TMR);
	writel(0x0002, regs + TB_ADDR_XCVR_PSM_A2OUT_TMR);
	writel(0x0002, regs + TB_ADDR_XCVR_PSM_A3OUT_TMR);
	writel(0x0002, regs + TB_ADDR_XCVR_PSM_A4OUT_TMR);
	writel(0x0002, regs + TB_ADDR_XCVR_PSM_A5OUT_TMR);

	/* Change rx detect parameter */
	writel(0x960, regs + TB_ADDR_TX_RCVDET_EN_TMR);
	writel(0x01e0, regs + TB_ADDR_TX_RCVDET_ST_TMR);
	writel(0x0090, regs + TB_ADDR_XCVR_DIAG_LANE_FCM_EN_MGN_TMR);

	/* RXDET_IN_P3_32KHZ, Receiver detect slow clock enable */
	value = readl(regs + TB_ADDR_TX_RCVDETSC_CTRL);
	value |= RXDET_IN_P3_32KHZ;
	writel(value, regs + TB_ADDR_TX_RCVDETSC_CTRL);

	udelay(10);

	pr_debug("end of %s\n", __func__);
}

static void cdns_set_role(struct cdns3 *cdns, enum cdns3_roles role)
{
	u32 value;
	int timeout_us = 100000;
	void __iomem *xhci_regs = cdns->xhci_regs;

	if (role == CDNS3_ROLE_END)
		return;

	/* Wait clk value */
	value = readl(cdns->none_core_regs + USB3_SSPHY_STATUS);
	writel(value, cdns->none_core_regs + USB3_SSPHY_STATUS);
	udelay(1);
	value = readl(cdns->none_core_regs + USB3_SSPHY_STATUS);
	while ((value & 0xf0000000) != 0xf0000000 && timeout_us-- > 0) {
		value = readl(cdns->none_core_regs + USB3_SSPHY_STATUS);
		dev_dbg(cdns->dev, "clkvld:0x%x\n", value);
		udelay(1);
	}

	if (timeout_us <= 0)
		dev_err(cdns->dev, "wait clkvld timeout\n");

	/* Set all Reset bits */
	value = readl(cdns->none_core_regs + USB3_CORE_CTRL1);
	value |= ALL_SW_RESET;
	writel(value, cdns->none_core_regs + USB3_CORE_CTRL1);
	udelay(1);

	if (role == CDNS3_ROLE_HOST) {
		value = readl(cdns->none_core_regs + USB3_CORE_CTRL1);
		value = (value & ~MODE_STRAP_MASK) | HOST_MODE | OC_DISABLE;
		writel(value, cdns->none_core_regs + USB3_CORE_CTRL1);
		value &= ~PHYAHB_SW_RESET;
		writel(value, cdns->none_core_regs + USB3_CORE_CTRL1);
		mdelay(1);
		cdns3_usb_phy_init(cdns->phy_regs);
		/* Force B Session Valid as 1 */
		writel(0x0060, cdns->phy_regs + 0x380a4);
		mdelay(1);

		value = readl(cdns->none_core_regs + USB3_INT_REG);
		value |= HOST_INT1_EN;
		writel(value, cdns->none_core_regs + USB3_INT_REG);

		value = readl(cdns->none_core_regs + USB3_CORE_CTRL1);
		value &= ~ALL_SW_RESET;
		writel(value, cdns->none_core_regs + USB3_CORE_CTRL1);

		dev_dbg(cdns->dev, "wait xhci_power_on_ready\n");

		value = readl(cdns->none_core_regs + USB3_CORE_STATUS);
		timeout_us = 100000;
		while (!(value & HOST_POWER_ON_READY) && timeout_us-- > 0) {
			value = readl(cdns->none_core_regs + USB3_CORE_STATUS);
			udelay(1);
		}

		if (timeout_us <= 0)
			dev_err(cdns->dev, "wait xhci_power_on_ready timeout\n");

		value = readl(xhci_regs + XECP_PORT_CAP_REG);
		value |= LPM_2_STB_SWITCH_EN;
		writel(value, xhci_regs + XECP_PORT_CAP_REG);

		mdelay(1);

		dev_dbg(cdns->dev, "switch to host role successfully\n");
	} else { /* gadget mode */
		value = readl(cdns->none_core_regs + USB3_CORE_CTRL1);
		value = (value & ~MODE_STRAP_MASK) | DEV_MODE;
		writel(value, cdns->none_core_regs + USB3_CORE_CTRL1);
		value &= ~PHYAHB_SW_RESET;
		writel(value, cdns->none_core_regs + USB3_CORE_CTRL1);

		cdns3_usb_phy_init(cdns->phy_regs);
		/* Force B Session Valid as 1 */
		writel(0x0060, cdns->phy_regs + 0x380a4);
		value = readl(cdns->none_core_regs + USB3_INT_REG);
		value |= DEV_INT_EN;
		writel(value, cdns->none_core_regs + USB3_INT_REG);

		value = readl(cdns->none_core_regs + USB3_CORE_CTRL1);
		value &= ~ALL_SW_RESET;
		writel(value, cdns->none_core_regs + USB3_CORE_CTRL1);

		dev_dbg(cdns->dev, "wait gadget_power_on_ready\n");

		value = readl(cdns->none_core_regs + USB3_CORE_STATUS);
		timeout_us = 100000;
		while (!(value & DEV_POWER_ON_READY) && timeout_us-- > 0) {
			value = readl(cdns->none_core_regs + USB3_CORE_STATUS);
			udelay(1);
		}

		if (timeout_us <= 0)
			dev_err(cdns->dev,
				"wait gadget_power_on_ready timeout\n");

		mdelay(1);

		dev_dbg(cdns->dev, "switch to gadget role successfully\n");
	}
}

static enum cdns3_roles cdns3_get_role(struct cdns3 *cdns)
{
	if (cdns->roles[CDNS3_ROLE_HOST] && cdns->roles[CDNS3_ROLE_GADGET]) {
		if (extcon_get_state(cdns->extcon, EXTCON_USB_HOST))
			return CDNS3_ROLE_HOST;
		else if (extcon_get_state(cdns->extcon, EXTCON_USB))
			return CDNS3_ROLE_GADGET;
		else
			return CDNS3_ROLE_END;
	} else {
		return cdns->roles[CDNS3_ROLE_HOST]
			? CDNS3_ROLE_HOST
			: CDNS3_ROLE_GADGET;
	}
}

/**
 * cdns3_core_init_role - initialize role of operation
 * @cdns: Pointer to cdns3 structure
 *
 * Returns 0 on success otherwise negative errno
 */
static int cdns3_core_init_role(struct cdns3 *cdns)
{
	struct device *dev = cdns->dev;
	enum usb_dr_mode best_dr_mode;
	enum usb_dr_mode dr_mode;
	int ret;

	cdns->role = CDNS3_ROLE_END;
	if (dr_mode == USB_DR_MODE_UNKNOWN)
		dr_mode = USB_DR_MODE_OTG;

	if (dr_mode == USB_DR_MODE_OTG || dr_mode == USB_DR_MODE_HOST) {
		if (cdns3_host_init(cdns))
			dev_info(dev, "doesn't support host\n");
	}

	if (dr_mode == USB_DR_MODE_OTG || dr_mode == USB_DR_MODE_PERIPHERAL) {
		if (cdns3_gadget_init(cdns))
			dev_info(dev, "doesn't support gadget\n");
	}

	if (!cdns->roles[CDNS3_ROLE_HOST] && !cdns->roles[CDNS3_ROLE_GADGET]) {
		dev_err(dev, "no supported roles\n");
		return -ENODEV;
	}

	return 0;
}

/**
 * cdns3_irq - interrupt handler for cdns3 core device
 *
 * @irq: irq number for cdns3 core device
 * @data: structure of cdns3
 *
 * Returns IRQ_HANDLED or IRQ_NONE
 */
static irqreturn_t cdns3_irq(int irq, void *data)
{
	struct cdns3 *cdns = data;
	irqreturn_t ret = IRQ_NONE;

	if (cdns->in_lpm) {
		disable_irq_nosync(cdns->irq);
		cdns->wakeup_int = true;
		pm_runtime_get(cdns->dev);
		return IRQ_HANDLED;
	}

	/* Handle device/host interrupt */
	if (cdns->role != CDNS3_ROLE_END)
		ret = cdns3_role(cdns)->irq(cdns);

	return ret;
}

static irqreturn_t cdns3_thread_irq(int irq, void *data)
{
	struct cdns3 *cdns = data;
	irqreturn_t ret = IRQ_NONE;

	/* Handle device/host interrupt */
	if (cdns->role != CDNS3_ROLE_END && cdns3_role(cdns)->thread_irq)
		ret = cdns3_role(cdns)->thread_irq(cdns);

	return ret;
}

static int cdns3_get_clks(struct device *dev)
{
	struct cdns3 *cdns = dev_get_drvdata(dev);
	int ret = 0;

	cdns->cdns3_clks[0] = devm_clk_get(dev, "usb3_lpm_clk");
	if (IS_ERR(cdns->cdns3_clks[0])) {
		ret = PTR_ERR(cdns->cdns3_clks[0]);
		dev_err(dev, "Failed to get usb3_lpm_clk, err=%d\n", ret);
		return ret;
	}

	cdns->cdns3_clks[1] = devm_clk_get(dev, "usb3_bus_clk");
	if (IS_ERR(cdns->cdns3_clks[1])) {
		ret = PTR_ERR(cdns->cdns3_clks[1]);
		dev_err(dev, "Failed to get usb3_bus_clk, err=%d\n", ret);
		return ret;
	}

	cdns->cdns3_clks[2] = devm_clk_get(dev, "usb3_aclk");
	if (IS_ERR(cdns->cdns3_clks[2])) {
		ret = PTR_ERR(cdns->cdns3_clks[2]);
		dev_err(dev, "Failed to get usb3_aclk, err=%d\n", ret);
		return ret;
	}

	cdns->cdns3_clks[3] = devm_clk_get(dev, "usb3_ipg_clk");
	if (IS_ERR(cdns->cdns3_clks[3])) {
		ret = PTR_ERR(cdns->cdns3_clks[3]);
		dev_err(dev, "Failed to get usb3_ipg_clk, err=%d\n", ret);
		return ret;
	}

	cdns->cdns3_clks[4] = devm_clk_get(dev, "usb3_core_pclk");
	if (IS_ERR(cdns->cdns3_clks[4])) {
		ret = PTR_ERR(cdns->cdns3_clks[4]);
		dev_err(dev, "Failed to get usb3_core_pclk, err=%d\n", ret);
		return ret;
	}

	return 0;
}

static int cdns3_prepare_enable_clks(struct device *dev)
{
	struct cdns3 *cdns = dev_get_drvdata(dev);
	int i, j, ret = 0;

	for (i = 0; i < CDNS3_NUM_OF_CLKS; i++) {
		ret = clk_prepare_enable(cdns->cdns3_clks[i]);
		if (ret) {
			dev_err(dev,
				"Failed to prepare/enable cdns3 clk, err=%d\n",
				ret);
			goto err;
		}
	}

	cdns->dr_mode = dr_mode;

	ret = cdns3_drd_update_mode(cdns);
	if (ret)
		goto err;

	/* Initialize idle role to start with */
	ret = cdns3_role_start(cdns, USB_ROLE_NONE);
	if (ret)
		goto err;

	switch (cdns->dr_mode) {
	case USB_DR_MODE_OTG:
		ret = cdns3_hw_role_switch(cdns);
		if (ret)
			goto err;
		break;
	case USB_DR_MODE_PERIPHERAL:
		ret = cdns3_role_start(cdns, USB_ROLE_DEVICE);
		if (ret)
			goto err;
		break;
	case USB_DR_MODE_HOST:
		ret = cdns3_role_start(cdns, USB_ROLE_HOST);
		if (ret)
			goto err;
		break;
	default:
		ret = -EINVAL;
		goto err;
	}

	return 0;
err:
	for (j = i; j > 0; j--)
		clk_disable_unprepare(cdns->cdns3_clks[j - 1]);

	return ret;
}

/**
 * cdns3_hw_role_state_machine  - role switch state machine based on hw events.
 * @cdns: Pointer to controller structure.
 *
 * Returns next role to be entered based on hw events.
 */
static enum usb_role cdns3_hw_role_state_machine(struct cdns3 *cdns)
{
	enum usb_role role = USB_ROLE_NONE;
	int id, vbus;

	if (cdns->dr_mode != USB_DR_MODE_OTG) {
		if (cdns3_is_host(cdns))
			role = USB_ROLE_HOST;
		if (cdns3_is_device(cdns))
			role = USB_ROLE_DEVICE;

		return role;
	}

	id = cdns3_get_id(cdns);
	vbus = cdns3_get_vbus(cdns);

	/*
	 * Role change state machine
	 * Inputs: ID, VBUS
	 * Previous state: cdns->role
	 * Next state: role
	 */
	role = cdns->role;

	switch (role) {
	case USB_ROLE_NONE:
		/*
		 * Driver treats USB_ROLE_NONE synonymous to IDLE state from
		 * controller specification.
		 */
		if (!id)
			role = USB_ROLE_HOST;
		else if (vbus)
			role = USB_ROLE_DEVICE;
		break;
	case USB_ROLE_HOST: /* from HOST, we can only change to NONE */
		if (id)
			role = USB_ROLE_NONE;
		break;
	case USB_ROLE_DEVICE: /* from GADGET, we can only change to NONE*/
		if (!vbus)
			role = USB_ROLE_NONE;
		break;
	}

	dev_dbg(cdns->dev, "role %d -> %d\n", cdns->role, role);

	return role;
}

static void cdns3_remove_roles(struct cdns3 *cdns)
{
	cdns3_gadget_exit(cdns);
	cdns3_host_remove(cdns);
}

static int cdns3_do_role_switch(struct cdns3 *cdns, enum cdns3_roles role)
{
	/* Program Lane swap and bring PHY out of RESET */
	phy_reset(cdns->usb3_phy);
}

static int cdns3_idle_init(struct cdns3 *cdns)
{
	struct cdns3_role_driver *rdrv;

	rdrv = devm_kzalloc(cdns->dev, sizeof(*rdrv), GFP_KERNEL);
	if (!rdrv)
		return -ENOMEM;

	rdrv->start = cdns3_idle_role_start;
	rdrv->stop = cdns3_idle_role_stop;
	rdrv->state = CDNS3_ROLE_STATE_INACTIVE;
	rdrv->suspend = NULL;
	rdrv->resume = NULL;
	rdrv->name = "idle";

	cdns->roles[USB_ROLE_NONE] = rdrv;

	return 0;
}

/**
 * cdns3_hw_role_switch - switch roles based on HW state
 * @cdns: controller
 */
int cdns3_hw_role_switch(struct cdns3 *cdns)
{
	enum usb_role real_role, current_role;
	int ret = 0;
	enum cdns3_roles current_role;

	/* Depends on role switch class */
	if (cdns->role_sw)
		return 0;

	pm_runtime_get_sync(cdns->dev);
	current_role = cdns->role;
	real_role = cdns3_hw_role_state_machine(cdns);

	/* Do nothing if nothing changed */
	if (current_role == real_role)
		goto exit;

	cdns3_role_stop(cdns);
	if (role == CDNS3_ROLE_END) {
		/* Force B Session Valid as 0 */
		writel(0x0040, cdns->phy_regs + 0x380a4);
		pm_runtime_put_sync(cdns->dev);
		return 0;
	}

	/*
	 * WORKAROUND: Mass storage gadget calls .ep_disable after
	 * disconnect with host, wait some time for .ep_disable completion.
	 */
	msleep(20);
	cdns_set_role(cdns, role);
	ret = cdns3_role_start(cdns, role);
	if (ret) {
		/* Back to current role */
		dev_err(cdns->dev, "set %d has failed, back to %d\n",
					role, current_role);
		cdns_set_role(cdns, current_role);
		ret = cdns3_role_start(cdns, current_role);
	}

	pm_runtime_put_sync(cdns->dev);
	return ret;
}

/**
 * cdns3_role_switch - work queue handler for role switch
 *
 * @sw: pointer to USB role switch structure
 *
 * Returns role
 */
static enum usb_role cdns3_role_get(struct usb_role_switch *sw)
{
	struct cdns3 *cdns = usb_role_switch_get_drvdata(sw);

	return cdns->role;
}

/**
 * cdns3_role_set - set current role of controller.
 *
 * @sw: pointer to USB role switch structure
 * @role: the previous role
 * Handles below events:
 * - Role switch for dual-role devices
 * - CDNS3_ROLE_GADGET <--> CDNS3_ROLE_END for peripheral-only devices
 */
static int cdns3_role_set(struct usb_role_switch *sw, enum usb_role role)
{
	struct cdns3 *cdns = usb_role_switch_get_drvdata(sw);
	int ret = 0;

	host = extcon_get_state(cdns->extcon, EXTCON_USB_HOST);
	device = extcon_get_state(cdns->extcon, EXTCON_USB);

	if (cdns->role == role)
		goto pm_put;

	if (cdns->dr_mode == USB_DR_MODE_HOST) {
		switch (role) {
		case USB_ROLE_NONE:
		case USB_ROLE_HOST:
			break;
		default:
			goto pm_put;
		}
	}

	if (cdns->dr_mode == USB_DR_MODE_PERIPHERAL) {
		switch (role) {
		case USB_ROLE_NONE:
		case USB_ROLE_DEVICE:
			break;
		default:
			goto pm_put;
		}
	}

	cdns3_role_stop(cdns);
	ret = cdns3_role_start(cdns, role);
	if (ret)
		dev_err(cdns->dev, "set role %d has failed\n", role);

pm_put:
	pm_runtime_put_sync(cdns->dev);
	return ret;
}

static int set_phy_power_on(struct cdns3 *cdns)
{
	int ret;

	ret = phy_power_on(cdns->usb2_phy);
	if (ret)
		return ret;

	ret = phy_power_on(cdns->usb3_phy);
	if (ret)
		phy_power_off(cdns->usb2_phy);

	return ret;
}

static void set_phy_power_off(struct cdns3 *cdns)
{
	phy_power_off(cdns->usb3_phy);
	phy_power_off(cdns->usb2_phy);
}

/**
 * cdns3_wakeup_irq - interrupt handler for wakeup events
 * @irq: irq number for cdns3 core device
 * @data: structure of cdns3
 *
 * Returns IRQ_HANDLED or IRQ_NONE
 */
static irqreturn_t cdns3_wakeup_irq(int irq, void *data)
{
	struct cdns3 *cdns = data;

	if (cdns->in_lpm) {
		disable_irq_nosync(irq);
		cdns->wakeup_pending = true;
		if ((cdns->role == USB_ROLE_HOST) && cdns->host_dev)
			pm_request_resume(&cdns->host_dev->dev);

		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

/**
 * cdns3_probe - probe for cdns3 core device
 * @pdev: Pointer to cdns3 core platform device
 *
 * Returns 0 on success otherwise negative errno
 */
static int cdns3_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource	*res;
	struct cdns3 *cdns;
	void __iomem *regs;
	int ret;

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(dev, "error setting dma mask: %d\n", ret);
		return ret;
	}

	cdns = devm_kzalloc(dev, sizeof(*cdns), GFP_KERNEL);
	if (!cdns)
		return -ENOMEM;

	cdns->dev = dev;
	cdns->pdata = dev_get_platdata(dev);

	platform_set_drvdata(pdev, cdns);

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(dev, "missing IRQ\n");
		return -ENODEV;
	}
	cdns->irq = res->start;

	/*
	 * Request memory region
	 * region-0: nxp wrap registers
	 * region-1: xHCI
	 * region-2: Peripheral
	 * region-3: PHY registers
	 * region-4: OTG registers
	 */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(regs))
		return PTR_ERR(regs);
	cdns->none_core_regs = regs;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(regs))
		return PTR_ERR(regs);
	cdns->xhci_regs = regs;
	cdns->xhci_res = res;

	cdns->xhci_res[1] = *res;

	cdns->dev_irq = platform_get_irq_byname(pdev, "peripheral");
	if (cdns->dev_irq == -EPROBE_DEFER)
		return cdns->dev_irq;

	if (cdns->dev_irq < 0)
		dev_err(dev, "couldn't get peripheral irq\n");

	regs = devm_platform_ioremap_resource_byname(pdev, "dev");
	if (IS_ERR(regs))
		return PTR_ERR(regs);
	cdns->dev_regs	= regs;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 3);
	regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(regs))
		return PTR_ERR(regs);
	cdns->phy_regs = regs;

	if (cdns->otg_irq < 0) {
		dev_err(dev, "couldn't get otg irq\n");
		return cdns->otg_irq;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "otg");
	if (!res) {
		dev_err(dev, "couldn't get otg resource\n");
		return -ENXIO;
	}

	cdns->phyrst_a_enable = device_property_read_bool(dev, "cdns,phyrst-a-enable");

	cdns->otg_res = *res;

	cdns->wakeup_irq = platform_get_irq_byname_optional(pdev, "wakeup");
	if (cdns->wakeup_irq == -EPROBE_DEFER)
		return cdns->wakeup_irq;
	else if (cdns->wakeup_irq == 0)
		return -EINVAL;

	if (cdns->wakeup_irq < 0) {
		dev_dbg(dev, "couldn't get wakeup irq\n");
		cdns->wakeup_irq = 0x0;
	}

	mutex_init(&cdns->mutex);
	ret = cdns3_get_clks(dev);
	if (ret)
		return ret;

	ret = cdns3_prepare_enable_clks(dev);
	if (ret)
		return ret;

	cdns->usbphy = devm_usb_get_phy_by_phandle(dev, "cdns3,usbphy", 0);
	if (IS_ERR(cdns->usbphy)) {
		ret = PTR_ERR(cdns->usbphy);
		if (ret == -ENODEV)
			ret = -EINVAL;
		goto err1;
	}

	ret = usb_phy_init(cdns->usbphy);
	if (ret)
		goto err1;

	ret = set_phy_power_on(cdns);
	if (ret)
		goto err2;

	if (device_property_read_bool(dev, "usb-role-switch")) {
		struct usb_role_switch_desc sw_desc = { };

		sw_desc.set = cdns3_role_set;
		sw_desc.get = cdns3_role_get;
		sw_desc.allow_userspace_control = true;
		sw_desc.driver_data = cdns;
		sw_desc.fwnode = dev->fwnode;

		cdns->role_sw = usb_role_switch_register(dev, &sw_desc);
		if (IS_ERR(cdns->role_sw)) {
			ret = PTR_ERR(cdns->role_sw);
			dev_warn(dev, "Unable to register Role Switch\n");
			goto err3;
		}
	}

	if (cdns->wakeup_irq) {
		ret = devm_request_irq(cdns->dev, cdns->wakeup_irq,
						cdns3_wakeup_irq,
						IRQF_SHARED,
						dev_name(cdns->dev), cdns);

		if (ret) {
			dev_err(cdns->dev, "couldn't register wakeup irq handler\n");
			goto err4;
		}
	}

	ret = cdns3_drd_init(cdns);
	if (ret)
		goto err4;

	ret = devm_request_threaded_irq(dev, cdns->irq, cdns3_irq,
			cdns3_thread_irq, IRQF_SHARED, dev_name(dev), cdns);
	if (ret)
		goto err4;

	spin_lock_init(&cdns->lock);
	device_set_wakeup_capable(dev, true);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	if (!(cdns->pdata && (cdns->pdata->quirks & CDNS3_DEFAULT_PM_RUNTIME_ALLOW)))
		pm_runtime_forbid(dev);

	/*
	 * The controller needs less time between bus and controller suspend,
	 * and we also needs a small delay to avoid frequently entering low
	 * power mode.
	 */
	pm_runtime_set_autosuspend_delay(dev, 20);
	pm_runtime_mark_last_busy(dev);
	pm_runtime_use_autosuspend(dev);
	dev_dbg(dev, "Cadence USB3 core: probe succeed\n");

	return 0;
err4:
	cdns3_drd_exit(cdns);
	if (cdns->role_sw)
		usb_role_switch_unregister(cdns->role_sw);
err3:
	set_phy_power_off(cdns);
err2:
	usb_phy_shutdown(cdns->usbphy);
err1:
	cdns3_disable_unprepare_clks(dev);
	return ret;
}

/**
 * cdns3_remove - unbind our drd driver and clean up
 * @pdev: Pointer to Linux platform device
 *
 * Returns 0 on success otherwise negative errno
 */
static int cdns3_remove(struct platform_device *pdev)
{
	struct cdns3 *cdns = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;

	pm_runtime_get_sync(dev);
	pm_runtime_disable(dev);
	pm_runtime_put_noidle(dev);
	sysfs_remove_group(&dev->kobj, &cdns3_attr_group);
	cdns3_remove_roles(cdns);
	usb_phy_shutdown(cdns->usbphy);
	cdns3_disable_unprepare_clks(dev);

	pm_runtime_get_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	pm_runtime_put_noidle(&pdev->dev);
	cdns3_exit_roles(cdns);
	usb_role_switch_unregister(cdns->role_sw);
	set_phy_power_off(cdns);
	phy_exit(cdns->usb2_phy);
	phy_exit(cdns->usb3_phy);
	return 0;
}

#ifdef CONFIG_PM

static int cdns3_set_platform_suspend(struct device *dev,
		bool suspend, bool wakeup)
{
	struct cdns3 *cdns = dev_get_drvdata(dev);
	int ret = 0;

	if (cdns->pdata && cdns->pdata->platform_suspend)
		ret = cdns->pdata->platform_suspend(dev, suspend, wakeup);

	return ret;
}

static int cdns3_controller_suspend(struct device *dev, pm_message_t msg)
{
	struct cdns3 *cdns = dev_get_drvdata(dev);
	bool wakeup;
	unsigned long flags;

	if (cdns->in_lpm)
		return 0;

	if (PMSG_IS_AUTO(msg))
		wakeup = true;
	else
		wakeup = device_may_wakeup(dev);

	cdns3_set_platform_suspend(cdns->dev, true, wakeup);
	set_phy_power_off(cdns);
	spin_lock_irqsave(&cdns->lock, flags);
	cdns->in_lpm = true;
	spin_unlock_irqrestore(&cdns->lock, flags);
	dev_dbg(cdns->dev, "%s ends\n", __func__);

	return 0;
}

static int cdns3_controller_resume(struct device *dev, pm_message_t msg)
{
	struct cdns3 *cdns = dev_get_drvdata(dev);
	int ret;
	unsigned long flags;

	if (!cdns->in_lpm)
		return 0;

	ret = set_phy_power_on(cdns);
	if (ret)
		return ret;

	cdns3_set_platform_suspend(cdns->dev, false, false);

	spin_lock_irqsave(&cdns->lock, flags);
	if (cdns->roles[cdns->role]->resume && !PMSG_IS_AUTO(msg))
		cdns->roles[cdns->role]->resume(cdns, false);

	cdns->in_lpm = false;
	spin_unlock_irqrestore(&cdns->lock, flags);
	if (cdns->wakeup_pending) {
		cdns->wakeup_pending = false;
		enable_irq(cdns->wakeup_irq);
	}
	dev_dbg(cdns->dev, "%s ends\n", __func__);

	return ret;
}

static int cdns3_runtime_suspend(struct device *dev)
{
	return cdns3_controller_suspend(dev, PMSG_AUTO_SUSPEND);
}

static int cdns3_runtime_resume(struct device *dev)
{
	return cdns3_controller_resume(dev, PMSG_AUTO_RESUME);
}
#ifdef CONFIG_PM_SLEEP

#ifdef CONFIG_PM
static inline bool controller_power_is_lost(struct cdns3 *cdns)
{
	u32 value;

	value = readl(cdns->none_core_regs + USB3_CORE_CTRL1);
	if ((value & SW_RESET_MASK) == ALL_SW_RESET)
		return true;
	else
		return false;
}

static void cdns3_set_wakeup(void *none_core_regs, bool enable)
{
	u32 value;

	if (enable) {
		/* Enable wakeup and phy_refclk_req */
		value = readl(none_core_regs + USB3_INT_REG);
		value |= OTG_WAKEUP_EN | DEVU3_WAEKUP_EN;
		writel(value, none_core_regs + USB3_INT_REG);
	} else {
		/* disable wakeup and phy_refclk_req */
		value = readl(none_core_regs + USB3_INT_REG);
		value &= ~(OTG_WAKEUP_EN | DEVU3_WAEKUP_EN);
		writel(value, none_core_regs + USB3_INT_REG);
	}
}

static int cdns3_enter_suspend(struct cdns3 *cdns, bool suspend, bool wakeup)
{
	void __iomem *otg_regs = cdns->otg_regs;
	void __iomem *xhci_regs = cdns->xhci_regs;
	void __iomem *none_core_regs = cdns->none_core_regs;
	u32 value;
	int timeout_us = 100000;
	int ret = 0;

	if (cdns->role == CDNS3_ROLE_GADGET) {
		if (suspend) {
			/* When at device mode, set controller at reset mode */
			value = readl(cdns->none_core_regs + USB3_CORE_CTRL1);
			value |= ALL_SW_RESET;
			writel(value, cdns->none_core_regs + USB3_CORE_CTRL1);
		}
		return 0;
	} else if (cdns->role == CDNS3_ROLE_END) {
		return 0;
	}

	if (suspend) {
		if (cdns3_role(cdns)->suspend)
			ret = cdns3_role(cdns)->suspend(cdns, wakeup);

		if (ret)
			return ret;

		/* SW request low power when all usb ports allow to it ??? */
		value = readl(xhci_regs + XECP_PM_PMCSR);
		value &= ~PS_MASK;
		value |= PS_D1;
		writel(value, xhci_regs + XECP_PM_PMCSR);

		/* mdctrl_clk_sel */
		value = readl(none_core_regs + USB3_CORE_CTRL1);
		value |= MDCTRL_CLK_SEL;
		writel(value, none_core_regs + USB3_CORE_CTRL1);

		/* wait for mdctrl_clk_status */
		value = readl(none_core_regs + USB3_CORE_STATUS);
		while (!(value & MDCTRL_CLK_STATUS) && timeout_us-- > 0) {
			value = readl(none_core_regs + USB3_CORE_STATUS);
			udelay(1);
		}

		if (timeout_us <= 0)
			dev_err(cdns->dev, "wait mdctrl_clk_status timeout\n");

		dev_dbg(cdns->dev, "mdctrl_clk_status is set\n");

		/* wait lpm_clk_req to be 0 */
		value = readl(none_core_regs + USB3_INT_REG);
		timeout_us = 100000;
		while ((value & LPM_CLK_REQ) && timeout_us-- > 0) {
			value = readl(none_core_regs + USB3_INT_REG);
			udelay(1);
		}

		if (timeout_us <= 0)
			dev_err(cdns->dev, "wait lpm_clk_req timeout\n");

		dev_dbg(cdns->dev, "lpm_clk_req cleared\n");

		/* wait phy_refclk_req to be 0 */
		value = readl(none_core_regs + USB3_SSPHY_STATUS);
		timeout_us = 100000;
		while ((value & PHY_REFCLK_REQ) && timeout_us-- > 0) {
			value = readl(none_core_regs + USB3_SSPHY_STATUS);
			udelay(1);
		}

		if (timeout_us <= 0)
			dev_err(cdns->dev, "wait phy_refclk_req timeout\n");

		dev_dbg(cdns->dev, "phy_refclk_req cleared\n");
		cdns3_set_wakeup(none_core_regs, true);
	} else {
		value = readl(none_core_regs + USB3_INT_REG);
		/* wait CLK_125_REQ to be 1 */
		value = readl(none_core_regs + USB3_INT_REG);
		while (!(value & CLK_125_REQ) && timeout_us-- > 0) {
			value = readl(none_core_regs + USB3_INT_REG);
			udelay(1);
		}

		cdns3_set_wakeup(none_core_regs, false);

		/* SW request D0 */
		value = readl(xhci_regs + XECP_PM_PMCSR);
		value &= ~PS_MASK;
		value |= PS_D0;
		writel(value, xhci_regs + XECP_PM_PMCSR);

		/* clr CFG_RXDET_P3_EN */
		value = readl(xhci_regs + XECP_AUX_CTRL_REG1);
		value &= ~CFG_RXDET_P3_EN;
		writel(value, xhci_regs + XECP_AUX_CTRL_REG1);

		/* clear mdctrl_clk_sel */
		value = readl(none_core_regs + USB3_CORE_CTRL1);
		value &= ~MDCTRL_CLK_SEL;
		writel(value, none_core_regs + USB3_CORE_CTRL1);

		/* wait for mdctrl_clk_status is cleared */
		value = readl(none_core_regs + USB3_CORE_STATUS);
		timeout_us = 100000;
		while ((value & MDCTRL_CLK_STATUS) && timeout_us-- > 0) {
			value = readl(none_core_regs + USB3_CORE_STATUS);
			udelay(1);
		}

		if (timeout_us <= 0)
			dev_err(cdns->dev, "wait mdctrl_clk_status timeout\n");

		dev_dbg(cdns->dev, "mdctrl_clk_status cleared\n");

		/* Wait until OTG_NRDY is 0 */
		value = readl(otg_regs + OTGSTS);
		timeout_us = 100000;
		while ((value & OTG_NRDY) && timeout_us-- > 0) {
			value = readl(otg_regs + OTGSTS);
			udelay(1);
		}

		if (timeout_us <= 0)
			dev_err(cdns->dev, "wait OTG ready timeout\n");

		value = readl(none_core_regs + USB3_CORE_STATUS);
		timeout_us = 100000;
		while (!(value & HOST_POWER_ON_READY) && timeout_us-- > 0) {
			value = readl(none_core_regs + USB3_CORE_STATUS);
			udelay(1);
		}

		if (timeout_us <= 0)
			dev_err(cdns->dev, "wait xhci_power_on_ready timeout\n");
	}

	return ret;
}

static int cdns3_controller_suspend(struct cdns3 *cdns, bool wakeup)
{
	int ret = 0;

	disable_irq(cdns->irq);
	ret = cdns3_enter_suspend(cdns, true, wakeup);
	if (ret) {
		enable_irq(cdns->irq);
		return ret;
	}

	usb_phy_set_suspend(cdns->usbphy, 1);
	cdns->in_lpm = true;
	enable_irq(cdns->irq);
	return ret;
}

#ifdef CONFIG_PM_SLEEP
static int cdns3_suspend(struct device *dev)
{
	struct cdns3 *cdns = dev_get_drvdata(dev);
	bool wakeup = device_may_wakeup(dev);
	int ret;

	if (pm_runtime_status_suspended(dev))
		pm_runtime_resume(dev);

	if (cdns->roles[cdns->role]->suspend) {
		spin_lock_irqsave(&cdns->lock, flags);
		cdns->roles[cdns->role]->suspend(cdns, false);
		spin_unlock_irqrestore(&cdns->lock, flags);
	}

	return cdns3_controller_suspend(dev, PMSG_SUSPEND);
}

static int cdns3_resume(struct device *dev)
{
	int ret;

	ret = cdns3_controller_resume(dev, PMSG_RESUME);
	if (ret)
		return ret;

	pm_runtime_disable(dev);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	return ret;
}
#endif /* CONFIG_PM_SLEEP */
#endif /* CONFIG_PM */

	dev_dbg(dev, "at the begin of %s\n", __func__);
	if (cdns->in_lpm) {
		WARN_ON(1);
		return 0;
	}

	ret = cdns3_controller_suspend(cdns, true);
	if (ret)
		return ret;

	cdns3_disable_unprepare_clks(dev);

	dev_dbg(dev, "at the end of %s\n", __func__);

	return ret;
}

static int cdns3_runtime_resume(struct device *dev)
{
	struct cdns3 *cdns = dev_get_drvdata(dev);
	int ret;

	if (!cdns->in_lpm) {
		WARN_ON(1);
		return 0;
	}

	ret = cdns3_prepare_enable_clks(dev);
	if (ret)
		return ret;

	usb_phy_set_suspend(cdns->usbphy, 0);
	/* At resume path, never return error */
	cdns3_enter_suspend(cdns, false, false);
	cdns->in_lpm = 0;

	if (cdns->role == CDNS3_ROLE_HOST) {
		if (cdns->wakeup_int) {
			cdns->wakeup_int = false;
			pm_runtime_mark_last_busy(cdns->dev);
			pm_runtime_put_autosuspend(cdns->dev);
			enable_irq(cdns->irq);
		}

		if ((cdns->role != CDNS3_ROLE_END) && cdns3_role(cdns)->resume)
			cdns3_role(cdns)->resume(cdns, false);
	}

	dev_dbg(dev, "at %s\n", __func__);
	return 0;
}
#endif /* CONFIG_PM */
static const struct dev_pm_ops cdns3_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(cdns3_suspend, cdns3_resume)
	SET_RUNTIME_PM_OPS(cdns3_runtime_suspend, cdns3_runtime_resume, NULL)
};

static struct platform_driver cdns3_driver = {
	.probe		= cdns3_probe,
	.remove		= cdns3_remove,
	.driver		= {
		.name	= "cdns-usb3",
		.of_match_table	= of_match_ptr(of_cdns3_match),
		.pm	= &cdns3_pm_ops,
	},
};

static int __init cdns3_driver_platform_register(void)
{
	cdns3_host_driver_init();
	return platform_driver_register(&cdns3_driver);
}
module_init(cdns3_driver_platform_register);

static void __exit cdns3_driver_platform_unregister(void)
{
	platform_driver_unregister(&cdns3_driver);
}
module_exit(cdns3_driver_platform_unregister);

MODULE_ALIAS("platform:cdns-usb3");
MODULE_AUTHOR("Peter Chen <peter.chen@nxp.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Cadence USB3 DRD Controller Driver");
