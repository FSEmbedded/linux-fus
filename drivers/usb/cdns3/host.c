/*
 * host.c - Cadence USB3 host controller driver
 *
 * Copyright 2017 NXP
 * Authors: Peter Chen <peter.chen@nxp.com>
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include <linux/pm_runtime.h>
#include <linux/usb/of.h>

#include "../host/xhci.h"

#include "core.h"
#include "host-export.h"
#include <linux/usb/hcd.h>
#include "../host/xhci.h"
#include "../host/xhci-plat.h"

#define XECP_PORT_CAP_REG	0x8000
#define XECP_AUX_CTRL_REG1	0x8120

#define CFG_RXDET_P3_EN		BIT(15)
#define LPM_2_STB_SWITCH_EN	BIT(25)

static const struct xhci_plat_priv xhci_plat_cdns3_xhci = {
	.quirks = XHCI_SKIP_PHY_INIT,
	.suspend_quirk = xhci_cdns3_suspend_quirk,
};

#define XHCI_WAKEUP_STATUS     (PORT_RC | PORT_PLC)

static struct hc_driver __read_mostly xhci_cdns3_hc_driver;

static void xhci_cdns3_quirks(struct device *dev, struct xhci_hcd *xhci)
{
	/*
	 * As of now platform drivers don't provide MSI support so we ensure
	 * here that the generic code does not try to make a pci_dev from our
	 * dev struct in order to setup MSI
	 */
	xhci->quirks |= (XHCI_PLAT | XHCI_AVOID_BEI | XHCI_CDNS_HOST);
}

static int xhci_cdns3_setup(struct usb_hcd *hcd)
{
	int ret;
	struct usb_hcd *hcd;

	cdns3_drd_host_on(cdns);

	xhci = platform_device_alloc("xhci-hcd", PLATFORM_DEVID_AUTO);
	if (!xhci) {
		dev_err(cdns->dev, "couldn't allocate xHCI device\n");
		return -ENOMEM;
	}

	xhci->dev.parent = cdns->dev;
	cdns->host_dev = xhci;

	ret = platform_device_add_resources(xhci, cdns->xhci_res,
					    CDNS3_XHCI_RESOURCES_NUM);
	if (ret) {
		dev_err(cdns->dev, "couldn't add resources to xHCI device\n");
		goto err1;
	}

	cdns->xhci_plat_data = kmemdup(&xhci_plat_cdns3_xhci,
			sizeof(struct xhci_plat_priv), GFP_KERNEL);
	if (!cdns->xhci_plat_data) {
		ret = -ENOMEM;
		goto err1;
	}

	if (cdns->pdata && (cdns->pdata->quirks & CDNS3_DEFAULT_PM_RUNTIME_ALLOW))
		cdns->xhci_plat_data->quirks |= XHCI_DEFAULT_PM_RUNTIME_ALLOW;

	ret = platform_device_add_data(xhci, cdns->xhci_plat_data,
			sizeof(struct xhci_plat_priv));
	if (ret)
		goto free_memory;

	ret = platform_device_add(xhci);
	if (ret) {
		dev_err(cdns->dev, "failed to register xHCI device\n");
		goto free_memory;
	}

	/* Glue needs to access xHCI region register for Power management */
	hcd = platform_get_drvdata(xhci);
	if (hcd)
		cdns->xhci_regs = hcd->regs;

	return 0;

free_memory:
	kfree(cdns->xhci_plat_data);
err1:
	put_device(dev);
	cdns->host_dev = NULL;
	return ret;
}

int xhci_cdns3_suspend_quirk(struct usb_hcd *hcd)
{
	struct xhci_hcd	*xhci = hcd_to_xhci(hcd);
	u32 value;

	if (pm_runtime_status_suspended(hcd->self.controller))
		return 0;

	/* set usbcmd.EU3S */
	value = readl(&xhci->op_regs->command);
	value |= CMD_PM_INDEX;
	writel(value, &xhci->op_regs->command);

	if (hcd->regs) {
		value = readl(hcd->regs + XECP_AUX_CTRL_REG1);
		value |= CFG_RXDET_P3_EN;
		writel(value, hcd->regs + XECP_AUX_CTRL_REG1);

		value = readl(hcd->regs + XECP_PORT_CAP_REG);
		value |= LPM_2_STB_SWITCH_EN;
		writel(value, hcd->regs + XECP_PORT_CAP_REG);
	}

	return 0;
}

static void cdns3_host_exit(struct cdns3 *cdns)
{
	kfree(cdns->xhci_plat_data);
	platform_device_unregister(cdns->host_dev);
	cdns->host_dev = NULL;
	cdns3_drd_host_off(cdns);
}

int cdns3_host_init(struct cdns3 *cdns)
{
	struct cdns3_role_driver *rdrv;

	rdrv = devm_kzalloc(cdns->dev, sizeof(*rdrv), GFP_KERNEL);
	if (!rdrv)
		return -ENOMEM;

	rdrv->start	= cdns3_host_start;
	rdrv->stop	= cdns3_host_stop;
	rdrv->irq	= cdns3_host_irq;
	rdrv->suspend	= cdns3_host_suspend;
	rdrv->resume	= cdns3_host_resume;
	rdrv->name	= "host";
	cdns->roles[CDNS3_ROLE_HOST] = rdrv;

	return 0;
}

void cdns3_host_remove(struct cdns3 *cdns)
{
	cdns3_host_stop(cdns);
}

void __init cdns3_host_driver_init(void)
{
	xhci_init_driver(&xhci_cdns3_hc_driver, &xhci_cdns3_overrides);
}
