/**
 * core.h - Cadence USB3 DRD Controller Core header file
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

#ifndef __DRIVERS_USB_CDNS3_CORE_H
#define __DRIVERS_USB_CDNS3_CORE_H

struct cdns3;
enum cdns3_roles {
	CDNS3_ROLE_HOST = 0,
	CDNS3_ROLE_GADGET,
	CDNS3_ROLE_END,
};

/**
 * struct cdns3_role_driver - host/gadget role driver
 * @start: start this role
 * @stop: stop this role
 * @suspend: suspend callback for this role
 * @resume: resume callback for this role
 * @irq: irq handler for this role
 * @thread_irq: thread irq handler for this role
 * @name: role name string (host/gadget)
 */
struct cdns3_role_driver {
	int (*start)(struct cdns3 *);
	void (*stop)(struct cdns3 *);
	int (*suspend)(struct cdns3 *, bool do_wakeup);
	int (*resume)(struct cdns3 *, bool hibernated);
	irqreturn_t (*irq)(struct cdns3 *);
	irqreturn_t (*thread_irq)(struct cdns3 *);
	const char *name;
};

#define CDNS3_XHCI_RESOURCES_NUM	2

struct cdns3_platform_data {
	int (*platform_suspend)(struct device *dev,
			bool suspend, bool wakeup);
	unsigned long quirks;
#define CDNS3_DEFAULT_PM_RUNTIME_ALLOW	BIT(0)
};

/**
 * struct cdns3 - Representation of Cadence USB3 DRD controller.
 * @dev: pointer to Cadence device struct
 * @xhci_regs: pointer to base of xhci registers
 * @xhci_res: the resource for xhci
 * @dev_regs: pointer to base of dev registers
 * @none_core_regs: pointer to base of nxp wrapper registers
 * @phy_regs: pointer to base of phy registers
 * @otg_regs: pointer to base of otg registers
 * @otg_irq: irq number for otg controller
 * @dev_irq: irq number for device controller
 * @wakeup_irq: irq number for wakeup event, it is optional
 * @roles: array of supported roles for this controller
 * @role: current role
 * @host_dev: the child host device pointer for cdns3 core
 * @gadget_dev: the child gadget device pointer for cdns3 core
 * @usbphy: usbphy for this controller
 * @cdns3_clks: Clock pointer array for cdns3 core
 * @extcon: Type-C extern connector
 * @extcon_nb: notifier block for Type-C extern connector
 * @role_switch_wq: work queue item for role switch
 * @in_lpm: the controller in low power mode
 * @wakeup_int: the wakeup interrupt
 * @mutex: the mutex for concurrent code at driver
 * @dr_mode: supported mode of operation it can be only Host, only Device
 *           or OTG mode that allow to switch between Device and Host mode.
 *           This field based on firmware setting, kernel configuration
 *           and hardware configuration.
 * @role_sw: pointer to role switch object.
 * @in_lpm: indicate the controller is in low power mode
 * @wakeup_pending: wakeup interrupt pending
 * @pdata: platform data from glue layer
 * @lock: spinlock structure
 * @xhci_plat_data: xhci private data structure pointer
 */
struct cdns3 {
	struct device			*dev;
	void __iomem			*xhci_regs;
	struct resource			xhci_res[CDNS3_XHCI_RESOURCES_NUM];
	struct cdns3_usb_regs __iomem	*dev_regs;

	struct resource			otg_res;
	struct cdns3_otg_legacy_regs	*otg_v0_regs;
	struct cdns3_otg_regs		*otg_v1_regs;
	struct cdns3_otg_common_regs	*otg_regs;
#define CDNS3_CONTROLLER_V0	0
#define CDNS3_CONTROLLER_V1	1
	u32				version;
	bool				phyrst_a_enable;

	int				otg_irq;
	int				dev_irq;
	int				wakeup_irq;
	struct cdns3_role_driver	*roles[USB_ROLE_DEVICE + 1];
	enum usb_role			role;
	struct platform_device		*host_dev;
	struct cdns3_device		*gadget_dev;
	struct phy			*usb2_phy;
	struct phy			*usb3_phy;
	/* mutext used in workqueue*/
	struct mutex			mutex;
	enum usb_dr_mode		dr_mode;
	struct usb_role_switch		*role_sw;
	bool				in_lpm;
	bool				wakeup_pending;
	struct cdns3_platform_data	*pdata;
	spinlock_t			lock;
	struct xhci_plat_priv		*xhci_plat_data;
};

static inline struct cdns3_role_driver *cdns3_role(struct cdns3 *cdns)
{
	WARN_ON(cdns->role >= CDNS3_ROLE_END || !cdns->roles[cdns->role]);
	return cdns->roles[cdns->role];
}

static inline int cdns3_role_start(struct cdns3 *cdns, enum cdns3_roles role)
{
	int ret;
	if (role >= CDNS3_ROLE_END)
		return 0;

	if (!cdns->roles[role])
		return -ENXIO;

	mutex_lock(&cdns->mutex);
	cdns->role = role;
	ret = cdns->roles[role]->start(cdns);
	mutex_unlock(&cdns->mutex);
	return ret;
}

static inline void cdns3_role_stop(struct cdns3 *cdns)
{
	enum cdns3_roles role = cdns->role;

	if (role == CDNS3_ROLE_END)
		return;

	mutex_lock(&cdns->mutex);
	cdns->roles[role]->stop(cdns);
	cdns->role = CDNS3_ROLE_END;
	mutex_unlock(&cdns->mutex);
}
int cdns3_handshake(void __iomem *ptr, u32 mask, u32 done, int usec);

#endif /* __DRIVERS_USB_CDNS3_CORE_H */
