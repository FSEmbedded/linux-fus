/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright 2004-2007, 2010-2015 Freescale Semiconductor, Inc.
 * Copyright (C) 2008 Juergen Beisert (kernel@pengutronix.de)
 */

#ifndef __ASM_ARCH_MXC_H__
#define __ASM_ARCH_MXC_H__

#include <linux/types.h>
#include <soc/imx/cpu.h>

#ifndef __ASM_ARCH_MXC_HARDWARE_H__
#error "Do not include directly."
#endif

#define IMX_DDR_TYPE_DDR3		0
#define IMX_DDR_TYPE_LPDDR2		1
#define IMX_DDR_TYPE_LPDDR3		2
#define IMX_MMDC_DDR_TYPE_LPDDR3	3

#define IMX_LPDDR2_1CH_MODE            0
#define IMX_LPDDR2_2CH_MODE            1

#ifndef __ASSEMBLY__

#ifdef CONFIG_SOC_IMX6SL
static inline bool cpu_is_imx6sl(void)
{
	return __mxc_cpu_type == MXC_CPU_IMX6SL;
}
#else
#define cpu_is_imx6sl() false
#endif

#ifdef CONFIG_SOC_IMX6Q
static inline bool cpu_is_imx6dl(void)
{
	return __mxc_cpu_type == MXC_CPU_IMX6DL;
}
#else
#define cpu_is_imx6dl() false
#endif

#ifdef CONFIG_SOC_IMX6SX
static inline bool cpu_is_imx6sx(void)
{
	return __mxc_cpu_type == MXC_CPU_IMX6SX;
}
#else
#define cpu_is_imx6sx() false
#endif

#ifdef CONFIG_SOC_IMX6UL
static inline bool cpu_is_imx6ul(void)
{
	return __mxc_cpu_type == MXC_CPU_IMX6UL;
}

static inline bool cpu_is_imx6ull(void)
{
	return __mxc_cpu_type == MXC_CPU_IMX6ULL;
}

static inline bool cpu_is_imx6ulz(void)
{
	return __mxc_cpu_type == MXC_CPU_IMX6ULZ;
}
#else
#define cpu_is_imx6ul() false
#define cpu_is_imx6ull() false
#define cpu_is_imx6ulz() false
#endif

#ifdef CONFIG_SOC_IMX6SLL
static inline bool cpu_is_imx6sll(void)
{
	return __mxc_cpu_type == MXC_CPU_IMX6SLL;
}
#else
#define cpu_is_imx6sll() false
#endif

#ifdef CONFIG_SOC_IMX6Q
static inline bool cpu_is_imx6q(void)
{
	return __mxc_cpu_type == MXC_CPU_IMX6Q;
}
#else
#define cpu_is_imx6q() false
#endif

static inline bool cpu_is_imx6(void)
{
	return cpu_is_imx6q() || cpu_is_imx6dl() || cpu_is_imx6sl()
		|| cpu_is_imx6sx() || cpu_is_imx6ul() || cpu_is_imx6ull()
		|| cpu_is_imx6sll();
}

#ifdef CONFIG_SOC_IMX7D
static inline bool cpu_is_imx7d(void)
{
	return __mxc_cpu_type == MXC_CPU_IMX7D;
}
#else
#define cpu_is_imx7d() false
#endif

#ifdef CONFIG_SOC_IMX7ULP
static inline bool cpu_is_imx7ulp(void)
{
	return __mxc_cpu_type == MXC_CPU_IMX7ULP;
}
#else
#define cpu_is_imx7ulp() false
#endif

struct cpu_op {
	u32 cpu_rate;
};

int tzic_enable_wake(void);

extern struct cpu_op *(*get_cpu_op)(int *op);
#endif

#define imx_readl	readl_relaxed
#define imx_readw	readw_relaxed
#define imx_writel	writel_relaxed
#define imx_writew	writew_relaxed

#endif /*  __ASM_ARCH_MXC_H__ */
