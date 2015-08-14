/*
 * Copyright 2011-2012 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __MACH_MXS_DMA_EXT_H__
#define __MACH_MXS_DMA_EXT_H__

#include <linux/dmaengine.h>

#define CCW_CMD_DMA_NO_XFER	(0 << 0)
#define CCW_CMD_DMA_WRITE	(1 << 0)
#define CCW_CMD_DMA_READ	(2 << 0)
#define CCW_CMD_DMA_SENSE	(3 << 0) /* not implemented! */
#define CCW_CMD_MASK		(3 << 0)
#define CCW_CMD_SHIFT		0
#define CCW_CHAIN		(1 << 2) /* automatically set by driver */
#define CCW_IRQ			(1 << 3)
#define CCW_NAND_LOCK		(1 << 4)
#define CCW_NAND_WAIT4READY	(1 << 5)
#define CCW_DEC_SEM		(1 << 6) /* automatically set by driver */
#define CCW_WAIT4END		(1 << 7)
#define CCW_HALT_ON_TERM	(1 << 8)
#define CCW_TERM_FLUSH		(1 << 9) /* not available on i.MX6 */
#define CCW_PIO_NUM(n)		((n) << 12)
#define CCW_PIO_NUM_MASK	(0xF << 12)
#define CCW_PIO_NUM_SHIFT	12
#define CCW_XFER_COUNT(n)	((n) << 16)
#define CCW_XFER_COUNT_MASK	(0xFFFF << 16)
#define CCW_XFER_COUNT_SHIFT	16

/* We currently only support GPMI, so 6 PIO words are enough */
#define MAX_XFER_BYTES	0xff00
#define MXS_PIO_WORDS	6

struct mxs_dma_ccw {
	u32 next;			/* set to zero, driver will replace */
	u32 cmd;			/* see CCW_* above */
	u32 bufaddr;			/* physical address! */
	u32 pio[MXS_PIO_WORDS];
};

struct mxs_dma_data {
	int chan_irq;
};

static inline int mxs_dma_ext_is_apbh(struct dma_chan *chan)
{
	return !strcmp(dev_name(chan->device->dev), "mxs-dma-ext-apbh");
}

static inline int mxs_dma_ext_is_apbx(struct dma_chan *chan)
{
	return !strcmp(dev_name(chan->device->dev), "mxs-dma-ext-apbx");
}

#endif /* __MACH_MXS_DMA_EXT_H__ */
