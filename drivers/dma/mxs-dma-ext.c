/*
 * Copyright 2011-2012 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * Refer to drivers/dma/imx-sdma.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/types.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/dmaengine.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/mxs-dma-ext.h>
#include <linux/stmp_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_dma.h>

#include <asm/irq.h>
#include "dmaengine.h"			/* Cookie stuff */

/*
 * NOTE: The term "PIO" throughout the mxs-dma implementation means
 * PIO mode of mxs apbh-dma and apbx-dma.  With this working mode,
 * dma can program the controller registers of peripheral devices.
 */

#define dma_is_apbh(mxs_dma)	((mxs_dma)->type == MXS_DMA_APBH)
#define apbh_is_old(mxs_dma)	((mxs_dma)->dev_id == IMX23_DMA)

#define HW_APBHX_CTRL0				0x000
#define BM_APBH_CTRL0_APB_BURST8_EN		(1 << 29)
#define BM_APBH_CTRL0_APB_BURST_EN		(1 << 28)
#define BP_APBH_CTRL0_RESET_CHANNEL		16
#define HW_APBHX_CTRL1				0x010
#define HW_APBHX_CTRL2				0x020
#define HW_APBHX_CHANNEL_CTRL			0x030
#define BP_APBHX_CHANNEL_CTRL_RESET_CHANNEL	16
/*
 * The offset of NXTCMDAR register is different per both dma type and version,
 * while stride for each channel is all the same 0x70.
 */
#define HW_APBHX_CHn_NXTCMDAR(d, n) \
	(((dma_is_apbh(d) && apbh_is_old(d)) ? 0x050 : 0x110) + (n) * 0x70)
#define HW_APBHX_CHn_SEMA(d, n) \
	(((dma_is_apbh(d) && apbh_is_old(d)) ? 0x080 : 0x140) + (n) * 0x70)

#define CCW_BLOCK_SIZE	(4 * PAGE_SIZE)
#define NUM_CCW	(int)(CCW_BLOCK_SIZE / sizeof(struct mxs_dma_ccw))

struct mxs_dma_chan {
	struct mxs_dma_engine		*mxs_dma;
	struct dma_chan			chan;
	struct dma_async_tx_descriptor	desc;
	struct tasklet_struct		tasklet;
	unsigned int			chan_irq;
	struct mxs_dma_ccw		*ccw;
	dma_addr_t			ccw_phys;
	unsigned int			desc_index;
	unsigned int			desc_count;
	enum dma_status			status;
#define MXS_DMA_SG_LOOP			(1 << 0)
};

#define MXS_DMA_CHANNELS		16
#define MXS_DMA_CHANNELS_MASK		0xffff

enum mxs_dma_devtype {
	MXS_DMA_APBH,
	MXS_DMA_APBX,
};

enum mxs_dma_id {
	IMX23_DMA,
	IMX28_DMA,
};

struct mxs_dma_engine {
	enum mxs_dma_id			dev_id;
	enum mxs_dma_devtype		type;
	void __iomem			*base;
	struct clk			*clk;
	struct dma_device		dma_device;
	struct device_dma_parameters	dma_parms;
	struct mxs_dma_chan		mxs_chans[MXS_DMA_CHANNELS];
	struct platform_device		*pdev;
	unsigned int			nr_channels;
};

struct mxs_dma_type {
	enum mxs_dma_id id;
	enum mxs_dma_devtype type;
};

static struct mxs_dma_type mxs_dma_types[] = {
	{
		.id = IMX23_DMA,
		.type = MXS_DMA_APBH,
	}, {
		.id = IMX23_DMA,
		.type = MXS_DMA_APBX,
	}, {
		.id = IMX28_DMA,
		.type = MXS_DMA_APBH,
	}, {
		.id = IMX28_DMA,
		.type = MXS_DMA_APBX,
	}
};

static struct platform_device_id mxs_dma_ids[] = {
	{
		.name = "imx23-dma-apbh",
		.driver_data = (kernel_ulong_t) &mxs_dma_types[0],
	}, {
		.name = "imx23-dma-apbx",
		.driver_data = (kernel_ulong_t) &mxs_dma_types[1],
	}, {
		.name = "imx28-dma-apbh",
		.driver_data = (kernel_ulong_t) &mxs_dma_types[2],
	}, {
		.name = "imx28-dma-apbx",
		.driver_data = (kernel_ulong_t) &mxs_dma_types[3],
	}, {
		/* end of list */
	}
};

static const struct of_device_id mxs_dma_dt_ids[] = {
	{ .compatible = "fus,imx23-dma-apbh", .data = &mxs_dma_ids[0], },
	{ .compatible = "fus,imx23-dma-apbx", .data = &mxs_dma_ids[1], },
	{ .compatible = "fus,imx28-dma-apbh", .data = &mxs_dma_ids[2], },
	{ .compatible = "fus,imx28-dma-apbx", .data = &mxs_dma_ids[3], },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mxs_dma_dt_ids);

static struct mxs_dma_chan *to_mxs_dma_ext_chan(struct dma_chan *chan)
{
	return container_of(chan, struct mxs_dma_chan, chan);
}

static void mxs_dma_ext_start_new_chain(struct mxs_dma_chan *mxs_chan)
{
	struct mxs_dma_engine *mxs_dma = mxs_chan->mxs_dma;
	int chan_id = mxs_chan->chan.chan_id;
	u32 current_pos;

	/* The NEXTCMDAR points to the next desc that will be executed */
	current_pos = readl(mxs_dma->base + HW_APBHX_CHn_NXTCMDAR(mxs_dma,
								  chan_id));
	mxs_chan->desc_index =	(struct mxs_dma_ccw *)current_pos
				- (struct mxs_dma_ccw *)mxs_chan->ccw_phys;
	mxs_chan->desc_count = 0;

//	printk("### DMA %d: next chain starting @ 0x%08x\n", chan_id, current_pos);
}

static void mxs_dma_ext_reset_chan(struct mxs_dma_chan *mxs_chan)
{
	struct mxs_dma_engine *mxs_dma = mxs_chan->mxs_dma;
	int chan_id = mxs_chan->chan.chan_id;

//	printk("### DMA %d: resetting channel\n", chan_id);
	if (dma_is_apbh(mxs_dma) && apbh_is_old(mxs_dma))
		writel(1 << (chan_id + BP_APBH_CTRL0_RESET_CHANNEL),
			mxs_dma->base + HW_APBHX_CTRL0 + STMP_OFFSET_REG_SET);
	else
		writel(1 << (chan_id + BP_APBHX_CHANNEL_CTRL_RESET_CHANNEL),
			mxs_dma->base + HW_APBHX_CHANNEL_CTRL + STMP_OFFSET_REG_SET);

	/* Set cmd_addr for first descriptor */
	writel(mxs_chan->ccw_phys,
		mxs_dma->base + HW_APBHX_CHn_NXTCMDAR(mxs_dma, chan_id));

	mxs_dma_ext_start_new_chain(mxs_chan);
}

static void mxs_dma_ext_enable_chan(struct mxs_dma_chan *mxs_chan)
{
	struct mxs_dma_engine *mxs_dma = mxs_chan->mxs_dma;
	int chan_id = mxs_chan->chan.chan_id;

#if 0
	/* set cmd_addr up */
	writel(mxs_chan->ccw_phys,
		mxs_dma->base + HW_APBHX_CHn_NXTCMDAR(mxs_dma, chan_id));
#endif
//	printk("### DMA %d go: 0x%08x\n", chan_id,
//	       readl(mxs_dma->base + HW_APBHX_CHn_NXTCMDAR(chan_id)));

	/* write 1 to SEMA to kick off the channel */
	writel(1, mxs_dma->base + HW_APBHX_CHn_SEMA(mxs_dma, chan_id));
}

static void mxs_dma_ext_disable_chan(struct mxs_dma_chan *mxs_chan)
{
	mxs_chan->status = DMA_COMPLETE;
}

static void mxs_dma_ext_pause_chan(struct mxs_dma_chan *mxs_chan)
{
	struct mxs_dma_engine *mxs_dma = mxs_chan->mxs_dma;
	int chan_id = mxs_chan->chan.chan_id;

	/* freeze the channel */
	if (dma_is_apbh(mxs_dma) && apbh_is_old(mxs_dma))
		writel(1 << chan_id,
			mxs_dma->base + HW_APBHX_CTRL0 + STMP_OFFSET_REG_SET);
	else
		writel(1 << chan_id,
			mxs_dma->base + HW_APBHX_CHANNEL_CTRL + STMP_OFFSET_REG_SET);

	mxs_chan->status = DMA_PAUSED;
}

static void mxs_dma_ext_resume_chan(struct mxs_dma_chan *mxs_chan)
{
	struct mxs_dma_engine *mxs_dma = mxs_chan->mxs_dma;
	int chan_id = mxs_chan->chan.chan_id;

	/* unfreeze the channel */
	if (dma_is_apbh(mxs_dma) && apbh_is_old(mxs_dma))
		writel(1 << chan_id,
			mxs_dma->base + HW_APBHX_CTRL0 + STMP_OFFSET_REG_CLR);
	else
		writel(1 << chan_id,
			mxs_dma->base + HW_APBHX_CHANNEL_CTRL + STMP_OFFSET_REG_CLR);

	mxs_chan->status = DMA_IN_PROGRESS;
}

static void mxs_dma_ext_tasklet(unsigned long data)
{
	struct mxs_dma_chan *mxs_chan = (struct mxs_dma_chan *)data;

	if (mxs_chan->desc.callback)
		mxs_chan->desc.callback(mxs_chan->desc.callback_param);
}

static dma_cookie_t mxs_dma_ext_tx_submit(struct dma_async_tx_descriptor *tx)
{
	return dma_cookie_assign(tx);
}

static irqreturn_t mxs_dma_ext_int_handler(int irq, void *dev_id)
{
	struct mxs_dma_engine *mxs_dma = dev_id;
	u32 stat1, stat2;

	/* completion status */
	stat1 = readl(mxs_dma->base + HW_APBHX_CTRL1);
	stat1 &= MXS_DMA_CHANNELS_MASK;
	writel(stat1, mxs_dma->base + HW_APBHX_CTRL1 + STMP_OFFSET_REG_CLR);

	/* error status */
	stat2 = readl(mxs_dma->base + HW_APBHX_CTRL2);
	writel(stat2, mxs_dma->base + HW_APBHX_CTRL2 + STMP_OFFSET_REG_CLR);

	/*
	 * When both completion and error of termination bits set at the
	 * same time, we do not take it as an error.  IOW, it only becomes
	 * an error we need to handle here in case of either it's (1) a bus
	 * error or (2) a termination error with no completion.
	 */
	stat2 = ((stat2 >> MXS_DMA_CHANNELS) & stat2) | /* (1) */
		(~(stat2 >> MXS_DMA_CHANNELS) & stat2 & ~stat1); /* (2) */

	/* combine error and completion status for checking */
	stat1 = (stat2 << MXS_DMA_CHANNELS) | stat1;
	while (stat1) {
		int channel = fls(stat1) - 1;
		struct mxs_dma_chan *mxs_chan =
			&mxs_dma->mxs_chans[channel % MXS_DMA_CHANNELS];

		if (channel >= MXS_DMA_CHANNELS) {
			dev_dbg(mxs_dma->dma_device.dev,
				"%s: error in channel %d\n", __func__,
				channel - MXS_DMA_CHANNELS);
			mxs_chan->status = DMA_ERROR;
			mxs_dma_ext_reset_chan(mxs_chan);
		} else {
			mxs_chan->status = DMA_COMPLETE;
			mxs_dma_ext_start_new_chain(mxs_chan);
		}

		stat1 &= ~(1 << channel);

		if (mxs_chan->status == DMA_COMPLETE)
			dma_cookie_complete(&mxs_chan->desc);

		/* schedule tasklet on this channel */
		tasklet_schedule(&mxs_chan->tasklet);
	}

	return IRQ_HANDLED;
}

static int mxs_dma_ext_alloc_chan_resources(struct dma_chan *chan)
{
	struct mxs_dma_chan *mxs_chan = to_mxs_dma_ext_chan(chan);
	struct mxs_dma_engine *mxs_dma = mxs_chan->mxs_dma;
	struct mxs_dma_ccw *ccw_phys;
	int ret;
	int i, n;

	mxs_chan->ccw = dma_alloc_coherent(mxs_dma->dma_device.dev,
				CCW_BLOCK_SIZE, &mxs_chan->ccw_phys,
				GFP_KERNEL);
	if (!mxs_chan->ccw) {
		ret = -ENOMEM;
		goto err_alloc;
	}

	memset(mxs_chan->ccw, 0, CCW_BLOCK_SIZE);

	/* Set the next pointer of all descriptors to build a ring buffer */
	ccw_phys = (struct mxs_dma_ccw *)mxs_chan->ccw_phys;
//	printk("### DMA %d: ring buffer @ 0x%p, %d descriptors of size 0x%x\n",
//	       mxs_chan->chan.chan_id, ccw_phys, NUM_CCW,
//	       sizeof(struct mxs_dma_ccw));
	for (i = 0; i < NUM_CCW; i++)
	{
		n = i + 1;
		if (n >= NUM_CCW)
			n = 0;
		mxs_chan->ccw[i].next = (u32)(ccw_phys + n);
	}

	if (mxs_chan->chan_irq != NO_IRQ) {
		ret = request_irq(mxs_chan->chan_irq, mxs_dma_ext_int_handler,
					0, "mxs-dma", mxs_dma);
		if (ret)
			goto err_irq;
	}

	ret = clk_prepare_enable(mxs_dma->clk);
	if (ret)
		goto err_clk;

	mxs_dma_ext_reset_chan(mxs_chan);

	dma_async_tx_descriptor_init(&mxs_chan->desc, chan);
	mxs_chan->desc.tx_submit = mxs_dma_ext_tx_submit;

	/* the descriptor is ready */
	async_tx_ack(&mxs_chan->desc);

	return 0;

err_clk:
	free_irq(mxs_chan->chan_irq, mxs_dma);
err_irq:
	dma_free_coherent(mxs_dma->dma_device.dev, CCW_BLOCK_SIZE,
			mxs_chan->ccw, mxs_chan->ccw_phys);
err_alloc:
	return ret;
}

static void mxs_dma_ext_free_chan_resources(struct dma_chan *chan)
{
	struct mxs_dma_chan *mxs_chan = to_mxs_dma_ext_chan(chan);
	struct mxs_dma_engine *mxs_dma = mxs_chan->mxs_dma;

	mxs_dma_ext_disable_chan(mxs_chan);

	free_irq(mxs_chan->chan_irq, mxs_dma);

	dma_free_coherent(mxs_dma->dma_device.dev, CCW_BLOCK_SIZE,
			mxs_chan->ccw, mxs_chan->ccw_phys);

	clk_disable_unprepare(mxs_dma->clk);
}

/*
 * Add further DMA descriptors. We misuse the sgl pointer for this purpose.
 */
static struct dma_async_tx_descriptor *mxs_dma_ext_prep_slave_sg(
		struct dma_chan *chan, struct scatterlist *sgl,
		unsigned int sg_len, enum dma_transfer_direction direction,
		unsigned long flags, void *context)
{
	struct mxs_dma_chan *mxs_chan = to_mxs_dma_ext_chan(chan);
	struct mxs_dma_engine *mxs_dma = mxs_chan->mxs_dma;
	struct mxs_dma_ccw *ccw;
	struct mxs_dma_ccw *ccw_in;
	unsigned int index;
	unsigned int i;
	u32 cmd;

	/* An empty descriptor array can be used to cancel the DMA chain */
	if (!sg_len || !sgl) {
		mxs_chan->status = DMA_ERROR;
		mxs_dma_ext_start_new_chain(mxs_chan);

		return NULL;
	}

	/* Check if there is room for the new descriptors */
	if (mxs_chan->desc_count + sg_len > NUM_CCW) {
		dev_err(mxs_dma->dma_device.dev,
			"%d descriptors exceeded, canceling whole DMA chain\n",
			NUM_CCW);

		mxs_chan->status = DMA_ERROR;
		mxs_dma_ext_start_new_chain(mxs_chan);

		return NULL;
	}

	/* Remove end-of-chain signalling from the previous descriptor */
	index = mxs_chan->desc_index;
	if (mxs_chan->desc_count) {
		unsigned int last = index;

		if (!last)
			last = NUM_CCW;
		last--;
		mxs_chan->ccw[last].cmd &= ~(CCW_IRQ | CCW_DEC_SEM);
	}

	/* Copy the passed descriptors to the ring buffer */
	mxs_chan->desc_count += sg_len;
	ccw_in = (struct mxs_dma_ccw *)sgl;
	do {
		/*
		 * Copy main part of descriptor. Ignore "next" entries, they
		 * are already set to build the ring buffer. Add end-of-chain
		 * signalling to the last descriptor.
		 */
		ccw = &mxs_chan->ccw[index];
		cmd = ccw_in->cmd | CCW_CHAIN;
		if (sg_len > 1)
			cmd &= ~(CCW_IRQ | CCW_DEC_SEM);
		else
			cmd |= CCW_IRQ | CCW_DEC_SEM;
		ccw->cmd = cmd;
		ccw->bufaddr = ccw_in->bufaddr;

		/* Copy PIO words. No need to use memcpy() for these few. */
		cmd = (cmd & CCW_PIO_NUM_MASK) >> CCW_PIO_NUM_SHIFT;
		for (i = 0; i < cmd; i++)
			ccw->pio[i] = ccw_in->pio[i];

		/* Go to next descriptor */
		ccw_in++;
		if (++index >= NUM_CCW)
			index = 0;
	} while (--sg_len);

	mxs_chan->desc_index = index;
	mxs_chan->status = DMA_IN_PROGRESS;

	return &mxs_chan->desc;
}

static int mxs_dma_ext_control(struct dma_chan *chan, enum dma_ctrl_cmd cmd,
		unsigned long arg)
{
	struct mxs_dma_chan *mxs_chan = to_mxs_dma_ext_chan(chan);
	int ret = 0;

	switch (cmd) {
	case DMA_TERMINATE_ALL:
		mxs_dma_ext_reset_chan(mxs_chan);
		mxs_dma_ext_disable_chan(mxs_chan);
		break;
	case DMA_PAUSE:
		mxs_dma_ext_pause_chan(mxs_chan);
		break;
	case DMA_RESUME:
		mxs_dma_ext_resume_chan(mxs_chan);
		break;
	default:
		ret = -ENOSYS;
	}

	return ret;
}

static enum dma_status mxs_dma_ext_tx_status(struct dma_chan *chan,
			dma_cookie_t cookie, struct dma_tx_state *txstate)
{
	struct mxs_dma_chan *mxs_chan = to_mxs_dma_ext_chan(chan);

	dma_set_tx_state(txstate, chan->completed_cookie, chan->cookie, 0);

	return mxs_chan->status;
}

static void mxs_dma_ext_issue_pending(struct dma_chan *chan)
{
	struct mxs_dma_chan *mxs_chan = to_mxs_dma_ext_chan(chan);

	if (mxs_chan->desc_count) {
//		printk("### DMA: executing chain with %d entries\n", mxs_chan->desc_count);
		mxs_dma_ext_enable_chan(mxs_chan);
	} else {
//		printk("### DMA: Empty chain, nothing to do\n");
		mxs_chan->status = DMA_COMPLETE;

		/* schedule tasklet for callback on this channel */
		tasklet_schedule(&mxs_chan->tasklet);
	}
}

static int __init mxs_dma_ext_init(struct mxs_dma_engine *mxs_dma)
{
	int ret;

	ret = clk_prepare_enable(mxs_dma->clk);
	if (ret)
		return ret;

	ret = stmp_reset_block(mxs_dma->base);
	if (ret)
		goto err_out;

	/* enable apbh burst */
	if (dma_is_apbh(mxs_dma)) {
		writel(BM_APBH_CTRL0_APB_BURST_EN,
			mxs_dma->base + HW_APBHX_CTRL0 + STMP_OFFSET_REG_SET);
		writel(BM_APBH_CTRL0_APB_BURST8_EN,
			mxs_dma->base + HW_APBHX_CTRL0 + STMP_OFFSET_REG_SET);
	}

	/* enable irq for all the channels */
	writel(MXS_DMA_CHANNELS_MASK << MXS_DMA_CHANNELS,
		mxs_dma->base + HW_APBHX_CTRL1 + STMP_OFFSET_REG_SET);

err_out:
	clk_disable_unprepare(mxs_dma->clk);
	return ret;
}

struct mxs_dma_filter_param {
	struct device_node *of_node;
	unsigned int chan_id;
};

static bool mxs_dma_filter_fn(struct dma_chan *chan, void *fn_param)
{
	struct mxs_dma_filter_param *param = fn_param;
	struct mxs_dma_chan *mxs_chan = to_mxs_dma_ext_chan(chan);
	struct mxs_dma_engine *mxs_dma = mxs_chan->mxs_dma;
	int chan_irq;

	if (mxs_dma->dma_device.dev->of_node != param->of_node)
		return false;

	if (chan->chan_id != param->chan_id)
		return false;

	chan_irq = platform_get_irq(mxs_dma->pdev, param->chan_id);
	if (chan_irq < 0)
		return false;

	mxs_chan->chan_irq = chan_irq;

	return true;
}

static struct dma_chan *mxs_dma_xlate(struct of_phandle_args *dma_spec,
			       struct of_dma *ofdma)
{
	struct mxs_dma_engine *mxs_dma = ofdma->of_dma_data;
	dma_cap_mask_t mask = mxs_dma->dma_device.cap_mask;
	struct mxs_dma_filter_param param;

	if (dma_spec->args_count != 1)
		return NULL;

	param.of_node = ofdma->of_node;
	param.chan_id = dma_spec->args[0];

	if (param.chan_id >= mxs_dma->nr_channels)
		return NULL;

	return dma_request_channel(mask, mxs_dma_filter_fn, &param);
}

static int __init mxs_dma_ext_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	const struct platform_device_id *id_entry;
	const struct of_device_id *of_id;
	const struct mxs_dma_type *dma_type;
	struct mxs_dma_engine *mxs_dma;
	struct resource *iores;
	int ret, i;

	mxs_dma = devm_kzalloc(&pdev->dev, sizeof(*mxs_dma), GFP_KERNEL);
	if (!mxs_dma)
		return -ENOMEM;

	ret = of_property_read_u32(np, "dma-channels", &mxs_dma->nr_channels);
	if (ret) {
		dev_err(&pdev->dev, "failed to read dma-channels\n");
		return ret;
	}

	of_id = of_match_device(mxs_dma_dt_ids, &pdev->dev);
	if (of_id)
		id_entry = of_id->data;
	else
		id_entry = platform_get_device_id(pdev);

	dma_type = (struct mxs_dma_type *)id_entry->driver_data;
	mxs_dma->type = dma_type->type;
	mxs_dma->dev_id = dma_type->id;

	iores = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	mxs_dma->base = devm_ioremap_resource(&pdev->dev, iores);
	if (IS_ERR(mxs_dma->base))
		return PTR_ERR(mxs_dma->base);

	mxs_dma->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(mxs_dma->clk))
		return PTR_ERR(mxs_dma->clk);

	dma_cap_set(DMA_SLAVE, mxs_dma->dma_device.cap_mask);

	INIT_LIST_HEAD(&mxs_dma->dma_device.channels);

	/* Initialize channel parameters */
	for (i = 0; i < MXS_DMA_CHANNELS; i++) {
		struct mxs_dma_chan *mxs_chan = &mxs_dma->mxs_chans[i];

		mxs_chan->mxs_dma = mxs_dma;
		mxs_chan->chan.device = &mxs_dma->dma_device;
		dma_cookie_init(&mxs_chan->chan);

		tasklet_init(&mxs_chan->tasklet, mxs_dma_ext_tasklet,
			     (unsigned long) mxs_chan);


		/* Add the channel to mxs_chan list */
		list_add_tail(&mxs_chan->chan.device_node,
			&mxs_dma->dma_device.channels);
	}

	ret = mxs_dma_ext_init(mxs_dma);
	if (ret)
		return ret;

	mxs_dma->pdev = pdev;
	mxs_dma->dma_device.dev = &pdev->dev;

	/* mxs_dma gets 65535 bytes maximum sg size */
	mxs_dma->dma_device.dev->dma_parms = &mxs_dma->dma_parms;
	dma_set_max_seg_size(mxs_dma->dma_device.dev, MAX_XFER_BYTES);

	mxs_dma->dma_device.device_alloc_chan_resources =
					mxs_dma_ext_alloc_chan_resources;
	mxs_dma->dma_device.device_free_chan_resources =
					mxs_dma_ext_free_chan_resources;
	mxs_dma->dma_device.device_tx_status = mxs_dma_ext_tx_status;
	mxs_dma->dma_device.device_prep_slave_sg = mxs_dma_ext_prep_slave_sg;
	mxs_dma->dma_device.device_control = mxs_dma_ext_control;
	mxs_dma->dma_device.device_issue_pending = mxs_dma_ext_issue_pending;

	ret = dma_async_device_register(&mxs_dma->dma_device);
	if (ret) {
		dev_err(mxs_dma->dma_device.dev, "unable to register\n");
		return ret;
	}

	ret = of_dma_controller_register(np, mxs_dma_xlate, mxs_dma);
	if (ret) {
		dev_err(mxs_dma->dma_device.dev,
			"failed to register controller\n");
		dma_async_device_unregister(&mxs_dma->dma_device);
	}

	dev_info(mxs_dma->dma_device.dev, "initialized\n");

	return 0;
}

static struct platform_driver mxs_dma_ext_driver = {
	.driver		= {
		.name	= "mxs-dma-ext",
		.of_match_table = mxs_dma_dt_ids,
	},
	.id_table	= mxs_dma_ids,
};

static int __init mxs_dma_ext_module_init(void)
{
	return platform_driver_probe(&mxs_dma_ext_driver, mxs_dma_ext_probe);
}
subsys_initcall(mxs_dma_ext_module_init);
