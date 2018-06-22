/*
 * F&S i.MX6 GPMI NAND flash driver
 *
 * Copyright (C) 2015 F&S Elektronik Systeme GmbH
 *
 * This driver uses parts of the code of the Freescale GPMI NAND flash driver
 * (see gpmi-nand.c and gpmi-lib.c).
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

/*
 * ### TODO-List:
 * - Set correct timeout values in TIMING1 for WAIT4READY descriptors
 * - Set clock to be compatible to NBoot/U-Boot.
 * - Check all gpmi-lib calls (duplicated here) if they can be removed.
 * - Probably make gpmi-regs.h and bch-regs.h easier.
 */

#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/mxs-dma-ext.h>		/* struct mxs_dma_ccw, CCW_*, ... */
#include <linux/platform_device.h>	/* GPMI_ */
#include <linux/dma-mapping.h>		/* dma_alloc_coherent(), ... */
#include <linux/delay.h>		/* udelay() */
#include <linux/pm_runtime.h>
#include <linux/busfreq-imx.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include "gpmi-regs.h"
#include "bch-regs.h"

/* Resource names for the GPMI NAND driver. */
#define GPMI_NAND_GPMI_REGS_ADDR_RES_NAME  "gpmi-nand"
#define GPMI_NAND_BCH_REGS_ADDR_RES_NAME   "bch"
#define GPMI_NAND_BCH_INTERRUPT_RES_NAME   "bch"

/* Default metadata size if no ecc_strength is given in platform data */
#define GPMI_NAND_METADATA_SIZE	32

/* Part of buffer that is used for commands */
#define GPMI_FUS_CMD_BUF_SIZE	64

/* Timeout values (in ms) */
#define GPMI_FUS_TIMEOUT_RESET	10
#define GPMI_FUS_TIMEOUT_DATA	100
#define GPMI_FUS_TIMEOUT_WRITE	100
#define GPMI_FUS_TIMEOUT_ERASE	400
#define GPMI_FUS_TIMEOUT_RPM	50

/* BCH : Status Block Completion Codes */
#define STATUS_GOOD		0x00
#define STATUS_ERASED		0xff
#define STATUS_UNCORRECTABLE	0xfe

/* Use the type to distinguish different Archs. */
#define GPMI_IS_MX23(x)		((x)->devdata->type == IS_MX23)
#define GPMI_IS_MX28(x)		((x)->devdata->type == IS_MX28)
#define GPMI_IS_MX6Q(x)		((x)->devdata->type == IS_MX6Q)
#define GPMI_IS_MX6QP(x)	((x)->devdata->type == IS_MX6QP)
#define GPMI_IS_MX6SX(x)	((x)->devdata->type == IS_MX6SX)
#define GPMI_IS_MX7D(x)		((x)->devdata->type == IS_MX7D)
#define GPMI_IS_MX6UL(x)	((x)->devdata->type == IS_MX6UL)
/* i.MX6UL version also works for i.MX6ULL */

#define GPMI_IS_MX6(x)		(GPMI_IS_MX6Q(x) || GPMI_IS_MX6QP(x)\
	   || GPMI_IS_MX6SX(x) || GPMI_IS_MX6UL(x))
#define GPMI_IS_MX7(x)		(GPMI_IS_MX7D(x))


#define GPMI_CLK_MAX 5 /* MX6Q needs five clocks */
struct resources {
	void __iomem  *gpmi_regs;
	void __iomem  *bch_regs;
	unsigned int  dma_low_channel;
	unsigned int  dma_high_channel;
	struct clk    *clock[GPMI_CLK_MAX];
};

/* DMA operations types */
enum dma_ops_type {
	DMA_FOR_COMMAND = 1,
	DMA_FOR_READ_DATA,
	DMA_FOR_WRITE_DATA,
	DMA_FOR_READ_ECC_PAGE,
	DMA_FOR_WRITE_ECC_PAGE
};

/**
 * struct nand_timing - Fundamental timing attributes for NAND.
 * @data_setup_in_ns:         The data setup time, in nanoseconds. Usually the
 *                            maximum of tDS and tWP. A negative value
 *                            indicates this characteristic isn't known.
 * @data_hold_in_ns:          The data hold time, in nanoseconds. Usually the
 *                            maximum of tDH, tWH and tREH. A negative value
 *                            indicates this characteristic isn't known.
 * @address_setup_in_ns:      The address setup time, in nanoseconds. Usually
 *                            the maximum of tCLS, tCS and tALS. A negative
 *                            value indicates this characteristic isn't known.
 * @gpmi_sample_delay_in_ns:  A GPMI-specific timing parameter. A negative value
 *                            indicates this characteristic isn't known.
 * @tREA_in_ns:               tREA, in nanoseconds, from the data sheet. A
 *                            negative value indicates this characteristic isn't
 *                            known.
 * @tRLOH_in_ns:              tRLOH, in nanoseconds, from the data sheet. A
 *                            negative value indicates this characteristic isn't
 *                            known.
 * @tRHOH_in_ns:              tRHOH, in nanoseconds, from the data sheet. A
 *                            negative value indicates this characteristic isn't
 *                            known.
 */
struct nand_timing {
	int8_t  data_setup_in_ns;
	int8_t  data_hold_in_ns;
	int8_t  address_setup_in_ns;
	int8_t  gpmi_sample_delay_in_ns;
	int8_t  tREA_in_ns;
	int8_t  tRLOH_in_ns;
	int8_t  tRHOH_in_ns;
};

enum gpmi_type {
	IS_MX23,
	IS_MX28,
	IS_MX6Q,
	IS_MX6QP,
	IS_MX6SX,
	IS_MX7D,
	IS_MX6UL
};

struct gpmi_devdata {
	enum gpmi_type type;
	int bch_max_ecc_strength;
	int max_chain_delay; /* See the async EDO mode */
};

#define ASYNC_EDO_ENABLED		1
#define ASYNC_EDO_TIMING_CONFIGED	2
struct gpmi_nand_data {
	/* Flags */
#define GPMI_ASYNC_EDO_ENABLED	(1 << 0)
#define GPMI_TIMING_INIT_OK	(1 << 1)
	int			flags;
	const struct gpmi_devdata *devdata;

	/* System Interface */
	struct device		*dev;
	struct platform_device	*pdev;

	/* Resources */
	struct resources	resources;

	/* Flash Hardware */
	struct nand_timing	timing;
	int			timing_mode;

	/* BCH */
	struct completion	bch_done;

	/* NAND Boot issue */
	bool			swap_block_mark;

	/* MTD / NAND */
	struct nand_chip	nand;
	struct mtd_info		mtd;

	/* General-use Variables */
	int			current_chip;
	unsigned int		command_length;

	/* for DMA operations */
	bool			direct_dma_map_ok;

	uint8_t			*cmd_buffer_virt; /* cmd/data/status */
	dma_addr_t		cmd_buffer_phys;

	uint8_t			*data_buffer_virt;
	dma_addr_t		data_buffer_phys;

	uint8_t			*page_buffer_virt; /* full page data */
	dma_addr_t		page_buffer_phys;

	uint8_t			*auxiliary_virt;
	dma_addr_t		auxiliary_phys;

	uint8_t			*status_virt;

	/* DMA channels */
#define DMA_CHANS		8
	struct dma_chan		*dma_chans[DMA_CHANS];
	struct completion	dma_done;

	/* private */
	void			*private;
	unsigned int		gf_len;
	unsigned int		bch_layout;
	uint8_t			column_cycles;
	uint8_t			row_cycles;
};

/**
 * struct gpmi_nfc_hardware_timing - GPMI hardware timing parameters.
 * @data_setup_in_cycles:      The data setup time, in cycles.
 * @data_hold_in_cycles:       The data hold time, in cycles.
 * @address_setup_in_cycles:   The address setup time, in cycles.
 * @device_busy_timeout:       The timeout waiting for NAND Ready/Busy,
 *                             this value is the number of cycles multiplied
 *                             by 4096.
 * @use_half_periods:          Indicates the clock is running slowly, so the
 *                             NFC DLL should use half-periods.
 * @sample_delay_factor:       The sample delay factor.
 * @wrn_dly_sel:               The delay on the GPMI write strobe.
 */
struct gpmi_nfc_hardware_timing {
	/* for GPMI_HW_GPMI_TIMING0 */
	uint8_t  data_setup_in_cycles;
	uint8_t  data_hold_in_cycles;
	uint8_t  address_setup_in_cycles;

	/* for GPMI_HW_GPMI_TIMING1 */
	uint16_t device_busy_timeout;

	/* for GPMI_HW_GPMI_CTRL1 */
	bool     use_half_periods;
	uint8_t  sample_delay_factor;
	uint8_t  wrn_dly_sel;
};

/**
 * struct timing_threshod - Timing threshold
 * @max_data_setup_cycles:       The maximum number of data setup cycles that
 *                               can be expressed in the hardware.
 * @internal_data_setup_in_ns:   The time, in ns, that the NFC hardware requires
 *                               for data read internal setup. In the Reference
 *                               Manual, see the chapter "High-Speed NAND
 *                               Timing" for more details.
 * @max_sample_delay_factor:     The maximum sample delay factor that can be
 *                               expressed in the hardware.
 * @max_dll_clock_period_in_ns:  The maximum period of the GPMI clock that the
 *                               sample delay DLL hardware can possibly work
 *                               with (the DLL is unusable with longer periods).
 *                               If the full-cycle period is greater than HALF
 *                               this value, the DLL must be configured to use
 *                               half-periods.
 * @max_dll_delay_in_ns:         The maximum amount of delay, in ns, that the
 *                               DLL can implement.
 * @clock_frequency_in_hz:       The clock frequency, in Hz, during the current
 *                               I/O transaction. If no I/O transaction is in
 *                               progress, this is the clock frequency during
 *                               the most recent I/O transaction.
 */
struct timing_threshod {
	const unsigned int      max_chip_count;
	const unsigned int      max_data_setup_cycles;
	const unsigned int      internal_data_setup_in_ns;
	const unsigned int      max_sample_delay_factor;
	const unsigned int      max_dll_clock_period_in_ns;
	const unsigned int      max_dll_delay_in_ns;
	unsigned long           clock_frequency_in_hz;

};

/*
 * F&S NAND page layout for i.MX6. The layout is heavily influenced by data
 * flow requirements of the BCH error correction engine.
 *
 * The OOB area is put first in the page. Then the main page data follows in
 * chunks of 512 or 1024 bytes, interleaved with ECC. For 1024 bytes chunk
 * size, GF14 is needed, which means 14 bits of ECC data per ECC step. For 512
 * bytes chunk size, GF13 is sufficient, which means 13 bits per ECC step. If
 * CONFIG_SYS_NAND_MXS_CHUNK_1K is set, the driver uses 1024 bytes chunks,
 * otherwise 512 bytes chunks.
 *
 * The bad block marker will also be located at byte 0 of the (main) page, NOT
 * on the first byte of the spare area anymore! We need at least 4 bytes of
 * OOB data for the bad block marker and the backup block number.
 *
 * |                  NAND flash main area                      | Spare area |
 * +-----+----------+--------+-------+--------+-------+-----+--------+-------+
 * | BBM | User OOB | Main 0 | ECC 0 | Main 1 | ECC 1 | ... | Main n | ECC n |
 * +-----+----------+--------+-------+--------+-------+-----+--------+-------+
 *    4    oobavail  512/1024         512/1024               512/1024
 *
 * The ECC size depends on the ECC strength and the chunk size. It is not
 * necessarily a multiple of 8 bits, which means that subsequent sections may
 * not be byte aligned anymore! But as long as we use NAND flashes with at
 * least 2K page sizes, the final ECC section will always end byte aligned, no
 * matter how odd the intermediate section crossings may be. This is even true
 * for GF13.
 *
 * Example 1
 * ---------
 * NAND page size is 2048 + 64 = 2112 bytes. Chunk size is set to 512 bytes
 * which means we can use GF13 and have four chunks. NBoot reports ECC2, which
 * means 13 bits * 2 = 26 bits ECC data (3.25 bytes). So the main data will
 * take 4x (512 + 3.25) = 2061 bytes. So we will have 2112 - 2061 = 51 bytes
 * free space for OOB, from which we need 4 bytes for internal purposes. The
 * remaining free space is 47 bytes for the User OOB data (mtd->oobavail).
 *
 * This results in the following rather complicated layout. Please note that
 * the ECC data is not byte aligned and therefore all subsequent sections will
 * also not be aligned to byte boundaries. However the final ECC section will
 * end byte aligned again.
 *
 *                       Raw Page Data
 *                      _______________
 *                     |               | 4 Bytes
 *                     | BBM           |
 *                     |_______________|
 *                     |               | 47 Bytes (oobavail)
 *                     | User-OOB      |
 *                     |_______________|
 *                     |               | 512 Bytes
 *                     | Main 0        |
 *                     |_______________|
 *                     |               | 26 Bits
 *                     | ECC 0         |
 *                     |___________    |
 *                     |           |___|
 *                     | Main 1        | 512 Bytes
 *                     |___________    |
 *                     |           |___|
 *                     | ECC 1         | 26 Bits
 *                     |_______        |
 *                     |       |_______|
 *                     |               | 512 Bytes
 *                     |_______ Main 2 |
 *                     |       |_______|
 *                     |               | 26 Bits
 *                     |___      ECC 2 |
 *                     |   |___________|
 *                     |               | 512 Bytes
 *                     |___     Main 3 |
 *                     |   |___________|
 *                     |               | 26 Bits
 *                     |         ECC 3 |
 *                     |_______________|
 *                      7 6 5 4 3 2 1 0
 *                            Bit        2112 Bytes total
 *
 * Example 2
 * ---------
 * NAND page size is 2048 + 64 = 2112 bytes. Chunk size is set to 1024 bytes
 * which means we must use GF14 and have two chunks. NBoot reports ECC8, which
 * means 14 bits * 8 = 112 bits ECC data (14 bytes). So the main data will
 * take 2x (1024 + 14) = 2076 bytes. So we will have 2112 - 2076 = 36 bytes
 * free space for OOB, from which we need 4 bytes for internal purposes. The
 * remaining free space is 32 bytes for the User OOB data (mtd->oobavail).
 *
 * This results in the following, significantly simpler layout where all
 * sections are byte aligned. This is the default layout for F&S usages.
 *
 *                       Raw Page Data
 *                      _______________
 *                     |               | 4 Bytes
 *                     | BBM           |
 *                     |_______________|
 *                     |               | 32 Bytes (oobavail)
 *                     | User-OOB      |
 *                     |_______________|
 *                     |               | 1024 Bytes
 *                     | Main 0        |
 *                     |_______________|
 *                     |               | 14 Bytes
 *                     | ECC 0         |
 *                     |_______________|
 *                     |               | 1024 Bytes
 *                     | Main 1        |
 *                     |_______________|
 *                     |               | 14 Bytes
 *                     | ECC 1         |
 *                     |_______________|
 *                      7 6 5 4 3 2 1 0
 *                            Bit        2112 Bytes total
 */

static int gpmi_ooblayout_fus_ecc(struct mtd_info *mtd, int section,
			      struct mtd_oob_region *oobregion)
{
	struct nand_chip *chip = mtd_to_nand(mtd);

	if (section)
		return -ERANGE;

	oobregion->offset = mtd->oobsize - chip->ecc.bytes;
	oobregion->length = chip->ecc.bytes;

	return 0;
}

static int gpmi_ooblayout_fus_free(struct mtd_info *mtd, int section,
			       struct mtd_oob_region *oobregion)
{
	struct nand_chip *chip = mtd_to_nand(mtd);

	if (section)
		return -ERANGE;

	/* The available oob size we have. */
	oobregion->offset = 4;
	oobregion->length = mtd->oobsize - chip->ecc.bytes - 4;

	return 0;
}

static const struct mtd_ooblayout_ops gpmi_ooblayout_fus_ops = {
	.ecc = gpmi_ooblayout_fus_ecc,
	.free = gpmi_ooblayout_fus_free,
};

static const struct gpmi_devdata gpmi_devdata_imx6q = {
	.type = IS_MX6Q,
	.bch_max_ecc_strength = 40,
	.max_chain_delay = 12,
};

static const struct gpmi_devdata gpmi_devdata_imx6qp = {
	.type = IS_MX6QP,
	.bch_max_ecc_strength = 40,
	.max_chain_delay = 12,
};

static const struct gpmi_devdata gpmi_devdata_imx6sx = {
	.type = IS_MX6SX,
	.bch_max_ecc_strength = 62,
	.max_chain_delay = 12,
};

static const struct gpmi_devdata gpmi_devdata_imx7d = {
	.type = IS_MX7D,
	.bch_max_ecc_strength = 62,
	.max_chain_delay = 12,
};

static const struct gpmi_devdata gpmi_devdata_imx6ul = {
	.type = IS_MX6UL,
	.bch_max_ecc_strength = 40,
	.max_chain_delay = 12,
};

static int showdesc; //###

static struct dma_chan *get_dma_chan(struct gpmi_nand_data *priv);

 
/* This function is very useful. It is called only when the bug occur. */
static void gpmi_dump_info(struct gpmi_nand_data *this)
{
	struct resources *r = &this->resources;
	struct nand_chip *chip = &this->nand;
	struct mtd_info *mtd = nand_to_mtd(chip);
	u32 reg;
	int i;

	dev_err(this->dev, "Show GPMI registers :\n");
	for (i = 0; i <= HW_GPMI_DEBUG / 0x10 + 1; i++) {
		reg = readl(r->gpmi_regs + i * 0x10);
		dev_err(this->dev, "offset 0x%.3x : 0x%.8x\n", i * 0x10, reg);
	}

	/* start to print out the BCH info */
	dev_err(this->dev, "BCH Geometry :\n");
	dev_err(this->dev, "GF length              : %u\n", this->gf_len);
	dev_err(this->dev, "ECC Strength           : %u\n", chip->ecc.strength);
	dev_err(this->dev, "Page Size in Bytes     : %u\n", mtd->writesize);
	dev_err(this->dev, "Metadata Size in Bytes : %u\n", mtd->oobavail);
	dev_err(this->dev, "ECC Chunk Size in Bytes: %u\n", chip->ecc.size);
	dev_err(this->dev, "ECC Chunk Count        : %u\n", chip->ecc.steps);
}

#if 0
static void gpmi_fus_show_regs(struct gpmi_nand_data *this)
{
	struct resources *r = &this->resources;
	u32 reg;
	int i;
	int n;

	n = 0xc0 / 0x10 + 1;

	pr_info("-------------- Show GPMI registers ----------\n");
	for (i = 0; i <= n; i++) {
		reg = __raw_readl(r->gpmi_regs + i * 0x10);
		pr_info("offset 0x%.3x : 0x%.8x\n", i * 0x10, reg);
	}
	pr_info("-------------- Show GPMI registers end ----------\n");
	pr_info("-------------- Show BCH registers ----------\n");
	for (i = 0; i <= n; i++) {
		reg = __raw_readl(r->bch_regs + i * 0x10);
		pr_info("offset 0x%.3x : 0x%.8x\n", i * 0x10, reg);
	}
	pr_info("-------------- Show BCH registers end ----------\n");
}
#endif

/* -------------------- IRQ ------------------------------------------------ */

static irqreturn_t bch_irq(int irq, void *cookie)
{
	struct gpmi_nand_data *this = cookie;
	struct resources *r = &this->resources;

	/* Clear BCH interrupt */
	writel(BM_BCH_CTRL_COMPLETE_IRQ, r->bch_regs + HW_BCH_CTRL_CLR);
	complete(&this->bch_done);

	return IRQ_HANDLED;
}

/* This will be called after the DMA operation is finished. */
static void dma_irq_callback(void *param)
{
	struct gpmi_nand_data *this = param;

	complete(&this->dma_done);
}

static int start_dma_without_bch_irq(struct gpmi_nand_data *this,
				     struct dma_async_tx_descriptor *desc)
{
	struct completion *dma_c = &this->dma_done;
	int err;
	dma_cookie_t cookie;

	init_completion(dma_c);

	desc->callback		= dma_irq_callback;
	desc->callback_param	= this;
	cookie = dmaengine_submit(desc);
	dma_async_issue_pending(get_dma_chan(this));

	this->command_length = 0;

	/* Wait for the interrupt from the DMA block. */
	err = wait_for_completion_timeout(dma_c, msecs_to_jiffies(1000));
	if (!err) {
		dev_err(this->dev, "DMA timeout\n");
		gpmi_dump_info(this);
		return -ETIMEDOUT;
	}

	/* Check if the DMA was successful */
	if (desc->chan->device->device_tx_status(desc->chan, cookie, NULL)
	    != DMA_COMPLETE) {
		dev_err(this->dev, "DMA chain execution failed\n");
		return -EIO;
	}

	return 0;
}

/*
 * This function is used when reading pages with BCH. First the DMA will
 * trigger an interrupt when done, but we will ignore this. We will only wait
 * for the COMPLETE_IRQ of the BCH which happens later.
 */
static int start_dma_with_bch_irq(struct gpmi_nand_data *priv,
				  struct dma_async_tx_descriptor *desc)
{
	struct completion *bch_c = &priv->bch_done;
	struct completion *dma_c = &priv->dma_done;
	struct resources *r = &priv->resources;
	unsigned long remain;
	dma_cookie_t cookie;

	/* Prepare to receive an interrupt from the BCH block. */
	init_completion(bch_c);
	init_completion(dma_c);

	/* Remove any COMPLETE_IRQ states from previous writes (we only check
	   COMPLETE_IRQ when reading); BCH has a pending state, too, so we may
	   have to clear it twice. We simply clear until it remains zero. */
	while (readl(r->bch_regs + HW_BCH_CTRL) & BM_BCH_CTRL_COMPLETE_IRQ)
		writel(BM_BCH_CTRL_COMPLETE_IRQ, r->bch_regs + HW_BCH_CTRL_CLR);

	/* Enable BCH complete interrupt */
	writel(BM_BCH_CTRL_COMPLETE_IRQ_EN, r->bch_regs + HW_BCH_CTRL_SET);

	/* Start the DMA */
	desc->callback		= dma_irq_callback;
	desc->callback_param	= priv;
	cookie = dmaengine_submit(desc);
	dma_async_issue_pending(get_dma_chan(priv));

	/* Wait for the interrupt from the DMA block. */
	remain = wait_for_completion_timeout(dma_c, msecs_to_jiffies(1000));
	if (!remain) {
		dev_err(priv->dev, "DMA (+BCH) timeout\n");
		gpmi_dump_info(priv);
		return -ETIMEDOUT;
	}

	/* Wait for the interrupt from the BCH block. */
	remain = wait_for_completion_timeout(bch_c, msecs_to_jiffies(1000));

	/* Disable BCH complete interrupt */
	writel(BM_BCH_CTRL_COMPLETE_IRQ_EN, r->bch_regs + HW_BCH_CTRL_CLR);

	if (!remain) {
		dev_err(priv->dev, "BCH timeout\n");
		gpmi_dump_info(priv);
		return -ETIMEDOUT;
	}

	/* Check if the DMA was successful */
	if (desc->chan->device->device_tx_status(desc->chan, cookie, NULL)
	    != DMA_COMPLETE) {
		dev_err(priv->dev,
			"DMA chain execution failed while waiting for BCH\n");
		return -EIO;
	}

	return 0;
}

static int acquire_bch_irq(struct gpmi_nand_data *this, irq_handler_t irq_h)
{
	struct platform_device *pdev = this->pdev;
	const char *res_name = GPMI_NAND_BCH_INTERRUPT_RES_NAME;
	struct resource *r;
	int err;

	r = platform_get_resource_byname(pdev, IORESOURCE_IRQ, res_name);
	if (!r) {
		dev_err(this->dev, "Can't get resource for %s\n", res_name);
		return -ENODEV;
	}

	err = devm_request_irq(this->dev, r->start, irq_h, 0, res_name, this);
	if (err)
		dev_err(this->dev, "error requesting BCH IRQ\n");

	return err;
}

/* -------------------- DMA ------------------------------------------------ */

static struct dma_chan *get_dma_chan(struct gpmi_nand_data *priv)
{
	int chipnr = priv->current_chip;

	return priv->dma_chans[chipnr];
}

/*
 * Wait until chip is ready.
 */
static int gpmi_fus_wait_ready(struct mtd_info *mtd, unsigned long timeout)
{
	struct nand_chip *chip = mtd->priv;
	struct gpmi_nand_data *priv = chip->priv;
	struct resources *r = &priv->resources;
	struct dma_chan *channel = get_dma_chan(priv);
	struct dma_async_tx_descriptor *desc;
	struct mxs_dma_ccw ccw;
	uint32_t tmp;
	int ret;

	/* ### TODO: Convert timeout value to GPMI ticks and set in
	   gpmi_regs->hw_gpmi_timing1 */

	/* Prepare DMA descriptor to wait for ready */
	ccw.next = 0;
	ccw.cmd = CCW_CMD_DMA_NO_XFER
		| CCW_NAND_WAIT4READY
		| CCW_WAIT4END
		| CCW_PIO_NUM(1);
	ccw.bufaddr = 0;
	ccw.pio[0] = BF_GPMI_CTRL0_CS(priv->current_chip, priv)
		| BF_GPMI_CTRL0_COMMAND_MODE(BV_GPMI_CTRL0_COMMAND_MODE__WAIT_FOR_READY)
		| BM_GPMI_CTRL0_WORD_LENGTH
		| BF_GPMI_CTRL0_ADDRESS(BV_GPMI_CTRL0_ADDRESS__NAND_DATA)
		| BF_GPMI_CTRL0_XFER_COUNT(0);

	if (showdesc) //###
		printk("### WAIT4READY-DESC\n");
	desc = dmaengine_prep_slave_sg(channel, (struct scatterlist *)&ccw,
				       1, 0, 0);
	if (!desc)
		ret = -EINVAL;
	else
		ret = start_dma_without_bch_irq(priv, desc);
	if (ret)
		return ret;

	/* Check for NAND timeout */
	tmp = readl(r->gpmi_regs + HW_GPMI_STAT);
	if (tmp & (1 << (priv->current_chip + 16))) {
		printk("### NAND timeout waiting for Ready\n");
		return -ETIMEDOUT;
	}

	return 0;
}

/*
 * Add a DMA descriptor for a command byte with optional arguments (row,
 * column or a one byte argument)
 */
static struct dma_async_tx_descriptor *gpmi_fus_add_cmd_desc(
				struct gpmi_nand_data *priv, uint command,
				int column, int row, int param)
{
	uint8_t *cmd_virt = priv->cmd_buffer_virt + priv->command_length;
	dma_addr_t cmd_phys = priv->cmd_buffer_phys + priv->command_length;
	struct dma_chan *channel = get_dma_chan(priv);
	struct mxs_dma_ccw ccw;
	int temp;
	unsigned i;
	uint32_t this_cmd_len = 0;

	/* Prepare the data to send in cmd_virt[] */
	cmd_virt[this_cmd_len++] = command;
	if (column != -1) {
		temp = column;
		for (i = 0; i < priv->column_cycles; i++) {
			cmd_virt[this_cmd_len++] = temp & 0xFF;
			temp >>= 8;
		}
	}
	if (row != -1) {
		temp = row;
		for (i = 0; i < priv->row_cycles; i++) {
			cmd_virt[this_cmd_len++] = temp & 0xFF;
			temp >>= 8;
		}
	}
	if (param != -1)
		cmd_virt[this_cmd_len++] = param & 0xFF;

	{ //###
		int i;
		if (showdesc) {
			printk("### CMD-DESC:");
			for (i = 0; i < this_cmd_len; i ++)
				printk(" %02X", cmd_virt[i]);
			printk("\n");
		}
	}
	priv->command_length += this_cmd_len;

	/* Prepare DMA descriptor that sends the command byte (and address) */
	ccw.next = 0;
	ccw.cmd = CCW_CMD_DMA_READ
		| CCW_NAND_LOCK
		| CCW_WAIT4END
		| CCW_PIO_NUM(3)
		| CCW_XFER_COUNT(this_cmd_len);
	ccw.bufaddr = cmd_phys;
	ccw.pio[0] = BF_GPMI_CTRL0_CS(priv->current_chip, priv)
		| BF_GPMI_CTRL0_LOCK_CS(LOCK_CS_ENABLE, priv)
		| BF_GPMI_CTRL0_COMMAND_MODE(BV_GPMI_CTRL0_COMMAND_MODE__WRITE)
		| BM_GPMI_CTRL0_WORD_LENGTH
		| BF_GPMI_CTRL0_ADDRESS(BV_GPMI_CTRL0_ADDRESS__NAND_CLE)
		| BF_GPMI_CTRL0_XFER_COUNT(this_cmd_len);
	if ((column != -1) || (row != -1) || (param != -1))
		ccw.pio[0] |= BM_GPMI_CTRL0_ADDRESS_INCREMENT;
	ccw.pio[1] = 0;
	ccw.pio[2] = 0;

	/* Add descriptor to DMA chain */
	return dmaengine_prep_slave_sg(channel, (struct scatterlist *)&ccw,
				       1, 0, 0);
}

/*
 * Add DMA descriptor to read data from flash to local buffer
 */
static struct dma_async_tx_descriptor *gpmi_fus_read_data_buf(
			struct mtd_info *mtd, dma_addr_t phys, uint32_t length)
{
	struct nand_chip *chip = mtd->priv;
	struct gpmi_nand_data *priv = chip->priv;
	struct dma_chan *channel = get_dma_chan(priv);
	struct mxs_dma_ccw ccw;

	/* Prepare DMA descriptor to read data */
	ccw.next = 0;
	ccw.cmd = CCW_CMD_DMA_WRITE
		| CCW_WAIT4END
		| CCW_PIO_NUM(1)
		| CCW_XFER_COUNT(length);
	ccw.bufaddr = phys;
	ccw.pio[0] = BF_GPMI_CTRL0_CS(priv->current_chip, priv)
		| BF_GPMI_CTRL0_COMMAND_MODE(BV_GPMI_CTRL0_COMMAND_MODE__READ)
		| BM_GPMI_CTRL0_WORD_LENGTH
		| BF_GPMI_CTRL0_ADDRESS(BV_GPMI_CTRL0_ADDRESS__NAND_DATA)
		| BF_GPMI_CTRL0_XFER_COUNT(length);

	if (showdesc) //###
		printk("### READ-DATA-DESC: phys=0x%08x, len=%d\n", phys, length);

	/* Add descriptor to DMA chain */
	return dmaengine_prep_slave_sg(channel, (struct scatterlist *)&ccw,
				       1, 0, 0);
}

/*
 * Add DMA descriptor to write data from local buffer to flash. This does
 * *not* execute the DMA chain right away, it will be executed later when the
 * command waits for ready.
 */
static struct dma_async_tx_descriptor *gpmi_fus_write_data_buf(
			struct mtd_info *mtd, dma_addr_t phys, uint32_t length)
{
	struct nand_chip *chip = mtd->priv;
	struct gpmi_nand_data *priv = chip->priv;
	struct dma_chan *channel = get_dma_chan(priv);
	struct mxs_dma_ccw ccw;

	/* Prepare DMA descriptor to write data */
	ccw.next = 0;
	ccw.cmd = CCW_CMD_DMA_READ
		| CCW_NAND_LOCK
		| CCW_WAIT4END
		| CCW_PIO_NUM(1)
		| CCW_XFER_COUNT(length);
	ccw.bufaddr = phys;
	ccw.pio[0] = BF_GPMI_CTRL0_CS(priv->current_chip, priv)
		| BF_GPMI_CTRL0_LOCK_CS(LOCK_CS_ENABLE, priv)
		| BF_GPMI_CTRL0_COMMAND_MODE(BV_GPMI_CTRL0_COMMAND_MODE__WRITE)
		| BM_GPMI_CTRL0_WORD_LENGTH
		| BF_GPMI_CTRL0_ADDRESS(BV_GPMI_CTRL0_ADDRESS__NAND_DATA)
		| BF_GPMI_CTRL0_XFER_COUNT(length);

	if (showdesc) //###
		printk("### WRITE-DATA-DESC: phys=0x%08x, len=%d\n", phys, length);

	/* Add descriptor to DMA chain */
	return dmaengine_prep_slave_sg(channel, (struct scatterlist *)&ccw,
				       1, 0, 0);
}

static void release_dma_channels(struct gpmi_nand_data *priv)
{
	unsigned int i;

	for (i = 0; i < DMA_CHANS; i++)
		if (priv->dma_chans[i]) {
			dma_release_channel(priv->dma_chans[i]);
			priv->dma_chans[i] = NULL;
		}
}

static int acquire_dma_channels(struct gpmi_nand_data *priv)
{
	struct platform_device *pdev = priv->pdev;
	struct dma_chan *dma_chan;

	/* request dma channel */
	dma_chan = dma_request_slave_channel(&pdev->dev, "rx-tx");

	if (!dma_chan) {
		dev_err(priv->dev, "Failed to request DMA channel.\n");
		goto acquire_err;
	}

	priv->dma_chans[0] = dma_chan;
	return 0;

acquire_err:
	release_dma_channels(priv);
	return -EINVAL;
}

/* -------------------- GPMI-LIB STUFF ------------------------------------- */

static struct timing_threshod timing_default_threshold = {
	.max_data_setup_cycles       = (BM_GPMI_TIMING0_DATA_SETUP >>
						BP_GPMI_TIMING0_DATA_SETUP),
	.internal_data_setup_in_ns   = 0,
	.max_sample_delay_factor     = (BM_GPMI_CTRL1_RDN_DELAY >>
						BP_GPMI_CTRL1_RDN_DELAY),
	.max_dll_clock_period_in_ns  = 32,
	.max_dll_delay_in_ns         = 16,
};

#define MXS_SET_ADDR		0x4
#define MXS_CLR_ADDR		0x8
/*
 * Clear the bit and poll it cleared.  This is usually called with
 * a reset address and mask being either SFTRST(bit 31) or CLKGATE
 * (bit 30).
 */
static int clear_poll_bit(void __iomem *addr, u32 mask)
{
	int timeout = 0x400;

	/* clear the bit */
	writel(mask, addr + MXS_CLR_ADDR);

	/*
	 * SFTRST needs 3 GPMI clocks to settle, the reference manual
	 * recommends to wait 1us.
	 */
	udelay(1);

	/* poll the bit becoming clear */
	while ((readl(addr) & mask) && --timeout)
		/* nothing */;

	return !timeout;
}

#define MODULE_CLKGATE		(1 << 30)
#define MODULE_SFTRST		(1 << 31)
/*
 * The current mxs_reset_block() will do two things:
 *  [1] enable the module.
 *  [2] reset the module.
 */
static int gpmi_reset_block(struct gpmi_nand_data *priv,
			    void __iomem *reset_addr)
{
	int ret;
	int timeout = 0x400;

	/* clear and poll SFTRST */
	ret = clear_poll_bit(reset_addr, MODULE_SFTRST);
	if (unlikely(ret))
		goto error;

	/* clear CLKGATE */
	writel(MODULE_CLKGATE, reset_addr + MXS_CLR_ADDR);

	/* set SFTRST to reset the block */
	writel(MODULE_SFTRST, reset_addr + MXS_SET_ADDR);
	udelay(1);

	/* poll CLKGATE becoming set */
	while ((!(readl(reset_addr) & MODULE_CLKGATE)) && --timeout)
		/* nothing */;
	if (unlikely(!timeout))
		goto error;

	/* clear and poll SFTRST */
	ret = clear_poll_bit(reset_addr, MODULE_SFTRST);
	if (unlikely(ret))
		goto error;

	/* clear and poll CLKGATE */
	ret = clear_poll_bit(reset_addr, MODULE_CLKGATE);
	if (unlikely(ret))
		goto error;

	return 0;

error:
	dev_err(priv->dev, "Module reset timeout for %p\n", reset_addr);
	return -ETIMEDOUT;
}

static int gpmi_enable_clk(struct gpmi_nand_data *this, bool v)
{
	struct clk *clk;
	int ret;
	int i;

	for (i = 0; i < GPMI_CLK_MAX; i++) {
		clk = this->resources.clock[i];
		if (!clk)
			break;

		if (v) {
			ret = clk_prepare_enable(clk);
			if (ret)
				goto err_clk;
		} else {
			clk_disable_unprepare(clk);
		}
	}
	return 0;

err_clk:
	for (; i > 0; i--)
		clk_disable_unprepare(this->resources.clock[i - 1]);
	return ret;
}

/* Configures the geometry for BCH.  */
static int bch_set_geometry(struct gpmi_nand_data *priv, unsigned int oobavail,
			    unsigned int index)
{
	struct resources *r = &priv->resources;
	struct nand_chip *chip = &priv->nand;
	struct mtd_info *mtd = nand_to_mtd(chip);
	unsigned int layout0, layout1;
	int ret;

	ret = pm_runtime_get_sync(priv->dev);
	if (ret < 0) {
		dev_err(priv->dev, "Failed to enable clock\n");
		goto err_out;
	}

	ret = gpmi_reset_block(priv, r->bch_regs);
	if (ret)
		goto err_out;

	/* Chunk #0 consists only of the metadata without any ECC */
	layout0 = BF_BCH_FLASH0LAYOUT0_NBLOCKS(chip->ecc.steps)
		| BF_BCH_FLASH0LAYOUT0_META_SIZE(oobavail + 4)
		| BF_BCH_FLASH0LAYOUT0_ECC0(0, priv)
		| BF_BCH_FLASH0LAYOUT0_DATA0_SIZE(0, priv);

	/* All other chunks have chunk size and use ECC */
	layout1 = BF_BCH_FLASH0LAYOUT1_PAGE_SIZE(mtd->writesize + mtd->oobsize)
		| BF_BCH_FLASH0LAYOUT1_ECCN(chip->ecc.strength >> 1, priv)
		| BF_BCH_FLASH0LAYOUT1_DATAN_SIZE(chip->ecc.size, priv);
	if (priv->gf_len == 14)
		layout1 |= BM_BCH_FLASH0LAYOUT1_GF13_0_GF14_1;

	/* Configure the appropriate layout */
	switch (index) {
	case 0:
		writel(layout0, r->bch_regs + HW_BCH_FLASH0LAYOUT0);
		writel(layout1, r->bch_regs + HW_BCH_FLASH0LAYOUT1);
		break;

	case 1:
		writel(layout0, r->bch_regs + HW_BCH_FLASH1LAYOUT0);
		writel(layout1, r->bch_regs + HW_BCH_FLASH1LAYOUT1);
		break;

	case 2:
		writel(layout0, r->bch_regs + HW_BCH_FLASH2LAYOUT0);
		writel(layout1, r->bch_regs + HW_BCH_FLASH2LAYOUT1);
		break;

	case 3:
		writel(layout0, r->bch_regs + HW_BCH_FLASH3LAYOUT0);
		writel(layout1, r->bch_regs + HW_BCH_FLASH3LAYOUT1);
		break;
	}

	/* Use index as layout number for all chip selects */
	index |= index << 2;
	index |= index << 4;
	priv->bch_layout = index;

	/* For now set *all* chip selects to use layout 0; we will change this
	   to the desired layout number when reading/writing pages with ECC */
	writel(0, r->bch_regs + HW_BCH_LAYOUTSELECT);

	/* Allow these many zero-bits in an empty page */
	writel(mtd->bitflip_threshold, r->bch_regs + HW_BCH_MODE);

	/* Enable BCH complete interrupt */
//###	writel(BM_BCH_CTRL_COMPLETE_IRQ_EN, r->bch_regs + HW_BCH_CTRL_SET);

err_out:
	pm_runtime_mark_last_busy(priv->dev);
	pm_runtime_put_autosuspend(priv->dev);

	return ret;
}

/* Converts time in nanoseconds to cycles. */
static unsigned int ns_to_cycles(unsigned int time,
				 unsigned int period, unsigned int min)
{
	unsigned int k;

	k = (time + period - 1) / period;
	return max(k, min);
}

#define DEF_MIN_PROP_DELAY	5
#define DEF_MAX_PROP_DELAY	9
/* Apply timing to current hardware conditions. */
static int gpmi_nfc_compute_hardware_timing(struct gpmi_nand_data *this,
					struct gpmi_nfc_hardware_timing *hw)
{
	struct timing_threshod *nfc = &timing_default_threshold;
	struct resources *r = &this->resources;
	struct nand_chip *nand = &this->nand;
	struct nand_timing target = this->timing;
	bool improved_timing_is_available;
	unsigned long clock_frequency_in_hz;
	unsigned int clock_period_in_ns;
	bool dll_use_half_periods;
	unsigned int dll_delay_shift;
	unsigned int max_sample_delay_in_ns;
	unsigned int address_setup_in_cycles;
	unsigned int data_setup_in_ns;
	unsigned int data_setup_in_cycles;
	unsigned int data_hold_in_cycles;
	int ideal_sample_delay_in_ns;
	unsigned int sample_delay_factor;
	int tEYE;
	unsigned int min_prop_delay_in_ns = DEF_MIN_PROP_DELAY;
	unsigned int max_prop_delay_in_ns = DEF_MAX_PROP_DELAY;

	/*
	 * If there are multiple chips, we need to relax the timings to allow
	 * for signal distortion due to higher capacitance.
	 */
	if (nand->numchips > 2) {
		target.data_setup_in_ns    += 10;
		target.data_hold_in_ns     += 10;
		target.address_setup_in_ns += 10;
	} else if (nand->numchips > 1) {
		target.data_setup_in_ns    += 5;
		target.data_hold_in_ns     += 5;
		target.address_setup_in_ns += 5;
	}

	/* Check if improved timing information is available. */
	improved_timing_is_available =
		(target.tREA_in_ns  >= 0) &&
		(target.tRLOH_in_ns >= 0) &&
		(target.tRHOH_in_ns >= 0);

	/* Inspect the clock. */
	nfc->clock_frequency_in_hz = clk_get_rate(r->clock[0]);
	clock_frequency_in_hz = nfc->clock_frequency_in_hz;
	clock_period_in_ns    = NSEC_PER_SEC / clock_frequency_in_hz;

	/*
	 * The NFC quantizes setup and hold parameters in terms of clock cycles.
	 * Here, we quantize the setup and hold timing parameters to the
	 * next-highest clock period to make sure we apply at least the
	 * specified times.
	 *
	 * For data setup and data hold, the hardware interprets a value of zero
	 * as the largest possible delay. This is not what's intended by a zero
	 * in the input parameter, so we impose a minimum of one cycle.
	 */
	data_setup_in_cycles    = ns_to_cycles(target.data_setup_in_ns,
							clock_period_in_ns, 1);
	data_hold_in_cycles     = ns_to_cycles(target.data_hold_in_ns,
							clock_period_in_ns, 1);
	address_setup_in_cycles = ns_to_cycles(target.address_setup_in_ns,
							clock_period_in_ns, 0);

	/*
	 * The clock's period affects the sample delay in a number of ways:
	 *
	 * (1) The NFC HAL tells us the maximum clock period the sample delay
	 *     DLL can tolerate. If the clock period is greater than half that
	 *     maximum, we must configure the DLL to be driven by half periods.
	 *
	 * (2) We need to convert from an ideal sample delay, in ns, to a
	 *     "sample delay factor," which the NFC uses. This factor depends on
	 *     whether we're driving the DLL with full or half periods.
	 *     Paraphrasing the reference manual:
	 *
	 *         AD = SDF x 0.125 x RP
	 *
	 * where:
	 *
	 *     AD   is the applied delay, in ns.
	 *     SDF  is the sample delay factor, which is dimensionless.
	 *     RP   is the reference period, in ns, which is a full clock period
	 *          if the DLL is being driven by full periods, or half that if
	 *          the DLL is being driven by half periods.
	 *
	 * Let's re-arrange this in a way that's more useful to us:
	 *
	 *                        8
	 *         SDF  =  AD x ----
	 *                       RP
	 *
	 * The reference period is either the clock period or half that, so this
	 * is:
	 *
	 *                        8       AD x DDF
	 *         SDF  =  AD x -----  =  --------
	 *                      f x P        P
	 *
	 * where:
	 *
	 *       f  is 1 or 1/2, depending on how we're driving the DLL.
	 *       P  is the clock period.
	 *     DDF  is the DLL Delay Factor, a dimensionless value that
	 *          incorporates all the constants in the conversion.
	 *
	 * DDF will be either 8 or 16, both of which are powers of two. We can
	 * reduce the cost of this conversion by using bit shifts instead of
	 * multiplication or division. Thus:
	 *
	 *                 AD << DDS
	 *         SDF  =  ---------
	 *                     P
	 *
	 *     or
	 *
	 *         AD  =  (SDF >> DDS) x P
	 *
	 * where:
	 *
	 *     DDS  is the DLL Delay Shift, the logarithm to base 2 of the DDF.
	 */
	if (clock_period_in_ns > (nfc->max_dll_clock_period_in_ns >> 1)) {
		dll_use_half_periods = true;
		dll_delay_shift      = 3 + 1;
	} else {
		dll_use_half_periods = false;
		dll_delay_shift      = 3;
	}

	/*
	 * Compute the maximum sample delay the NFC allows, under current
	 * conditions. If the clock is running too slowly, no sample delay is
	 * possible.
	 */
	if (clock_period_in_ns > nfc->max_dll_clock_period_in_ns)
		max_sample_delay_in_ns = 0;
	else {
		/*
		 * Compute the delay implied by the largest sample delay factor
		 * the NFC allows.
		 */
		max_sample_delay_in_ns =
			(nfc->max_sample_delay_factor * clock_period_in_ns) >>
								dll_delay_shift;

		/*
		 * Check if the implied sample delay larger than the NFC
		 * actually allows.
		 */
		if (max_sample_delay_in_ns > nfc->max_dll_delay_in_ns)
			max_sample_delay_in_ns = nfc->max_dll_delay_in_ns;
	}

	/*
	 * Check if improved timing information is available. If not, we have to
	 * use a less-sophisticated algorithm.
	 */
	if (!improved_timing_is_available) {
		/*
		 * Fold the read setup time required by the NFC into the ideal
		 * sample delay.
		 */
		ideal_sample_delay_in_ns = target.gpmi_sample_delay_in_ns +
						nfc->internal_data_setup_in_ns;

		/*
		 * The ideal sample delay may be greater than the maximum
		 * allowed by the NFC. If so, we can trade off sample delay time
		 * for more data setup time.
		 *
		 * In each iteration of the following loop, we add a cycle to
		 * the data setup time and subtract a corresponding amount from
		 * the sample delay until we've satisified the constraints or
		 * can't do any better.
		 */
		while ((ideal_sample_delay_in_ns > max_sample_delay_in_ns) &&
			(data_setup_in_cycles < nfc->max_data_setup_cycles)) {

			data_setup_in_cycles++;
			ideal_sample_delay_in_ns -= clock_period_in_ns;

			if (ideal_sample_delay_in_ns < 0)
				ideal_sample_delay_in_ns = 0;

		}

		/*
		 * Compute the sample delay factor that corresponds most closely
		 * to the ideal sample delay. If the result is too large for the
		 * NFC, use the maximum value.
		 *
		 * Notice that we use the ns_to_cycles function to compute the
		 * sample delay factor. We do this because the form of the
		 * computation is the same as that for calculating cycles.
		 */
		sample_delay_factor =
			ns_to_cycles(
				ideal_sample_delay_in_ns << dll_delay_shift,
							clock_period_in_ns, 0);

		if (sample_delay_factor > nfc->max_sample_delay_factor)
			sample_delay_factor = nfc->max_sample_delay_factor;

		/* Skip to the part where we return our results. */
		goto return_results;
	}

	/*
	 * If control arrives here, we have more detailed timing information,
	 * so we can use a better algorithm.
	 */

	/*
	 * Fold the read setup time required by the NFC into the maximum
	 * propagation delay.
	 */
	max_prop_delay_in_ns += nfc->internal_data_setup_in_ns;

	/*
	 * Earlier, we computed the number of clock cycles required to satisfy
	 * the data setup time. Now, we need to know the actual nanoseconds.
	 */
	data_setup_in_ns = clock_period_in_ns * data_setup_in_cycles;

	/*
	 * Compute tEYE, the width of the data eye when reading from the NAND
	 * Flash. The eye width is fundamentally determined by the data setup
	 * time, perturbed by propagation delays and some characteristics of the
	 * NAND Flash device.
	 *
	 * start of the eye = max_prop_delay + tREA
	 * end of the eye   = min_prop_delay + tRHOH + data_setup
	 */
	tEYE = (int)min_prop_delay_in_ns + (int)target.tRHOH_in_ns +
							(int)data_setup_in_ns;

	tEYE -= (int)max_prop_delay_in_ns + (int)target.tREA_in_ns;

	/*
	 * The eye must be open. If it's not, we can try to open it by
	 * increasing its main forcer, the data setup time.
	 *
	 * In each iteration of the following loop, we increase the data setup
	 * time by a single clock cycle. We do this until either the eye is
	 * open or we run into NFC limits.
	 */
	while ((tEYE <= 0) &&
			(data_setup_in_cycles < nfc->max_data_setup_cycles)) {
		/* Give a cycle to data setup. */
		data_setup_in_cycles++;
		/* Synchronize the data setup time with the cycles. */
		data_setup_in_ns += clock_period_in_ns;
		/* Adjust tEYE accordingly. */
		tEYE += clock_period_in_ns;
	}

	/*
	 * When control arrives here, the eye is open. The ideal time to sample
	 * the data is in the center of the eye:
	 *
	 *     end of the eye + start of the eye
	 *     ---------------------------------  -  data_setup
	 *                    2
	 *
	 * After some algebra, this simplifies to the code immediately below.
	 */
	ideal_sample_delay_in_ns =
		((int)max_prop_delay_in_ns +
			(int)target.tREA_in_ns +
				(int)min_prop_delay_in_ns +
					(int)target.tRHOH_in_ns -
						(int)data_setup_in_ns) >> 1;

	/*
	 * The following figure illustrates some aspects of a NAND Flash read:
	 *
	 *
	 *           __                   _____________________________________
	 * RDN         \_________________/
	 *
	 *                                         <---- tEYE ----->
	 *                                        /-----------------\
	 * Read Data ----------------------------<                   >---------
	 *                                        \-----------------/
	 *             ^                 ^                 ^              ^
	 *             |                 |                 |              |
	 *             |<--Data Setup -->|<--Delay Time -->|              |
	 *             |                 |                 |              |
	 *             |                 |                                |
	 *             |                 |<--   Quantized Delay Time   -->|
	 *             |                 |                                |
	 *
	 *
	 * We have some issues we must now address:
	 *
	 * (1) The *ideal* sample delay time must not be negative. If it is, we
	 *     jam it to zero.
	 *
	 * (2) The *ideal* sample delay time must not be greater than that
	 *     allowed by the NFC. If it is, we can increase the data setup
	 *     time, which will reduce the delay between the end of the data
	 *     setup and the center of the eye. It will also make the eye
	 *     larger, which might help with the next issue...
	 *
	 * (3) The *quantized* sample delay time must not fall either before the
	 *     eye opens or after it closes (the latter is the problem
	 *     illustrated in the above figure).
	 */

	/* Jam a negative ideal sample delay to zero. */
	if (ideal_sample_delay_in_ns < 0)
		ideal_sample_delay_in_ns = 0;

	/*
	 * Extend the data setup as needed to reduce the ideal sample delay
	 * below the maximum permitted by the NFC.
	 */
	while ((ideal_sample_delay_in_ns > max_sample_delay_in_ns) &&
			(data_setup_in_cycles < nfc->max_data_setup_cycles)) {

		/* Give a cycle to data setup. */
		data_setup_in_cycles++;
		/* Synchronize the data setup time with the cycles. */
		data_setup_in_ns += clock_period_in_ns;
		/* Adjust tEYE accordingly. */
		tEYE += clock_period_in_ns;

		/*
		 * Decrease the ideal sample delay by one half cycle, to keep it
		 * in the middle of the eye.
		 */
		ideal_sample_delay_in_ns -= (clock_period_in_ns >> 1);

		/* Jam a negative ideal sample delay to zero. */
		if (ideal_sample_delay_in_ns < 0)
			ideal_sample_delay_in_ns = 0;
	}

	/*
	 * Compute the sample delay factor that corresponds to the ideal sample
	 * delay. If the result is too large, then use the maximum allowed
	 * value.
	 *
	 * Notice that we use the ns_to_cycles function to compute the sample
	 * delay factor. We do this because the form of the computation is the
	 * same as that for calculating cycles.
	 */
	sample_delay_factor =
		ns_to_cycles(ideal_sample_delay_in_ns << dll_delay_shift,
							clock_period_in_ns, 0);

	if (sample_delay_factor > nfc->max_sample_delay_factor)
		sample_delay_factor = nfc->max_sample_delay_factor;

	/*
	 * These macros conveniently encapsulate a computation we'll use to
	 * continuously evaluate whether or not the data sample delay is inside
	 * the eye.
	 */
	#define IDEAL_DELAY  ((int) ideal_sample_delay_in_ns)

	#define QUANTIZED_DELAY  \
		((int) ((sample_delay_factor * clock_period_in_ns) >> \
							dll_delay_shift))

	#define DELAY_ERROR  (abs(QUANTIZED_DELAY - IDEAL_DELAY))

	#define SAMPLE_IS_NOT_WITHIN_THE_EYE  (DELAY_ERROR > (tEYE >> 1))

	/*
	 * While the quantized sample time falls outside the eye, reduce the
	 * sample delay or extend the data setup to move the sampling point back
	 * toward the eye. Do not allow the number of data setup cycles to
	 * exceed the maximum allowed by the NFC.
	 */
	while (SAMPLE_IS_NOT_WITHIN_THE_EYE &&
			(data_setup_in_cycles < nfc->max_data_setup_cycles)) {
		/*
		 * If control arrives here, the quantized sample delay falls
		 * outside the eye. Check if it's before the eye opens, or after
		 * the eye closes.
		 */
		if (QUANTIZED_DELAY > IDEAL_DELAY) {
			/*
			 * If control arrives here, the quantized sample delay
			 * falls after the eye closes. Decrease the quantized
			 * delay time and then go back to re-evaluate.
			 */
			if (sample_delay_factor != 0)
				sample_delay_factor--;
			continue;
		}

		/*
		 * If control arrives here, the quantized sample delay falls
		 * before the eye opens. Shift the sample point by increasing
		 * data setup time. This will also make the eye larger.
		 */

		/* Give a cycle to data setup. */
		data_setup_in_cycles++;
		/* Synchronize the data setup time with the cycles. */
		data_setup_in_ns += clock_period_in_ns;
		/* Adjust tEYE accordingly. */
		tEYE += clock_period_in_ns;

		/*
		 * Decrease the ideal sample delay by one half cycle, to keep it
		 * in the middle of the eye.
		 */
		ideal_sample_delay_in_ns -= (clock_period_in_ns >> 1);

		/* ...and one less period for the delay time. */
		ideal_sample_delay_in_ns -= clock_period_in_ns;

		/* Jam a negative ideal sample delay to zero. */
		if (ideal_sample_delay_in_ns < 0)
			ideal_sample_delay_in_ns = 0;

		/*
		 * We have a new ideal sample delay, so re-compute the quantized
		 * delay.
		 */
		sample_delay_factor =
			ns_to_cycles(
				ideal_sample_delay_in_ns << dll_delay_shift,
							clock_period_in_ns, 0);

		if (sample_delay_factor > nfc->max_sample_delay_factor)
			sample_delay_factor = nfc->max_sample_delay_factor;
	}

	/* Control arrives here when we're ready to return our results. */
return_results:
	hw->data_setup_in_cycles    = data_setup_in_cycles;
	hw->data_hold_in_cycles     = data_hold_in_cycles;
	hw->address_setup_in_cycles = address_setup_in_cycles;
	hw->use_half_periods        = dll_use_half_periods;
	hw->sample_delay_factor     = sample_delay_factor;
//###	hw->device_busy_timeout     = GPMI_DEFAULT_BUSY_TIMEOUT;
	hw->device_busy_timeout     = 0xFFFF; /* default busy timeout value. */
	hw->wrn_dly_sel             = BV_GPMI_CTRL1_WRN_DLY_SEL_4_TO_8NS;

	/* Return success. */
	return 0;
}

/*
 * <1> Firstly, we should know what's the GPMI-clock means.
 *     The GPMI-clock is the internal clock in the gpmi nand controller.
 *     If you set 100MHz to gpmi nand controller, the GPMI-clock's period
 *     is 10ns. Mark the GPMI-clock's period as GPMI-clock-period.
 *
 * <2> Secondly, we should know what's the frequency on the nand chip pins.
 *     The frequency on the nand chip pins is derived from the GPMI-clock.
 *     We can get it from the following equation:
 *
 *         F = G / (DS + DH)
 *
 *         F  : the frequency on the nand chip pins.
 *         G  : the GPMI clock, such as 100MHz.
 *         DS : GPMI_HW_GPMI_TIMING0:DATA_SETUP
 *         DH : GPMI_HW_GPMI_TIMING0:DATA_HOLD
 *
 * <3> Thirdly, when the frequency on the nand chip pins is above 33MHz,
 *     the nand EDO(extended Data Out) timing could be applied.
 *     The GPMI implements a feedback read strobe to sample the read data.
 *     The feedback read strobe can be delayed to support the nand EDO timing
 *     where the read strobe may deasserts before the read data is valid, and
 *     read data is valid for some time after read strobe.
 *
 *     The following figure illustrates some aspects of a NAND Flash read:
 *
 *                   |<---tREA---->|
 *                   |             |
 *                   |         |   |
 *                   |<--tRP-->|   |
 *                   |         |   |
 *                  __          ___|__________________________________
 *     RDN            \________/   |
 *                                 |
 *                                 /---------\
 *     Read Data    --------------<           >---------
 *                                 \---------/
 *                                |     |
 *                                |<-D->|
 *     FeedbackRDN  ________             ____________
 *                          \___________/
 *
 *          D stands for delay, set in the HW_GPMI_CTRL1:RDN_DELAY.
 *
 *
 * <4> Now, we begin to describe how to compute the right RDN_DELAY.
 *
 *  4.1) From the aspect of the nand chip pins:
 *        Delay = (tREA + C - tRP)               {1}
 *
 *        tREA : the maximum read access time. From the ONFI nand standards,
 *               we know that tREA is 16ns in mode 5, tREA is 20ns is mode 4.
 *               Please check it in : www.onfi.org
 *        C    : a constant for adjust the delay. default is 4.
 *        tRP  : the read pulse width.
 *               Specified by the HW_GPMI_TIMING0:DATA_SETUP:
 *                    tRP = (GPMI-clock-period) * DATA_SETUP
 *
 *  4.2) From the aspect of the GPMI nand controller:
 *         Delay = RDN_DELAY * 0.125 * RP        {2}
 *
 *         RP   : the DLL reference period.
 *            if (GPMI-clock-period > DLL_THRETHOLD)
 *                   RP = GPMI-clock-period / 2;
 *            else
 *                   RP = GPMI-clock-period;
 *
 *            Set the HW_GPMI_CTRL1:HALF_PERIOD if GPMI-clock-period
 *            is greater DLL_THRETHOLD. In other SOCs, the DLL_THRETHOLD
 *            is 16ns, but in mx6q, we use 12ns.
 *
 *  4.3) since {1} equals {2}, we get:
 *
 *                    (tREA + 4 - tRP) * 8
 *         RDN_DELAY = ---------------------     {3}
 *                           RP
 *
 *  4.4) We only support the fastest asynchronous mode of ONFI nand.
 *       For some ONFI nand, the mode 4 is the fastest mode;
 *       while for some ONFI nand, the mode 5 is the fastest mode.
 *       So we only support the mode 4 and mode 5. It is no need to
 *       support other modes.
 */
static void gpmi_compute_edo_timing(struct gpmi_nand_data *this,
			struct gpmi_nfc_hardware_timing *hw)
{
	struct resources *r = &this->resources;
	unsigned long rate = clk_get_rate(r->clock[0]);
	int mode = this->timing_mode;
	int dll_threshold = this->devdata->max_chain_delay;
	unsigned long delay;
	unsigned long clk_period;
	int t_rea;
	int c = 4;
	int t_rp;
	int rp;

	/*
	 * [1] for GPMI_HW_GPMI_TIMING0:
	 *     The async mode requires 40MHz for mode 4, 50MHz for mode 5.
	 *     The GPMI can support 100MHz at most. So if we want to
	 *     get the 40MHz or 50MHz, we have to set DS=1, DH=1.
	 *     Set the ADDRESS_SETUP to 0 in mode 4.
	 */
	hw->data_setup_in_cycles = 1;
	hw->data_hold_in_cycles = 1;
	hw->address_setup_in_cycles = ((mode == 5) ? 1 : 0);

	/* [2] for GPMI_HW_GPMI_TIMING1 */
//###	hw->device_busy_timeout = 0x9000;
	hw->device_busy_timeout = 0xFFFF;

	/* [3] for GPMI_HW_GPMI_CTRL1 */
	hw->wrn_dly_sel = BV_GPMI_CTRL1_WRN_DLY_SEL_NO_DELAY;

	/*
	 * Enlarge 10 times for the numerator and denominator in {3}.
	 * This make us to get more accurate result.
	 */
	clk_period = NSEC_PER_SEC / (rate / 10);
	dll_threshold *= 10;
	t_rea = ((mode == 5) ? 16 : 20) * 10;
	c *= 10;

	t_rp = clk_period * 1; /* DATA_SETUP is 1 */

	if (clk_period > dll_threshold) {
		hw->use_half_periods = 1;
		rp = clk_period / 2;
	} else {
		hw->use_half_periods = 0;
		rp = clk_period;
	}

	/*
	 * Multiply the numerator with 10, we could do a round off:
	 *      7.8 round up to 8; 7.4 round down to 7.
	 */
	delay  = (((t_rea + c - t_rp) * 8) * 10) / rp;
	delay = (delay + 5) / 10;

	hw->sample_delay_factor = delay;
}

static int enable_edo_mode(struct gpmi_nand_data *this, int mode)
{
	struct resources  *r = &this->resources;
	struct nand_chip *nand = &this->nand;
	struct mtd_info *mtd = nand_to_mtd(nand);
	uint8_t feature[ONFI_SUBFEATURE_PARAM_LEN] = {};
	unsigned long rate;
	int ret;

	nand->select_chip(mtd, 0);

	/* [1] send SET FEATURE commond to NAND */
	feature[0] = mode;
	ret = nand->onfi_set_features(mtd, nand,
				ONFI_FEATURE_ADDR_TIMING_MODE, feature);
	if (ret)
		goto err_out;

	/* [2] send GET FEATURE command to double-check the timing mode */
	ret = nand->onfi_get_features(mtd, nand,
				ONFI_FEATURE_ADDR_TIMING_MODE, feature);
	if (ret || feature[0] != mode)
		goto err_out;

	nand->select_chip(mtd, -1);

	pm_runtime_get_sync(this->dev);
	clk_disable_unprepare(r->clock[0]);
	/* [3] set the main IO clock, 100MHz for mode 5, 80MHz for mode 4. */
	rate = (mode == 5) ? 100000000 : 80000000;
	clk_set_rate(r->clock[0], rate);
	clk_prepare_enable(r->clock[0]);
	pm_runtime_mark_last_busy(this->dev);
        pm_runtime_put_autosuspend(this->dev);	

	/* Let the gpmi_begin() re-compute the timing again. */
	this->flags &= ~GPMI_TIMING_INIT_OK;

	this->flags |= GPMI_ASYNC_EDO_ENABLED;
	this->timing_mode = mode;
	dev_info(this->dev, "enable asynchronous EDO mode %d\n", mode);

	return 0;

err_out:
	nand->select_chip(mtd, -1);
	dev_err(this->dev, "mode: %d, failed in set feature.\n", mode);
	return -EINVAL;
}

static int gpmi_extra_init(struct gpmi_nand_data *this)
{
	struct nand_chip *chip = &this->nand;

	/* If flash is ONFI flash and supports GET/SET_FEATURE command and
	   asynchronous EDO timing mode 4 or 5, enable appropriate EDO mode */
	if (onfi_get_opt_cmd(chip) & ONFI_OPT_CMD_SET_GET_FEATURES) {
		int mode = onfi_get_async_timing_mode(chip);

		if (mode & ONFI_TIMING_MODE_5)
			enable_edo_mode(this, 5);
		else if (mode & ONFI_TIMING_MODE_4)
			enable_edo_mode(this, 4);
	}

	return 0;
}

/* Begin the I/O */
static void gpmi_begin(struct gpmi_nand_data *this)
{
	struct resources *r = &this->resources;
	void __iomem *gpmi_regs = r->gpmi_regs;
	unsigned int   clock_period_in_ns;
	uint32_t       reg;
	unsigned int   dll_wait_time_in_us;
	struct gpmi_nfc_hardware_timing  hw;
	int ret;

	/* Enable the clock. */
	ret = pm_runtime_get_sync(this->dev);
	if (ret < 0) {
		dev_err(this->dev, "Failed to enable clock\n");
		goto err_out;
	}

	/* Only initialize the timing once */
	if (this->flags & GPMI_TIMING_INIT_OK)
		return;
	this->flags |= GPMI_TIMING_INIT_OK;

	if (this->flags & GPMI_ASYNC_EDO_ENABLED)
		gpmi_compute_edo_timing(this, &hw);
	else
		gpmi_nfc_compute_hardware_timing(this, &hw);

	/* [1] Set HW_GPMI_TIMING0 */
	reg = BF_GPMI_TIMING0_ADDRESS_SETUP(hw.address_setup_in_cycles) |
		BF_GPMI_TIMING0_DATA_HOLD(hw.data_hold_in_cycles)         |
		BF_GPMI_TIMING0_DATA_SETUP(hw.data_setup_in_cycles)       ;

	writel(reg, gpmi_regs + HW_GPMI_TIMING0);

	/* [2] Set HW_GPMI_TIMING1 */
	writel(BF_GPMI_TIMING1_BUSY_TIMEOUT(hw.device_busy_timeout),
		gpmi_regs + HW_GPMI_TIMING1);

	/* [3] The following code is to set the HW_GPMI_CTRL1. */

	/* Set the WRN_DLY_SEL */
	writel(BM_GPMI_CTRL1_WRN_DLY_SEL, gpmi_regs + HW_GPMI_CTRL1_CLR);
	writel(BF_GPMI_CTRL1_WRN_DLY_SEL(hw.wrn_dly_sel),
					gpmi_regs + HW_GPMI_CTRL1_SET);

	/* DLL_ENABLE must be set to 0 when setting RDN_DELAY or HALF_PERIOD. */
	writel(BM_GPMI_CTRL1_DLL_ENABLE, gpmi_regs + HW_GPMI_CTRL1_CLR);

	/* Clear out the DLL control fields. */
	reg = BM_GPMI_CTRL1_RDN_DELAY | BM_GPMI_CTRL1_HALF_PERIOD;
	writel(reg, gpmi_regs + HW_GPMI_CTRL1_CLR);

	/* If no sample delay is called for, return immediately. */
	if (!hw.sample_delay_factor)
		return;

	/* Set RDN_DELAY or HALF_PERIOD. */
	reg = BF_GPMI_CTRL1_RDN_DELAY(hw.sample_delay_factor);
	if (hw.use_half_periods)
		reg |= BM_GPMI_CTRL1_HALF_PERIOD;
	writel(reg, gpmi_regs + HW_GPMI_CTRL1_SET);

	/* At last, we enable the DLL. */
	writel(BM_GPMI_CTRL1_DLL_ENABLE, gpmi_regs + HW_GPMI_CTRL1_SET);

	/*
	 * After we enable the GPMI DLL, we have to wait 64 clock cycles before
	 * we can use the GPMI. Calculate the amount of time we need to wait,
	 * in microseconds.
	 */
	clock_period_in_ns = NSEC_PER_SEC / clk_get_rate(r->clock[0]);
	dll_wait_time_in_us = (clock_period_in_ns * 64) / 1000;

	if (!dll_wait_time_in_us)
		dll_wait_time_in_us = 1;

	/* Wait for the DLL to settle. */
	udelay(dll_wait_time_in_us);

err_out:
	return;
}

static void gpmi_end(struct gpmi_nand_data *this)
{
	pm_runtime_mark_last_busy(this->dev);
	pm_runtime_put_autosuspend(this->dev);
}

/* -------------------- LOCAL HELPER FUNCTIONS ----------------------------- */


static inline int gpmi_fus_get_ecc_strength(struct mtd_info *mtd,
					    unsigned int chunk_shift)
{
	unsigned int oob_ecc;
	unsigned int ecc_steps;
	unsigned int gf = (chunk_shift > 9) ? 14 : 13;

	/* Get chunk count */
	ecc_steps = mtd->writesize >> chunk_shift;

	/* Get available space for ECC, in bits */
	oob_ecc = mtd->oobsize - GPMI_NAND_METADATA_SIZE - 4;
	oob_ecc <<= 3;

	/* We need an even number for the ECC strength, therefore divide by
	   twice the value and multiply by two again */
	oob_ecc /= gf * 2 * ecc_steps;
	oob_ecc <<= 1;

	return oob_ecc;
}

/*
 * Copy the main data from the raw page data in the DMA page buffer to buf[].
 * The main data is interleaved with the ECC data and ECC data is not
 * necessarily aligned to bytes. So for some chunks we might need to shift
 * bits.
 *
 * The following example shows a rather complicated case on a NAND flash with
 * 2048+64 = 2112 bytes pages. The layout uses ECC2 on 512 bytes chunks (i.e.
 * 13 bits ECC data needed per ECC step = 26 bits). This requires four data
 * chunks, and we have 51 free bytes for the Bad Block Marker and User OOB.
 * The markings show the section boundaries and the number of bits in each
 * section at the transitions.
 *
 *   Raw Page Data   page_buffer_virt[]
 *  _______________
 * |               | 4 Bytes
 * | BBM           |
 * |_______________|
 * |               | 47 Bytes (oobavail)
 * | User-OOB      |
 * |_______________|
 * |               | 512 Bytes
 * | Main 0        |<-----------+
 * |_______________|            |             Main Data     buf[]
 * |               | 26 Bits    |          _______________
 * | ECC 0         |            |     1:1 |               | 512 Bytes
 * |___________    |            +-------->| Main 0        |
 * |           |___|                      |_______________|
 * | Main 1        | 512 Bytes      shift |               | 512 Bytes
 * |___________    |--------------------->| Main 1        |
 * |           |___|                      |_______________|
 * | ECC 1         | 26 Bits        shift |               | 512 Bytes
 * |_______        |            +-------->| Main 2        |
 * |       |_______|            |         |_______________|
 * |               | 512 Bytes  |   shift |               | 512 Bytes
 * |_______ Main 2 |<-----------+    +--->| Main 3        |
 * |       |_______|                 |    |_______________|
 * |               | 26 Bits         |     7 6 5 4 3 2 1 0
 * |___      ECC 2 |                 |           Bit        2048 Bytes total
 * |   |___________|                 |
 * |               | 512 Bytes       |
 * |___     Main 3 |<----------------+
 * |   |___________|
 * |               | 26 Bits
 * |         ECC 3 |
 * |_______________|
 *  7 6 5 4 3 2 1 0
 *        Bit        2112 Bytes total
 *
 * The standard F&S layout with 1024 bytes chunks and ECC8 is far less
 * complicated as the ECC data will always match byte boundaries. However we
 * want the code to work in all possible cases.
 */
static void gpmi_fus_read_main_data(struct mtd_info *mtd, uint8_t *buf)
{
	struct nand_chip *chip = mtd->priv;
	struct gpmi_nand_data *priv = chip->priv;
	uint val;
	uint bit = 0;
	int chunk, remaining;
	int chunk_size = chip->ecc.size;
	uint8_t *raw_data = priv->page_buffer_virt + mtd->oobavail + 4;
	unsigned int ecc_bits;

	ecc_bits = priv->gf_len * chip->ecc.strength;
	for (chunk = 0; chunk < chip->ecc.steps; chunk++) {
		if (bit) {
			/* Copy split bytes with shifting */
			val = *raw_data >> bit;
			remaining = chunk_size;
			do {
				val |= *(++raw_data) << (8 - bit);
				*buf++ = (uint8_t)val;
				val >>= 8;
			} while (--remaining);
		} else {
			/* Copy whole bytes */
			memcpy(buf, raw_data, chunk_size);
			buf += chunk_size;
			raw_data += chunk_size;
		}
		bit += ecc_bits;
		raw_data += bit >> 3;
		bit &= 7;
	}
}

/*
 * Copy the main data from buf[] to the raw page data in the DMA page buffer.
 * The main data is interleaved with the ECC data and ECC data is not
 * necessarily aligned to bytes. So for some chunks we might need to shift
 * bits.
 *
 * See gpmi_fus_read_main_data() for a detailed explanation and an example.
 */
static void gpmi_fus_write_main_data(struct mtd_info *mtd, const uint8_t *buf)
{
	struct nand_chip *chip = mtd->priv;
	struct gpmi_nand_data *priv = chip->priv;
	uint val, tmp, mask;
	uint bit = 0;
	int chunk, remaining;
	int chunk_size = chip->ecc.size;
	uint8_t *raw_data = priv->page_buffer_virt + mtd->oobavail + 4;
	unsigned int ecc_bits;

	ecc_bits = priv->gf_len * chip->ecc.strength;
	for (chunk = 0; chunk < chip->ecc.steps; chunk++) {
		if (bit) {
			/* Copy split bytes with shifting */
			mask = (1 << bit) - 1;
			val = *raw_data & mask;
			remaining = chunk_size;
			do {
				tmp = *buf++;
				val |= tmp << bit;
				*raw_data++ = (uint8_t)val;
				val = tmp >> (8 - bit);
			} while (--remaining);
			val |= *raw_data & ~mask;
			*raw_data = (uint8_t)val;
		} else {
			/* Copy whole bytes */
			memcpy(raw_data, buf, chunk_size);
			buf += chunk_size;
			raw_data += chunk_size;
		}
		bit += ecc_bits;
		raw_data += bit >> 3;
		bit &= 7;
	}
}

/*
 * Copy the ECC data from raw page data in the DMA page buffer to oob[]. The
 * BBM and user part of the OOB area are not touched!
 *
 * The complicated part is the ECC data. It is interleaved with the main data
 * and ECC data is not necessarily aligned to bytes. But because the main data
 * chunks always consist of full bytes, the ECC parts will always fit together
 * without the need for bit shifting.
 *
 * The following example shows a layout with ECC2 on 512 bytes chunks (13 bits
 * ECC data per ECC step = 26 bits). This requires four data chunks on a NAND
 * flash with 2048+64 bytes pages, and we have 51 free bytes for the Bad Block
 * Marker and User OOB. The markings show the section boundaries and the
 * number of bits in each section at the transitions.
 *
 *   Raw Page Data   page_buffer_virt[]
 *  _______________
 * |               | 4 Bytes
 * | BBM           |
 * |_______________|
 * |               | 47 Bytes (oobavail)
 * | User-OOB      |
 * |_______________|                          OOB Data      oob[]
 * |               | 512 Bytes             _______________
 * | Main 0        |                      |               | 4 Bytes
 * |_______________|                      | BBM           |
 * |               | 26 Bits              |_______________|
 * | ECC 0         |<-------------+       |               | 47 Bytes (oobavail)
 * |___________    |              |       | User-OOB      |
 * |           |___|              |       |_______________|
 * | Main 1        | 512 Bytes    |       |               | 26 Bits
 * |___________    |              +------>| ECC 0         |
 * |           |___|                      |___________    |
 * | ECC 1         | 26 Bits              |           |___|
 * |_______        |<-------------------->| ECC 1         | 26 Bits
 * |       |_______|                      |_______        |
 * |               | 512 Bytes            |       |_______|
 * |_______ Main 2 |              +------>|               | 26 Bits
 * |       |_______|              |       |___      ECC 2 |
 * |               | 26 Bits      |       |   |___________|
 * |___      ECC 2 |<-------------+       |               | 26 Bits
 * |   |___________|                 +--->|         ECC 3 |
 * |               | 512 Bytes       |    |_______________|
 * |___     Main 3 |                 |     7 6 5 4 3 2 1 0
 * |   |___________|                 |           Bit        64 Bytes total
 * |               | 26 Bits         |
 * |         ECC 3 |<----------------+
 * |_______________|
 *  7 6 5 4 3 2 1 0
 *        Bit        2112 Bytes total
 *
 * The standard F&S layout with 1024 bytes chunks and ECC8 is far less
 * complicated as the ECC data will always match byte boundaries. However we
 * want the code to work in all possible cases.
 */
static void gpmi_fus_read_ecc_data(struct mtd_info *mtd, uint8_t *oob)
{
	struct nand_chip *chip = mtd->priv;
	struct gpmi_nand_data *priv = chip->priv;
	int chunk, remaining = 0;
	int chunk_size = chip->ecc.size;
	uint8_t val, mask;
	uint headersize = mtd->oobavail + 4;
	uint8_t *raw_data = priv->page_buffer_virt;
	unsigned int ecc_bits;

	/* Adjust pointers to first ECC section */
	oob += headersize;
	raw_data += headersize + chunk_size;

	/* Copy ECC sections of all chunks */
	ecc_bits = priv->gf_len * chip->ecc.strength;
	for (chunk = 0; chunk < chip->ecc.steps; chunk++) {
		remaining += ecc_bits;
		while (remaining >= 8) {
			/* Copy whole bytes; the sections are only a few bytes
			   long, so don't bother to use memcpy() */
			*oob++ = *raw_data++;
			remaining -= 8;
		}
		if (remaining) {
			/* The transition to the next section of ECC bytes is
			   within a byte. Get part of the byte from the
			   previous section and part of the byte from the next
			   section. These will always fit together without
			   shifting, we just need to mask the non-ECC bits. */
			mask = (1 << remaining) - 1;
			val = *raw_data & mask;
			raw_data += chunk_size;
			val |= *raw_data++ & ~mask;
			*oob++ = val;
			remaining -= 8;
			/* Now remaining is negative because we have borrowed
			   a few bits from the next chunk already. The next
			   chunk always has more than 8 bits of ECC data, so
			   we do not have to care about special cases.*/
		} else
			raw_data += chunk_size;
	}
}

/*
 * Copy the ECC data from oob[] to raw page data in the DMA page buffer. The
 * BBM and user part of the OOB area are not touched!
 *
 * The complicated part is the ECC data. It is interleaved with the main data
 * and ECC data is not necessarily aligned to bytes. But because the main data
 * chunks always consist of full bytes, the ECC parts will always fit together
 * without the need for bit shifting.
 *
 * See gpmi_fus_read_ecc_data() for a detailed explanation and an example.
 */
static void gpmi_fus_write_ecc_data(struct mtd_info *mtd, const uint8_t *oob)
{
	struct nand_chip *chip = mtd->priv;
	struct gpmi_nand_data *priv = chip->priv;
	int chunk, remaining = 0;
	int chunk_size = chip->ecc.size;
	uint8_t val, mask;
	uint headersize = mtd->oobavail + 4;
	uint8_t *raw_data = priv->page_buffer_virt;
	unsigned int ecc_bits;

	/* Adjust pointers to first ECC section */
	oob += headersize;
	raw_data += headersize + chunk_size;

	/* Copy ECC sections of all chunks */
	ecc_bits = priv->gf_len * chip->ecc.strength;
	for (chunk = 0; chunk < chip->ecc.steps; chunk++) {
		remaining += ecc_bits;
		while (remaining >= 8) {
			/* Copy whole bytes; the sections are only a few bytes
			   long, so don't bother to use memcpy() */
			*raw_data++ = *oob++;
			remaining -= 8;
		}
		if (remaining) {
			/* The transition to the next section of ECC bytes is
			   within a byte. This goes without shifting, we just
			   need to split the byte in a part that goes to the
			   previous section and a part that goes to the next
			   section. Remark: we fill unused bits with 1 to
			   make it possible to store the OOB only, where we
			   only write the ECC and not the main data. 1-bits
			   should keep the old content. */
			val = *oob++;
			mask = (1 << remaining) - 1;
			*raw_data = ~mask | (val & mask);
			raw_data += chunk_size;
			*raw_data++ = mask | (val & ~mask);
			remaining -= 8;
			/* Now remaining is negative because we have borrowed
			   a few bits from the next chunk already. The next
			   chunk always has more than 8 bits of ECC data, so
			   we do not have to care about special cases.*/
		} else
			raw_data += chunk_size;
	}
}

/*
 * Read OOB and create virtual OOB area. If column is set, skip a part at the
 * beginning.
 *
 * See gpmi_fus_read_ecc_data() for a description of the OOB area layout.
 */
static int gpmi_fus_do_read_oob(struct mtd_info *mtd, struct nand_chip *chip,
				int page, int raw)
{
	struct gpmi_nand_data *priv = chip->priv;
	struct dma_async_tx_descriptor *desc;
	uint32_t boffs;
	int column = raw ? 0 : 4;
	int length = mtd->oobavail + 4 - column;
	uint32_t from, to;
	int i, ret;
	dma_addr_t phys;
	unsigned int ecc_bits;

	/* Issue read command for the page */
	chip->cmdfunc(mtd, NAND_CMD_READ0, column, page);
	chip->cmdfunc(mtd, NAND_CMD_READSTART, -1, -1);
	ret = gpmi_fus_wait_ready(mtd, GPMI_FUS_TIMEOUT_DATA);
	if (ret)
		return ret;

	/* Read the BBM (optional) and the user part of the OOB area */
	phys = priv->page_buffer_phys;
	desc = gpmi_fus_read_data_buf(mtd, phys, length);
	if (!desc)
		return -EINVAL;
	if (raw) {
		/* Now read all the ECC bytes, section by section. The data is
		   still in the NAND page buffer, so we can use RNDOUT */
		ecc_bits = priv->gf_len	* chip->ecc.strength;
		boffs = (mtd->oobavail + 4) << 3;
		i = chip->ecc.steps;
		do {
			/* Add DMA descriptor to read only a few bytes */
			boffs += chip->ecc.size << 3;
			from = boffs >> 3;
			boffs += ecc_bits;
			to = (boffs - 1) >> 3;
			chip->cmdfunc(mtd, NAND_CMD_RNDOUT, from, -1);
			chip->cmdfunc(mtd, NAND_CMD_RNDOUTSTART, -1, -1);
			desc = gpmi_fus_read_data_buf(mtd, phys + from,
						      to - from + 1);
			if (!desc)
				return -EINVAL;
		} while (--i);
	}

	/* Run the DMA chain that reads all in one go */
	ret = start_dma_without_bch_irq(priv, desc);
	if (ret)
		return ret;

	/* Read BBM (optional) and user part of the OOB area */
	memcpy(chip->oob_poi + column, priv->page_buffer_virt, length);

	if (raw) {
		/* Compile the ECC data at the end of the OOB area */
		gpmi_fus_read_ecc_data(mtd, chip->oob_poi);
	}

	return 0;
}

/*
 * Write the virtual OOB area to NAND flash.
 *
 * See gpmi_fus_read_ecc_data() for a description of the OOB area layout.
 */
static int gpmi_fus_do_write_oob(struct mtd_info *mtd, struct nand_chip *chip,
				 int page, int raw)
{
	struct gpmi_nand_data *priv = chip->priv;
	struct dma_async_tx_descriptor *desc;
	uint32_t boffs;
	int column = raw ? 0 : 4;
	int length = mtd->oobavail + 4 - column;
	uint32_t from, to;
	int i;
	dma_addr_t phys;
	unsigned int ecc_bits;

	/* Write the BBM (optional) and the user part of the OOB area */
	chip->cmdfunc(mtd, NAND_CMD_SEQIN, column, page);
	memcpy(priv->page_buffer_virt, chip->oob_poi + column, length);
	gpmi_fus_write_data_buf(mtd, 0, length);

	phys = priv->page_buffer_phys;
	if (raw) {
		/* Store ECC data in data_buf at the appropriate positions */
		gpmi_fus_write_ecc_data(mtd, chip->oob_poi);

		/* Write all the ECC bytes to the page, section by section */
		ecc_bits = priv->gf_len	* chip->ecc.strength;
		boffs = (mtd->oobavail + 4) << 3;
		i = chip->ecc.steps;
		do {
			/* We actually only write a few bytes */
			boffs += chip->ecc.size << 3;
			from = boffs >> 3;
			boffs += ecc_bits;
			to = (boffs - 1) >> 3;
			chip->cmdfunc(mtd, NAND_CMD_RNDIN, from, -1);
			desc = gpmi_fus_write_data_buf(mtd, phys + from,
						       to - from + 1);
			if (!desc)
				return -EINVAL;
		} while (--i);
	}

	/* Now actually do the programming */
	chip->cmdfunc(mtd, NAND_CMD_PAGEPROG, -1, -1);

	/* Check if it worked */
	if (chip->waitfunc(mtd, chip) & NAND_STATUS_FAIL)
		return -EIO;

	return 0;
}

static int
acquire_register_block(struct gpmi_nand_data *this, const char *res_name)
{
	struct platform_device *pdev = this->pdev;
	struct resources *res = &this->resources;
	struct resource *r;
	void __iomem *p;

	r = platform_get_resource_byname(pdev, IORESOURCE_MEM, res_name);
	p = devm_ioremap_resource(&pdev->dev, r);
	if (IS_ERR(p))
		return PTR_ERR(p);

	if (!strcmp(res_name, GPMI_NAND_GPMI_REGS_ADDR_RES_NAME))
		res->gpmi_regs = p;
	else if (!strcmp(res_name, GPMI_NAND_BCH_REGS_ADDR_RES_NAME))
		res->bch_regs = p;
	else
		dev_err(this->dev, "unknown resource name : %s\n", res_name);

	return 0;
}

static char *extra_clks_for_mx6q[GPMI_CLK_MAX] = {
	"gpmi_apb", "gpmi_bch", "gpmi_bch_apb", "per1_bch",
};

static char *extra_clks_for_mx7d[GPMI_CLK_MAX] = {
	"gpmi_bch_apb",
};

static int gpmi_get_clks(struct gpmi_nand_data *this)
{
	struct resources *r = &this->resources;
	char **extra_clks = NULL;
	struct clk *clk;
	int err, i;

	/* The main clock is stored in the first. */
	r->clock[0] = devm_clk_get(this->dev, "gpmi_io");
	if (IS_ERR(r->clock[0])) {
		err = PTR_ERR(r->clock[0]);
		goto err_clock;
	}

	/* Get extra clocks */
	if (GPMI_IS_MX6(this))
		extra_clks = extra_clks_for_mx6q;
	if (GPMI_IS_MX7(this))
		extra_clks = extra_clks_for_mx7d;
	if (!extra_clks)
		return 0;

	for (i = 1; i < GPMI_CLK_MAX; i++) {
		if (extra_clks[i - 1] == NULL)
			break;

		clk = devm_clk_get(this->dev, extra_clks[i - 1]);
		if (IS_ERR(clk)) {
			err = PTR_ERR(clk);
			goto err_clock;
		}

		r->clock[i] = clk;
	}

	if (GPMI_IS_MX6(this) || GPMI_IS_MX7(this))
		/*
		 * Set the default value for the gpmi clock in mx6q:
		 *
		 * If you want to use the ONFI nand which is in the
		 * Synchronous Mode, you should change the clock as you need.
		 */
		clk_set_rate(r->clock[0], 22000000);

	return 0;

err_clock:
	dev_dbg(this->dev, "failed in finding the clocks.\n");
	return err;
}

static int init_rpm(struct gpmi_nand_data *this)
{
	pm_runtime_enable(this->dev);
	pm_runtime_set_autosuspend_delay(this->dev, GPMI_FUS_TIMEOUT_RPM);
	pm_runtime_use_autosuspend(this->dev);

	return 0;
}

static int acquire_resources(struct gpmi_nand_data *this)
{
	int ret;

	ret = acquire_register_block(this, GPMI_NAND_GPMI_REGS_ADDR_RES_NAME);
	if (ret)
		goto exit_regs;

	ret = acquire_register_block(this, GPMI_NAND_BCH_REGS_ADDR_RES_NAME);
	if (ret)
		goto exit_regs;

	ret = acquire_bch_irq(this, bch_irq);
	if (ret)
		goto exit_regs;

	ret = acquire_dma_channels(this);
	if (ret)
		goto exit_regs;

	ret = gpmi_get_clks(this);
	if (ret)
		goto exit_clock;

	return 0;

exit_clock:
	release_dma_channels(this);
exit_regs:
	return ret;
}

static void release_resources(struct gpmi_nand_data *this)
{
	release_dma_channels(this);
}


static int gpmi_init(struct gpmi_nand_data *this)
{
	struct resources *r = &this->resources;
	int ret;

	ret = pm_runtime_get_sync(this->dev);
	if (ret < 0) {
		dev_err(this->dev, "Failed to enable clock\n");
		return ret;
	}

	ret = gpmi_reset_block(this, r->gpmi_regs);
	if (!ret) {
		u32 ctrl1 = readl(r->gpmi_regs + HW_GPMI_CTRL1);

		/* Choose NAND mode. */
		ctrl1 &= ~BM_GPMI_CTRL1_GPMI_MODE;

		/* Set IRQ polarity, disable Write-Protection, select BCH ECC */
		ctrl1 |= BM_GPMI_CTRL1_ATA_IRQRDY_POLARITY
			| BM_GPMI_CTRL1_DEV_RESET
			| BM_GPMI_CTRL1_DECOUPLE_CS
			| BM_GPMI_CTRL1_BCH_MODE;

		writel(ctrl1, r->gpmi_regs + HW_GPMI_CTRL1);
	}

	/*
	 * Decouple the chip select from dma channel. We use dma0 for all
	 * the chips.
	 */
//###??	writel(BM_GPMI_CTRL1_DECOUPLE_CS, r->gpmi_regs + HW_GPMI_CTRL1_SET);

	pm_runtime_mark_last_busy(this->dev);
	pm_runtime_put_autosuspend(this->dev);

	return ret;
}

static int init_hardware(struct gpmi_nand_data *this)
{
	int ret;

	/*
	 * This structure contains the "safe" GPMI timing that should succeed
	 * with any NAND Flash device
	 * (although, with less-than-optimal performance).
	 */
	struct nand_timing  safe_timing = {
		.data_setup_in_ns        = 80,
		.data_hold_in_ns         = 60,
		.address_setup_in_ns     = 25,
		.gpmi_sample_delay_in_ns =  6,
		.tREA_in_ns              = -1,
		.tRLOH_in_ns             = -1,
		.tRHOH_in_ns             = -1,
	};

	ret = gpmi_init(this);
	if (!ret)
		this->timing = safe_timing;

	return ret;

}

/* -------------------- INTERFACE FUNCTIONS -------------------------------- */

/*
 * Select, change or deselect the NAND chip, activate or deactivate hardware
 */
static void gpmi_fus_select_chip(struct mtd_info *mtd, int chipnr)
{
	struct nand_chip *chip = mtd->priv;
	struct gpmi_nand_data *this = chip->priv;

	if ((this->current_chip < 0) && (chipnr >= 0))
		gpmi_begin(this);
	else if ((this->current_chip >= 0) && (chipnr < 0))
		gpmi_end(this);

	this->current_chip = chipnr;
}

/*
 * Test if the NAND flash is ready.
 */
static int gpmi_fus_dev_ready(struct mtd_info *mtd)
{
	/* We have called gpmi_fus_wait_ready() before, so we can be sure that
	   our command is already completed. So just return "done". */
	return 1;
}

/*
 * Read a single byte from NAND.
 */
static uint8_t gpmi_fus_read_byte(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	struct gpmi_nand_data *priv = chip->priv;
	struct dma_async_tx_descriptor *desc;

	desc = gpmi_fus_read_data_buf(mtd, priv->data_buffer_phys, 1);
	if (desc) {
		if (!start_dma_without_bch_irq(priv, desc))
			return priv->data_buffer_virt[0];
	}

	return 0xFF;
}

/*
 * Read a word from NAND.
 */
static u16 gpmi_fus_read_word(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	struct gpmi_nand_data *priv = chip->priv;
	struct dma_async_tx_descriptor *desc;

	desc = gpmi_fus_read_data_buf(mtd, priv->data_buffer_phys, 2);
	if (desc) {
		if (!start_dma_without_bch_irq(priv, desc))
			return priv->data_buffer_virt[0]
				| (priv->data_buffer_virt[1] << 8);
	}

	return 0xFFFF;
}

/*
 * Read arbitrary data from NAND.
 */
static void gpmi_fus_read_buf(struct mtd_info *mtd, uint8_t *buf, int length)
{
	struct nand_chip *chip = mtd->priv;
	struct gpmi_nand_data *priv = chip->priv;
	struct dma_async_tx_descriptor *desc;

	desc = gpmi_fus_read_data_buf(mtd, priv->data_buffer_phys, length);
	if (desc) {
		if (!start_dma_without_bch_irq(priv, desc))
			memcpy(buf, priv->data_buffer_virt, length);
	}
}

/*
 * Write arbitrary data to NAND.
 */
static void gpmi_fus_write_buf(struct mtd_info *mtd, const uint8_t *buf,
			       int length)
{
	struct nand_chip *chip = mtd->priv;
	struct gpmi_nand_data *priv = chip->priv;

	memcpy(priv->data_buffer_virt, buf, length);
	gpmi_fus_write_data_buf(mtd, priv->data_buffer_phys, length);
}

/*
 * Wait until chip is ready, then read status.
 */
static int gpmi_fus_waitfunc(struct mtd_info *mtd, struct nand_chip *chip)
{
	unsigned long timeout;

	if (chip->state == FL_ERASING)
		timeout = GPMI_FUS_TIMEOUT_ERASE;
	else
		timeout = GPMI_FUS_TIMEOUT_WRITE;

	/* Add DMA descriptor to wait for ready and execute the DMA chain */
	if (gpmi_fus_wait_ready(mtd, timeout))
		return NAND_STATUS_FAIL;

	/* Issue NAND_CMD_STATUS command to read status */
	chip->cmdfunc(mtd, NAND_CMD_STATUS, -1, -1);

	/* Read and return status byte */
	return gpmi_fus_read_byte(mtd);
}

/* Write command to NAND flash. This usually results in creating a chain of
   DMA descriptors which is executed at the end. */
static void gpmi_fus_command(struct mtd_info *mtd, uint command, int column,
			     int page)
{
	struct nand_chip *chip = mtd->priv;
	struct gpmi_nand_data *priv = chip->priv;

	/* Simulate NAND_CMD_READOOB with NAND_CMD_READ0; NAND_CMD_READOOB is
	   only used in nand_block_bad() to read the BBM. So simply use the
	   command that will load the BBM correctly. */
	if (command == NAND_CMD_READOOB)
		command = NAND_CMD_READ0;
	/*
	 * The command sequence should be:
	 *   NAND_CMD_READ0 Col+Row -> NAND_CMD_READSTART -> Ready -> Read data
	 *	  {-> NAND_CMD_RNDOUT col -> NAND_CMD_RNDOUTSTART -> Read data}
	 *   NAND_CMD_SEQIN Col+Row -> Write data
	 *	  {-> NAND_CMD_RNDIN Col -> Write Data}
	 *	  -> NAND_CMD_PAGEPROG -> Ready
	 *   NAND_CMD_ERASE1 Row -> NAND_CMD_ERASE2 -> Ready
	 *   NAND_CMD_STATUS -> Read 1 data byte
	 *   NAND_CMD_READID 0x00/0x20 -> Read ID/Read ONFI ID
	 *   NAND_CMD_PARAM 0x00 -> Ready -> Read ONFI parameters
	 *   NAND_CMD_SET_FEATURES addr -> Write 4 data bytes -> Ready
	 *   NAND_CMD_GET_FEATURES addr -> Ready -> Read 4 data bytes
	 *   NAND_CMD_RESET -> Ready
	 */
	switch (command) {
	case NAND_CMD_RNDIN:
	case NAND_CMD_READ0:
	case NAND_CMD_READSTART:
	case NAND_CMD_RNDOUT:
	case NAND_CMD_SEQIN:
	case NAND_CMD_PAGEPROG:
	case NAND_CMD_ERASE1:
	case NAND_CMD_ERASE2:
		/* Add DMA descriptor with command + column and/or row */
		gpmi_fus_add_cmd_desc(priv, command, column, page, -1);
		/* In case of NAND_CMD_PAGEPROG andNAND_CMD_ERASE2,
		   chip->waitfunc() is called later, so we need not wait now.
		   FIXME: In case of NAND_CMD_RNDIN we should have a delay of
		   t_CCS here so that the chip can switch columns */
		break;

	case NAND_CMD_PARAM:
	case NAND_CMD_GET_FEATURES:
		/* Add DMA descriptor with command and one address byte */
		gpmi_fus_add_cmd_desc(priv, command, -1, -1, column);
		/* Add DMA descriptor to wait for ready and run DMA chain */
		gpmi_fus_wait_ready(mtd, GPMI_FUS_TIMEOUT_DATA);
		break;

	case NAND_CMD_RESET:
		/* Add DMA descriptor with command byte */
		gpmi_fus_add_cmd_desc(priv, command, -1, -1, -1);
		/* Add DMA descriptor to wait for ready and run DMA chain */
		gpmi_fus_wait_ready(mtd, GPMI_FUS_TIMEOUT_RESET);
		break;

	case NAND_CMD_RNDOUTSTART:
	case NAND_CMD_STATUS:
		column = -1;
		/* Fall through to case NAND_CMD_READID */
	case NAND_CMD_READID:
	case NAND_CMD_SET_FEATURES:
		/* Add DMA descriptor with command and up to one byte */
		gpmi_fus_add_cmd_desc(priv, command, -1, -1, column);
		/* According to the ONFI specification these commands never
		   set the busy signal so we need not wait for ready here.
		   FIXME: But in case of NAND_CMD_RNDOUTSTART we should have a
		   delay of t_CCS here so that the chip can switch columns.
		   FIXME: In case of NAND_CMD_SET_FEATURES we should have a
		   delay of t_ADL here so that the chip can switch mode. */
		break;

	default:
		printk("### NAND_CMD 0x%02x not supported\n", command);
		break;
	}
}

/*
 * Read a page from NAND without ECC
 */
static int gpmi_fus_read_page_raw(struct mtd_info *mtd, struct nand_chip *chip,
				  uint8_t *buf, int oob_required, int page)
{
	struct gpmi_nand_data *priv = chip->priv;
	struct dma_async_tx_descriptor *desc;
	int ret;

	/*
	 * A DMA descriptor for the first command byte NAND_CMD_READ0 and the
	 * column/row bytes was already created in nand_do_read_ops(). Now add
	 * a DMA descriptor for the second command byte NAND_CMD_READSTART and
	 * a DMA descriptor to wait for ready. Execute this DMA chain and
	 * check for timeout.
	 */
	chip->cmdfunc(mtd, NAND_CMD_READSTART, -1, -1);
	ret = gpmi_fus_wait_ready(mtd, GPMI_FUS_TIMEOUT_DATA);
	if (ret)
		return ret;

	/* Add DMA descriptor to read the whole page and execute DMA chain */
	desc = gpmi_fus_read_data_buf(mtd, priv->page_buffer_phys,
				      mtd->writesize + mtd->oobsize);
	if (!desc)
		return -EINVAL;
	ret = start_dma_without_bch_irq(priv, desc);
	if (ret)
		return ret;

	/* Copy the main data to buf */
	gpmi_fus_read_main_data(mtd, buf);

	if (oob_required) {
		/* Copy Bad Block Marker and User OOB section */
		memcpy(chip->oob_poi, priv->page_buffer_virt,
		       mtd->oobavail + 4);

		/* Copy the ECC data */
		gpmi_fus_read_ecc_data(mtd, chip->oob_poi);
	} else {
		/* Simply clear OOB area */
		memset(chip->oob_poi, 0xFF, mtd->oobsize);
	}

	return 0;
}

/*
 * Write a page to NAND without ECC
 */
static int gpmi_fus_write_page_raw(struct mtd_info *mtd, struct nand_chip *chip,
				   const uint8_t *buf, int oob_required, int page)
{
	struct gpmi_nand_data *priv = chip->priv;

	/* NAND_CMD_SEQIN for column 0 was already issued by the caller */

	/* We must write OOB data as it is interleaved with main data. But if
	   no OOB data is available, use 0xFF instead. This does not modify
	   the existing values. */
	if (!oob_required)
		memset(chip->oob_poi, 0xFF, mtd->oobsize);

	/* Copy the OOB data to data_buf[]. Please note that this has to be
	   done before the main data, because gpmi_fus_write_ecc_data() will
	   use 1-bits in unused positions that must be overwritten by the
	   main data later. */
	gpmi_fus_write_ecc_data(mtd, chip->oob_poi);

	/* Copy the main data to data_buf[] */
	gpmi_fus_write_main_data(mtd, buf);

	/* Copy Bad Block Marker and User OOB section */
	memcpy(priv->page_buffer_virt, chip->oob_poi, mtd->oobavail + 4);

	/* Add DMA descriptor to write DMA page buffer to NAND flash */
	if (!gpmi_fus_write_data_buf(mtd, priv->page_buffer_phys,
				     mtd->writesize + mtd->oobsize))
		return -EINVAL;

	/* The actual programming will take place in gpmi_fus_command() when
	   command NAND_CMD_PAGEPROG is sent */
	return 0;
}

/*
 * Read a page from NAND with ECC
 */
static int gpmi_fus_read_page(struct mtd_info *mtd, struct nand_chip *chip,
			      uint8_t *buf, int oob_required, int page)
{
	struct gpmi_nand_data *priv = chip->priv;
	struct resources *r = &priv->resources;
	struct dma_chan *channel = get_dma_chan(priv);
	struct dma_async_tx_descriptor *desc;
	struct mxs_dma_ccw ccw[3];
	uint32_t corrected = 0;
	unsigned char *status;
	int i, ret;
	bool direct_dma_map_ok;
	dma_addr_t try_phys;
	dma_addr_t payload_phys;

//	showdesc = 1;//###

	/* Select the desired flash layout */
	writel(priv->bch_layout, r->bch_regs + HW_BCH_LAYOUTSELECT);

	/*
	 * A DMA descriptor for the first command byte NAND_CMD_READ0 and the
	 * column/row bytes was already created in nand_do_read_ops(). Now add
	 * a DMA descriptor for the second command byte NAND_CMD_READSTART and
	 * a DMA descriptor to wait for ready. Execute this DMA chain and
	 * check for timeout.
	 */
	chip->cmdfunc(mtd, NAND_CMD_READSTART, -1, -1);
	ret = gpmi_fus_wait_ready(mtd, GPMI_FUS_TIMEOUT_DATA);
	if (ret)
		return ret;

	/* Try to map buf directly for DMA. If this fails, use the local page
	   buffer. */
	direct_dma_map_ok = 0;
	payload_phys = priv->page_buffer_phys;
	if (virt_addr_valid(buf)) {
		try_phys = dma_map_single(priv->dev, buf, mtd->writesize,
					  DMA_FROM_DEVICE);
		if (!dma_mapping_error(priv->dev, try_phys)) {
			direct_dma_map_ok = 1;
			payload_phys = try_phys;
		}
	}

	/* Prepare DMA descriptor to enable the BCH block and read */
	ccw[0].next = 0;
	ccw[0].cmd = CCW_CMD_DMA_NO_XFER
		| CCW_NAND_LOCK
		| CCW_WAIT4END
		| CCW_PIO_NUM(6);
	ccw[0].bufaddr = 0;
	ccw[0].pio[0] = BF_GPMI_CTRL0_CS(priv->current_chip, priv)
		| BF_GPMI_CTRL0_LOCK_CS(LOCK_CS_ENABLE, priv)
		| BF_GPMI_CTRL0_COMMAND_MODE(BV_GPMI_CTRL0_COMMAND_MODE__READ)
		| BM_GPMI_CTRL0_WORD_LENGTH
		| BF_GPMI_CTRL0_ADDRESS(BV_GPMI_CTRL0_ADDRESS__NAND_DATA)
		| BF_GPMI_CTRL0_XFER_COUNT(mtd->writesize + mtd->oobsize);
	ccw[0].pio[1] = 0;
	ccw[0].pio[2] = BM_GPMI_ECCCTRL_ENABLE_ECC
		| BF_GPMI_ECCCTRL_ECC_CMD(BV_GPMI_ECCCTRL_ECC_CMD__BCH_DECODE)
		| BV_GPMI_ECCCTRL_BUFFER_MASK__BCH_PAGE;
	ccw[0].pio[3] = mtd->writesize + mtd->oobsize;
	ccw[0].pio[4] = payload_phys;
	ccw[0].pio[5] = priv->auxiliary_phys;

	/* Prepare DMA descriptor to disable the BCH block (wait-for-ready is
	   a NOP here) */
	ccw[1].next = 0;
	ccw[1].cmd = CCW_CMD_DMA_NO_XFER
		| CCW_NAND_LOCK
		| CCW_NAND_WAIT4READY
		| CCW_WAIT4END
		| CCW_PIO_NUM(3);
	ccw[1].bufaddr = 0;
	ccw[1].pio[0] = BF_GPMI_CTRL0_CS(priv->current_chip, priv)
		| BF_GPMI_CTRL0_COMMAND_MODE(BV_GPMI_CTRL0_COMMAND_MODE__WAIT_FOR_READY)
		| BM_GPMI_CTRL0_WORD_LENGTH
		| BF_GPMI_CTRL0_ADDRESS(BV_GPMI_CTRL0_ADDRESS__NAND_DATA);
	ccw[1].pio[1] = 0;
	ccw[1].pio[2] = 0;

	/* Prepare DMA descriptor to deassert NAND lock and issue interrupt */
	ccw[2].next = 0;
	ccw[2].cmd = CCW_CMD_DMA_NO_XFER
		| CCW_PIO_NUM(0);
	ccw[2].bufaddr = 0;

	if (showdesc) //###
		printk("### READ-PAGE-DESC: phys=0x%08x\n", payload_phys);

	/* Add all three DMA descriptors to DMA chain */
	desc = dmaengine_prep_slave_sg(channel, (struct scatterlist *)ccw,
				       3, 0, 0);
	if (!desc)
		return -EINVAL;

	/* Execute DMA chain */
	ret = start_dma_with_bch_irq(priv, desc);

	/* Before handling any errors, unmap buf again if it was mapped */
	if (direct_dma_map_ok)
		dma_unmap_single(priv->dev, payload_phys, mtd->writesize,
				 DMA_FROM_DEVICE);
	if (ret)
		return ret;

	/*
	 * Loop over status bytes, accumulating ECC status. The status bytes
	 * are located at the first 32 bit boundary behind the auxiliary data.
	 *   0x00:  no errors
	 *   0xFF:  empty (all bytes 0xFF)
	 *   0xFE:  uncorrectable
	 *   other: number of bitflips
	 * Save the maximum number of bitflips of all chunks in variable ret.
	 *
	 * Remark: Please note that from the point of view of the BCH engine
	 * the OOB area at the beginning of the page is a separate chunk. So
	 * it will have its own status byte, even if ECC is set to ECC0. So we
	 * have to check one more status byte than we have main chunks or we
	 * would miss bitflips in the last main chunk. Hence the +1 in the
	 * loop below.
	 */
	status = priv->status_virt;

	{ //###
		if (showdesc) {
			printk("### Status bytes:");
			for (i = 0; i < chip->ecc.steps + 1; i++)
				printk(" %02X", status[i]);
			printk("\n");
		}
	} //###
	ret = 0;
	for (i = 0; i < chip->ecc.steps + 1; i++) {
		switch (status[i]) {
		case STATUS_GOOD:
		case STATUS_ERASED:
			break;

		case STATUS_UNCORRECTABLE:
			mtd->ecc_stats.failed++;

			/* Note: buf may be changed if it was mapped directly;
			   otherwise it is unchanged. chip->oob_poi is
			   unchanged in any case. */
			return 0;

		default:
			/* Remember maximum bitflips of a chunk in ret; don't
			   modify mtd->ecc_stats.corrected directly as this
			   would be wrong in case a later chunk fails. */
			corrected += status[i];
			if (status[i] > ret)
				ret = status[i];
			break;
		}
	}
	mtd->ecc_stats.corrected += corrected;

	/* Copy the main data if DMA did not write directly to buf already */
	if (!direct_dma_map_ok)
		memcpy(buf, priv->page_buffer_virt, mtd->writesize);

	/* Copy the user OOB data if requested */
	if (oob_required) {
		memset(chip->oob_poi, 0xff, mtd->oobsize);
		memcpy(chip->oob_poi + 4,
		       priv->page_buffer_virt + mtd->writesize + 4,
		       mtd->oobavail);
	}

	/* Return maximum number of bitflips across all chunks */
	return ret;
}

/*
 * Write a page to NAND with ECC.
 */
static int gpmi_fus_write_page(struct mtd_info *mtd, struct nand_chip *chip,
			       const uint8_t *buf, int oob_required, int page)
{
	struct gpmi_nand_data *priv = chip->priv;
	struct resources *r = &priv->resources;
	struct dma_async_tx_descriptor *desc;
	struct dma_chan *channel = get_dma_chan(priv);
	struct mxs_dma_ccw ccw;
	bool direct_dma_map_ok;
	dma_addr_t try_phys;
	dma_addr_t payload_phys;
	int ret;

	/* Select the desired flash layout */
	writel(priv->bch_layout, r->bch_regs + HW_BCH_LAYOUTSELECT);

	/* Try to map buf directly for DMA. If this fails, use the local page
	   buffer and copy the data to it. */
	payload_phys = priv->page_buffer_phys;
	direct_dma_map_ok = 0;
	if (virt_addr_valid(buf)) {
		try_phys = dma_map_single(priv->dev, (void *)buf,
					  mtd->writesize, DMA_TO_DEVICE);
		if (!dma_mapping_error(priv->dev, try_phys)) {
			payload_phys = try_phys;
			direct_dma_map_ok = 1;
		}
	}
	if (!direct_dma_map_ok)
		memcpy(priv->page_buffer_virt, buf, mtd->writesize);

	/* We must provide OOB data because it is written in one go with the
	   main data. If no OOB data is available, use 0xFF instead. This
	   does not modify the existing values. */
	memset(priv->auxiliary_virt, 0xFF, mtd->oobavail + 4);
	if (oob_required)
		memcpy(priv->auxiliary_virt + 4, chip->oob_poi + 4,
		       mtd->oobavail);

	/* Prepare DMA descriptor to enable BCH and write data */
	ccw.next = 0;
	ccw.cmd = CCW_CMD_DMA_NO_XFER
		| CCW_NAND_LOCK
		| CCW_WAIT4END
		| CCW_PIO_NUM(6);
	ccw.bufaddr = 0;
	ccw.pio[0] = BF_GPMI_CTRL0_CS(priv->current_chip, priv)
		| BF_GPMI_CTRL0_LOCK_CS(LOCK_CS_ENABLE, priv)
		| BF_GPMI_CTRL0_COMMAND_MODE(BV_GPMI_CTRL0_COMMAND_MODE__WRITE)
		| BM_GPMI_CTRL0_WORD_LENGTH
		| BF_GPMI_CTRL0_ADDRESS(BV_GPMI_CTRL0_ADDRESS__NAND_DATA)
		| BF_GPMI_CTRL0_XFER_COUNT(0);
	ccw.pio[1] = 0;
	ccw.pio[2] = BM_GPMI_ECCCTRL_ENABLE_ECC
		| BF_GPMI_ECCCTRL_ECC_CMD(BV_GPMI_ECCCTRL_ECC_CMD__BCH_ENCODE)
		| BV_GPMI_ECCCTRL_BUFFER_MASK__BCH_PAGE;
	ccw.pio[3] = mtd->writesize + mtd->oobsize;
	ccw.pio[4] = payload_phys;
	ccw.pio[5] = priv->auxiliary_phys;

	/*
	 * Add the DMA descriptor to DMA chain and execute DMA chain.
	 *
	 * Remark:
	 * Theoretically we could wait with the DMA until NAND_CMD_PAGEPROG,
	 * but this would be complicated as we still need to unmap buf and
	 * have to use BCH when running the chain. So let's do it right here.
	 */
	if (showdesc) //###
		printk("### WRITE-PAGE-DESC: phys=0x%08x\n", payload_phys);
	desc = dmaengine_prep_slave_sg(channel, (struct scatterlist *)&ccw,
				       1, 0, 0);
	if (!desc)
		ret = -EINVAL;
	else
		ret = start_dma_with_bch_irq(priv, desc);

	if (direct_dma_map_ok)
		dma_unmap_single(priv->dev, payload_phys,
				 mtd->writesize, DMA_TO_DEVICE);

	/* The actual programming will take place in when command
	   NAND_CMD_PAGEPROG is sent. This also disables the BCH. */
	return 0;
}


/*
 * Read the OOB area, including the BBM and ECC info. Unfortunately the ECC
 * system of the i.MX6 CPU does not match the common view of NAND chips. So we
 * must compile a virtual OOB area by reading different parts of the page.
 *
 * See gpmi_fus_read_ecc_data() for a description of the OOB area layout.
 */
static int gpmi_fus_read_oob_raw(struct mtd_info *mtd,
				 struct nand_chip *chip, int page)
{
	return gpmi_fus_do_read_oob(mtd, chip, page, 1);
}

/*
 * Read the user part of the OOB area (behind the BBM, before the ECC data)
 *
 * See gpmi-fus_read_ecc_data() for a description of the OOB area layout.
 */
static int gpmi_fus_read_oob(struct mtd_info *mtd, struct nand_chip *chip,
			     int page)
{
	return gpmi_fus_do_read_oob(mtd, chip, page, 0);
}

/*
 * Write the OOB area, including the BBM and ECC info. Unfortunately the ECC
 * system of the i.MX6 CPU does not match the common view of NAND chips. So we
 * must store the (virtual) OOB area to different parts of the page.
 *
 * See gpmi_fus_read_ecc_data() for a description of the OOB area layout.
 */
static int gpmi_fus_write_oob_raw(struct mtd_info *mtd,
				  struct nand_chip *chip, int page)
{
	return gpmi_fus_do_write_oob(mtd, chip, page, 1);
}

/*
 * Write the user part of the OOB area (behind the BBM, before the ECC data)
 *
 * See gpmi_fus_read_ecc_data() for a description of the OOB area layout.
 */
static int gpmi_fus_write_oob(struct mtd_info *mtd, struct nand_chip *chip,
			      int page)
{
	return gpmi_fus_do_write_oob(mtd, chip, page, 0);
}

static void gpmi_fus_exit(struct gpmi_nand_data *priv)
{
	struct mtd_info *mtd = nand_to_mtd(&priv->nand);

	nand_release(mtd);

	/* Free the DMA buffers */
	if (priv->cmd_buffer_virt && virt_addr_valid(priv->cmd_buffer_virt))
		dma_free_coherent(priv->dev, PAGE_SIZE, priv->cmd_buffer_virt,
				  priv->cmd_buffer_phys);
	if (priv->page_buffer_virt && virt_addr_valid(priv->page_buffer_virt))
		dma_free_coherent(priv->dev, mtd->writesize + mtd->oobsize,
			       priv->page_buffer_virt, priv->page_buffer_phys);

	priv->cmd_buffer_virt	= NULL;
	priv->page_buffer_virt	= NULL;
}

static int gpmi_fus_init(struct gpmi_nand_data *priv)
{
	struct nand_chip *chip = &priv->nand;
	struct mtd_info *mtd = nand_to_mtd(chip);
	int ret;
	unsigned int chunk_shift;
	unsigned int ecc_strength;
	unsigned int skipblocks;
	unsigned int oobavail;
	unsigned int ecc_bytes;
	struct mtd_part_parser_data ppdata = {};

	/* init current chip */
	priv->current_chip	= -1;

	/* init the MTD data structures */
	mtd->priv		= chip;
	mtd->name		= "gpmi-nand";
	mtd->owner		= THIS_MODULE;

	/* init the nand_chip{}, we don't support a 16-bit NAND Flash bus. */
	chip->priv		= priv;
	chip->select_chip	= gpmi_fus_select_chip;
	chip->dev_ready		= gpmi_fus_dev_ready;
	chip->cmdfunc		= gpmi_fus_command;
	chip->read_byte		= gpmi_fus_read_byte;
	chip->read_word		= gpmi_fus_read_word;
	chip->read_buf		= gpmi_fus_read_buf;
	chip->write_buf		= gpmi_fus_write_buf;
	chip->waitfunc		= gpmi_fus_waitfunc;
	chip->options		= NAND_BBT_SCAN2NDPAGE | NAND_NO_SUBPAGE_WRITE;

	/* Allocate a combined comman/data/status buffer. This is used for
	   reading the NAND ID and ONFI data. PAGE_SIZE is enough. */
	priv->cmd_buffer_virt =
		dma_alloc_coherent(priv->dev, PAGE_SIZE,
				   &priv->cmd_buffer_phys, GFP_DMA);
	if (!priv->cmd_buffer_virt) {
		ret = -ENOMEM;
		goto err_out;
	}
	priv->data_buffer_virt = priv->cmd_buffer_virt + GPMI_FUS_CMD_BUF_SIZE;
	priv->data_buffer_phys = priv->cmd_buffer_phys + GPMI_FUS_CMD_BUF_SIZE;

	/* Read NAND ID and ONFI parameters, set basic MTD geometry info */
	ret = nand_scan_ident(mtd, 1 /*### OF->max_chips */, NULL);
	if (ret) {
		mtd->name = NULL;
		goto err_out;
	}

	chunk_shift = 9;
	ecc_strength = 0;
	if (of_property_read_u32(priv->dev->of_node, "fus,skipblocks",
				 &skipblocks))
		skipblocks = 0;
	mtd->skip = skipblocks * mtd->erasesize;
	if (of_property_read_bool(priv->dev->of_node, "fus,skip_inverse")) {
		mtd->size = mtd->skip;
		mtd->skip = 0;
	}
	if (of_property_read_bool(priv->dev->of_node, "fus,chunk1k"))
		chunk_shift = 10;

	if (of_property_read_u32(priv->dev->of_node, "fus,ecc_strength",
				 &ecc_strength))
		ecc_strength = 0;
	if (!ecc_strength
	    || (ecc_strength > priv->devdata->bch_max_ecc_strength))
		ecc_strength = gpmi_fus_get_ecc_strength(mtd, chunk_shift);

	chip->ecc.strength = ecc_strength;
	priv->gf_len = (chunk_shift > 9) ? 14 : 13;

	/* Please note that mtd->oobavail is not yet set here */
	chip->ecc.mode = NAND_ECC_HW;
	chip->ecc.size = 1 << chunk_shift;
	chip->ecc.steps = mtd->writesize >> chunk_shift;
	ecc_bytes = ecc_strength * chip->ecc.steps * priv->gf_len / 8;
	chip->ecc.bytes = ecc_bytes;
	oobavail = mtd->oobsize - ecc_bytes - 4;
	mtd_set_ooblayout(mtd, &gpmi_ooblayout_fus_ops);

	mtd->oobavail = oobavail;
	mtd->ecc_strength = ecc_strength;

	/* Set ECC access functions */
	chip->ecc.read_page = gpmi_fus_read_page;
	chip->ecc.write_page = gpmi_fus_write_page;
	chip->ecc.read_oob = gpmi_fus_read_oob;
	chip->ecc.write_oob = gpmi_fus_write_oob;
	chip->ecc.read_page_raw = gpmi_fus_read_page_raw;
	chip->ecc.write_page_raw = gpmi_fus_write_page_raw;
	chip->ecc.read_oob_raw = gpmi_fus_read_oob_raw;
	chip->ecc.write_oob_raw = gpmi_fus_write_oob_raw;

	if (ecc_strength >= 20)
		mtd->bitflip_threshold = ecc_strength - 2;
	else
		mtd->bitflip_threshold = ecc_strength - 1;

	if (chip->onfi_version) {
		/* Get address cycles from ONFI data:
		   [7:4] column cycles, [3:0] row cycles */
		priv->column_cycles = chip->onfi_params.addr_cycles >> 4;
		priv->row_cycles = chip->onfi_params.addr_cycles & 0x0F;
	} else {
		/* Use two column cycles and decide from the size whether we
		   need two or three row cycles */
		priv->column_cycles = 2;
		if (chip->chipsize > (mtd->writesize << 16))
			priv->row_cycles = 3;
		else
			priv->row_cycles = 2;
	}

	/* Set up the NFC geometry which is used by BCH. */
	ret = bch_set_geometry(priv, oobavail, 0); /* ### TODO: set MTD device index */
	if (ret) {
		dev_err(priv->dev, "Set geometry failed with %d\n", ret);
		return ret;
	}

	/*
	 * Allocate the page buffer.
	 *
	 * Both the payload buffer and the auxiliary buffer must appear on
	 * 32-bit boundaries. We presume the size of the payload buffer is a
	 * power of two and is much larger than four, which guarantees the
	 * auxiliary buffer will appear on a 32-bit boundary.
	 *
	 * The status bytes are located at the first 32 bit boundary behind
	 * the auxiliary data.
	 */
	priv->page_buffer_virt =
		dma_alloc_coherent(priv->dev, mtd->writesize + mtd->oobsize,
				   &priv->page_buffer_phys, GFP_DMA);
	if (!priv->page_buffer_virt) {
		ret = -ENOMEM;
		goto err_out;
	}
	priv->auxiliary_virt = priv->page_buffer_virt + mtd->writesize;
	priv->auxiliary_phys = priv->page_buffer_phys + mtd->writesize;
	priv->status_virt = priv->auxiliary_virt + ((oobavail + 4 + 3) & ~3);

	/* Activate DDR or EDO mode if possible */
	ret = gpmi_extra_init(priv);
	if (ret != 0)
		goto err_out;

	/* Fill remaining mtd structure and create bad block table (BBT) */
	ret = nand_scan_tail(mtd);
	if (ret) {
		mtd->name = NULL;
		goto err_out;
	}

	/* Parse MTD partition table and register MTD device */
	ret = mtd_device_parse_register(mtd, NULL, &ppdata, NULL, 0);
	if (!ret)
		return 0;

err_out:
	gpmi_fus_exit(priv);
	return ret;
}

static const struct platform_device_id gpmi_fus_ids[] = {
	{ .name = "imx6q-gpmi-nand", .driver_data = IS_MX6Q, },
	{}
};

static const struct of_device_id gpmi_nand_fus_id_table[] = {
	{
		.compatible = "fus,imx6q-gpmi-nand",
		.data = (void *)&gpmi_devdata_imx6q,
	}, {
		.compatible = "fus,imx6qp-gpmi-nand",
		.data = (void *)&gpmi_devdata_imx6qp,
	}, {
		.compatible = "fus,imx6sx-gpmi-nand",
		.data = (void *)&gpmi_devdata_imx6sx,
	}, {
		.compatible = "fus,imx6ul-gpmi-nand",
		.data = (void *)&gpmi_devdata_imx6ul,
	}, {
		.compatible = "fus,imx7d-gpmi-nand",
		.data = (void *)&gpmi_devdata_imx7d,
	}, { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, gpmi_nand_fus_id_table);

static int gpmi_nand_fus_probe(struct platform_device *pdev)
{
	struct gpmi_nand_data *this;
	const struct of_device_id *of_id;
	int ret;

	this = devm_kzalloc(&pdev->dev, sizeof(*this), GFP_KERNEL);
	if (!this) {
		dev_err(&pdev->dev, "Failed to allocate per-device memory\n");
		return -ENOMEM;
	}

	of_id = of_match_device(gpmi_nand_fus_id_table, &pdev->dev);
	if (of_id) {
		this->devdata = of_id->data;
	} else {
		dev_err(&pdev->dev, "Failed to find the right device id\n");
		return -ENODEV;
	}

	platform_set_drvdata(pdev, this);
	this->pdev  = pdev;
	this->dev   = &pdev->dev;

	ret = acquire_resources(this);
	if (ret)
		goto exit_acquire_resources;

	ret = init_rpm(this);
	if (ret)
		goto exit_nfc_init;

	ret = init_hardware(this);
	if (ret)
		goto exit_nfc_init;

	ret = gpmi_fus_init(this);
	if (ret)
		goto exit_nfc_init;

	dev_info(this->dev, "driver registered.\n");

	return 0;

exit_nfc_init:
	release_resources(this);
exit_acquire_resources:
	dev_err(this->dev, "driver registration failed: %d\n", ret);

	return ret;
}

static int gpmi_nand_fus_remove(struct platform_device *pdev)
{
	struct gpmi_nand_data *this = platform_get_drvdata(pdev);

	gpmi_fus_exit(this);
	pm_runtime_disable(this->dev);
	release_resources(this);
	return 0;
}

static int gpmi_nand_fus_pm_suspend(struct device *dev)
{
	struct gpmi_nand_data *this = dev_get_drvdata(dev);

	release_dma_channels(this);
	pinctrl_pm_select_sleep_state(dev);
	return 0;
}

static int gpmi_nand_fus_pm_resume(struct device *dev)
{
	struct gpmi_nand_data *this = dev_get_drvdata(dev);
	int ret;

	pinctrl_pm_select_default_state(dev);

	ret = acquire_dma_channels(this);
	if (ret < 0)
		return ret;

	/* re-init the GPMI registers */
	this->flags &= ~GPMI_TIMING_INIT_OK;
	ret = gpmi_init(this);
	if (ret) {
		dev_err(this->dev, "Error setting GPMI : %d\n", ret);
		return ret;
	}

	/* re-init the BCH registers */
	ret = bch_set_geometry(this, this->mtd.oobavail, 0);
	if (ret) {
		dev_err(this->dev, "Error setting BCH : %d\n", ret);
		return ret;
	}

	/* re-init others */
	gpmi_extra_init(this);

	return 0;
}

int gpmi_nand_fus_runtime_suspend(struct device *dev)
{
	struct gpmi_nand_data *this = dev_get_drvdata(dev);

	gpmi_enable_clk(this, false);
	release_bus_freq(BUS_FREQ_HIGH);

	return 0;
}

int gpmi_nand_fus_runtime_resume(struct device *dev)
{
	struct gpmi_nand_data *this = dev_get_drvdata(dev);
	int ret;

	ret = gpmi_enable_clk(this, true);
	if (ret)
		return ret;

	request_bus_freq(BUS_FREQ_HIGH);

	return 0;
}

static const struct dev_pm_ops gpmi_nand_fus_pm_ops = {
	SET_RUNTIME_PM_OPS(gpmi_nand_fus_runtime_suspend,
			   gpmi_nand_fus_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(gpmi_nand_fus_pm_suspend,
				gpmi_nand_fus_pm_resume)
};

static struct platform_driver gpmi_nand_fus_driver = {
	.driver = {
		.name = "gpmi-nand-fus",
		.pm = &gpmi_nand_fus_pm_ops,
		.of_match_table = gpmi_nand_fus_id_table,
	},
	.probe   = gpmi_nand_fus_probe,
	.remove  = gpmi_nand_fus_remove,
	.id_table = gpmi_fus_ids,
};

module_platform_driver(gpmi_nand_fus_driver);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("i.MX GPMI NAND Flash Controller Driver");
MODULE_LICENSE("GPL");
