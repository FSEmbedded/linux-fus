/*
 * Vybrid NAND Flash Controller Driver with Hardware ECC
 *
 * Copyright (C) 2019 F&S Elektronik Systeme GmbH
 *
 * This is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/rawnand.h>
#include <linux/mtd/partitions.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/platform_data/vf610_nfc_fus.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/nmi.h>

#define	DRV_NAME		"vf610_nfc_fus"
#define	DRV_VERSION		"V3.0"

#define ECC_STATUS_OFFS		0x8F8
#define ECC_STATUS_MASK		0x80
#define ECC_ERR_COUNT		0x3F

/*
 * Timeout values. In our case they must enclose the transmission time for the
 * command, address and data byte cycles. The timer resolution in U-Boot is
 * 1/1000s, i.e. CONFIG_SYS_HZ=1000 and any final value < 2 will not work well.
 */
#define NFC_TIMEOUT_DATA	(HZ * 100) / 1000
#define NFC_TIMEOUT_WRITE	(HZ * 100) / 1000
#define NFC_TIMEOUT_ERASE	(HZ * 400) / 1000

/* Addresses for NFC MAIN RAM BUFFER areas */
#define NFC_MAIN_AREA(n)		((n) *  0x1000)

/* Typical Flash Commands */
#define READ_PAGE_CMD_CODE		0x7EE0
#define PROGRAM_PAGE_CMD_CODE		0x7FC0 /* Without status */
#define SEQIN_PROGRAM_PAGE_CMD_CODE	0x7FD8 /* With PROGRAM_PAGE_CMD_BYTE1 */
#define RNDIN_PROGRAM_PAGE_CMD_CODE	0x71D8 /* With RANDOM_IN_CMD_BYTE */
#define ERASE_CMD_CODE			0x4EC0 /* Without status */
#define ERASESTAT_CMD_CODE		0x4ED8 /* With status */
//####define READ_ID_CMD_CODE		0x4804
#define READ_ID_CMD_CODE		0x6004
#define RESET_CMD_CODE			0x4040
#define DMA_PROGRAM_PAGE_CMD_CODE	0xFFC8
#define SEQIN_RANDOM_IN_CMD_CODE	0x7F40 /* With PROGRAM_PAGE_CMD_BYTE1 */
#define RNDIN_RANDOM_IN_CMD_CODE	0x7140 /* With RANDOM_IN_CMD_BYTE */
#define RANDOM_OUT_CMD_CODE		0x70E0
//####define STATUS_READ_CMD_CODE	0x4068
#define STATUS_READ_CMD_CODE		0x4048
#define READ_PARAM_CMD_CODE		0x6060

#define PAGE_READ_CMD_BYTE1		0x00
#define PAGE_READ_CMD_BYTE2		0x30
#define PROGRAM_PAGE_CMD_BYTE1		0x80
#define PROGRAM_PAGE_CMD_BYTE2		0x10
#define READ_STATUS_CMD_BYTE		0x70
#define ERASE_CMD_BYTE1			0x60
#define ERASE_CMD_BYTE2			0xD0
#define READ_ID_CMD_BYTE		0x90
#define RESET_CMD_BYTE			0xFF
#define RANDOM_IN_CMD_BYTE		0x85
#define RANDOM_OUT_CMD_BYTE1		0x05
#define RANDOM_OUT_CMD_BYTE2		0xE0

/* NFC ECC mode define */
#define ECC_BYPASS			0x0
#define ECC_8_BYTE			0x1
#define ECC_12_BYTE			0x2
#define ECC_15_BYTE			0x3
#define ECC_23_BYTE			0x4
#define ECC_30_BYTE			0x5
#define ECC_45_BYTE			0x6
#define ECC_60_BYTE			0x7
#define ECC_ERROR			1
#define ECC_RIGHT			0

/* Module-Relative Register Offsets */
#define NFC_SRAM_BUFFER			0x0000
#define NFC_FLASH_CMD1			0x3F00
#define NFC_FLASH_CMD2			0x3F04
#define NFC_COL_ADDR			0x3F08
#define NFC_ROW_ADDR			0x3F0c
#define NFC_FLASH_COMMAND_REPEAT	0x3F10
#define NFC_ROW_ADDR_INC		0x3F14
#define NFC_FLASH_STATUS1		0x3F18
#define NFC_FLASH_STATUS2		0x3F1c
#define NFC_DMA1_ADDR			0x3F20
#define NFC_DMA2_ADDR			0x3F34
#define NFC_DMA_CONFIG			0x3F24
#define NFC_CACHE_SWAP			0x3F28
#define NFC_SECTOR_SIZE			0x3F2c
#define NFC_FLASH_CONFIG		0x3F30
#define NFC_IRQ_STATUS			0x3F38

/* NFC_FLASH_CMD1 Field */
#define CMD1_MASK			0xFFFF0000
#define CMD1_SHIFT			0
#define CMD_BYTE2_MASK			0xFF000000
#define CMD_BYTE2_SHIFT			24
#define CMD_BYTE3_MASK			0x00FF0000
#define CMD_BYTE3_SHIFT			16

/* NFC_FLASH_CM2 Field */
#define CMD2_MASK			0xFFFFFF07
#define CMD2_SHIFT			0
#define CMD_BYTE1_MASK			0xFF000000
#define CMD_BYTE1_SHIFT			24
#define CMD_CODE_MASK			0x00FFFF00
#define CMD_CODE_SHIFT			8
#define BUFNO_MASK			0x00000006
#define BUFNO_SHIFT			1
#define BUSY_MASK			0x00000001
#define BUSY_SHIFT			0
#define START_MASK			0x00000001
#define START_SHIFT			0

/* NFC_COL_ADDR Field */
#define COL_ADDR_MASK			0x0000FFFF
#define COL_ADDR_SHIFT			0
#define COL_ADDR_COL_ADDR2_MASK		0x0000FF00
#define COL_ADDR_COL_ADDR2_SHIFT	8
#define COL_ADDR_COL_ADDR1_MASK		0x000000FF
#define COL_ADDR_COL_ADDR1_SHIFT	0

/* NFC_ROW_ADDR Field */
#define ROW_ADDR_MASK			0x00FFFFFF
#define ROW_ADDR_SHIFT			0
#define ROW_ADDR_CHIP_SEL_RB_MASK	0xF0000000
#define ROW_ADDR_CHIP_SEL_RB_SHIFT	28
#define ROW_ADDR_CHIP_SEL_MASK		0x0F000000
#define ROW_ADDR_CHIP_SEL_SHIFT		24
#define ROW_ADDR_ROW_ADDR3_MASK		0x00FF0000
#define ROW_ADDR_ROW_ADDR3_SHIFT	16
#define ROW_ADDR_ROW_ADDR2_MASK		0x0000FF00
#define ROW_ADDR_ROW_ADDR2_SHIFT	8
#define ROW_ADDR_ROW_ADDR1_MASK		0x000000FF
#define ROW_ADDR_ROW_ADDR1_SHIFT	0

/* NFC_FLASH_STATUS1 Field */
#define STATUS1_MASK			0xFFFFFFFF
#define STATUS1_SHIFT			0
#define STATUS1_ID_BYTE1_MASK		0xFF000000
#define STATUS1_ID_BYTE1_SHIFT		24
#define STATUS1_ID_BYTE2_MASK		0x00FF0000
#define STATUS1_ID_BYTE2_SHIFT		16
#define STATUS1_ID_BYTE3_MASK		0x0000FF00
#define STATUS1_ID_BYTE3_SHIFT		8
#define STATUS1_ID_BYTE4_MASK		0x000000FF
#define STATUS1_ID_BYTE4_SHIFT		0

/* NFC_FLASH_STATUS2 Field */
#define STATUS2_MASK			0xFF0000FF
#define STATUS2_SHIFT			0
#define STATUS2_ID_BYTE5_MASK		0xFF000000
#define STATUS2_ID_BYTE5_SHIFT		24
#define STATUS_BYTE1_MASK		0x000000FF
#define STATUS2_STATUS_BYTE1_SHIFT	0

/* NFC_FLASH_CONFIG Field */
#define CONFIG_MASK			0xFFFFFFFF
#define CONFIG_SHIFT			0
#define CONFIG_STOP_ON_WERR_MASK	0x80000000
#define CONFIG_STOP_ON_WERR_SHIFT	31
#define CONFIG_ECC_SRAM_ADDR_MASK	0x7FC00000
#define CONFIG_ECC_SRAM_ADDR_SHIFT	22
#define CONFIG_ECC_SRAM_REQ_MASK	0x00200000
#define CONFIG_ECC_SRAM_REQ_SHIFT	21
#define CONFIG_DMA_REQ_MASK		0x00100000
#define CONFIG_DMA_REQ_SHIFT		20
#define CONFIG_ECC_MODE_MASK		0x000E0000
#define CONFIG_ECC_MODE_SHIFT		17
#define CONFIG_FAST_FLASH_MASK		0x00010000
#define CONFIG_FAST_FLASH_SHIFT		16
#define CONFIG_ID_COUNT_MASK		0x0000E000
#define CONFIG_ID_COUNT_SHIFT		13
#define CONFIG_CMD_TIMEOUT_MASK		0x00001F00
#define CONFIG_CMD_TIMEOUT_SHIFT	8
#define CONFIG_16BIT_MASK		0x00000080
#define CONFIG_16BIT_SHIFT		7
#define CONFIG_BOOT_MODE_MASK		0x00000040
#define CONFIG_BOOT_MODE_SHIFT		6
#define CONFIG_ADDR_AUTO_INCR_MASK	0x00000020
#define CONFIG_ADDR_AUTO_INCR_SHIFT	5
#define CONFIG_BUFNO_AUTO_INCR_MASK	0x00000010
#define CONFIG_BUFNO_AUTO_INCR_SHIFT	4
#define CONFIG_PAGE_CNT_MASK		0x0000000F
#define CONFIG_PAGE_CNT_SHIFT		0

/* NFC_IRQ_STATUS Field */
#define MASK				0xEFFC003F
#define SHIFT				0
#define WERR_IRQ_MASK			0x80000000
#define WERR_IRQ_SHIFT			31
#define CMD_DONE_IRQ_MASK		0x40000000
#define CMD_DONE_IRQ_SHIFT		30
#define IDLE_IRQ_MASK			0x20000000
#define IDLE_IRQ_SHIFT			29
#define WERR_STATUS_MASK		0x08000000
#define WERR_STATUS_SHIFT		27
#define FLASH_CMD_BUSY_MASK		0x04000000
#define FLASH_CMD_BUSY_SHIFT		26
#define RESIDUE_BUSY_MASK		0x02000000
#define RESIDUE_BUSY_SHIFT		25
#define ECC_BUSY_MASK			0x01000000
#define ECC_BUSY_SHIFT			24
#define DMA_BUSY_MASK			0x00800000
#define DMA_BUSY_SHIFT			23
#define WERR_EN_MASK			0x00400000
#define WERR_EN_SHIFT			22
#define CMD_DONE_EN_MASK		0x00200000
#define CMD_DONE_EN_SHIFT		21
#define IDLE_EN_MASK			0x00100000
#define IDLE_EN_SHIFT			20
#define WERR_CLEAR_MASK			0x00080000
#define WERR_CLEAR_SHIFT		19
#define CMD_DONE_CLEAR_MASK		0x00040000
#define CMD_DONE_CLEAR_SHIFT		18
#define IDLE_CLEAR_MASK			0x00020000
#define IDLE_CLEAR_SHIFT		17
#define RESIDUE_BUFF_NO_MASK		0x00000030
#define RESIDUE_BUFF_NO_SHIFT		4
#define ECC_BUFF_NO_MASK		0x000000C0
#define ECC_BUFF_NO_SHIFT		2
#define DMA_BUFF_NO_MASK		0x00000003

/*
 * NAND page layouts for all possible Vybrid ECC modes. The OOB area has the
 * following appearance:
 *
 *   Offset          Bytes               Meaning
 *   ---------------------------------------------------------------
 *   0               4                   Bad Block Marker (BBM)
 *   4               eccbytes            Error Correction Code (ECC)
 *   4 + eccbytes    oobfree.length      Free OOB area (user part)
 *
 * As the size of the OOB is in mtd->oobsize and the number of free bytes
 * (= size of the user part) is stored in mtd->oobavail, we can compute the
 * offset of the free OOB area also by mtd->oobsize - mtd->oobavail.
 */
struct vf610_nfc_fus_prv {
	struct nand_chip	chip;
	int			irq;
	void __iomem		*regs;
	struct clk		*clk;
	wait_queue_head_t	irq_waitq;
	struct device		*dev;
	volatile u32		irq_handled;
	uint			column;		/* Column for read_byte() */
	uint			last_command;	/* Previous command issued */
	u32			eccmode;	/* ECC_BYPASS .. ECC_60_BYTE */
	u32			cfgbase;	/* fix: timeout, buswidth */
	u32			cmdclr;		/* Address cycle bits that
						   must be cleared in NFC cmd */
};

static const uint bitflip_threshold[8] = {
	1, 3, 5, 7, 11, 15, 22, 30
};

static const uint ecc_strength[8] = {
	0, 4, 6, 8, 12, 16, 24, 32
};

static const uint ecc_bytes[8] = {
	0, 8, 12, 15, 23, 30, 45, 60
};


/* -------------------- LOCAL HELPER FUNCTIONS ----------------------------- */

/* Read NFC register */
static inline u32 nfc_read(const struct nand_chip *chip, uint reg)
{
	return readl(chip->IO_ADDR_R + reg);
}


/* Write NFC register */
static inline void nfc_write(const struct nand_chip *chip, uint reg, u32 val)
{
	writel(val, chip->IO_ADDR_R + reg);
}


/* Read a byte from NFC RAM; the RAM is connected in 32 bit big endian mode to
   the bus, so we have to be careful with the lower two offset bits */
static inline u8 nfc_read_ram(const struct nand_chip *chip, uint offset)
{
	return readb(chip->IO_ADDR_R + NFC_MAIN_AREA(0) + (offset ^ 3));
}


/* Write a byte to NFC RAM; the RAM is connected in 32 bit big endian mode to
   the bus, so we have to be careful with the lower two offset bits */
static inline void nfc_write_ram(const struct nand_chip *chip,
				 uint offset, u8 val)
{
	writeb(val, chip->IO_ADDR_R + NFC_MAIN_AREA(0) + (offset ^ 3));
}


/* Copy data from NFC RAM */
static void nfc_copy_from_nfc(const struct nand_chip *chip, u8 *buffer,
			      uint size, uint ramoffset)
{
	while (size--)
		*buffer++ = nfc_read_ram(chip, ramoffset++);
}


/* Copy data to NFC RAM */
static void nfc_copy_to_nfc(const struct nand_chip *chip, const u8 *buffer,
			    uint size, uint ramoffset)
{
	while (size--)
		nfc_write_ram(chip, ramoffset++, *buffer++);
}


/* Start a comand with one command byte and wait for completion */
static inline void nfc_send_cmd_1(const struct nand_chip *chip,
				  u32 cmd_byte1, u32 cmd_code)
{
	struct vf610_nfc_fus_prv *prv = chip->priv;

	prv->irq_handled = 0;

	/* Write first command byte, command code, buffer 0, start transfer */
	nfc_write(chip, NFC_FLASH_CMD2,
		  (cmd_byte1 << CMD_BYTE1_SHIFT)
		  | (cmd_code << CMD_CODE_SHIFT)
		  | (0 << BUFNO_SHIFT)
		  | (1 << START_SHIFT));
}


/* Start a command with two (or three) command bytes, e.g. program page */
static inline void nfc_send_cmd_2(const struct nand_chip *chip,
				  u32 cmd_byte1, u32 cmd_byte2, u32 cmd_code)
{
	/* Write second command byte, the third command byte (if used) is
	   always NAND_CMD_STATUS */
	nfc_write(chip, NFC_FLASH_CMD1, (cmd_byte2 << CMD_BYTE2_SHIFT)
		  | (NAND_CMD_STATUS << CMD_BYTE3_SHIFT));

	/* The rest is like the one byte command */
	nfc_send_cmd_1(chip, cmd_byte1, cmd_code);
}


/* Set column for command */
static void nfc_set_column(struct nand_chip *chip, int column, uint command)
{
	if (column < 0) {
		printk("### Missing column for NAND_CMD 0x%02x\n", command);
		return;
	}

	nfc_write(chip, NFC_COL_ADDR, column << COL_ADDR_SHIFT);
}


/* Set row for command */
static void nfc_set_row(struct nand_chip *chip, int row, uint command)
{
	u32 rowreg;

	if (row < 0) {
		printk("### Missing row for NAND_CMD 0x%02x\n", command);
		return;
	}

	rowreg = nfc_read(chip, NFC_ROW_ADDR);
	rowreg &= ~ROW_ADDR_MASK;
	rowreg |= row << ROW_ADDR_SHIFT;
	nfc_write(chip, NFC_ROW_ADDR, rowreg);
}


static int nfc_do_write_oob(struct mtd_info *mtd, struct nand_chip *chip,
			    int page, uint offs)
{
	struct vf610_nfc_fus_prv *prv = chip->priv;

	/* Start programming sequence with NAND_CMD_SEQIN */
	chip->cmdfunc(mtd, NAND_CMD_SEQIN, offs + mtd->writesize, page);

	/* Copy OOB data to NFC RAM; as we only write these few bytes and no
	   main data, they have to be set at offset 0 in NFC RAM */
	nfc_copy_to_nfc(chip, chip->oob_poi + offs, mtd->oobsize - offs, 0);

	/* Write some OOB bytes */
	nfc_write(chip, NFC_SECTOR_SIZE, mtd->oobsize - offs);

	/* Set ECC mode to BYPASS, one virtual page */
	nfc_write(chip, NFC_FLASH_CONFIG,
		  prv->cfgbase
		  | (0 << CONFIG_STOP_ON_WERR_SHIFT)
		  | (0 << CONFIG_ECC_SRAM_ADDR_SHIFT)
		  | (0 << CONFIG_ECC_SRAM_REQ_SHIFT)
		  | (0 << CONFIG_DMA_REQ_SHIFT)
		  | (ECC_BYPASS << CONFIG_ECC_MODE_SHIFT)
		  | (0 << CONFIG_FAST_FLASH_SHIFT)
		  | (0 << CONFIG_ID_COUNT_SHIFT)
		  | (0 << CONFIG_BOOT_MODE_SHIFT)
		  | (0 << CONFIG_ADDR_AUTO_INCR_SHIFT)
		  | (0 << CONFIG_BUFNO_AUTO_INCR_SHIFT)
		  | (1 << CONFIG_PAGE_CNT_SHIFT));

	/* Now actually trigger the programming by sending NAND_CMD_PAGEPROG */
	chip->cmdfunc(mtd, NAND_CMD_PAGEPROG, -1, -1);

	/* Read programming status */
	if (chip->waitfunc(mtd, chip) & NAND_STATUS_FAIL)
		return -EIO;

	return 0;
}


/* NFC interrupt handler */
static irqreturn_t fus_nfc_irq(int irq, void *data)
{
	struct mtd_info *mtd = data;
	struct nand_chip *chip = mtd->priv;
	struct vf610_nfc_fus_prv *prv = chip->priv;

	/* Clear IDLE and DONE status bits by setting the clear bits, disable
	   interrupt */
	nfc_write(chip, NFC_IRQ_STATUS,
		  (0 << IDLE_EN_SHIFT)
		  | (1 << CMD_DONE_CLEAR_SHIFT)
		  | (1 << IDLE_CLEAR_SHIFT));

	prv->irq_handled = 1;

	wake_up(&prv->irq_waitq);

	return IRQ_HANDLED;
}


/* Wait for operation complete */
static void nfc_wait_ready(struct mtd_info *mtd, unsigned long timeout)
{
	struct nand_chip *chip = mtd->priv;
	struct vf610_nfc_fus_prv *prv = chip->priv;
	int rv;
	int repeat = 200;

	/* Remark: We should trigger the LED event for NAND here when entering
	   and leaving the wait function, but we can not access the
	   nand_led_trigger here that is statically defined in nand_base.c */

	/* Try a few times with busy wait. */
	do {
		if (nfc_read(chip, NFC_IRQ_STATUS) & IDLE_IRQ_MASK) {
			nfc_write(chip, NFC_IRQ_STATUS,
				  (1 << CMD_DONE_CLEAR_SHIFT)
				  | (1 << IDLE_CLEAR_SHIFT));
			break;
		}
		touch_softlockup_watchdog();
	} while (--repeat);
	if (!repeat) {
		/* Busy wait did not succeed yet, enable and wait for the
		   interrupt. */
		nfc_write(chip, NFC_IRQ_STATUS, 1 << IDLE_EN_SHIFT);
		rv = wait_event_timeout(prv->irq_waitq,
					prv->irq_handled,
					timeout);
		if (!rv && !prv->irq_handled)
			dev_warn(prv->dev, "timeout waiting for Ready\n");
	}
}


/* Count the number of zero bits in a value */
static int count_zeroes(u32 value)
{
	int count = 0;

	/*
	 * In each cycle, the following loop will flip the least significant
	 * 0-bit to a 1-bit. So we only need as many loop cycles as we have
	 * 0-bits in the value.
	 *
	 * Example for one cycle:
	 *
	 *   value:     1111 1111 1111 1111 0100 1011 1111 1111
	 *   value + 1: 1111 1111 1111 1111 0100 1100 0000 0000
	 *   --------------------------------------------------
	 *   result:    1111 1111 1111 1111 0100 1111 1111 1111
	 */
	while (value != 0xFFFFFFFF) {
		value |= value + 1;
		count++;
	}

	return count;
}


/* -------------------- INTERFACE FUNCTIONS -------------------------------- */

/* Control chip select signal on the board */
static void fus_nfc_select_chip(struct mtd_info *mtd, int chipnr)
{
	struct nand_chip *chip = mtd->priv;
	u32 rowreg;

	rowreg = nfc_read(chip, NFC_ROW_ADDR);
	rowreg &= ~ROW_ADDR_CHIP_SEL_MASK;
	if (chipnr >= 0)
		rowreg |= (1 << chipnr) << ROW_ADDR_CHIP_SEL_SHIFT;
	rowreg |= 1 << ROW_ADDR_CHIP_SEL_RB_SHIFT;
	nfc_write(chip, NFC_ROW_ADDR, rowreg);
}


/* Check if command is done (in nand_wait_ready()) */
static int fus_nfc_dev_ready(struct mtd_info *mtd)
{
	/* We have called nfc_wait_ready() before, so we can be sure that our
	   command is already completed. So just return "done". */
	return 1;
}


/* Read byte from NFC buffers */
static uint8_t fus_nfc_read_byte(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	struct vf610_nfc_fus_prv *prv = chip->priv;
	u32 val;
	uint col = prv->column++;

	if (prv->last_command == NAND_CMD_STATUS)
		val = nfc_read(chip, NFC_FLASH_STATUS2) & STATUS_BYTE1_MASK;
	else if (prv->last_command == NAND_CMD_READID) {
		val = nfc_read(chip,
			       col < 4 ? NFC_FLASH_STATUS1 : NFC_FLASH_STATUS2);
		val >>= ((col & 3) ^ 3) << 3;
	} else
		val = nfc_read_ram(chip, col);

	return (uint8_t)val;
}


/* Read word from NFC buffers */
static u16 fus_nfc_read_word(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;

	return chip->read_byte(mtd) | (chip->read_byte(mtd) << 8);
}


/* Read data from NFC buffers */
static void fus_nfc_read_buf(struct mtd_info *mtd, u_char *buf, int len)
{
	struct nand_chip *chip = mtd->priv;
	struct vf610_nfc_fus_prv *prv = chip->priv;

	/* Never call this function for the main area of a page, as the main
	   area should not be swapped like nfc_copy_from_nfc() does. */
	nfc_copy_from_nfc(chip, buf, len, prv->column);
	prv->column += len;
}


/* Write data to NFC buffers */
static void fus_nfc_write_buf(struct mtd_info *mtd, const u_char *buf, int len)
{
	struct nand_chip *chip = mtd->priv;

	/* Never call this function for the main area of a page, as the main
	   area should not be swapped like nfc_copy_to_nfc() does. */
	nfc_copy_to_nfc(chip, buf, len, 0);
}


/* Wait until page is written or block is erased, return status */
static int fus_nfc_waitfunc(struct mtd_info *mtd, struct nand_chip *chip)
{
	unsigned long timeout;

	if (chip->state == FL_ERASING)
		timeout = NFC_TIMEOUT_ERASE;
	else
		timeout = NFC_TIMEOUT_WRITE;

	nfc_wait_ready(mtd, timeout);

	return nfc_read(chip, NFC_FLASH_STATUS2) & STATUS_BYTE1_MASK;
}


/*
 * Write command to NAND flash. As the Vybrid NFC combines all steps in one
 * compound command, some of these commands are only dummies and the main task
 * is done at the second command byte or in the read/write functions.
 */
static void fus_nfc_command(struct mtd_info *mtd, uint command, int column,
			    int page)
{
	struct nand_chip *chip = mtd->priv;
	struct vf610_nfc_fus_prv *prv = chip->priv;

	/* Restore column for fus_nfc_read_byte() to 0 on each new command */
	prv->column = 0;

	/*
	 * Some commands just store column and/or row for the address cycle.
	 * As the Vybrid NFC does all in one go, the real action takes place
	 * when the second command byte is sent:
	 *   NAND_CMD_READ0 -> NAND_CMD_READSTART
	 *   NAND_CMD_SEQIN -> NAND_CMD_RNDIN or NAND_CMD_PAGEPROG
	 *   NAND_CMD_RNDIN -> NAND_CMD_RNDIN or NAND_CMD_PAGEPROG
	 *   NAND_CMD_ERASE1 -> NAND_CMD_ERASE2
	 *   NAND_CMD_RNDOUT -> NAND_CMD_RNDOUTSTART
	 */
	switch (command) {
	case NAND_CMD_RNDIN:
		/* NAND_CMD_RNDIN must move all the data since the last
		   NAND_CMD_SEQIN or NAND_CMD_RNDIN to the flash. */
		if (prv->last_command == NAND_CMD_SEQIN) {
			/* Write the data since NAND_CMD_SEQIN to the flash */
			nfc_send_cmd_1(chip, NAND_CMD_SEQIN,
				       SEQIN_RANDOM_IN_CMD_CODE);
		} else if (prv->last_command == NAND_CMD_RNDIN) {
			/* Write the data since NAND_CMD_RNDIN to the flash */
			nfc_send_cmd_1(chip, NAND_CMD_RNDIN,
				       RNDIN_RANDOM_IN_CMD_CODE);
		} else {
			dev_err(prv->dev, "no NAND_CMD_SEQIN or NAND_CMD_RNDIN "
			       "before NAND_CMD_RNDIN, ignored\n");
			return;
		}

		/* Wait for command completion */
		nfc_wait_ready(mtd, NFC_TIMEOUT_DATA);
		/* Fall through to case NAND_CMD_RNDOUT */

	case NAND_CMD_RNDOUT:
		nfc_set_column(chip, column, command);
		goto done;

	case NAND_CMD_READ0:
	case NAND_CMD_SEQIN:
		nfc_set_column(chip, column, command);
		/* Fall through to case NAND_CMD_ERASE1 */

	case NAND_CMD_ERASE1:
		nfc_set_row(chip, page, command);
		goto done;

	case NAND_CMD_READSTART:
		if (prv->last_command != NAND_CMD_READ0)
			dev_err(prv->dev, "no NAND_CMD_READ0 before"
				" NAND_CMD_READSTART\n");

		/* Start reading transfer */
		nfc_send_cmd_2(chip, NAND_CMD_READ0, NAND_CMD_READSTART,
			       READ_PAGE_CMD_CODE);
		break;

	case NAND_CMD_RNDOUTSTART:
		if (prv->last_command != NAND_CMD_RNDOUT)
			dev_err(prv->dev, "no NAND_CMD_RNDOUT before"
				" NAND_CMD_RNDOUTSTART\n");
		/* Actually do the reading after NAND_CMD_RNDOUTSTART */
		nfc_send_cmd_2(chip, NAND_CMD_RNDOUT, NAND_CMD_RNDOUTSTART,
			       RANDOM_OUT_CMD_CODE);
		break;

	case NAND_CMD_PAGEPROG:
		/* Actually do the programming. This is in fact a 3-byte
		   command with NAND_CMD_STATUS as last command. */ 
		if (prv->last_command == NAND_CMD_SEQIN) {
			/* Write the data since NAND_CMD_SEQIN to the
			   flash and start programming. */
			nfc_send_cmd_2(chip, NAND_CMD_SEQIN, NAND_CMD_PAGEPROG,
				       SEQIN_PROGRAM_PAGE_CMD_CODE);
		} else if (prv->last_command == NAND_CMD_RNDIN) {
			/* Write the data since the last NAND_CMD_RNDIN to the
			   flash and start programming. */
			nfc_send_cmd_2(chip, NAND_CMD_RNDIN, NAND_CMD_PAGEPROG,
				       RNDIN_PROGRAM_PAGE_CMD_CODE);
		} else {
			dev_err(prv->dev, "no NAND_CMD_SEQIN or NAND_CMD_RNDIN "
			       "before NAND_CMD_PAGEPROG, ignored\n");
			return;
		}
		/* chip->waitfunc() is called later, so we need not wait now */
		goto done;

	case NAND_CMD_ERASE2:
		/* Actually do the erasing after a NAND_CMD_ERASE1 */
		if (prv->last_command != NAND_CMD_ERASE1) {
			dev_err(prv->dev, "no NAND_CMD_ERASE1 before"
				" NAND_CMD_ERASE2, ignored\n");
			return;
		}

		/* Start erasing. This is in fact a 3-byte command with
		   NAND_CMD_STATUS as last command. */
		nfc_send_cmd_2(chip, NAND_CMD_ERASE1, NAND_CMD_ERASE2,
			       ERASESTAT_CMD_CODE);
		/* chip->waitfunc() is called later, so we need not wait now */
		goto done;

	case NAND_CMD_READOOB:
		/* Emulate with NAND_CMD_READ0 */
		if (column < 0) {
			dev_err(prv->dev, "missing column for NAND_CMD_READOOB\n");
			return;
		}
		chip->cmdfunc(mtd, NAND_CMD_READ0, column + mtd->writesize,
			      page);

		/* Read some OOB bytes */
		nfc_write(chip, NFC_SECTOR_SIZE, mtd->oobsize - column);

		/* Set ECC mode to BYPASS, one virtual page */
		nfc_write(chip, NFC_FLASH_CONFIG,
			  prv->cfgbase
			  | (0 << CONFIG_STOP_ON_WERR_SHIFT)
			  | (0 << CONFIG_ECC_SRAM_ADDR_SHIFT)
			  | (0 << CONFIG_ECC_SRAM_REQ_SHIFT)
			  | (0 << CONFIG_DMA_REQ_SHIFT)
			  | (ECC_BYPASS << CONFIG_ECC_MODE_SHIFT)
			  | (0 << CONFIG_FAST_FLASH_SHIFT)
			  | (0 << CONFIG_ID_COUNT_SHIFT)
			  | (0 << CONFIG_BOOT_MODE_SHIFT)
			  | (0 << CONFIG_ADDR_AUTO_INCR_SHIFT)
			  | (0 << CONFIG_BUFNO_AUTO_INCR_SHIFT)
			  | (1 << CONFIG_PAGE_CNT_SHIFT));

		/* Now actually trigger reading by sending NAND_CMD_READSTART;
		   wait for completion */
		chip->cmdfunc(mtd, NAND_CMD_READSTART, -1, -1);

		/* Copy OOB data from NFC RAM; as we have only read these few
		   bytes and no main data, they are at offset 0 in NFC RAM */
		nfc_copy_from_nfc(chip, chip->oob_poi + column,
				  mtd->oobsize - column, 0);
		return;

	case NAND_CMD_READID:
		/* Set ECC mode to BYPASS, 5 ID bytes, one virtual page */
		nfc_set_column(chip, column, command);
		nfc_write(chip, NFC_FLASH_CONFIG,
			  prv->cfgbase
			  | (0 << CONFIG_STOP_ON_WERR_SHIFT)
			  | (0 << CONFIG_ECC_SRAM_ADDR_SHIFT)
			  | (0 << CONFIG_ECC_SRAM_REQ_SHIFT)
			  | (0 << CONFIG_DMA_REQ_SHIFT)
			  | (ECC_BYPASS << CONFIG_ECC_MODE_SHIFT)
			  | (0 << CONFIG_FAST_FLASH_SHIFT)
			  | (5 << CONFIG_ID_COUNT_SHIFT)
			  | (0 << CONFIG_BOOT_MODE_SHIFT)
			  | (0 << CONFIG_ADDR_AUTO_INCR_SHIFT)
			  | (0 << CONFIG_BUFNO_AUTO_INCR_SHIFT)
			  | (1 << CONFIG_PAGE_CNT_SHIFT));

		/* Start ID request */
		nfc_send_cmd_1(chip, NAND_CMD_READID, READ_ID_CMD_CODE);
		break;

	case NAND_CMD_STATUS:
		/* Set ECC mode to BYPASS, one virtual page */
		nfc_write(chip, NFC_FLASH_CONFIG,
			  prv->cfgbase
			  | (0 << CONFIG_STOP_ON_WERR_SHIFT)
			  | (0 << CONFIG_ECC_SRAM_ADDR_SHIFT)
			  | (0 << CONFIG_ECC_SRAM_REQ_SHIFT)
			  | (0 << CONFIG_DMA_REQ_SHIFT)
			  | (ECC_BYPASS << CONFIG_ECC_MODE_SHIFT)
			  | (0 << CONFIG_FAST_FLASH_SHIFT)
			  | (0 << CONFIG_ID_COUNT_SHIFT)
			  | (0 << CONFIG_BOOT_MODE_SHIFT)
			  | (0 << CONFIG_ADDR_AUTO_INCR_SHIFT)
			  | (0 << CONFIG_BUFNO_AUTO_INCR_SHIFT)
			  | (1 << CONFIG_PAGE_CNT_SHIFT));

		/* Start status request */
		nfc_send_cmd_1(chip, NAND_CMD_STATUS, STATUS_READ_CMD_CODE);
		break;

	case NAND_CMD_RESET:
		/* Set ECC mode to BYPASS, one virtual page */
		nfc_write(chip, NFC_FLASH_CONFIG,
			  prv->cfgbase
			  | (0 << CONFIG_STOP_ON_WERR_SHIFT)
			  | (0 << CONFIG_ECC_SRAM_ADDR_SHIFT)
			  | (0 << CONFIG_ECC_SRAM_REQ_SHIFT)
			  | (0 << CONFIG_DMA_REQ_SHIFT)
			  | (ECC_BYPASS << CONFIG_ECC_MODE_SHIFT)
			  | (0 << CONFIG_FAST_FLASH_SHIFT)
			  | (0 << CONFIG_ID_COUNT_SHIFT)
			  | (0 << CONFIG_BOOT_MODE_SHIFT)
			  | (0 << CONFIG_ADDR_AUTO_INCR_SHIFT)
			  | (0 << CONFIG_BUFNO_AUTO_INCR_SHIFT)
			  | (1 << CONFIG_PAGE_CNT_SHIFT));

		/* Send reset */
		nfc_send_cmd_1(chip, NAND_CMD_RESET, RESET_CMD_CODE);
		break;

	case NAND_CMD_PARAM:
		/* Read ONFI parameter pages (3 copies) */
		nfc_set_column(chip, column, command);
		nfc_write(chip, NFC_SECTOR_SIZE, 3 * 256);

		/* Set ECC mode to BYPASS, one virtual page */
		nfc_write(chip, NFC_FLASH_CONFIG,
			  prv->cfgbase
			  | (0 << CONFIG_STOP_ON_WERR_SHIFT)
			  | (0 << CONFIG_ECC_SRAM_ADDR_SHIFT)
			  | (0 << CONFIG_ECC_SRAM_REQ_SHIFT)
			  | (0 << CONFIG_DMA_REQ_SHIFT)
			  | (ECC_BYPASS << CONFIG_ECC_MODE_SHIFT)
			  | (0 << CONFIG_FAST_FLASH_SHIFT)
			  | (0 << CONFIG_ID_COUNT_SHIFT)
			  | (0 << CONFIG_BOOT_MODE_SHIFT)
			  | (0 << CONFIG_ADDR_AUTO_INCR_SHIFT)
			  | (0 << CONFIG_BUFNO_AUTO_INCR_SHIFT)
			  | (1 << CONFIG_PAGE_CNT_SHIFT));

		/* Start ID request */
		nfc_send_cmd_1(chip, NAND_CMD_PARAM, READ_PARAM_CMD_CODE);
		break;

	default:
		dev_err(prv->dev, "NAND_CMD 0x%02x not supported\n", command);
		return;
	}

	/* Wait for command completion */
	nfc_wait_ready(mtd, NFC_TIMEOUT_DATA);

done:
	prv->last_command = command;
}

static int fus_nfc_read_oob_raw(struct mtd_info *mtd, struct nand_chip *chip,
				int page)
{
	/* Send command to read oob data from flash */
	chip->cmdfunc(mtd, NAND_CMD_READOOB, 0, page);

	return 0;
}


/* Read OOB data from given offset */
static int fus_nfc_read_oob(struct mtd_info *mtd, struct nand_chip *chip,
			    int page)
{
	/* Send command to read oob data from flash */
	chip->cmdfunc(mtd, NAND_CMD_READOOB,
		      mtd->oobsize - mtd->oobavail, page);

	return 0;
}


/* Write OOB data at given offset */
static int fus_nfc_write_oob_raw(struct mtd_info *mtd, struct nand_chip *chip,
				 int page)
{
	return nfc_do_write_oob(mtd, chip, page, 0);
}


/* Write OOB data at given offset */
static int fus_nfc_write_oob(struct mtd_info *mtd, struct nand_chip *chip,
			     int page)
{
	return nfc_do_write_oob(mtd, chip, page, mtd->oobsize - mtd->oobavail);
}


/* Read the whole main and the whole OOB area in one go (without ECC) */
static int fus_nfc_read_page_raw(struct mtd_info *mtd, struct nand_chip *chip,
				 uint8_t *buf, int oob_required, int page)
{
	struct vf610_nfc_fus_prv *prv = chip->priv;
	uint size = mtd->writesize;

	/* This code assumes that the NAND_CMD_READ0 command was
	   already issued before */

	/* Set number of bytes to transfer */
	if (oob_required)
		size += mtd->oobsize;
	nfc_write(chip, NFC_SECTOR_SIZE, size);

	/* Set ECC mode to BYPASS, position of ECC status, one virtual page */
	nfc_write(chip, NFC_FLASH_CONFIG,
		  prv->cfgbase
		  | (0 << CONFIG_STOP_ON_WERR_SHIFT)
		  | (0 << CONFIG_ECC_SRAM_ADDR_SHIFT)
		  | (0 << CONFIG_ECC_SRAM_REQ_SHIFT)
		  | (0 << CONFIG_DMA_REQ_SHIFT)
		  | (ECC_BYPASS << CONFIG_ECC_MODE_SHIFT)
		  | (0 << CONFIG_FAST_FLASH_SHIFT)
		  | (0 << CONFIG_ID_COUNT_SHIFT)
		  | (0 << CONFIG_BOOT_MODE_SHIFT)
		  | (0 << CONFIG_ADDR_AUTO_INCR_SHIFT)
		  | (0 << CONFIG_BUFNO_AUTO_INCR_SHIFT)
		  | (1 << CONFIG_PAGE_CNT_SHIFT));

	/* Actually trigger read transfer and wait for completion */
	chip->cmdfunc(mtd, NAND_CMD_READSTART, -1, -1);

	/*
	 * Copy main and OOB data from NFC RAM. Please note that we don't swap
	 * the main data from Big Endian byte order. DMA does not swap data
	 * either, so if we want to use DMA in the future, we have to keep it
	 * this way. See also fus_nfc_write_page_raw(). However we do swap
	 * the OOB data.
	 */
	memcpy_fromio(buf, chip->IO_ADDR_R + NFC_MAIN_AREA(0), mtd->writesize);
	if (oob_required) {
		memset(chip->oob_poi, 0xFF, mtd->oobsize);
		nfc_copy_from_nfc(chip, chip->oob_poi, mtd->oobsize,
				  mtd->writesize);
	}

	return 0;
}


/* Write the whole main and the whole OOB area without ECC in one go */
static int fus_nfc_write_page_raw(struct mtd_info *mtd, struct nand_chip *chip,
				const uint8_t *buf, int oob_required, int page)
{
	struct vf610_nfc_fus_prv *prv = chip->priv;
	uint size = mtd->writesize;

	/* NAND_CMD_SEQIN for column 0 was already issued by the caller */

	/*
	 * Copy main and OOB data to NFC RAM. Please note that we don't swap
	 * the main data to Big Endian byte order. DMA does not swap the data
	 * either, so if we want to use DMA in the future, we have to keep it
	 * this way.
	 */
	memcpy_toio(chip->IO_ADDR_R + NFC_MAIN_AREA(0), buf, size);
	if (oob_required) {
		nfc_copy_to_nfc(chip, chip->oob_poi, mtd->oobsize, size);
		size += mtd->oobsize;
	}

	/* Set number of bytes to transfer */
	nfc_write(chip, NFC_SECTOR_SIZE, size);

	/* Set ECC mode to BYPASS, one virtual page */
	nfc_write(chip, NFC_FLASH_CONFIG,
		  prv->cfgbase
		  | (0 << CONFIG_STOP_ON_WERR_SHIFT)
		  | (0 << CONFIG_ECC_SRAM_ADDR_SHIFT)
		  | (0 << CONFIG_ECC_SRAM_REQ_SHIFT)
		  | (0 << CONFIG_DMA_REQ_SHIFT)
		  | (ECC_BYPASS << CONFIG_ECC_MODE_SHIFT)
		  | (0 << CONFIG_FAST_FLASH_SHIFT)
		  | (0 << CONFIG_ID_COUNT_SHIFT)
		  | (0 << CONFIG_BOOT_MODE_SHIFT)
		  | (0 << CONFIG_ADDR_AUTO_INCR_SHIFT)
		  | (0 << CONFIG_BUFNO_AUTO_INCR_SHIFT)
		  | (1 << CONFIG_PAGE_CNT_SHIFT));

	/* The actual programming will take place in fus_nfc_command() when
	   command NAND_CMD_PAGEPROG is sent */
	return 0;
}


/* Read main data of page with ECC enabled; if there is a problem with the ECC
   correction, e.g. too many bits flipped, return the raw (uncorrected) data */
static int fus_nfc_read_page(struct mtd_info *mtd, struct nand_chip *chip,
			     uint8_t *buf, int oob_required, int page)
{
	u8 ecc_status;
	uint i;
	uint size;
	int zerobits;
	int limit;
	struct vf610_nfc_fus_prv *prv = chip->priv;

	/* This code assumes that the NAND_CMD_READ0 command was
	   already issued before */

	/* Set size to load main area, BBM and ECC */
	size = mtd->oobsize - mtd->oobavail;
	nfc_write(chip, NFC_SECTOR_SIZE, mtd->writesize + size);

	/* Set ECC mode, position of ECC status, one virtual page */
	nfc_write(chip, NFC_FLASH_CONFIG,
		  prv->cfgbase
		  | (0 << CONFIG_STOP_ON_WERR_SHIFT)
		  | ((ECC_STATUS_OFFS >> 3) << CONFIG_ECC_SRAM_ADDR_SHIFT)
		  | (1 << CONFIG_ECC_SRAM_REQ_SHIFT)
		  | (0 << CONFIG_DMA_REQ_SHIFT)
		  | (prv->eccmode << CONFIG_ECC_MODE_SHIFT)
		  | (0 << CONFIG_FAST_FLASH_SHIFT)
		  | (0 << CONFIG_ID_COUNT_SHIFT)
		  | (0 << CONFIG_BOOT_MODE_SHIFT)
		  | (0 << CONFIG_ADDR_AUTO_INCR_SHIFT)
		  | (0 << CONFIG_BUFNO_AUTO_INCR_SHIFT)
		  | (1 << CONFIG_PAGE_CNT_SHIFT));

	/* Actually trigger read transfer and wait for completion */
	chip->cmdfunc(mtd, NAND_CMD_READSTART, -1, -1);

	/* Get the ECC status */
	ecc_status = nfc_read_ram(chip, ECC_STATUS_OFFS + 7);
	if (!(ecc_status & ECC_STATUS_MASK)) {
		int bitflips = ecc_status & ECC_ERR_COUNT;

		/* Correctable error or no error at all: update ecc_stats */
		mtd->ecc_stats.corrected += bitflips;

		/*
		 * Copy main data from NFC RAM. Please note that we don't swap
		 * the data from Big Endian byte order. DMA does not swap data
		 * either, so if we want to use DMA in the future, we have to
		 * keep it this way. See also fus_nfc_write_page().
		 */
		memcpy_fromio(buf, chip->IO_ADDR_R + NFC_MAIN_AREA(0),
			      mtd->writesize);

		// ### TODO: Read Refresh-Number

		if (oob_required) {
			memset(chip->oob_poi, 0xFF, mtd->oobsize);

			/*
			 * Reload the user part of the OOB area to NFC RAM. We
			 * can use NAND_CMD_RNDOUT as the NAND flash still has
			 * the data in its page cache.
			 */
			chip->cmdfunc(mtd, NAND_CMD_RNDOUT,
				      mtd->writesize + size, -1);

			/* Set size to load user OOB area */
			nfc_write(chip, NFC_SECTOR_SIZE, mtd->oobavail);

			/* Set ECC mode to BYPASS, one virtual page */
			nfc_write(chip, NFC_FLASH_CONFIG,
				  prv->cfgbase
				  | (0 << CONFIG_STOP_ON_WERR_SHIFT)
				  | (0 << CONFIG_ECC_SRAM_ADDR_SHIFT)
				  | (0 << CONFIG_ECC_SRAM_REQ_SHIFT)
				  | (0 << CONFIG_DMA_REQ_SHIFT)
				  | (ECC_BYPASS << CONFIG_ECC_MODE_SHIFT)
				  | (0 << CONFIG_FAST_FLASH_SHIFT)
				  | (0 << CONFIG_ID_COUNT_SHIFT)
				  | (0 << CONFIG_BOOT_MODE_SHIFT)
				  | (0 << CONFIG_ADDR_AUTO_INCR_SHIFT)
				  | (0 << CONFIG_BUFNO_AUTO_INCR_SHIFT)
				  | (1 << CONFIG_PAGE_CNT_SHIFT));

			/* Actually trigger reading and wait until done */
			chip->cmdfunc(mtd, NAND_CMD_RNDOUTSTART, -1, -1);

			/* Copy data to OOB buffer; here we do swap the bytes;
			   the data is at the beginning of the NFC RAM */
			nfc_copy_from_nfc(chip, chip->oob_poi + size,
					  mtd->oobavail, 0);
		}

		return bitflips;
	}

	/*
	 * The page is uncorrectable; however this can also happen with a
	 * totally empty (=erased) page, in which case we should return OK.
	 * But this means that we must reload the page in raw mode. However we
	 * can use NAND_CMD_RNDOUT as the NAND flash still has the data in its
	 * page cache.
	 */
	chip->cmdfunc(mtd, NAND_CMD_RNDOUT, 0, -1);

	/* Set size to load main and OOB area in one go */
	nfc_write(chip, NFC_SECTOR_SIZE, mtd->writesize + mtd->oobsize);

	/* Set ECC mode to BYPASS, one virtual page */
	nfc_write(chip, NFC_FLASH_CONFIG,
		  prv->cfgbase
		  | (0 << CONFIG_STOP_ON_WERR_SHIFT)
		  | (0 << CONFIG_ECC_SRAM_ADDR_SHIFT)
		  | (0 << CONFIG_ECC_SRAM_REQ_SHIFT)
		  | (0 << CONFIG_DMA_REQ_SHIFT)
		  | (ECC_BYPASS << CONFIG_ECC_MODE_SHIFT)
		  | (0 << CONFIG_FAST_FLASH_SHIFT)
		  | (0 << CONFIG_ID_COUNT_SHIFT)
		  | (0 << CONFIG_BOOT_MODE_SHIFT)
		  | (0 << CONFIG_ADDR_AUTO_INCR_SHIFT)
		  | (0 << CONFIG_BUFNO_AUTO_INCR_SHIFT)
		  | (1 << CONFIG_PAGE_CNT_SHIFT));

	/* Actually trigger reading and wait until done */
	chip->cmdfunc(mtd, NAND_CMD_RNDOUTSTART, -1, -1);

	/* If requested, copy the user part of the OOB to the internal OOB
	   buffer. This may or may not be empty. Here we do swap the bytes. */
	if (oob_required)
		nfc_copy_from_nfc(chip, chip->oob_poi + size, mtd->oobavail,
				  mtd->writesize + size);
	/*
	 * Check the main part up to and including the ECC for empty.
	 * Because also an empty page may show bitflips (e.g. by write
	 * disturbs caused by writes to a nearby page), we accept up to
	 * bitflip_threshold non-empty bits.
	 *
	 * Remark:
	 * The last up to three bytes may be unaligned, so we read them as
	 * bytes with byte swappping. But for everything before that do 4-byte
	 * compares to be faster; byte order does not matter when comparing to
	 * 0xFFFFFFFF or when counting zero bits.
	 */
	zerobits = 0;
	limit = (int)mtd->bitflip_threshold;
	i = mtd->writesize + size;
	do {
		u32 data;

		if (i & 3)
			data = nfc_read_ram(chip, --i) | 0xFFFFFF00;
		else {
			i -= 4;
			data = nfc_read(chip, NFC_MAIN_AREA(0) + i);
		}
		zerobits += count_zeroes(data);
		if (zerobits > limit)
			break;
	} while (i);

	/* If this is an empty page, "correct" any bitflips by returning 0xFF,
	   not the actually read data */
	if (zerobits <= limit) {
		memset(buf, 0xFF, mtd->writesize);
		memset(chip->oob_poi, 0xFF, size);

		mtd->ecc_stats.corrected += zerobits;

		return zerobits;
	}

	/* The page is not empty, it is a real read error. Update ecc_stats
	   and return the raw data. Do byte swaps on ECC data only. */
	memcpy(buf, chip->IO_ADDR_R + NFC_MAIN_AREA(0), mtd->writesize);
	nfc_copy_from_nfc(chip, chip->oob_poi, size, mtd->writesize);

	mtd->ecc_stats.failed++;

	return 0;
}


/* Write main data of page with ECC enabled */
static int fus_nfc_write_page(struct mtd_info *mtd, struct nand_chip *chip,
			      const uint8_t *buf, int oob_required, int page)
{
	struct vf610_nfc_fus_prv *prv = chip->priv;
	uint size = mtd->oobsize - mtd->oobavail;
	uint i;
	uint8_t *oob;

	/* NAND_CMD_SEQIN for column 0 was already issued by the caller */

	/*
	 * Copy main data to NFC RAM. Please note that we don't swap the data
	 * to Big Endian byte order. DMA does not swap data either, so if we
	 * want to use DMA in the future, we have to keep it this way. Also
	 * don't forget to write 0xFFFFFFFF to the BBM area.
	 */
	memcpy_toio(chip->IO_ADDR_R + NFC_MAIN_AREA(0), buf, mtd->writesize);
	nfc_write(chip, NFC_MAIN_AREA(0) + mtd->writesize, 0xFFFFFFFF);

	// ### TODO: Write Refresh-Number

	/* Set number of bytes to transfer */
	nfc_write(chip, NFC_SECTOR_SIZE, size + mtd->writesize);

	/* Set ECC mode, position of ECC status, one virtual page */
	nfc_write(chip, NFC_FLASH_CONFIG,
		  prv->cfgbase
		  | (0 << CONFIG_STOP_ON_WERR_SHIFT)
		  | ((ECC_STATUS_OFFS >> 3) << CONFIG_ECC_SRAM_ADDR_SHIFT)
		  | (1 << CONFIG_ECC_SRAM_REQ_SHIFT)
		  | (0 << CONFIG_DMA_REQ_SHIFT)
		  | (prv->eccmode << CONFIG_ECC_MODE_SHIFT)
		  | (0 << CONFIG_FAST_FLASH_SHIFT)
		  | (0 << CONFIG_ID_COUNT_SHIFT)
		  | (0 << CONFIG_BOOT_MODE_SHIFT)
		  | (0 << CONFIG_ADDR_AUTO_INCR_SHIFT)
		  | (0 << CONFIG_BUFNO_AUTO_INCR_SHIFT)
		  | (1 << CONFIG_PAGE_CNT_SHIFT));

	if (!oob_required)
		return 0;

	/* Check if user OOB area is empty */
	oob = chip->oob_poi + size;
	for (i = mtd->oobavail; i > 0; i--) {
		if (oob[i - 1] != 0xFF)
			break;
	}
	if (i) {
		/* No: transfer main data, BBM and ECC to NAND chip, then move
		   to user OOB column */
		chip->cmdfunc(mtd, NAND_CMD_RNDIN, size + mtd->writesize, -1);

		/* Copy the user part of the OOB to NFC RAM */
		nfc_copy_to_nfc(chip, oob, mtd->oobavail, 0);

		/* Set number of bytes to transfer */
		nfc_write(chip, NFC_SECTOR_SIZE, mtd->oobavail);

		/* Set ECC mode to BYPASS, one virtual page */
		nfc_write(chip, NFC_FLASH_CONFIG,
			  prv->cfgbase
			  | (0 << CONFIG_STOP_ON_WERR_SHIFT)
			  | (0 << CONFIG_ECC_SRAM_ADDR_SHIFT)
			  | (0 << CONFIG_ECC_SRAM_REQ_SHIFT)
			  | (0 << CONFIG_DMA_REQ_SHIFT)
			  | (ECC_BYPASS << CONFIG_ECC_MODE_SHIFT)
			  | (0 << CONFIG_FAST_FLASH_SHIFT)
			  | (0 << CONFIG_ID_COUNT_SHIFT)
			  | (0 << CONFIG_BOOT_MODE_SHIFT)
			  | (0 << CONFIG_ADDR_AUTO_INCR_SHIFT)
			  | (0 << CONFIG_BUFNO_AUTO_INCR_SHIFT)
			  | (1 << CONFIG_PAGE_CNT_SHIFT));
	}

	/* The actual programming will take place in fus_nfc_command() when
	   command NAND_CMD_PAGEPROG is sent */
	return 0;
}

/* Free driver resources */
static void fus_nfc_free(struct platform_device *dev, struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	struct vf610_nfc_fus_prv *prv = chip->priv;

	clk_disable_unprepare(prv->clk);
}

static int fus_nfc_ooblayout_ecc(struct mtd_info *mtd, int section,
				 struct mtd_oob_region *oobregion)
{
	struct nand_chip *chip = mtd_to_nand(mtd);

	if (section)
		return -ERANGE;

	oobregion->offset = 4;
	oobregion->length = chip->ecc.bytes;

	return 0;
}

static int fus_nfc_ooblayout_free(struct mtd_info *mtd, int section,
				  struct mtd_oob_region *oobregion)
{
	if (section)
		return -ERANGE;

	/* The available oob size we have. */
	oobregion->offset = mtd->oobsize - mtd->oobavail;
	oobregion->length = mtd->oobavail;

	return 0;
}

static const struct mtd_ooblayout_ops fus_nfc_ooblayout_ops = {
	.ecc = fus_nfc_ooblayout_ecc,
	.free = fus_nfc_ooblayout_free,
};

static const struct platform_device_id fus_nfc_ids[] = {
	{ .name = "vf610_nfc_fus", .driver_data = 0, },
	{}
};

#ifdef CONFIG_OF
static int fus_nfc_parse_dt(struct device *dev,
			    struct vf610_nfc_fus_platform_data *pdata)
{
	struct device_node *node = dev->of_node;

	if (!node)
		return -ENXIO;

	if (of_property_read_u32(node, "fus,skip_blocks", &pdata->skipblocks))
		pdata->skipblocks = 0;
	if (of_property_read_bool(node, "fus,skip_inverse"))
		pdata->flags |= VYBRID_NFC_SKIP_INVERSE;
	if (of_property_read_u32(node, "fus,ecc_mode", &pdata->eccmode))
		pdata->eccmode = VYBRID_NFC_ECCMODE_16BIT;

	return 0;
}

static const struct of_device_id fus_nfc_id_table[] = {
	{
		.compatible = "fus,vf610-nfc",
	}, { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, fus_nfc_id_table);
#else
static int fus_nfc_parse_dt(struct device *dev,
			    struct vf610_nfc_fus_platform_data *pdata)
{
	return -ENODEV;
}
#endif /* CONFIG_OF */

static int fus_nfc_probe(struct platform_device *pdev)
{
	struct vf610_nfc_fus_prv *prv;
	struct resource *res;
	struct mtd_info *mtd;
	struct nand_chip *chip;
	struct vf610_nfc_fus_platform_data *pdata = pdev->dev.platform_data;
	struct vf610_nfc_fus_platform_data data;
//###	const struct of_device_id *of_id;
	int ret;
	int irq;
	u32 oobavail;
	struct mtd_part_parser_data ppdata = {};

	dev_info(&pdev->dev, "F&S VF610 NFC MTD NAND driver %s\n", DRV_VERSION);
	prv = devm_kzalloc(&pdev->dev, sizeof(*prv), GFP_KERNEL);
	if (!prv) {
		dev_err(&pdev->dev, "Failed to allocate per-device memory\n");
		return -ENOMEM;
	}

#if 0 //###
	of_id = of_match_device(fus_nfc_id_table, &pdev->dev);
	if (!of_id) {
		this->devdata = of_id->data;
	} else {
		dev_err(&pdev->dev, "Failed to find the right device id\n");
		return -ENODEV;
	}
#endif

	if (!pdata) {
		memset(&data, 0, sizeof(data));
		ret = fus_nfc_parse_dt(&pdev->dev, &data);
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to find platform data\n");
			return ret;
		}

		pdata = &data;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq <= 0) {
		dev_err(&pdev->dev, "error getting IRQ!\n");
		return -EINVAL;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	prv->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(prv->regs)) {
		dev_err(&pdev->dev, "Can't map NFC register area\n");
		return PTR_ERR(prv->regs);
	}

	/* Init the mtd device, most of it is done in nand_scan_ident() */
	platform_set_drvdata(pdev, prv);

	prv->dev = &pdev->dev;

	/* Setup all things in chip that are required to detect the chip */
	chip = &prv->chip;
	chip->priv = prv;
	chip->IO_ADDR_R = prv->regs;
	chip->IO_ADDR_W = chip->IO_ADDR_R;
	chip->select_chip = fus_nfc_select_chip;
	chip->dev_ready = fus_nfc_dev_ready;
	chip->cmdfunc = fus_nfc_command;
	chip->read_byte = fus_nfc_read_byte;
	chip->read_word = fus_nfc_read_word;
	chip->read_buf = fus_nfc_read_buf;
	chip->write_buf = fus_nfc_write_buf;
	chip->waitfunc = fus_nfc_waitfunc;
	chip->options = pdata ? pdata->options : 0;
	chip->options |= NAND_BBT_SCAN2NDPAGE;
	chip->badblockpos = 0; //###???

	/* Basic settings of MTD data structure */
	mtd = &chip->mtd;
	mtd->priv = chip;
	mtd->name = "NAND";
	mtd->owner = THIS_MODULE;

	//### power-manager initialisieren

	/* Set up our remaining private data */
	init_waitqueue_head(&prv->irq_waitq);

	ret = devm_request_irq(&pdev->dev, irq, fus_nfc_irq, 0, mtd->name, prv);
	if (ret) {
		dev_err(&pdev->dev, "error requesting IRQ!\n");
		return ret;
	}

	prv->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(prv->clk))
		return PTR_ERR(prv->clk);

	ret = clk_prepare_enable(prv->clk);
	if (ret) {
		dev_err(&pdev->dev, "Unable to enable clock!\n");
		return ret;
	}

	prv->cfgbase =
		nfc_read(chip, NFC_FLASH_CONFIG) & CONFIG_CMD_TIMEOUT_MASK;
	prv->eccmode = pdata->eccmode;
	if (pdata->t_wb && (pdata->t_wb <= 0x1f))
		prv->cfgbase = pdata->t_wb << CONFIG_CMD_TIMEOUT_SHIFT;
	if (chip->options & NAND_BUSWIDTH_16)
		prv->cfgbase |= 1 << CONFIG_16BIT_SHIFT;

	/* Clear IDLE and DONE status bits by setting the clear bits. The
	   interrupt is enabled in nfc_wait_ready() if required. */
	nfc_write(chip, NFC_IRQ_STATUS,
		  (0 << IDLE_EN_SHIFT)
		  | (1 << CMD_DONE_CLEAR_SHIFT)
		  | (1 << IDLE_CLEAR_SHIFT));

	/* Identify the device, set page and block sizes, etc. */
	if (nand_scan_ident(mtd, 1 /*### OF->max_chips */, NULL)) {
		dev_err(&pdev->dev, "NAND Flash not found!\n");
		free_irq(prv->irq, mtd);
		ret = -ENXIO;
		goto error;
	}

	/* Set skipped region */
	mtd->skip = pdata->skipblocks * mtd->erasesize;
	if (pdata->flags & VYBRID_NFC_SKIP_INVERSE) {
		mtd->size = mtd->skip;
		mtd->skip = 0;
	}

	/* Set up ECC configuration */
	chip->ecc.read_page = fus_nfc_read_page;
	chip->ecc.write_page = fus_nfc_write_page;
	chip->ecc.read_oob = fus_nfc_read_oob;
	chip->ecc.write_oob = fus_nfc_write_oob;
	chip->ecc.read_page_raw = fus_nfc_read_page_raw;
	chip->ecc.write_page_raw = fus_nfc_write_page_raw;
	chip->ecc.read_oob_raw = fus_nfc_read_oob_raw;
	chip->ecc.write_oob_raw = fus_nfc_write_oob_raw;
	chip->ecc.mode = NAND_ECC_HW;
	chip->ecc.steps = 1;
	chip->ecc.size = mtd->writesize;
	chip->ecc.bytes = ecc_bytes[prv->eccmode];
	chip->ecc.strength = ecc_strength[prv->eccmode];
	oobavail = mtd->oobsize - chip->ecc.bytes - 4;
	mtd->oobavail = oobavail;
	mtd->bitflip_threshold = bitflip_threshold[prv->eccmode];
	mtd_set_ooblayout(mtd, &fus_nfc_ooblayout_ops);

	if (chip->onfi_version) {
		u8 addr_cycles;

		/* Get address cycles from ONFI data:
		   [7:4] column cycles, [3:0] row cycles */
		addr_cycles = chip->onfi_params.addr_cycles;
		prv->cmdclr = 0;
		if ((addr_cycles >> 4) < 2)
			prv->cmdclr |= (1 << 12); /* No column byte 2 */
		addr_cycles &= 0x0F;
		if (addr_cycles < 3)
			prv->cmdclr |= (1 << 9);  /* No row byte 3 */
		if (addr_cycles < 2)
			prv->cmdclr |= (1 << 10); /* No row byte 2 */
	} else
		/* Use only two row cycles if NAND is smaller than 32 MB */
		prv->cmdclr = (chip->chipsize > (32 << 20)) ? 0 : (1 << 9);

	if (nand_scan_tail(mtd)) {
		dev_err(&pdev->dev, "Can not init NAND Flash!\n");
		free_irq(prv->irq, mtd);
		ret = -ENXIO;
		goto error;
	}

	/* Parse MTD partition table and register MTD device */
	ret = mtd_device_parse_register(mtd, NULL, &ppdata, NULL, 0);
	if (!ret)
		return 0;

error:
	fus_nfc_free(pdev, mtd);
	return ret;
}

static int __exit fus_nfc_remove(struct platform_device *pdev)
{
	struct mtd_info *mtd = platform_get_drvdata(pdev);
	struct nand_chip *chip = mtd->priv;
	struct vf610_nfc_fus_prv *prv = chip->priv;

	nand_release(mtd);
	free_irq(prv->irq, mtd);
	fus_nfc_free(pdev, mtd);

	return 0;
}

static struct platform_driver fus_nfc_driver = {
	.driver		= {
		.name	= DRV_NAME,
//###		.of_match_table = of_match_ptr(fus_nfc_id_table),
		.of_match_table = fus_nfc_id_table,
	},
	.probe = fus_nfc_probe,
	.remove = fus_nfc_remove,
	.id_table = fus_nfc_ids,
};

module_platform_driver(fus_nfc_driver);

MODULE_AUTHOR("F&S Elektronik Systeme GmbH");
MODULE_DESCRIPTION("F&S VF610 NFC NAND MTD driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);
