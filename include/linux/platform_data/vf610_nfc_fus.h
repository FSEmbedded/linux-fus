/*
 * Vybrid NAND Flash Controller Driver with Hardware ECC
 *
 * Copyright 2014 F&S Elektronik Systeme GmbH
 *
 * This is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __VF610_NFC_FUS_H__
#define __VF610_NFC_FUS_H__

/* Possible values for flags entry */
#define VYBRID_NFC_SKIP_INVERSE 0x01	/* Use skip region only, skip rest */

/* Possible values for eccmode entry */
#define VYBRID_NFC_ECCMODE_OFF   0	/* No ECC */
#define VYBRID_NFC_ECCMODE_4BIT  1	/* 8 ECC bytes */
#define VYBRID_NFC_ECCMODE_6BIT  2	/* 12 ECC bytes */
#define VYBRID_NFC_ECCMODE_8BIT  3	/* 15 ECC bytes */
#define VYBRID_NFC_ECCMODE_12BIT 4	/* 23 ECC bytes */
#define VYBRID_NFC_ECCMODE_16BIT 5	/* 30 ECC bytes */
#define VYBRID_NFC_ECCMODE_24BIT 6	/* 45 ECC bytes */
#define VYBRID_NFC_ECCMODE_32BIT 7	/* 60 ECC bytes */

/* Sizes and offsets are given in blocks because the NAND parameters (like
   block size) are not yet known when this structure is filled in. */
struct vf610_nfc_fus_platform_data {
	unsigned int	options;	/* Basic set of options */
	unsigned int	t_wb;		/* Wait cycles for T_WB and T_WHR
					   (0: use current value from NFC) */
	unsigned int	eccmode;	/* See VYBRID_NFC_ECCMODE_* above */
	unsigned int	flags;		/* See flag values above */
	unsigned int	skipblocks;	/* Number of blocks to skip at
					   beginning of device */
};

#endif /* !__VF610_NFC_FUS_H__ */
