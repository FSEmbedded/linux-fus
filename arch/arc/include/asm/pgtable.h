/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2004, 2007-2010, 2011-2012 Synopsys, Inc. (www.synopsys.com)
 */

#ifndef _ASM_ARC_PGTABLE_H
#define _ASM_ARC_PGTABLE_H

#include <linux/bits.h>
#include <asm-generic/pgtable-nopmd.h>
#include <asm/page.h>
#include <asm/mmu.h>	/* to propagate CONFIG_ARC_MMU_VER <n> */

/**************************************************************************
 * Page Table Flags
 *
 * ARC700 MMU only deals with softare managed TLB entries.
 * Page Tables are purely for Linux VM's consumption and the bits below are
 * suited to that (uniqueness). Hence some are not implemented in the TLB and
 * some have different value in TLB.
 * e.g. MMU v2: K_READ bit is 8 and so is GLOBAL (possible because they live in
 *      seperate PD0 and PD1, which combined forms a translation entry)
 *      while for PTE perspective, they are 8 and 9 respectively
 * with MMU v3: Most bits (except SHARED) represent the exact hardware pos
 *      (saves some bit shift ops in TLB Miss hdlrs)
 */

#if (CONFIG_ARC_MMU_VER <= 2)

#define _PAGE_ACCESSED      (1<<1)	/* Page is accessed (S) */
#define _PAGE_CACHEABLE     (1<<2)	/* Page is cached (H) */
#define _PAGE_EXECUTE       (1<<3)	/* Page has user execute perm (H) */
#define _PAGE_WRITE         (1<<4)	/* Page has user write perm (H) */
#define _PAGE_READ          (1<<5)	/* Page has user read perm (H) */
#define _PAGE_DIRTY         (1<<6)	/* Page modified (dirty) (S) */
#define _PAGE_SPECIAL       (1<<7)
#define _PAGE_GLOBAL        (1<<8)	/* Page is global (H) */
#define _PAGE_PRESENT       (1<<10)	/* TLB entry is valid (H) */

#else	/* MMU v3 onwards */

#define _PAGE_CACHEABLE     (1<<0)	/* Page is cached (H) */
#define _PAGE_EXECUTE       (1<<1)	/* Page has user execute perm (H) */
#define _PAGE_WRITE         (1<<2)	/* Page has user write perm (H) */
#define _PAGE_READ          (1<<3)	/* Page has user read perm (H) */
#define _PAGE_ACCESSED      (1<<4)	/* Page is accessed (S) */
#define _PAGE_DIRTY         (1<<5)	/* Page modified (dirty) (S) */
#define _PAGE_SPECIAL       (1<<6)

#if (CONFIG_ARC_MMU_VER >= 4)
#define _PAGE_WTHRU         (1<<7)	/* Page cache mode write-thru (H) */
#endif

#define _PAGE_GLOBAL        (1<<8)	/* Page is global (H) */
#define _PAGE_PRESENT       (1<<9)	/* TLB entry is valid (H) */

#if (CONFIG_ARC_MMU_VER >= 4)
#define _PAGE_HW_SZ         (1<<10)	/* Page Size indicator (H): 0 normal, 1 super */
#endif

#define _PAGE_SHARED_CODE   (1<<11)	/* Shared Code page with cmn vaddr
					   usable for shared TLB entries (H) */

#define _PAGE_UNUSED_BIT    (1<<12)
#endif

/* vmalloc permissions */
#define _K_PAGE_PERMS  (_PAGE_EXECUTE | _PAGE_WRITE | _PAGE_READ | \
			_PAGE_GLOBAL | _PAGE_PRESENT)

#ifndef CONFIG_ARC_CACHE_PAGES
#undef _PAGE_CACHEABLE
#define _PAGE_CACHEABLE 0
#endif

#ifndef _PAGE_HW_SZ
#define _PAGE_HW_SZ	0
#endif

/* Defaults for every user page */
#define ___DEF (_PAGE_PRESENT | _PAGE_CACHEABLE)

/* Set of bits not changed in pte_modify */
#define _PAGE_CHG_MASK	(PAGE_MASK_PHYS | _PAGE_ACCESSED | _PAGE_DIRTY | \
							   _PAGE_SPECIAL)
/* More Abbrevaited helpers */
#define PAGE_U_NONE     __pgprot(___DEF)
#define PAGE_U_R        __pgprot(___DEF | _PAGE_READ)
#define PAGE_U_W_R      __pgprot(___DEF | _PAGE_READ | _PAGE_WRITE)
#define PAGE_U_X_R      __pgprot(___DEF | _PAGE_READ | _PAGE_EXECUTE)
#define PAGE_U_X_W_R    __pgprot(___DEF | _PAGE_READ | _PAGE_WRITE | \
						       _PAGE_EXECUTE)

#define PAGE_SHARED	PAGE_U_W_R

/* While kernel runs out of unstranslated space, vmalloc/modules use a chunk of
 * user vaddr space - visible in all addr spaces, but kernel mode only
 * Thus Global, all-kernel-access, no-user-access, cached
 */
#define PAGE_KERNEL          __pgprot(_K_PAGE_PERMS | _PAGE_CACHEABLE)

/* ioremap */
#define PAGE_KERNEL_NO_CACHE __pgprot(_K_PAGE_PERMS)

/* Masks for actual TLB "PD"s */
#define PTE_BITS_IN_PD0		(_PAGE_GLOBAL | _PAGE_PRESENT | _PAGE_HW_SZ)
#define PTE_BITS_RWX		(_PAGE_EXECUTE | _PAGE_WRITE | _PAGE_READ)

#define PTE_BITS_NON_RWX_IN_PD1	(PAGE_MASK_PHYS | _PAGE_CACHEABLE)

/**************************************************************************
 * Mapping of vm_flags (Generic VM) to PTE flags (arch specific)
 *
 * Certain cases have 1:1 mapping
 *  e.g. __P101 means VM_READ, VM_EXEC and !VM_SHARED
 *       which directly corresponds to  PAGE_U_X_R
 *
 * Other rules which cause the divergence from 1:1 mapping
 *
 *  1. Although ARC700 can do exclusive execute/write protection (meaning R
 *     can be tracked independet of X/W unlike some other CPUs), still to
 *     keep things consistent with other archs:
 *      -Write implies Read:   W => R
 *      -Execute implies Read: X => R
 *
 *  2. Pvt Writable doesn't have Write Enabled initially: Pvt-W => !W
 *     This is to enable COW mechanism
 */
	/* xwr */
#define __P000  PAGE_U_NONE
#define __P001  PAGE_U_R
#define __P010  PAGE_U_R	/* Pvt-W => !W */
#define __P011  PAGE_U_R	/* Pvt-W => !W */
#define __P100  PAGE_U_X_R	/* X => R */
#define __P101  PAGE_U_X_R
#define __P110  PAGE_U_X_R	/* Pvt-W => !W and X => R */
#define __P111  PAGE_U_X_R	/* Pvt-W => !W */

#define __S000  PAGE_U_NONE
#define __S001  PAGE_U_R
#define __S010  PAGE_U_W_R	/* W => R */
#define __S011  PAGE_U_W_R
#define __S100  PAGE_U_X_R	/* X => R */
#define __S101  PAGE_U_X_R
#define __S110  PAGE_U_X_W_R	/* X => R */
#define __S111  PAGE_U_X_W_R

/****************************************************************
 * 2 tier (PGD:PTE) software page walker
 *
 * [31]		    32 bit virtual address              [0]
 * -------------------------------------------------------
 * |               | <------------ PGDIR_SHIFT ----------> |
 * |		   |					 |
 * | BITS_FOR_PGD  |  BITS_FOR_PTE  | <-- PAGE_SHIFT --> |
 * -------------------------------------------------------
 *       |                  |                |
 *       |                  |                --> off in page frame
 *       |                  ---> index into Page Table
 *       ----> index into Page Directory
 *
 * In a single page size configuration, only PAGE_SHIFT is fixed
 * So both PGD and PTE sizing can be tweaked
 *  e.g. 8K page (PAGE_SHIFT 13) can have
 *  - PGDIR_SHIFT 21  -> 11:8:13 address split
 *  - PGDIR_SHIFT 24  -> 8:11:13 address split
 *
 * If Super Page is configured, PGDIR_SHIFT becomes fixed too,
 * so the sizing flexibility is gone.
 */

#if defined(CONFIG_ARC_HUGEPAGE_16M)
#define PGDIR_SHIFT	24
#elif defined(CONFIG_ARC_HUGEPAGE_2M)
#define PGDIR_SHIFT	21
#else
/*
 * Only Normal page support so "hackable" (see comment above)
 * Default value provides 11:8:13 (8K), 11:9:12 (4K)
 */
#define PGDIR_SHIFT	21
#endif

#define BITS_FOR_PTE	(PGDIR_SHIFT - PAGE_SHIFT)
#define BITS_FOR_PGD	(32 - PGDIR_SHIFT)

#define PGDIR_SIZE	BIT(PGDIR_SHIFT)	/* vaddr span, not PDG sz */
#define PGDIR_MASK	(~(PGDIR_SIZE-1))

#include <asm/pgtable-levels.h>
#include <asm/pgtable-bits-arcv2.h>
#include <asm/page.h>
#include <asm/mmu.h>

/*
 * Number of entries a user land program use.
 * TASK_SIZE is the maximum vaddr that can be used by a userland program.
 */
#define	USER_PTRS_PER_PGD	(TASK_SIZE / PGDIR_SIZE)

#ifndef __ASSEMBLY__

extern char empty_zero_page[PAGE_SIZE];
#define ZERO_PAGE(vaddr)	(virt_to_page(empty_zero_page))

extern pgd_t swapper_pg_dir[] __aligned(PAGE_SIZE);

/* to cope with aliasing VIPT cache */
#define HAVE_ARCH_UNMAPPED_AREA

#endif /* __ASSEMBLY__ */

#endif
