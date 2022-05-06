/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef __ASM_SPINLOCK_H
#define __ASM_SPINLOCK_H
#ifdef __KERNEL__

/*
 * Simple spin lock operations.  
 *
 * Copyright (C) 2001-2004 Paul Mackerras <paulus@au.ibm.com>, IBM
 * Copyright (C) 2001 Anton Blanchard <anton@au.ibm.com>, IBM
 * Copyright (C) 2002 Dave Engebretsen <engebret@us.ibm.com>, IBM
 *	Rework to support virtual processors
 *
 * Type of int is used as a full 64b word is not necessary.
 *
 * (the type definitions are in asm/spinlock_types.h)
 */
#include <linux/jump_label.h>
#include <linux/irqflags.h>
#ifdef CONFIG_PPC64
#include <asm/paca.h>
#include <asm/hvcall.h>
#endif
#include <asm/synch.h>
#include <asm/ppc-opcode.h>
#include <asm/asm-405.h>

#ifdef CONFIG_PPC64
/* use 0x800000yy when locked, where yy == CPU number */
#ifdef __BIG_ENDIAN__
#define LOCK_TOKEN	(*(u32 *)(&get_paca()->lock_token))
#else
#define LOCK_TOKEN	(*(u32 *)(&get_paca()->paca_index))
#endif
#else
#include <asm/simple_spinlock.h>
#endif

#ifdef CONFIG_PPC_PSERIES
DECLARE_STATIC_KEY_FALSE(shared_processor);

#define vcpu_is_preempted vcpu_is_preempted
static inline bool vcpu_is_preempted(int cpu)
{
	if (!static_branch_unlikely(&shared_processor))
		return false;
	return !!(be32_to_cpu(lppaca_of(cpu).yield_count) & 1);
}
#endif

#endif /* __KERNEL__ */
#endif /* __ASM_SPINLOCK_H */
