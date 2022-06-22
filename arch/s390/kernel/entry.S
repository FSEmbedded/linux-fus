/* SPDX-License-Identifier: GPL-2.0 */
/*
 *    S390 low-level entry points.
 *
 *    Copyright IBM Corp. 1999, 2012
 *    Author(s): Martin Schwidefsky (schwidefsky@de.ibm.com),
 *		 Hartmut Penner (hp@de.ibm.com),
 *		 Denis Joseph Barrow (djbarrow@de.ibm.com,barrow_dj@yahoo.com),
 *		 Heiko Carstens <heiko.carstens@de.ibm.com>
 */

#include <linux/init.h>
#include <linux/linkage.h>
#include <asm/alternative-asm.h>
#include <asm/processor.h>
#include <asm/cache.h>
#include <asm/dwarf.h>
#include <asm/errno.h>
#include <asm/ptrace.h>
#include <asm/thread_info.h>
#include <asm/asm-offsets.h>
#include <asm/unistd.h>
#include <asm/page.h>
#include <asm/sigp.h>
#include <asm/irq.h>
#include <asm/vx-insn.h>
#include <asm/setup.h>
#include <asm/nmi.h>
#include <asm/export.h>
#include <asm/nospec-insn.h>

__PT_R0      =	__PT_GPRS
__PT_R1      =	__PT_GPRS + 8
__PT_R2      =	__PT_GPRS + 16
__PT_R3      =	__PT_GPRS + 24
__PT_R4      =	__PT_GPRS + 32
__PT_R5      =	__PT_GPRS + 40
__PT_R6      =	__PT_GPRS + 48
__PT_R7      =	__PT_GPRS + 56
__PT_R8      =	__PT_GPRS + 64
__PT_R9      =	__PT_GPRS + 72
__PT_R10     =	__PT_GPRS + 80
__PT_R11     =	__PT_GPRS + 88
__PT_R12     =	__PT_GPRS + 96
__PT_R13     =	__PT_GPRS + 104
__PT_R14     =	__PT_GPRS + 112
__PT_R15     =	__PT_GPRS + 120

STACK_SHIFT = PAGE_SHIFT + THREAD_SIZE_ORDER
STACK_SIZE  = 1 << STACK_SHIFT
STACK_INIT = STACK_SIZE - STACK_FRAME_OVERHEAD - __PT_SIZE

_LPP_OFFSET	= __LC_LPP

	.macro	CHECK_STACK savearea
#ifdef CONFIG_CHECK_STACK
	tml	%r15,STACK_SIZE - CONFIG_STACK_GUARD
	lghi	%r14,\savearea
	jz	stack_overflow
#endif
	.endm

	.macro	CHECK_VMAP_STACK savearea,oklabel
#ifdef CONFIG_VMAP_STACK
	lgr	%r14,%r15
	nill	%r14,0x10000 - STACK_SIZE
	oill	%r14,STACK_INIT
	clg	%r14,__LC_KERNEL_STACK
	je	\oklabel
	clg	%r14,__LC_ASYNC_STACK
	je	\oklabel
	clg	%r14,__LC_MCCK_STACK
	je	\oklabel
	clg	%r14,__LC_NODAT_STACK
	je	\oklabel
	clg	%r14,__LC_RESTART_STACK
	je	\oklabel
	lghi	%r14,\savearea
	j	stack_overflow
#else
	j	\oklabel
#endif
	.endm

	.macro	SWITCH_ASYNC savearea,timer,clock
	tmhh	%r8,0x0001		# interrupting from user ?
	jnz	4f
#if IS_ENABLED(CONFIG_KVM)
	lgr	%r14,%r9
	larl	%r13,.Lsie_gmap
	slgr	%r14,%r13
	lghi	%r13,.Lsie_done - .Lsie_gmap
	clgr	%r14,%r13
	jhe	0f
	lghi	%r11,\savearea		# inside critical section, do cleanup
	brasl	%r14,.Lcleanup_sie
#endif
0:	larl	%r13,.Lpsw_idle_exit
	cgr	%r13,%r9
	jne	3f

	larl	%r1,smp_cpu_mtid
	llgf	%r1,0(%r1)
	ltgr	%r1,%r1
	jz	2f			# no SMT, skip mt_cycles calculation
	.insn	rsy,0xeb0000000017,%r1,5,__SF_EMPTY+80(%r15)
	larl	%r3,mt_cycles
	ag	%r3,__LC_PERCPU_OFFSET
	la	%r4,__SF_EMPTY+16(%r15)
1:	lg	%r0,0(%r3)
	slg	%r0,0(%r4)
	alg	%r0,64(%r4)
	stg	%r0,0(%r3)
	la	%r3,8(%r3)
	la	%r4,8(%r4)
	brct	%r1,1b

2:	mvc	__CLOCK_IDLE_EXIT(8,%r2), \clock
	mvc	__TIMER_IDLE_EXIT(8,%r2), \timer
	# account system time going idle
	ni	__LC_CPU_FLAGS+7,255-_CIF_ENABLED_WAIT

	lg	%r13,__LC_STEAL_TIMER
	alg	%r13,__CLOCK_IDLE_ENTER(%r2)
	slg	%r13,__LC_LAST_UPDATE_CLOCK
	stg	%r13,__LC_STEAL_TIMER

	mvc	__LC_LAST_UPDATE_CLOCK(8),__CLOCK_IDLE_EXIT(%r2)

	lg	%r13,__LC_SYSTEM_TIMER
	alg	%r13,__LC_LAST_UPDATE_TIMER
	slg	%r13,__TIMER_IDLE_ENTER(%r2)
	stg	%r13,__LC_SYSTEM_TIMER
	mvc	__LC_LAST_UPDATE_TIMER(8),__TIMER_IDLE_EXIT(%r2)

	nihh	%r8,0xfcfd		# clear wait state and irq bits
3:	lg	%r14,__LC_ASYNC_STACK	# are we already on the target stack?
	slgr	%r14,%r15
	srag	%r14,%r14,STACK_SHIFT
	jnz	5f
	CHECK_STACK \savearea
	aghi	%r15,-(STACK_FRAME_OVERHEAD + __PT_SIZE)
	j	6f
4:	UPDATE_VTIME %r14,%r15,\timer
	BPENTER __TI_flags(%r12),_TIF_ISOLATE_BP
5:	lg	%r15,__LC_ASYNC_STACK	# load async stack
6:	la	%r11,STACK_FRAME_OVERHEAD(%r15)
	.endm

	.macro UPDATE_VTIME w1,w2,enter_timer
	lg	\w1,__LC_EXIT_TIMER
	lg	\w2,__LC_LAST_UPDATE_TIMER
	slg	\w1,\enter_timer
	slg	\w2,__LC_EXIT_TIMER
	alg	\w1,__LC_USER_TIMER
	alg	\w2,__LC_SYSTEM_TIMER
	stg	\w1,__LC_USER_TIMER
	stg	\w2,__LC_SYSTEM_TIMER
	mvc	__LC_LAST_UPDATE_TIMER(8),\enter_timer
	.endm

	.macro RESTORE_SM_CLEAR_PER
	stg	%r8,__LC_RETURN_PSW
	ni	__LC_RETURN_PSW,0xbf
	ssm	__LC_RETURN_PSW
	.endm

	.macro ENABLE_INTS
	stosm	__SF_EMPTY(%r15),3
	.endm

	.macro ENABLE_INTS_TRACE
	TRACE_IRQS_ON
	ENABLE_INTS
	.endm

	.macro DISABLE_INTS
	stnsm	__SF_EMPTY(%r15),0xfc
	.endm

	.macro DISABLE_INTS_TRACE
	DISABLE_INTS
	TRACE_IRQS_OFF
	.endm

	.macro STCK savearea
	ALTERNATIVE ".insn	s,0xb2050000,\savearea", \
		    ".insn	s,0xb27c0000,\savearea", 25
	.endm

	/*
	 * The TSTMSK macro generates a test-under-mask instruction by
	 * calculating the memory offset for the specified mask value.
	 * Mask value can be any constant.  The macro shifts the mask
	 * value to calculate the memory offset for the test-under-mask
	 * instruction.
	 */
	.macro TSTMSK addr, mask, size=8, bytepos=0
		.if (\bytepos < \size) && (\mask >> 8)
			.if (\mask & 0xff)
				.error "Mask exceeds byte boundary"
			.endif
			TSTMSK \addr, "(\mask >> 8)", \size, "(\bytepos + 1)"
			.exitm
		.endif
		.ifeq \mask
			.error "Mask must not be zero"
		.endif
		off = \size - \bytepos - 1
		tm	off+\addr, \mask
	.endm

	.macro BPOFF
	ALTERNATIVE "", ".long 0xb2e8c000", 82
	.endm

	.macro BPON
	ALTERNATIVE "", ".long 0xb2e8d000", 82
	.endm

	.macro BPENTER tif_ptr,tif_mask
	ALTERNATIVE "TSTMSK \tif_ptr,\tif_mask; jz .+8; .long 0xb2e8d000", \
		    "", 82
	.endm

	.macro BPEXIT tif_ptr,tif_mask
	TSTMSK	\tif_ptr,\tif_mask
	ALTERNATIVE "jz .+8;  .long 0xb2e8c000", \
		    "jnz .+8; .long 0xb2e8d000", 82
	.endm

	/*
	 * The CHKSTG macro jumps to the provided label in case the
	 * machine check interruption code reports one of unrecoverable
	 * storage errors:
	 * - Storage error uncorrected
	 * - Storage key error uncorrected
	 * - Storage degradation with Failing-storage-address validity
	 */
	.macro CHKSTG errlabel
	TSTMSK	__LC_MCCK_CODE,(MCCK_CODE_STG_ERROR|MCCK_CODE_STG_KEY_ERROR)
	jnz	\errlabel
	TSTMSK	__LC_MCCK_CODE,MCCK_CODE_STG_DEGRAD
	jz	.Loklabel\@
	TSTMSK	__LC_MCCK_CODE,MCCK_CODE_STG_FAIL_ADDR
	jnz	\errlabel
.Loklabel\@:
	.endm

#if IS_ENABLED(CONFIG_KVM)
	/*
	 * The OUTSIDE macro jumps to the provided label in case the value
	 * in the provided register is outside of the provided range. The
	 * macro is useful for checking whether a PSW stored in a register
	 * pair points inside or outside of a block of instructions.
	 * @reg: register to check
	 * @start: start of the range
	 * @end: end of the range
	 * @outside_label: jump here if @reg is outside of [@start..@end)
	 */
	.macro OUTSIDE reg,start,end,outside_label
	lgr	%r14,\reg
	larl	%r13,\start
	slgr	%r14,%r13
	lghi	%r13,\end - \start
	clgr	%r14,%r13
	jhe	\outside_label
	.endm

	.macro SIEEXIT
	lg	%r9,__SF_SIE_CONTROL(%r15)	# get control block pointer
	ni	__SIE_PROG0C+3(%r9),0xfe	# no longer in SIE
	lctlg	%c1,%c1,__LC_KERNEL_ASCE	# load primary asce
	larl	%r9,sie_exit			# skip forward to sie_exit
	.endm
#endif

	GEN_BR_THUNK %r14
	GEN_BR_THUNK %r14,%r13

	.section .kprobes.text, "ax"
.Ldummy:
	/*
	 * This nop exists only in order to avoid that __bpon starts at
	 * the beginning of the kprobes text section. In that case we would
	 * have several symbols at the same address. E.g. objdump would take
	 * an arbitrary symbol name when disassembling this code.
	 * With the added nop in between the __bpon symbol is unique
	 * again.
	 */
	nop	0

ENTRY(__bpon)
	.globl __bpon
	BPON
	BR_EX	%r14
ENDPROC(__bpon)

/*
 * Scheduler resume function, called by switch_to
 *  gpr2 = (task_struct *) prev
 *  gpr3 = (task_struct *) next
 * Returns:
 *  gpr2 = prev
 */
ENTRY(__switch_to)
	stmg	%r6,%r15,__SF_GPRS(%r15)	# store gprs of prev task
	lghi	%r4,__TASK_stack
	lghi	%r1,__TASK_thread
	llill	%r5,STACK_INIT
	stg	%r15,__THREAD_ksp(%r1,%r2)	# store kernel stack of prev
	lg	%r15,0(%r4,%r3)			# start of kernel stack of next
	agr	%r15,%r5			# end of kernel stack of next
	stg	%r3,__LC_CURRENT		# store task struct of next
	stg	%r15,__LC_KERNEL_STACK		# store end of kernel stack
	lg	%r15,__THREAD_ksp(%r1,%r3)	# load kernel stack of next
	aghi	%r3,__TASK_pid
	mvc	__LC_CURRENT_PID(4,%r0),0(%r3)	# store pid of next
	lmg	%r6,%r15,__SF_GPRS(%r15)	# load gprs of next task
	ALTERNATIVE "", ".insn s,0xb2800000,_LPP_OFFSET", 40
	BR_EX	%r14
ENDPROC(__switch_to)

#if IS_ENABLED(CONFIG_KVM)
/*
 * sie64a calling convention:
 * %r2 pointer to sie control block
 * %r3 guest register save area
 */
ENTRY(sie64a)
	stmg	%r6,%r14,__SF_GPRS(%r15)	# save kernel registers
	lg	%r12,__LC_CURRENT
	stg	%r2,__SF_SIE_CONTROL(%r15)	# save control block pointer
	stg	%r3,__SF_SIE_SAVEAREA(%r15)	# save guest register save area
	xc	__SF_SIE_REASON(8,%r15),__SF_SIE_REASON(%r15) # reason code = 0
	mvc	__SF_SIE_FLAGS(8,%r15),__TI_flags(%r12) # copy thread flags
	lmg	%r0,%r13,0(%r3)			# load guest gprs 0-13
	lg	%r14,__LC_GMAP			# get gmap pointer
	ltgr	%r14,%r14
	jz	.Lsie_gmap
	lctlg	%c1,%c1,__GMAP_ASCE(%r14)	# load primary asce
.Lsie_gmap:
	lg	%r14,__SF_SIE_CONTROL(%r15)	# get control block pointer
	oi	__SIE_PROG0C+3(%r14),1		# we are going into SIE now
	tm	__SIE_PROG20+3(%r14),3		# last exit...
	jnz	.Lsie_skip
	TSTMSK	__LC_CPU_FLAGS,_CIF_FPU
	jo	.Lsie_skip			# exit if fp/vx regs changed
	BPEXIT	__SF_SIE_FLAGS(%r15),(_TIF_ISOLATE_BP|_TIF_ISOLATE_BP_GUEST)
.Lsie_entry:
	sie	0(%r14)
	BPOFF
	BPENTER	__SF_SIE_FLAGS(%r15),(_TIF_ISOLATE_BP|_TIF_ISOLATE_BP_GUEST)
.Lsie_skip:
	ni	__SIE_PROG0C+3(%r14),0xfe	# no longer in SIE
	lctlg	%c1,%c1,__LC_KERNEL_ASCE	# load primary asce
.Lsie_done:
# some program checks are suppressing. C code (e.g. do_protection_exception)
# will rewind the PSW by the ILC, which is often 4 bytes in case of SIE. There
# are some corner cases (e.g. runtime instrumentation) where ILC is unpredictable.
# Other instructions between sie64a and .Lsie_done should not cause program
# interrupts. So lets use 3 nops as a landing pad for all possible rewinds.
.Lrewind_pad6:
	nopr	7
.Lrewind_pad4:
	nopr	7
.Lrewind_pad2:
	nopr	7
	.globl sie_exit
sie_exit:
	lg	%r14,__SF_SIE_SAVEAREA(%r15)	# load guest register save area
	stmg	%r0,%r13,0(%r14)		# save guest gprs 0-13
	xgr	%r0,%r0				# clear guest registers to
	xgr	%r1,%r1				# prevent speculative use
	xgr	%r3,%r3
	xgr	%r4,%r4
	xgr	%r5,%r5
	lmg	%r6,%r14,__SF_GPRS(%r15)	# restore kernel registers
	lg	%r2,__SF_SIE_REASON(%r15)	# return exit reason code
	BR_EX	%r14
.Lsie_fault:
	lghi	%r14,-EFAULT
	stg	%r14,__SF_SIE_REASON(%r15)	# set exit reason code
	j	sie_exit

	EX_TABLE(.Lrewind_pad6,.Lsie_fault)
	EX_TABLE(.Lrewind_pad4,.Lsie_fault)
	EX_TABLE(.Lrewind_pad2,.Lsie_fault)
	EX_TABLE(sie_exit,.Lsie_fault)
ENDPROC(sie64a)
EXPORT_SYMBOL(sie64a)
EXPORT_SYMBOL(sie_exit)
#endif

/*
 * SVC interrupt handler routine. System calls are synchronous events and
 * are entered with interrupts disabled.
 */

ENTRY(system_call)
	stpt	__LC_SYS_ENTER_TIMER
	stmg	%r8,%r15,__LC_SAVE_AREA_SYNC
	BPOFF
	lghi	%r14,0
.Lsysc_per:
	lctlg	%c1,%c1,__LC_KERNEL_ASCE
	lg	%r12,__LC_CURRENT
	lg	%r15,__LC_KERNEL_STACK
	xc	__SF_BACKCHAIN(8,%r15),__SF_BACKCHAIN(%r15)
	stmg	%r0,%r7,STACK_FRAME_OVERHEAD+__PT_R0(%r15)
	BPENTER __TI_flags(%r12),_TIF_ISOLATE_BP
	stmg	%r0,%r7,__PT_R0(%r11)
	mvc	__PT_R8(64,%r11),__LC_SAVE_AREA_SYNC
	mvc	__PT_PSW(16,%r11),__LC_SVC_OLD_PSW
	mvc	__PT_INT_CODE(4,%r11),__LC_SVC_ILC
	stg	%r14,__PT_FLAGS(%r11)
	xc	__SF_BACKCHAIN(8,%r15),__SF_BACKCHAIN(%r15)
	ENABLE_INTS
.Lsysc_do_svc:
	# clear user controlled register to prevent speculative use
	xgr	%r0,%r0
	# load address of system call table
	lg	%r10,__THREAD_sysc_table(%r13,%r12)
	llgh	%r8,__PT_INT_CODE+2(%r11)
	slag	%r8,%r8,3			# shift and test for svc 0
	jnz	.Lsysc_nr_ok
	# svc 0: system call number in %r1
	llgfr	%r1,%r1				# clear high word in r1
	sth	%r1,__PT_INT_CODE+2(%r11)
	cghi	%r1,NR_syscalls
	jnl	.Lsysc_nr_ok
	slag	%r8,%r1,3
.Lsysc_nr_ok:
	stg	%r2,__PT_ORIG_GPR2(%r11)
	stg	%r7,STACK_FRAME_OVERHEAD(%r15)
	lg	%r9,0(%r8,%r10)			# get system call add.
	TSTMSK	__TI_flags(%r12),_TIF_TRACE
	jnz	.Lsysc_tracesys
	BASR_EX	%r14,%r9			# call sys_xxxx
	stg	%r2,__PT_R2(%r11)		# store return value

.Lsysc_return:
#ifdef CONFIG_DEBUG_RSEQ
	lgr	%r2,%r11
	brasl	%r14,rseq_syscall
#endif
	LOCKDEP_SYS_EXIT
.Lsysc_tif:
	DISABLE_INTS
	TSTMSK	__PT_FLAGS(%r11),_PIF_WORK
	jnz	.Lsysc_work
	TSTMSK	__TI_flags(%r12),_TIF_WORK
	jnz	.Lsysc_work			# check for work
	TSTMSK	__LC_CPU_FLAGS,(_CIF_WORK-_CIF_FPU)
	jnz	.Lsysc_work
	BPEXIT	__TI_flags(%r12),_TIF_ISOLATE_BP
.Lsysc_restore:
	DISABLE_INTS
	TSTMSK	__LC_CPU_FLAGS, _CIF_FPU
	jz	.Lsysc_skip_fpu
	brasl	%r14,load_fpu_regs
.Lsysc_skip_fpu:
	mvc	__LC_RETURN_PSW(16),__PT_PSW(%r11)
	stpt	__LC_EXIT_TIMER
	b	__LC_RETURN_LPSWE
ENDPROC(system_call)

#
# a new process exits the kernel with ret_from_fork
#
ENTRY(ret_from_fork)
	lgr	%r3,%r11
	brasl	%r14,__ret_from_fork
	lctlg	%c1,%c1,__LC_USER_ASCE
	mvc	__LC_RETURN_PSW(16),STACK_FRAME_OVERHEAD+__PT_PSW(%r15)
	BPEXIT __TI_flags(%r12),_TIF_ISOLATE_BP
	lmg	%r0,%r15,STACK_FRAME_OVERHEAD+__PT_R0(%r15)
	stpt	__LC_EXIT_TIMER
	b	__LC_RETURN_LPSWE
ENDPROC(ret_from_fork)

/*
 * Program check handler routine
 */

ENTRY(pgm_check_handler)
	stpt	__LC_SYS_ENTER_TIMER
	BPOFF
	stmg	%r8,%r15,__LC_SAVE_AREA_SYNC
	lg	%r12,__LC_CURRENT
	lghi	%r10,0
	lmg	%r8,%r9,__LC_PGM_OLD_PSW
	tmhh	%r8,0x0001		# coming from user space?
	jno	.Lpgm_skip_asce
	lctlg	%c1,%c1,__LC_KERNEL_ASCE
	j	3f			# -> fault in user space
.Lpgm_skip_asce:
#if IS_ENABLED(CONFIG_KVM)
	# cleanup critical section for program checks in sie64a
	OUTSIDE	%r9,.Lsie_gmap,.Lsie_done,1f
	SIEEXIT
	lghi	%r10,_PIF_GUEST_FAULT
#endif
1:	tmhh	%r8,0x4000		# PER bit set in old PSW ?
	jnz	2f			# -> enabled, can't be a double fault
	tm	__LC_PGM_ILC+3,0x80	# check for per exception
	jnz	.Lpgm_svcper		# -> single stepped svc
2:	CHECK_STACK __LC_SAVE_AREA_SYNC
	aghi	%r15,-(STACK_FRAME_OVERHEAD + __PT_SIZE)
	# CHECK_VMAP_STACK branches to stack_overflow or 4f
	CHECK_VMAP_STACK __LC_SAVE_AREA_SYNC,4f
3:	BPENTER __TI_flags(%r12),_TIF_ISOLATE_BP
	lg	%r15,__LC_KERNEL_STACK
4:	la	%r11,STACK_FRAME_OVERHEAD(%r15)
	stg	%r10,__PT_FLAGS(%r11)
	xc	__SF_BACKCHAIN(8,%r15),__SF_BACKCHAIN(%r15)
	stmg	%r0,%r7,__PT_R0(%r11)
	mvc	__PT_R8(64,%r11),__LC_SAVE_AREA_SYNC
	stmg	%r8,%r9,__PT_PSW(%r11)

	# clear user controlled registers to prevent speculative use
	xgr	%r0,%r0
	xgr	%r1,%r1
	xgr	%r3,%r3
	xgr	%r4,%r4
	xgr	%r5,%r5
	xgr	%r6,%r6
	xgr	%r7,%r7
	mvc	__PT_R8(64,%r11),__LC_SAVE_AREA_SYNC
	stmg	%r8,%r9,__PT_PSW(%r11)
	mvc	__PT_INT_CODE(4,%r11),__LC_PGM_ILC
	mvc	__PT_INT_PARM_LONG(8,%r11),__LC_TRANS_EXC_CODE
	stg	%r13,__PT_FLAGS(%r11)
	stg	%r10,__PT_ARGS(%r11)
	tm	__LC_PGM_ILC+3,0x80	# check for per exception
	jz	6f
	tmhh	%r8,0x0001		# kernel per event ?
	jz	.Lpgm_kprobe
	oi	__PT_FLAGS+7(%r11),_PIF_PER_TRAP
	mvc	__THREAD_per_address(8,%r14),__LC_PER_ADDRESS
	mvc	__THREAD_per_cause(2,%r14),__LC_PER_CODE
	mvc	__THREAD_per_paid(1,%r14),__LC_PER_ACCESS_ID
6:	xc	__SF_BACKCHAIN(8,%r15),__SF_BACKCHAIN(%r15)
	RESTORE_SM_CLEAR_PER
	larl	%r1,pgm_check_table
	llgh	%r10,__PT_INT_CODE+2(%r11)
	nill	%r10,0x007f
	sll	%r10,3
	je	.Lpgm_return
	lg	%r9,0(%r10,%r1)		# load address of handler routine
	lgr	%r2,%r11		# pass pointer to pt_regs
	BASR_EX	%r14,%r9		# branch to interrupt-handler
.Lpgm_return:
	LOCKDEP_SYS_EXIT
	tm	__PT_PSW+1(%r11),0x01	# returning to user ?
	jno	.Lsysc_restore
	TSTMSK	__PT_FLAGS(%r11),_PIF_SYSCALL
	jo	.Lsysc_do_syscall
	j	.Lsysc_tif

#
# PER event in supervisor state, must be kprobes
#
.Lpgm_kprobe:
	xc	__SF_BACKCHAIN(8,%r15),__SF_BACKCHAIN(%r15)
	RESTORE_SM_CLEAR_PER
	lgr	%r2,%r11		# pass pointer to pt_regs
	brasl	%r14,do_per_trap
	j	.Lpgm_return

#
# single stepped system call
#
.Lpgm_svcper:
	mvc	__LC_RETURN_PSW(8),__LC_SVC_NEW_PSW
	larl	%r14,.Lsysc_per
	stg	%r14,__LC_RETURN_PSW+8
	lghi	%r14,1
	lpswe	__LC_RETURN_PSW		# branch to .Lsysc_per
ENDPROC(pgm_check_handler)

/*
 * Interrupt handler macro used for external and IO interrupts.
 */
.macro INT_HANDLER name,lc_old_psw,handler
ENTRY(\name)
	STCK	__LC_INT_CLOCK
	stpt	__LC_SYS_ENTER_TIMER
	BPOFF
	stmg	%r8,%r15,__LC_SAVE_AREA_ASYNC
	lg	%r12,__LC_CURRENT
	lmg	%r8,%r9,__LC_IO_OLD_PSW
	SWITCH_ASYNC __LC_SAVE_AREA_ASYNC,__LC_ASYNC_ENTER_TIMER,__LC_INT_CLOCK
	stmg	%r0,%r7,__PT_R0(%r11)
	# clear user controlled registers to prevent speculative use
	xgr	%r0,%r0
	xgr	%r1,%r1
	xgr	%r3,%r3
	xgr	%r4,%r4
	xgr	%r5,%r5
	xgr	%r6,%r6
	xgr	%r7,%r7
	xgr	%r10,%r10
	xc	__PT_FLAGS(8,%r11),__PT_FLAGS(%r11)
	mvc	__PT_R8(64,%r11),__LC_SAVE_AREA_ASYNC
	stmg	%r8,%r9,__PT_PSW(%r11)
	mvc	__PT_INT_CODE(12,%r11),__LC_SUBCHANNEL_ID
	xc	__PT_FLAGS(8,%r11),__PT_FLAGS(%r11)
	xc	__SF_BACKCHAIN(8,%r15),__SF_BACKCHAIN(%r15)
	TSTMSK	__LC_CPU_FLAGS,_CIF_IGNORE_IRQ
	jo	.Lio_restore
	TRACE_IRQS_OFF
.Lio_loop:
	lgr	%r2,%r11		# pass pointer to pt_regs
	lghi	%r3,IO_INTERRUPT
	tm	__PT_INT_CODE+8(%r11),0x80	# adapter interrupt ?
	jz	.Lio_call
	lghi	%r3,THIN_INTERRUPT
.Lio_call:
	brasl	%r14,do_IRQ
	TSTMSK	__LC_MACHINE_FLAGS,MACHINE_FLAG_LPAR
	jz	.Lio_return
	tpi	0
	jz	.Lio_return
	mvc	__PT_INT_CODE(12,%r11),__LC_SUBCHANNEL_ID
	j	.Lio_loop
.Lio_return:
	LOCKDEP_SYS_EXIT
	TSTMSK	__TI_flags(%r12),_TIF_WORK
	jnz	.Lio_work		# there is work to do (signals etc.)
	TSTMSK	__LC_CPU_FLAGS,_CIF_WORK
	jnz	.Lio_work
.Lio_restore:
	TRACE_IRQS_ON
	mvc	__LC_RETURN_PSW(16),__PT_PSW(%r11)
	tmhh	%r8,0x0001		# returning to user ?
	jno	2f
	lctlg	%c1,%c1,__LC_USER_ASCE
	BPEXIT	__TI_flags(%r12),_TIF_ISOLATE_BP
	stpt	__LC_EXIT_TIMER
2:	lmg	%r0,%r15,__PT_R0(%r11)
	b	__LC_RETURN_LPSWE
ENDPROC(\name)
.endm

#
# _TIF_SIGPENDING or is set, call do_signal
#
.Lio_sigpending:
	ENABLE_INTS_TRACE
	lgr	%r2,%r11		# pass pointer to pt_regs
	brasl	%r14,do_signal
	DISABLE_INTS_TRACE
	j	.Lio_return

#
# _TIF_NOTIFY_RESUME or is set, call do_notify_resume
#
.Lio_notify_resume:
	ENABLE_INTS_TRACE
	lgr	%r2,%r11		# pass pointer to pt_regs
	brasl	%r14,do_notify_resume
	DISABLE_INTS_TRACE
	j	.Lio_return
ENDPROC(io_int_handler)

/*
 * External interrupt handler routine
 */
ENTRY(ext_int_handler)
	STCK	__LC_INT_CLOCK
	stpt	__LC_ASYNC_ENTER_TIMER
	BPOFF
	stmg	%r8,%r15,__LC_SAVE_AREA_ASYNC
	lg	%r12,__LC_CURRENT
	lmg	%r8,%r9,__LC_EXT_OLD_PSW
	SWITCH_ASYNC __LC_SAVE_AREA_ASYNC,__LC_ASYNC_ENTER_TIMER,__LC_INT_CLOCK
	stmg	%r0,%r7,__PT_R0(%r11)
	# clear user controlled registers to prevent speculative use
	xgr	%r0,%r0
	xgr	%r1,%r1
	xgr	%r2,%r2
	xgr	%r3,%r3
	xgr	%r4,%r4
	xgr	%r5,%r5
	xgr	%r6,%r6
	xgr	%r7,%r7
	xgr	%r10,%r10
	mvc	__PT_R8(64,%r11),__LC_SAVE_AREA_ASYNC
	stmg	%r8,%r9,__PT_PSW(%r11)
	lghi	%r1,__LC_EXT_PARAMS2
	mvc	__PT_INT_CODE(4,%r11),__LC_EXT_CPU_ADDR
	mvc	__PT_INT_PARM(4,%r11),__LC_EXT_PARAMS
	mvc	__PT_INT_PARM_LONG(8,%r11),0(%r1)
	xc	__PT_FLAGS(8,%r11),__PT_FLAGS(%r11)
	xc	__SF_BACKCHAIN(8,%r15),__SF_BACKCHAIN(%r15)
	TSTMSK	__LC_CPU_FLAGS,_CIF_IGNORE_IRQ
	jo	.Lio_restore
	TRACE_IRQS_OFF
	lgr	%r2,%r11		# pass pointer to pt_regs
	lghi	%r3,EXT_INTERRUPT
	brasl	%r14,do_IRQ
	j	.Lio_return
ENDPROC(ext_int_handler)

/*
 * Load idle PSW.
 */
ENTRY(psw_idle)
	stg	%r14,(__SF_GPRS+8*8)(%r15)
	stg	%r3,__SF_EMPTY(%r15)
	larl	%r1,psw_idle_exit
	stg	%r1,__SF_EMPTY+8(%r15)
	larl	%r1,smp_cpu_mtid
	llgf	%r1,0(%r1)
	ltgr	%r1,%r1
	jz	.Lpsw_idle_stcctm
	.insn	rsy,0xeb0000000017,%r1,5,__MT_CYCLES_ENTER(%r2)
.Lpsw_idle_stcctm:
	oi	__LC_CPU_FLAGS+7,_CIF_ENABLED_WAIT
	BPON
	STCK	__CLOCK_IDLE_ENTER(%r2)
	stpt	__TIMER_IDLE_ENTER(%r2)
	lpswe	__SF_EMPTY(%r15)
.globl psw_idle_exit
psw_idle_exit:
	BR_EX	%r14
ENDPROC(psw_idle)

/*
 * Machine check handler routines
 */
ENTRY(mcck_int_handler)
	STCK	__LC_MCCK_CLOCK
	BPOFF
	la	%r1,4095		# validate r1
	spt	__LC_CPU_TIMER_SAVE_AREA-4095(%r1)	# validate cpu timer
	lmg	%r0,%r15,__LC_GPREGS_SAVE_AREA-4095(%r1)# validate gprs
	lg	%r12,__LC_CURRENT
	lmg	%r8,%r9,__LC_MCK_OLD_PSW
	TSTMSK	__LC_MCCK_CODE,MCCK_CODE_SYSTEM_DAMAGE
	jo	.Lmcck_panic		# yes -> rest of mcck code invalid
	TSTMSK	__LC_MCCK_CODE,MCCK_CODE_CR_VALID
	jno	.Lmcck_panic		# control registers invalid -> panic
	la	%r14,4095
	lctlg	%c0,%c15,__LC_CREGS_SAVE_AREA-4095(%r14) # validate ctl regs
	ptlb
	lghi	%r14,__LC_CPU_TIMER_SAVE_AREA
	mvc	__LC_MCCK_ENTER_TIMER(8),0(%r14)
	TSTMSK	__LC_MCCK_CODE,MCCK_CODE_CPU_TIMER_VALID
	jo	3f
	la	%r14,__LC_SYS_ENTER_TIMER
	clc	0(8,%r14),__LC_EXIT_TIMER
	jl	1f
	la	%r14,__LC_EXIT_TIMER
1:	clc	0(8,%r14),__LC_LAST_UPDATE_TIMER
	jl	2f
	la	%r14,__LC_LAST_UPDATE_TIMER
2:	spt	0(%r14)
	mvc	__LC_MCCK_ENTER_TIMER(8),0(%r14)
3:	TSTMSK	__LC_MCCK_CODE,MCCK_CODE_PSW_MWP_VALID
	jno	.Lmcck_panic
	tmhh	%r8,0x0001		# interrupting from user ?
	jnz	6f
	TSTMSK	__LC_MCCK_CODE,MCCK_CODE_PSW_IA_VALID
	jno	.Lmcck_panic
4:	ssm	__LC_PGM_NEW_PSW	# turn dat on, keep irqs off
	SWITCH_ASYNC __LC_GPREGS_SAVE_AREA+64,__LC_MCCK_ENTER_TIMER,__LC_MCCK_CLOCK
.Lmcck_skip:
	lghi	%r14,__LC_GPREGS_SAVE_AREA+64
	stmg	%r0,%r7,__PT_R0(%r11)
	# clear user controlled registers to prevent speculative use
	xgr	%r0,%r0
	xgr	%r1,%r1
	xgr	%r3,%r3
	xgr	%r4,%r4
	xgr	%r5,%r5
	xgr	%r6,%r6
	xgr	%r7,%r7
	xgr	%r10,%r10
	mvc	__PT_R8(64,%r11),0(%r14)
	stmg	%r8,%r9,__PT_PSW(%r11)
	xc	__PT_FLAGS(8,%r11),__PT_FLAGS(%r11)
	xc	__SF_BACKCHAIN(8,%r15),__SF_BACKCHAIN(%r15)
	lgr	%r2,%r11		# pass pointer to pt_regs
	brasl	%r14,s390_do_machine_check
	cghi	%r2,0
	je	.Lmcck_return
	lg	%r1,__LC_KERNEL_STACK	# switch to kernel stack
	mvc	STACK_FRAME_OVERHEAD(__PT_SIZE,%r1),0(%r11)
	xc	__SF_BACKCHAIN(8,%r1),__SF_BACKCHAIN(%r1)
	la	%r11,STACK_FRAME_OVERHEAD(%r1)
	lgr	%r15,%r1
	brasl	%r14,s390_handle_mcck
.Lmcck_return:
	lctlg	%c1,%c1,__PT_CR1(%r11)
	lmg	%r0,%r10,__PT_R0(%r11)
	mvc	__LC_RETURN_MCCK_PSW(16),__PT_PSW(%r11) # move return PSW
	tm	__LC_RETURN_MCCK_PSW+1,0x01 # returning to user ?
	jno	0f
	BPEXIT	__TI_flags(%r12),_TIF_ISOLATE_BP
	stpt	__LC_EXIT_TIMER
0:	lmg	%r11,%r15,__PT_R11(%r11)
	b	__LC_RETURN_MCCK_LPSWE

.Lmcck_panic:
	/*
	 * Iterate over all possible CPU addresses in the range 0..0xffff
	 * and stop each CPU using signal processor. Use compare and swap
	 * to allow just one CPU-stopper and prevent concurrent CPUs from
	 * stopping each other while leaving the others running.
	 */
	lhi	%r5,0
	lhi	%r6,1
	larl	%r7,.Lstop_lock
	cs	%r5,%r6,0(%r7)		# single CPU-stopper only
	jnz	4f
	larl	%r7,.Lthis_cpu
	stap	0(%r7)			# this CPU address
	lh	%r4,0(%r7)
	nilh	%r4,0
	lhi	%r0,1
	sll	%r0,16			# CPU counter
	lhi	%r3,0			# next CPU address
0:	cr	%r3,%r4
	je	2f
1:	sigp	%r1,%r3,SIGP_STOP	# stop next CPU
	brc	SIGP_CC_BUSY,1b
2:	ahi	%r3,1
	brct	%r0,0b
3:	sigp	%r1,%r4,SIGP_STOP	# stop this CPU
	brc	SIGP_CC_BUSY,3b
4:	j	4b
ENDPROC(mcck_int_handler)

ENTRY(restart_int_handler)
	ALTERNATIVE "", ".insn s,0xb2800000,_LPP_OFFSET", 40
	stg	%r15,__LC_SAVE_AREA_RESTART
	TSTMSK	__LC_RESTART_FLAGS,RESTART_FLAG_CTLREGS,4
	jz	0f
	la	%r15,4095
	lctlg	%c0,%c15,__LC_CREGS_SAVE_AREA-4095(%r15)
0:	larl	%r15,.Lstosm_tmp
	stosm	0(%r15),0x04			# turn dat on, keep irqs off
	lg	%r15,__LC_RESTART_STACK
	xc	STACK_FRAME_OVERHEAD(__PT_SIZE,%r15),STACK_FRAME_OVERHEAD(%r15)
	stmg	%r0,%r14,STACK_FRAME_OVERHEAD+__PT_R0(%r15)
	mvc	STACK_FRAME_OVERHEAD+__PT_R15(8,%r15),__LC_SAVE_AREA_RESTART
	mvc	STACK_FRAME_OVERHEAD+__PT_PSW(16,%r15),__LC_RST_OLD_PSW
	xc	0(STACK_FRAME_OVERHEAD,%r15),0(%r15)
	lg	%r1,__LC_RESTART_FN		# load fn, parm & source cpu
	lg	%r2,__LC_RESTART_DATA
	lgf	%r3,__LC_RESTART_SOURCE
	ltgr	%r3,%r3				# test source cpu address
	jm	1f				# negative -> skip source stop
0:	sigp	%r4,%r3,SIGP_SENSE		# sigp sense to source cpu
	brc	10,0b				# wait for status stored
1:	basr	%r14,%r1			# call function
	stap	__SF_EMPTY(%r15)		# store cpu address
	llgh	%r3,__SF_EMPTY(%r15)
2:	sigp	%r4,%r3,SIGP_STOP		# sigp stop to current cpu
	brc	2,2b
3:	j	3b
ENDPROC(restart_int_handler)

	.section .kprobes.text, "ax"

#if defined(CONFIG_CHECK_STACK) || defined(CONFIG_VMAP_STACK)
/*
 * The synchronous or the asynchronous stack overflowed. We are dead.
 * No need to properly save the registers, we are going to panic anyway.
 * Setup a pt_regs so that show_trace can provide a good call trace.
 */
ENTRY(stack_overflow)
	lg	%r15,__LC_NODAT_STACK	# change to panic stack
	la	%r11,STACK_FRAME_OVERHEAD(%r15)
	stmg	%r0,%r7,__PT_R0(%r11)
	stmg	%r8,%r9,__PT_PSW(%r11)
	mvc	__PT_R8(64,%r11),0(%r14)
	stg	%r10,__PT_ORIG_GPR2(%r11) # store last break to orig_gpr2
	xc	__SF_BACKCHAIN(8,%r15),__SF_BACKCHAIN(%r15)
	lgr	%r2,%r11		# pass pointer to pt_regs
	jg	kernel_stack_overflow
ENDPROC(stack_overflow)
#endif

#if IS_ENABLED(CONFIG_KVM)
.Lcleanup_sie:
	cghi	%r11,__LC_SAVE_AREA_ASYNC	#Is this in normal interrupt?
	je	1f
	larl	%r13,.Lsie_entry
	slgr	%r9,%r13
	lghi	%r13,.Lsie_skip - .Lsie_entry
	clgr	%r9,%r13
	jh	1f
	oi	__LC_CPU_FLAGS+7, _CIF_MCCK_GUEST
1:	BPENTER	__SF_SIE_FLAGS(%r15),(_TIF_ISOLATE_BP|_TIF_ISOLATE_BP_GUEST)
	lg	%r9,__SF_SIE_CONTROL(%r15)	# get control block pointer
	ni	__SIE_PROG0C+3(%r9),0xfe	# no longer in SIE
	lctlg	%c1,%c1,__LC_USER_ASCE		# load primary asce
	larl	%r9,sie_exit			# skip forward to sie_exit
	BR_EX	%r14,%r11

#endif
	.section .rodata, "a"
#define SYSCALL(esame,emu)	.quad __s390x_ ## esame
	.globl	sys_call_table
sys_call_table:
#include "asm/syscall_table.h"
#undef SYSCALL

#ifdef CONFIG_COMPAT

#define SYSCALL(esame,emu)	.quad __s390_ ## emu
	.globl	sys_call_table_emu
sys_call_table_emu:
#include "asm/syscall_table.h"
#undef SYSCALL
#endif
