/* SPDX-License-Identifier: GPL-2.0 */
/*
 * syscall_wrapper.h - x86 specific wrappers to syscall definitions
 */

#ifndef _ASM_X86_SYSCALL_WRAPPER_H
#define _ASM_X86_SYSCALL_WRAPPER_H

struct pt_regs;

/* Mapping of registers to parameters for syscalls on x86-64 and x32 */
#define SC_X86_64_REGS_TO_ARGS(x, ...)					\
	__MAP(x,__SC_ARGS						\
		,,regs->di,,regs->si,,regs->dx				\
		,,regs->r10,,regs->r8,,regs->r9)			\

/* Mapping of registers to parameters for syscalls on i386 */
#define SC_IA32_REGS_TO_ARGS(x, ...)					\
	__MAP(x,__SC_ARGS						\
	      ,,(unsigned int)regs->bx,,(unsigned int)regs->cx		\
	      ,,(unsigned int)regs->dx,,(unsigned int)regs->si		\
	      ,,(unsigned int)regs->di,,(unsigned int)regs->bp)

#define __SYS_STUB0(abi, name)						\
	long __##abi##_##name(const struct pt_regs *regs);		\
	ALLOW_ERROR_INJECTION(__##abi##_##name, ERRNO);			\
	long __##abi##_##name(const struct pt_regs *regs)		\
		__alias(__do_##name);

#define __SYS_STUBx(abi, name, ...)					\
	long __##abi##_##name(const struct pt_regs *regs);		\
	ALLOW_ERROR_INJECTION(__##abi##_##name, ERRNO);			\
	long __##abi##_##name(const struct pt_regs *regs)		\
	{								\
		return __se_##name(__VA_ARGS__);			\
	}

#define __COND_SYSCALL(abi, name)					\
	__weak long __##abi##_##name(const struct pt_regs *__unused)	\
	{								\
		return sys_ni_syscall();				\
	}

#define __SYS_NI(abi, name)						\
	SYSCALL_ALIAS(__##abi##_##name, sys_ni_posix_timers);

#ifdef CONFIG_X86_64
#define __X64_SYS_STUB0(name)						\
	__SYS_STUB0(x64, sys_##name)

#define __X64_SYS_STUBx(x, name, ...)					\
	__SYS_STUBx(x64, sys##name,					\
		    SC_X86_64_REGS_TO_ARGS(x, __VA_ARGS__))

#define __X64_COND_SYSCALL(name)					\
	__COND_SYSCALL(x64, sys_##name)

#define __X64_SYS_NI(name)						\
	__SYS_NI(x64, sys_##name)
#else /* CONFIG_X86_64 */
#define __X64_SYS_STUB0(name)
#define __X64_SYS_STUBx(x, name, ...)
#define __X64_COND_SYSCALL(name)
#define __X64_SYS_NI(name)
#endif /* CONFIG_X86_64 */

#if defined(CONFIG_X86_32) || defined(CONFIG_IA32_EMULATION)
#define __IA32_SYS_STUB0(name)						\
	__SYS_STUB0(ia32, sys_##name)

#define __IA32_SYS_STUBx(x, name, ...)					\
	__SYS_STUBx(ia32, sys##name,					\
		    SC_IA32_REGS_TO_ARGS(x, __VA_ARGS__))

#define __IA32_COND_SYSCALL(name)					\
	__COND_SYSCALL(ia32, sys_##name)

#define __IA32_SYS_NI(name)						\
	__SYS_NI(ia32, sys_##name)
#else /* CONFIG_X86_32 || CONFIG_IA32_EMULATION */
#define __IA32_SYS_STUB0(name)
#define __IA32_SYS_STUBx(x, name, ...)
#define __IA32_COND_SYSCALL(name)
#define __IA32_SYS_NI(name)
#endif /* CONFIG_X86_32 || CONFIG_IA32_EMULATION */

#ifdef CONFIG_IA32_EMULATION
/*
 * For IA32 emulation, we need to handle "compat" syscalls *and* create
 * additional wrappers (aptly named __ia32_sys_xyzzy) which decode the
 * ia32 regs in the proper order for shared or "common" syscalls. As some
 * syscalls may not be implemented, we need to expand COND_SYSCALL in
 * kernel/sys_ni.c and SYS_NI in kernel/time/posix-stubs.c to cover this
 * case as well.
 */
#define __IA32_COMPAT_SYS_STUB0(x, name)				\
	asmlinkage long __ia32_compat_sys_##name(const struct pt_regs *regs);\
	ALLOW_ERROR_INJECTION(__ia32_compat_sys_##name, ERRNO);		\
	asmlinkage long __ia32_compat_sys_##name(const struct pt_regs *regs)\
	{								\
		return __se_compat_sys_##name();			\
	}

#define __IA32_COMPAT_SYS_STUBx(x, name, ...)				\
	asmlinkage long __ia32_compat_sys##name(const struct pt_regs *regs);\
	ALLOW_ERROR_INJECTION(__ia32_compat_sys##name, ERRNO);		\
	asmlinkage long __ia32_compat_sys##name(const struct pt_regs *regs)\
	{								\
		return __se_compat_sys##name(SC_IA32_REGS_TO_ARGS(x,__VA_ARGS__));\
	}

#define __IA32_SYS_STUBx(x, name, ...)					\
	asmlinkage long __ia32_sys##name(const struct pt_regs *regs);	\
	ALLOW_ERROR_INJECTION(__ia32_sys##name, ERRNO);			\
	asmlinkage long __ia32_sys##name(const struct pt_regs *regs)	\
	{								\
		return __se_sys##name(SC_IA32_REGS_TO_ARGS(x,__VA_ARGS__));\
	}

/*
 * To keep the naming coherent, re-define SYSCALL_DEFINE0 to create an alias
 * named __ia32_sys_*()
 */

#define SYSCALL_DEFINE0(sname)						\
	SYSCALL_METADATA(_##sname, 0);					\
	asmlinkage long __x64_sys_##sname(const struct pt_regs *__unused);\
	ALLOW_ERROR_INJECTION(__x64_sys_##sname, ERRNO);		\
	SYSCALL_ALIAS(__ia32_sys_##sname, __x64_sys_##sname);		\
	asmlinkage long __x64_sys_##sname(const struct pt_regs *__unused)

#define COND_SYSCALL(name)							\
	asmlinkage __weak long __x64_sys_##name(const struct pt_regs *__unused)	\
	{									\
		return sys_ni_syscall();					\
	}									\
	asmlinkage __weak long __ia32_sys_##name(const struct pt_regs *__unused)\
	{									\
		return sys_ni_syscall();					\
	}

#define __IA32_COMPAT_SYS_NI(name)					\
	__SYS_NI(ia32, compat_sys_##name)

#else /* CONFIG_IA32_EMULATION */
#define __IA32_COMPAT_SYS_STUB0(name)
#define __IA32_COMPAT_SYS_STUBx(x, name, ...)
#define __IA32_COMPAT_COND_SYSCALL(name)
#define __IA32_COMPAT_SYS_NI(name)
#endif /* CONFIG_IA32_EMULATION */


#ifdef CONFIG_X86_X32
/*
 * For the x32 ABI, we need to create a stub for compat_sys_*() which is aware
 * of the x86-64-style parameter ordering of x32 syscalls. The syscalls common
 * with x86_64 obviously do not need such care.
 */
#define __X32_COMPAT_SYS_STUB0(x, name, ...)				\
	asmlinkage long __x32_compat_sys_##name(const struct pt_regs *regs);\
	ALLOW_ERROR_INJECTION(__x32_compat_sys_##name, ERRNO);		\
	asmlinkage long __x32_compat_sys_##name(const struct pt_regs *regs)\
	{								\
		return __se_compat_sys_##name();\
	}

#define __X32_COMPAT_SYS_STUBx(x, name, ...)				\
	asmlinkage long __x32_compat_sys##name(const struct pt_regs *regs);\
	ALLOW_ERROR_INJECTION(__x32_compat_sys##name, ERRNO);		\
	asmlinkage long __x32_compat_sys##name(const struct pt_regs *regs)\
	{								\
		return __se_compat_sys##name(SC_X86_64_REGS_TO_ARGS(x,__VA_ARGS__));\
	}

#define __X32_COMPAT_COND_SYSCALL(name)					\
	__COND_SYSCALL(x32, compat_sys_##name)

#define __X32_COMPAT_SYS_NI(name)					\
	__SYS_NI(x32, compat_sys_##name)
#else /* CONFIG_X86_X32 */
#define __X32_COMPAT_SYS_STUB0(x, name)
#define __X32_COMPAT_SYS_STUBx(x, name, ...)
#define __X32_COMPAT_COND_SYSCALL(name)
#define __X32_COMPAT_SYS_NI(name)
#endif /* CONFIG_X86_X32 */


#ifdef CONFIG_COMPAT
/*
 * Compat means IA32_EMULATION and/or X86_X32. As they use a different
 * mapping of registers to parameters, we need to generate stubs for each
 * of them.
 */
#define COMPAT_SYSCALL_DEFINE0(name)					\
	static long __se_compat_sys_##name(void);			\
	static inline long __do_compat_sys_##name(void);		\
	__IA32_COMPAT_SYS_STUB0(x, name)				\
	__X32_COMPAT_SYS_STUB0(x, name)					\
	static long __se_compat_sys_##name(void)			\
	{								\
		return __do_compat_sys_##name();			\
	}								\
	static inline long __do_compat_sys_##name(void)

#define COMPAT_SYSCALL_DEFINEx(x, name, ...)					\
	static long __se_compat_sys##name(__MAP(x,__SC_LONG,__VA_ARGS__));	\
	static inline long __do_compat_sys##name(__MAP(x,__SC_DECL,__VA_ARGS__));\
	__IA32_COMPAT_SYS_STUBx(x, name, __VA_ARGS__)				\
	__X32_COMPAT_SYS_STUBx(x, name, __VA_ARGS__)				\
	static long __se_compat_sys##name(__MAP(x,__SC_LONG,__VA_ARGS__))	\
	{									\
		return __do_compat_sys##name(__MAP(x,__SC_DELOUSE,__VA_ARGS__));\
	}									\
	static inline long __do_compat_sys##name(__MAP(x,__SC_DECL,__VA_ARGS__))

/*
 * As some compat syscalls may not be implemented, we need to expand
 * COND_SYSCALL_COMPAT in kernel/sys_ni.c and COMPAT_SYS_NI in
 * kernel/time/posix-stubs.c to cover this case as well.
 */
#define COND_SYSCALL_COMPAT(name) 					\
	__IA32_COMPAT_COND_SYSCALL(name)				\
	__X32_COMPAT_COND_SYSCALL(name)

#define COMPAT_SYS_NI(name)						\
	__IA32_COMPAT_SYS_NI(name)					\
	__X32_COMPAT_SYS_NI(name)

#endif /* CONFIG_COMPAT */

#define __SYSCALL_DEFINEx(x, name, ...)					\
	static long __se_sys##name(__MAP(x,__SC_LONG,__VA_ARGS__));	\
	static inline long __do_sys##name(__MAP(x,__SC_DECL,__VA_ARGS__));\
	__X64_SYS_STUBx(x, name, __VA_ARGS__)				\
	__IA32_SYS_STUBx(x, name, __VA_ARGS__)				\
	static long __se_sys##name(__MAP(x,__SC_LONG,__VA_ARGS__))	\
	{								\
		long ret = __do_sys##name(__MAP(x,__SC_CAST,__VA_ARGS__));\
		__MAP(x,__SC_TEST,__VA_ARGS__);				\
		__PROTECT(x, ret,__MAP(x,__SC_ARGS,__VA_ARGS__));	\
		return ret;						\
	}								\
	static inline long __do_sys##name(__MAP(x,__SC_DECL,__VA_ARGS__))

/*
 * As the generic SYSCALL_DEFINE0() macro does not decode any parameters for
 * obvious reasons, and passing struct pt_regs *regs to it in %rdi does not
 * hurt, we only need to re-define it here to keep the naming congruent to
 * SYSCALL_DEFINEx() -- which is essential for the COND_SYSCALL() and SYS_NI()
 * macros to work correctly.
 */
#ifndef SYSCALL_DEFINE0
#define SYSCALL_DEFINE0(sname)						\
	SYSCALL_METADATA(_##sname, 0);					\
	asmlinkage long __x64_sys_##sname(const struct pt_regs *__unused);\
	ALLOW_ERROR_INJECTION(__x64_sys_##sname, ERRNO);		\
	asmlinkage long __x64_sys_##sname(const struct pt_regs *__unused)
#endif

#ifndef COND_SYSCALL
#define COND_SYSCALL(name) 							\
	asmlinkage __weak long __x64_sys_##name(const struct pt_regs *__unused)	\
	{									\
		return sys_ni_syscall();					\
	}
#endif

#define SYS_NI(name)							\
	__X64_SYS_NI(name)						\
	__IA32_SYS_NI(name)


/*
 * For VSYSCALLS, we need to declare these three syscalls with the new
 * pt_regs-based calling convention for in-kernel use.
 */
asmlinkage long __x64_sys_getcpu(const struct pt_regs *regs);
asmlinkage long __x64_sys_gettimeofday(const struct pt_regs *regs);
asmlinkage long __x64_sys_time(const struct pt_regs *regs);

#endif /* _ASM_X86_SYSCALL_WRAPPER_H */
