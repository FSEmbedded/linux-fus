// SPDX-License-Identifier: MIT
/*
 * Copyright © 2014 Intel Corporation
 */

#include "gem/i915_gem_lmem.h"

#include "gen8_engine_cs.h"
#include "i915_drv.h"
#include "i915_perf.h"
#include "intel_engine.h"
#include "intel_gpu_commands.h"
#include "intel_gt.h"
#include "intel_lrc.h"
#include "intel_lrc_reg.h"
#include "intel_ring.h"
#include "shmem_utils.h"

static void set_offsets(u32 *regs,
			const u8 *data,
			const struct intel_engine_cs *engine,
			bool close)
#define NOP(x) (BIT(7) | (x))
#define LRI(count, flags) ((flags) << 6 | (count) | BUILD_BUG_ON_ZERO(count >= BIT(6)))
#define POSTED BIT(0)
#define REG(x) (((x) >> 2) | BUILD_BUG_ON_ZERO(x >= 0x200))
#define REG16(x) \
	(((x) >> 9) | BIT(7) | BUILD_BUG_ON_ZERO(x >= 0x10000)), \
	(((x) >> 2) & 0x7f)
#define END 0
{
	const u32 base = engine->mmio_base;

	while (*data) {
		u8 count, flags;

		if (*data & BIT(7)) { /* skip */
			count = *data++ & ~BIT(7);
			regs += count;
			continue;
		}

		count = *data & 0x3f;
		flags = *data >> 6;
		data++;

		*regs = MI_LOAD_REGISTER_IMM(count);
		if (flags & POSTED)
			*regs |= MI_LRI_FORCE_POSTED;
		if (GRAPHICS_VER(engine->i915) >= 11)
			*regs |= MI_LRI_LRM_CS_MMIO;
		regs++;

		GEM_BUG_ON(!count);
		do {
			u32 offset = 0;
			u8 v;

			do {
				v = *data++;
				offset <<= 7;
				offset |= v & ~BIT(7);
			} while (v & BIT(7));

			regs[0] = base + (offset << 2);
			regs += 2;
		} while (--count);
	}

	if (close) {
		/* Close the batch; used mainly by live_lrc_layout() */
		*regs = MI_BATCH_BUFFER_END;
		if (GRAPHICS_VER(engine->i915) >= 11)
			*regs |= BIT(0);
	}
}

static const u8 gen8_xcs_offsets[] = {
	NOP(1),
	LRI(11, 0),
	REG16(0x244),
	REG(0x034),
	REG(0x030),
	REG(0x038),
	REG(0x03c),
	REG(0x168),
	REG(0x140),
	REG(0x110),
	REG(0x11c),
	REG(0x114),
	REG(0x118),

	NOP(9),
	LRI(9, 0),
	REG16(0x3a8),
	REG16(0x28c),
	REG16(0x288),
	REG16(0x284),
	REG16(0x280),
	REG16(0x27c),
	REG16(0x278),
	REG16(0x274),
	REG16(0x270),

	NOP(13),
	LRI(2, 0),
	REG16(0x200),
	REG(0x028),

	END
};

static const u8 gen9_xcs_offsets[] = {
	NOP(1),
	LRI(14, POSTED),
	REG16(0x244),
	REG(0x034),
	REG(0x030),
	REG(0x038),
	REG(0x03c),
	REG(0x168),
	REG(0x140),
	REG(0x110),
	REG(0x11c),
	REG(0x114),
	REG(0x118),
	REG(0x1c0),
	REG(0x1c4),
	REG(0x1c8),

	NOP(3),
	LRI(9, POSTED),
	REG16(0x3a8),
	REG16(0x28c),
	REG16(0x288),
	REG16(0x284),
	REG16(0x280),
	REG16(0x27c),
	REG16(0x278),
	REG16(0x274),
	REG16(0x270),

	NOP(13),
	LRI(1, POSTED),
	REG16(0x200),

	NOP(13),
	LRI(44, POSTED),
	REG(0x028),
	REG(0x09c),
	REG(0x0c0),
	REG(0x178),
	REG(0x17c),
	REG16(0x358),
	REG(0x170),
	REG(0x150),
	REG(0x154),
	REG(0x158),
	REG16(0x41c),
	REG16(0x600),
	REG16(0x604),
	REG16(0x608),
	REG16(0x60c),
	REG16(0x610),
	REG16(0x614),
	REG16(0x618),
	REG16(0x61c),
	REG16(0x620),
	REG16(0x624),
	REG16(0x628),
	REG16(0x62c),
	REG16(0x630),
	REG16(0x634),
	REG16(0x638),
	REG16(0x63c),
	REG16(0x640),
	REG16(0x644),
	REG16(0x648),
	REG16(0x64c),
	REG16(0x650),
	REG16(0x654),
	REG16(0x658),
	REG16(0x65c),
	REG16(0x660),
	REG16(0x664),
	REG16(0x668),
	REG16(0x66c),
	REG16(0x670),
	REG16(0x674),
	REG16(0x678),
	REG16(0x67c),
	REG(0x068),

	END
};

static const u8 gen12_xcs_offsets[] = {
	NOP(1),
	LRI(13, POSTED),
	REG16(0x244),
	REG(0x034),
	REG(0x030),
	REG(0x038),
	REG(0x03c),
	REG(0x168),
	REG(0x140),
	REG(0x110),
	REG(0x1c0),
	REG(0x1c4),
	REG(0x1c8),
	REG(0x180),
	REG16(0x2b4),

	NOP(5),
	LRI(9, POSTED),
	REG16(0x3a8),
	REG16(0x28c),
	REG16(0x288),
	REG16(0x284),
	REG16(0x280),
	REG16(0x27c),
	REG16(0x278),
	REG16(0x274),
	REG16(0x270),

	END
};

static const u8 gen8_rcs_offsets[] = {
	NOP(1),
	LRI(14, POSTED),
	REG16(0x244),
	REG(0x034),
	REG(0x030),
	REG(0x038),
	REG(0x03c),
	REG(0x168),
	REG(0x140),
	REG(0x110),
	REG(0x11c),
	REG(0x114),
	REG(0x118),
	REG(0x1c0),
	REG(0x1c4),
	REG(0x1c8),

	NOP(3),
	LRI(9, POSTED),
	REG16(0x3a8),
	REG16(0x28c),
	REG16(0x288),
	REG16(0x284),
	REG16(0x280),
	REG16(0x27c),
	REG16(0x278),
	REG16(0x274),
	REG16(0x270),

	NOP(13),
	LRI(1, 0),
	REG(0x0c8),

	END
};

static const u8 gen9_rcs_offsets[] = {
	NOP(1),
	LRI(14, POSTED),
	REG16(0x244),
	REG(0x34),
	REG(0x30),
	REG(0x38),
	REG(0x3c),
	REG(0x168),
	REG(0x140),
	REG(0x110),
	REG(0x11c),
	REG(0x114),
	REG(0x118),
	REG(0x1c0),
	REG(0x1c4),
	REG(0x1c8),

	NOP(3),
	LRI(9, POSTED),
	REG16(0x3a8),
	REG16(0x28c),
	REG16(0x288),
	REG16(0x284),
	REG16(0x280),
	REG16(0x27c),
	REG16(0x278),
	REG16(0x274),
	REG16(0x270),

	NOP(13),
	LRI(1, 0),
	REG(0xc8),

	NOP(13),
	LRI(44, POSTED),
	REG(0x28),
	REG(0x9c),
	REG(0xc0),
	REG(0x178),
	REG(0x17c),
	REG16(0x358),
	REG(0x170),
	REG(0x150),
	REG(0x154),
	REG(0x158),
	REG16(0x41c),
	REG16(0x600),
	REG16(0x604),
	REG16(0x608),
	REG16(0x60c),
	REG16(0x610),
	REG16(0x614),
	REG16(0x618),
	REG16(0x61c),
	REG16(0x620),
	REG16(0x624),
	REG16(0x628),
	REG16(0x62c),
	REG16(0x630),
	REG16(0x634),
	REG16(0x638),
	REG16(0x63c),
	REG16(0x640),
	REG16(0x644),
	REG16(0x648),
	REG16(0x64c),
	REG16(0x650),
	REG16(0x654),
	REG16(0x658),
	REG16(0x65c),
	REG16(0x660),
	REG16(0x664),
	REG16(0x668),
	REG16(0x66c),
	REG16(0x670),
	REG16(0x674),
	REG16(0x678),
	REG16(0x67c),
	REG(0x68),

	END
};

static const u8 gen11_rcs_offsets[] = {
	NOP(1),
	LRI(15, POSTED),
	REG16(0x244),
	REG(0x034),
	REG(0x030),
	REG(0x038),
	REG(0x03c),
	REG(0x168),
	REG(0x140),
	REG(0x110),
	REG(0x11c),
	REG(0x114),
	REG(0x118),
	REG(0x1c0),
	REG(0x1c4),
	REG(0x1c8),
	REG(0x180),

	NOP(1),
	LRI(9, POSTED),
	REG16(0x3a8),
	REG16(0x28c),
	REG16(0x288),
	REG16(0x284),
	REG16(0x280),
	REG16(0x27c),
	REG16(0x278),
	REG16(0x274),
	REG16(0x270),

	LRI(1, POSTED),
	REG(0x1b0),

	NOP(10),
	LRI(1, 0),
	REG(0x0c8),

	END(80)
};

static const u8 gen12_rcs_offsets[] = {
	NOP(1),
	LRI(13, POSTED),
	REG16(0x244),
	REG(0x034),
	REG(0x030),
	REG(0x038),
	REG(0x03c),
	REG(0x168),
	REG(0x140),
	REG(0x110),
	REG(0x1c0),
	REG(0x1c4),
	REG(0x1c8),
	REG(0x180),
	REG16(0x2b4),

	NOP(5),
	LRI(9, POSTED),
	REG16(0x3a8),
	REG16(0x28c),
	REG16(0x288),
	REG16(0x284),
	REG16(0x280),
	REG16(0x27c),
	REG16(0x278),
	REG16(0x274),
	REG16(0x270),

	LRI(3, POSTED),
	REG(0x1b0),
	REG16(0x5a8),
	REG16(0x5ac),

	NOP(6),
	LRI(1, 0),
	REG(0x0c8),
	NOP(3 + 9 + 1),

	LRI(51, POSTED),
	REG16(0x588),
	REG16(0x588),
	REG16(0x588),
	REG16(0x588),
	REG16(0x588),
	REG16(0x588),
	REG(0x028),
	REG(0x09c),
	REG(0x0c0),
	REG(0x178),
	REG(0x17c),
	REG16(0x358),
	REG(0x170),
	REG(0x150),
	REG(0x154),
	REG(0x158),
	REG16(0x41c),
	REG16(0x600),
	REG16(0x604),
	REG16(0x608),
	REG16(0x60c),
	REG16(0x610),
	REG16(0x614),
	REG16(0x618),
	REG16(0x61c),
	REG16(0x620),
	REG16(0x624),
	REG16(0x628),
	REG16(0x62c),
	REG16(0x630),
	REG16(0x634),
	REG16(0x638),
	REG16(0x63c),
	REG16(0x640),
	REG16(0x644),
	REG16(0x648),
	REG16(0x64c),
	REG16(0x650),
	REG16(0x654),
	REG16(0x658),
	REG16(0x65c),
	REG16(0x660),
	REG16(0x664),
	REG16(0x668),
	REG16(0x66c),
	REG16(0x670),
	REG16(0x674),
	REG16(0x678),
	REG16(0x67c),
	REG(0x068),
	REG(0x084),
	NOP(1),

	END(192)
};

#undef END
#undef REG16
#undef REG
#undef LRI
#undef NOP

static const u8 *reg_offsets(const struct intel_engine_cs *engine)
{
	/*
	 * The gen12+ lists only have the registers we program in the basic
	 * default state. We rely on the context image using relative
	 * addressing to automatic fixup the register state between the
	 * physical engines for virtual engine.
	 */
	GEM_BUG_ON(INTEL_GEN(engine->i915) >= 12 &&
		   !intel_engine_has_relative_mmio(engine));

	if (engine->class == RENDER_CLASS) {
		if (INTEL_GEN(engine->i915) >= 12)
			return gen12_rcs_offsets;
		else if (INTEL_GEN(engine->i915) >= 11)
			return gen11_rcs_offsets;
		else if (INTEL_GEN(engine->i915) >= 9)
			return gen9_rcs_offsets;
		else
			return gen8_rcs_offsets;
	} else {
		if (INTEL_GEN(engine->i915) >= 12)
			return gen12_xcs_offsets;
		else if (INTEL_GEN(engine->i915) >= 9)
			return gen9_xcs_offsets;
		else
			return gen8_xcs_offsets;
	}
}

static struct i915_request *
__unwind_incomplete_requests(struct intel_engine_cs *engine)
{
	struct i915_request *rq, *rn, *active = NULL;
	struct list_head *pl;
	int prio = I915_PRIORITY_INVALID;

	lockdep_assert_held(&engine->active.lock);

	list_for_each_entry_safe_reverse(rq, rn,
					 &engine->active.requests,
					 sched.link) {
		if (i915_request_completed(rq))
			continue; /* XXX */

		__i915_request_unsubmit(rq);

		/*
		 * Push the request back into the queue for later resubmission.
		 * If this request is not native to this physical engine (i.e.
		 * it came from a virtual source), push it back onto the virtual
		 * engine so that it can be moved across onto another physical
		 * engine as load dictates.
		 */
		if (likely(rq->execution_mask == engine->mask)) {
			GEM_BUG_ON(rq_prio(rq) == I915_PRIORITY_INVALID);
			if (rq_prio(rq) != prio) {
				prio = rq_prio(rq);
				pl = i915_sched_lookup_priolist(engine, prio);
			}
			GEM_BUG_ON(RB_EMPTY_ROOT(&engine->execlists.queue.rb_root));

			list_move(&rq->sched.link, pl);
			set_bit(I915_FENCE_FLAG_PQUEUE, &rq->fence.flags);

			/* Check in case we rollback so far we wrap [size/2] */
			if (intel_ring_direction(rq->ring,
						 rq->tail,
						 rq->ring->tail + 8) > 0)
				rq->context->lrc.desc |= CTX_DESC_FORCE_RESTORE;

			active = rq;
		} else {
			struct intel_engine_cs *owner = rq->context->engine;

			WRITE_ONCE(rq->engine, owner);
			owner->submit_request(rq);
			active = NULL;
		}
	}

	return active;
}

struct i915_request *
execlists_unwind_incomplete_requests(struct intel_engine_execlists *execlists)
{
	struct intel_engine_cs *engine =
		container_of(execlists, typeof(*engine), execlists);

	return __unwind_incomplete_requests(engine);
}

static inline void
execlists_context_status_change(struct i915_request *rq, unsigned long status)
{
	/*
	 * Only used when GVT-g is enabled now. When GVT-g is disabled,
	 * The compiler should eliminate this function as dead-code.
	 */
	if (!IS_ENABLED(CONFIG_DRM_I915_GVT))
		return;

	atomic_notifier_call_chain(&rq->engine->context_status_notifier,
				   status, rq);
}

static void intel_engine_context_in(struct intel_engine_cs *engine)
{
	unsigned long flags;

	if (atomic_add_unless(&engine->stats.active, 1, 0))
		return;

	write_seqlock_irqsave(&engine->stats.lock, flags);
	if (!atomic_add_unless(&engine->stats.active, 1, 0)) {
		engine->stats.start = ktime_get();
		atomic_inc(&engine->stats.active);
	}
	write_sequnlock_irqrestore(&engine->stats.lock, flags);
}

static void intel_engine_context_out(struct intel_engine_cs *engine)
{
	unsigned long flags;

	GEM_BUG_ON(!atomic_read(&engine->stats.active));

	if (atomic_add_unless(&engine->stats.active, -1, 1))
		return;

	write_seqlock_irqsave(&engine->stats.lock, flags);
	if (atomic_dec_and_test(&engine->stats.active)) {
		engine->stats.total =
			ktime_add(engine->stats.total,
				  ktime_sub(ktime_get(), engine->stats.start));
	}
	write_sequnlock_irqrestore(&engine->stats.lock, flags);
}

static void
execlists_check_context(const struct intel_context *ce,
			const struct intel_engine_cs *engine)
{
	const struct intel_ring *ring = ce->ring;
	u32 *regs = ce->lrc_reg_state;
	bool valid = true;
	int x;

	if (regs[CTX_RING_START] != i915_ggtt_offset(ring->vma)) {
		pr_err("%s: context submitted with incorrect RING_START [%08x], expected %08x\n",
		       engine->name,
		       regs[CTX_RING_START],
		       i915_ggtt_offset(ring->vma));
		regs[CTX_RING_START] = i915_ggtt_offset(ring->vma);
		valid = false;
	}

	if ((regs[CTX_RING_CTL] & ~(RING_WAIT | RING_WAIT_SEMAPHORE)) !=
	    (RING_CTL_SIZE(ring->size) | RING_VALID)) {
		pr_err("%s: context submitted with incorrect RING_CTL [%08x], expected %08x\n",
		       engine->name,
		       regs[CTX_RING_CTL],
		       (u32)(RING_CTL_SIZE(ring->size) | RING_VALID));
		regs[CTX_RING_CTL] = RING_CTL_SIZE(ring->size) | RING_VALID;
		valid = false;
	}

	x = lrc_ring_mi_mode(engine);
	if (x != -1 && regs[x + 1] & (regs[x + 1] >> 16) & STOP_RING) {
		pr_err("%s: context submitted with STOP_RING [%08x] in RING_MI_MODE\n",
		       engine->name, regs[x + 1]);
		regs[x + 1] &= ~STOP_RING;
		regs[x + 1] |= STOP_RING << 16;
		valid = false;
	}

	WARN_ONCE(!valid, "Invalid lrc state found before submission\n");
}

static void restore_default_state(struct intel_context *ce,
				  struct intel_engine_cs *engine)
{
	u32 *regs;

	regs = memset(ce->lrc_reg_state, 0, engine->context_size - PAGE_SIZE);
	execlists_init_reg_state(regs, ce, engine, ce->ring, true);

	ce->runtime.last = intel_context_get_runtime(ce);
}

static void reset_active(struct i915_request *rq,
			 struct intel_engine_cs *engine)
{
	struct intel_context * const ce = rq->context;
	u32 head;

	/*
	 * The executing context has been cancelled. We want to prevent
	 * further execution along this context and propagate the error on
	 * to anything depending on its results.
	 *
	 * In __i915_request_submit(), we apply the -EIO and remove the
	 * requests' payloads for any banned requests. But first, we must
	 * rewind the context back to the start of the incomplete request so
	 * that we do not jump back into the middle of the batch.
	 *
	 * We preserve the breadcrumbs and semaphores of the incomplete
	 * requests so that inter-timeline dependencies (i.e other timelines)
	 * remain correctly ordered. And we defer to __i915_request_submit()
	 * so that all asynchronous waits are correctly handled.
	 */
	ENGINE_TRACE(engine, "{ rq=%llx:%lld }\n",
		     rq->fence.context, rq->fence.seqno);

	/* On resubmission of the active request, payload will be scrubbed */
	if (i915_request_completed(rq))
		head = rq->tail;
	else
		head = active_request(ce->timeline, rq)->head;
	head = intel_ring_wrap(ce->ring, head);

	/* Scrub the context image to prevent replaying the previous batch */
	restore_default_state(ce, engine);
	__execlists_update_reg_state(ce, engine, head);

	/* We've switched away, so this should be a no-op, but intent matters */
	ce->lrc.desc |= CTX_DESC_FORCE_RESTORE;
}

static void st_update_runtime_underflow(struct intel_context *ce, s32 dt)
{
#if IS_ENABLED(CONFIG_DRM_I915_SELFTEST)
	ce->runtime.num_underflow += dt < 0;
	ce->runtime.max_underflow = max_t(u32, ce->runtime.max_underflow, -dt);
#endif
}

static void intel_context_update_runtime(struct intel_context *ce)
{
	u32 old;
	s32 dt;

	if (intel_context_is_barrier(ce))
		return;

	old = ce->runtime.last;
	ce->runtime.last = intel_context_get_runtime(ce);
	dt = ce->runtime.last - old;

	if (unlikely(dt <= 0)) {
		CE_TRACE(ce, "runtime underflow: last=%u, new=%u, delta=%d\n",
			 old, ce->runtime.last, dt);
		st_update_runtime_underflow(ce, dt);
		return;
	}

	ewma_runtime_add(&ce->runtime.avg, dt);
	ce->runtime.total += dt;
}

static inline struct intel_engine_cs *
__execlists_schedule_in(struct i915_request *rq)
{
	struct intel_engine_cs * const engine = rq->engine;
	struct intel_context * const ce = rq->context;

	intel_context_get(ce);

	if (unlikely(intel_context_is_banned(ce)))
		reset_active(rq, engine);

	if (IS_ENABLED(CONFIG_DRM_I915_DEBUG_GEM))
		execlists_check_context(ce, engine);

	if (ce->tag) {
		/* Use a fixed tag for OA and friends */
		GEM_BUG_ON(ce->tag <= BITS_PER_LONG);
		ce->lrc.ccid = ce->tag;
	} else {
		/* We don't need a strict matching tag, just different values */
		unsigned int tag = ffs(READ_ONCE(engine->context_tag));

		GEM_BUG_ON(tag == 0 || tag >= BITS_PER_LONG);
		clear_bit(tag - 1, &engine->context_tag);
		ce->lrc.ccid = tag << (GEN11_SW_CTX_ID_SHIFT - 32);

		BUILD_BUG_ON(BITS_PER_LONG > GEN12_MAX_CONTEXT_HW_ID);
	}

	ce->lrc.ccid |= engine->execlists.ccid;

	__intel_gt_pm_get(engine->gt);
	if (engine->fw_domain && !atomic_fetch_inc(&engine->fw_active))
		intel_uncore_forcewake_get(engine->uncore, engine->fw_domain);
	execlists_context_status_change(rq, INTEL_CONTEXT_SCHEDULE_IN);
	intel_engine_context_in(engine);

	return engine;
}

static inline struct i915_request *
execlists_schedule_in(struct i915_request *rq, int idx)
{
	struct intel_context * const ce = rq->context;
	struct intel_engine_cs *old;

	GEM_BUG_ON(!intel_engine_pm_is_awake(rq->engine));
	trace_i915_request_in(rq, idx);

	old = READ_ONCE(ce->inflight);
	do {
		if (!old) {
			WRITE_ONCE(ce->inflight, __execlists_schedule_in(rq));
			break;
		}
	} while (!try_cmpxchg(&ce->inflight, &old, ptr_inc(old)));

	GEM_BUG_ON(intel_context_inflight(ce) != rq->engine);
	return i915_request_get(rq);
}

static void kick_siblings(struct i915_request *rq, struct intel_context *ce)
{
	struct virtual_engine *ve = container_of(ce, typeof(*ve), context);
	struct i915_request *next = READ_ONCE(ve->request);

	if (next == rq || (next && next->execution_mask & ~rq->execution_mask))
		tasklet_hi_schedule(&ve->base.execlists.tasklet);
}

static inline void
__execlists_schedule_out(struct i915_request *rq,
			 struct intel_engine_cs * const engine,
			 unsigned int ccid)
{
	struct intel_context * const ce = rq->context;

	/*
	 * NB process_csb() is not under the engine->active.lock and hence
	 * schedule_out can race with schedule_in meaning that we should
	 * refrain from doing non-trivial work here.
	 */

	/*
	 * If we have just completed this context, the engine may now be
	 * idle and we want to re-enter powersaving.
	 */
	if (list_is_last_rcu(&rq->link, &ce->timeline->requests) &&
	    i915_request_completed(rq))
		intel_engine_add_retire(engine, ce->timeline);

	ccid >>= GEN11_SW_CTX_ID_SHIFT - 32;
	ccid &= GEN12_MAX_CONTEXT_HW_ID;
	if (ccid < BITS_PER_LONG) {
		GEM_BUG_ON(ccid == 0);
		GEM_BUG_ON(test_bit(ccid - 1, &engine->context_tag));
		set_bit(ccid - 1, &engine->context_tag);
	}

	intel_context_update_runtime(ce);
	intel_engine_context_out(engine);
	execlists_context_status_change(rq, INTEL_CONTEXT_SCHEDULE_OUT);
	if (engine->fw_domain && !atomic_dec_return(&engine->fw_active))
		intel_uncore_forcewake_put(engine->uncore, engine->fw_domain);
	intel_gt_pm_put_async(engine->gt);

	/*
	 * If this is part of a virtual engine, its next request may
	 * have been blocked waiting for access to the active context.
	 * We have to kick all the siblings again in case we need to
	 * switch (e.g. the next request is not runnable on this
	 * engine). Hopefully, we will already have submitted the next
	 * request before the tasklet runs and do not need to rebuild
	 * each virtual tree and kick everyone again.
	 */
	if (ce->engine != engine)
		kick_siblings(rq, ce);

	intel_context_put(ce);
}

static inline void
execlists_schedule_out(struct i915_request *rq)
{
	struct intel_context * const ce = rq->context;
	struct intel_engine_cs *cur, *old;
	u32 ccid;

	trace_i915_request_out(rq);

	ccid = rq->context->lrc.ccid;
	old = READ_ONCE(ce->inflight);
	do
		cur = ptr_unmask_bits(old, 2) ? ptr_dec(old) : NULL;
	while (!try_cmpxchg(&ce->inflight, &old, cur));
	if (!cur)
		__execlists_schedule_out(rq, old, ccid);

	i915_request_put(rq);
}

static u64 execlists_update_context(struct i915_request *rq)
{
	struct intel_context *ce = rq->context;
	u64 desc = ce->lrc.desc;
	u32 tail, prev;

	/*
	 * WaIdleLiteRestore:bdw,skl
	 *
	 * We should never submit the context with the same RING_TAIL twice
	 * just in case we submit an empty ring, which confuses the HW.
	 *
	 * We append a couple of NOOPs (gen8_emit_wa_tail) after the end of
	 * the normal request to be able to always advance the RING_TAIL on
	 * subsequent resubmissions (for lite restore). Should that fail us,
	 * and we try and submit the same tail again, force the context
	 * reload.
	 *
	 * If we need to return to a preempted context, we need to skip the
	 * lite-restore and force it to reload the RING_TAIL. Otherwise, the
	 * HW has a tendency to ignore us rewinding the TAIL to the end of
	 * an earlier request.
	 */
	GEM_BUG_ON(ce->lrc_reg_state[CTX_RING_TAIL] != rq->ring->tail);
	prev = rq->ring->tail;
	tail = intel_ring_set_tail(rq->ring, rq->tail);
	if (unlikely(intel_ring_direction(rq->ring, tail, prev) <= 0))
		desc |= CTX_DESC_FORCE_RESTORE;
	ce->lrc_reg_state[CTX_RING_TAIL] = tail;
	rq->tail = rq->wa_tail;

	/*
	 * Make sure the context image is complete before we submit it to HW.
	 *
	 * Ostensibly, writes (including the WCB) should be flushed prior to
	 * an uncached write such as our mmio register access, the empirical
	 * evidence (esp. on Braswell) suggests that the WC write into memory
	 * may not be visible to the HW prior to the completion of the UC
	 * register write and that we may begin execution from the context
	 * before its image is complete leading to invalid PD chasing.
	 */
	wmb();

	ce->lrc.desc &= ~CTX_DESC_FORCE_RESTORE;
	return desc;
}

static inline void write_desc(struct intel_engine_execlists *execlists, u64 desc, u32 port)
{
	if (execlists->ctrl_reg) {
		writel(lower_32_bits(desc), execlists->submit_reg + port * 2);
		writel(upper_32_bits(desc), execlists->submit_reg + port * 2 + 1);
	} else {
		writel(upper_32_bits(desc), execlists->submit_reg);
		writel(lower_32_bits(desc), execlists->submit_reg);
	}
}

static __maybe_unused char *
dump_port(char *buf, int buflen, const char *prefix, struct i915_request *rq)
{
	if (!rq)
		return "";

	snprintf(buf, buflen, "%sccid:%x %llx:%lld%s prio %d",
		 prefix,
		 rq->context->lrc.ccid,
		 rq->fence.context, rq->fence.seqno,
		 i915_request_completed(rq) ? "!" :
		 i915_request_started(rq) ? "*" :
		 "",
		 rq_prio(rq));

	return buf;
}

static __maybe_unused void
trace_ports(const struct intel_engine_execlists *execlists,
	    const char *msg,
	    struct i915_request * const *ports)
{
	const struct intel_engine_cs *engine =
		container_of(execlists, typeof(*engine), execlists);
	char __maybe_unused p0[40], p1[40];

	if (!ports[0])
		return;

	ENGINE_TRACE(engine, "%s { %s%s }\n", msg,
		     dump_port(p0, sizeof(p0), "", ports[0]),
		     dump_port(p1, sizeof(p1), ", ", ports[1]));
}

static inline bool
reset_in_progress(const struct intel_engine_execlists *execlists)
{
	return unlikely(!__tasklet_is_enabled(&execlists->tasklet));
}

static __maybe_unused bool
assert_pending_valid(const struct intel_engine_execlists *execlists,
		     const char *msg)
{
	struct intel_engine_cs *engine =
		container_of(execlists, typeof(*engine), execlists);
	struct i915_request * const *port, *rq;
	struct intel_context *ce = NULL;
	bool sentinel = false;
	u32 ccid = -1;

	trace_ports(execlists, msg, execlists->pending);

	/* We may be messing around with the lists during reset, lalala */
	if (reset_in_progress(execlists))
		return true;

	if (!execlists->pending[0]) {
		GEM_TRACE_ERR("%s: Nothing pending for promotion!\n",
			      engine->name);
		return false;
	}

	if (execlists->pending[execlists_num_ports(execlists)]) {
		GEM_TRACE_ERR("%s: Excess pending[%d] for promotion!\n",
			      engine->name, execlists_num_ports(execlists));
		return false;
	}

	for (port = execlists->pending; (rq = *port); port++) {
		unsigned long flags;
		bool ok = true;

		GEM_BUG_ON(!kref_read(&rq->fence.refcount));
		GEM_BUG_ON(!i915_request_is_active(rq));

		if (ce == rq->context) {
			GEM_TRACE_ERR("%s: Dup context:%llx in pending[%zd]\n",
				      engine->name,
				      ce->timeline->fence_context,
				      port - execlists->pending);
			return false;
		}
		ce = rq->context;

		if (ccid == ce->lrc.ccid) {
			GEM_TRACE_ERR("%s: Dup ccid:%x context:%llx in pending[%zd]\n",
				      engine->name,
				      ccid, ce->timeline->fence_context,
				      port - execlists->pending);
			return false;
		}
		ccid = ce->lrc.ccid;

		/*
		 * Sentinels are supposed to be the last request so they flush
		 * the current execution off the HW. Check that they are the only
		 * request in the pending submission.
		 */
		if (sentinel) {
			GEM_TRACE_ERR("%s: context:%llx after sentinel in pending[%zd]\n",
				      engine->name,
				      ce->timeline->fence_context,
				      port - execlists->pending);
			return false;
		}
		sentinel = i915_request_has_sentinel(rq);

		/* Hold tightly onto the lock to prevent concurrent retires! */
		if (!spin_trylock_irqsave(&rq->lock, flags))
			continue;

		if (i915_request_completed(rq))
			goto unlock;

		if (i915_active_is_idle(&ce->active) &&
		    !intel_context_is_barrier(ce)) {
			GEM_TRACE_ERR("%s: Inactive context:%llx in pending[%zd]\n",
				      engine->name,
				      ce->timeline->fence_context,
				      port - execlists->pending);
			ok = false;
			goto unlock;
		}

		if (!i915_vma_is_pinned(ce->state)) {
			GEM_TRACE_ERR("%s: Unpinned context:%llx in pending[%zd]\n",
				      engine->name,
				      ce->timeline->fence_context,
				      port - execlists->pending);
			ok = false;
			goto unlock;
		}

		if (!i915_vma_is_pinned(ce->ring->vma)) {
			GEM_TRACE_ERR("%s: Unpinned ring:%llx in pending[%zd]\n",
				      engine->name,
				      ce->timeline->fence_context,
				      port - execlists->pending);
			ok = false;
			goto unlock;
		}

unlock:
		spin_unlock_irqrestore(&rq->lock, flags);
		if (!ok)
			return false;
	}

	return ce;
}

static void execlists_submit_ports(struct intel_engine_cs *engine)
{
	struct intel_engine_execlists *execlists = &engine->execlists;
	unsigned int n;

	GEM_BUG_ON(!assert_pending_valid(execlists, "submit"));

	/*
	 * We can skip acquiring intel_runtime_pm_get() here as it was taken
	 * on our behalf by the request (see i915_gem_mark_busy()) and it will
	 * not be relinquished until the device is idle (see
	 * i915_gem_idle_work_handler()). As a precaution, we make sure
	 * that all ELSP are drained i.e. we have processed the CSB,
	 * before allowing ourselves to idle and calling intel_runtime_pm_put().
	 */
	GEM_BUG_ON(!intel_engine_pm_is_awake(engine));

	/*
	 * ELSQ note: the submit queue is not cleared after being submitted
	 * to the HW so we need to make sure we always clean it up. This is
	 * currently ensured by the fact that we always write the same number
	 * of elsq entries, keep this in mind before changing the loop below.
	 */
	for (n = execlists_num_ports(execlists); n--; ) {
		struct i915_request *rq = execlists->pending[n];

		write_desc(execlists,
			   rq ? execlists_update_context(rq) : 0,
			   n);
	}

	/* we need to manually load the submit queue */
	if (execlists->ctrl_reg)
		writel(EL_CTRL_LOAD, execlists->ctrl_reg);
}

static bool ctx_single_port_submission(const struct intel_context *ce)
{
	return (IS_ENABLED(CONFIG_DRM_I915_GVT) &&
		intel_context_force_single_submission(ce));
}

static bool can_merge_ctx(const struct intel_context *prev,
			  const struct intel_context *next)
{
	if (prev != next)
		return false;

	if (ctx_single_port_submission(prev))
		return false;

	return true;
}

static unsigned long i915_request_flags(const struct i915_request *rq)
{
	return READ_ONCE(rq->fence.flags);
}

static bool can_merge_rq(const struct i915_request *prev,
			 const struct i915_request *next)
{
	GEM_BUG_ON(prev == next);
	GEM_BUG_ON(!assert_priority_queue(prev, next));

	/*
	 * We do not submit known completed requests. Therefore if the next
	 * request is already completed, we can pretend to merge it in
	 * with the previous context (and we will skip updating the ELSP
	 * and tracking). Thus hopefully keeping the ELSP full with active
	 * contexts, despite the best efforts of preempt-to-busy to confuse
	 * us.
	 */
	if (i915_request_completed(next))
		return true;

	if (unlikely((i915_request_flags(prev) ^ i915_request_flags(next)) &
		     (BIT(I915_FENCE_FLAG_NOPREEMPT) |
		      BIT(I915_FENCE_FLAG_SENTINEL))))
		return false;

	if (!can_merge_ctx(prev->context, next->context))
		return false;

	GEM_BUG_ON(i915_seqno_passed(prev->fence.seqno, next->fence.seqno));
	return true;
}

static void virtual_update_register_offsets(u32 *regs,
					    struct intel_engine_cs *engine)
{
	set_offsets(regs, reg_offsets(engine), engine, false);
}

static bool virtual_matches(const struct virtual_engine *ve,
			    const struct i915_request *rq,
			    const struct intel_engine_cs *engine)
{
	const struct intel_engine_cs *inflight;

	if (!(rq->execution_mask & engine->mask)) /* We peeked too soon! */
		return false;

	/*
	 * We track when the HW has completed saving the context image
	 * (i.e. when we have seen the final CS event switching out of
	 * the context) and must not overwrite the context image before
	 * then. This restricts us to only using the active engine
	 * while the previous virtualized request is inflight (so
	 * we reuse the register offsets). This is a very small
	 * hystersis on the greedy seelction algorithm.
	 */
	inflight = intel_context_inflight(&ve->context);
	if (inflight && inflight != engine)
		return false;

	return true;
}

static void virtual_xfer_context(struct virtual_engine *ve,
				 struct intel_engine_cs *engine)
{
	unsigned int n;

	if (likely(engine == ve->siblings[0]))
		return;

	GEM_BUG_ON(READ_ONCE(ve->context.inflight));
	if (!intel_engine_has_relative_mmio(engine))
		virtual_update_register_offsets(ve->context.lrc_reg_state,
						engine);

	/*
	 * Move the bound engine to the top of the list for
	 * future execution. We then kick this tasklet first
	 * before checking others, so that we preferentially
	 * reuse this set of bound registers.
	 */
	for (n = 1; n < ve->num_siblings; n++) {
		if (ve->siblings[n] == engine) {
			swap(ve->siblings[n], ve->siblings[0]);
			break;
		}
	}
}

#define for_each_waiter(p__, rq__) \
	list_for_each_entry_lockless(p__, \
				     &(rq__)->sched.waiters_list, \
				     wait_link)

#define for_each_signaler(p__, rq__) \
	list_for_each_entry_rcu(p__, \
				&(rq__)->sched.signalers_list, \
				signal_link)

static void defer_request(struct i915_request *rq, struct list_head * const pl)
{
	LIST_HEAD(list);

	/*
	 * We want to move the interrupted request to the back of
	 * the round-robin list (i.e. its priority level), but
	 * in doing so, we must then move all requests that were in
	 * flight and were waiting for the interrupted request to
	 * be run after it again.
	 */
	do {
		struct i915_dependency *p;

		GEM_BUG_ON(i915_request_is_active(rq));
		list_move_tail(&rq->sched.link, pl);

		for_each_waiter(p, rq) {
			struct i915_request *w =
				container_of(p->waiter, typeof(*w), sched);

			if (p->flags & I915_DEPENDENCY_WEAK)
				continue;

			/* Leave semaphores spinning on the other engines */
			if (w->engine != rq->engine)
				continue;

			/* No waiter should start before its signaler */
			GEM_BUG_ON(i915_request_has_initial_breadcrumb(w) &&
				   i915_request_started(w) &&
				   !i915_request_completed(rq));

			GEM_BUG_ON(i915_request_is_active(w));
			if (!i915_request_is_ready(w))
				continue;

			if (rq_prio(w) < rq_prio(rq))
				continue;

			GEM_BUG_ON(rq_prio(w) > rq_prio(rq));
			list_move_tail(&w->sched.link, &list);
		}

		rq = list_first_entry_or_null(&list, typeof(*rq), sched.link);
	} while (rq);
}

static void defer_active(struct intel_engine_cs *engine)
{
	struct i915_request *rq;

	rq = __unwind_incomplete_requests(engine);
	if (!rq)
		return;

	defer_request(rq, i915_sched_lookup_priolist(engine, rq_prio(rq)));
}

static bool
need_timeslice(const struct intel_engine_cs *engine,
	       const struct i915_request *rq,
	       const struct rb_node *rb)
{
	int hint;

	if (!intel_engine_has_timeslices(engine))
		return false;

	hint = engine->execlists.queue_priority_hint;

	if (rb) {
		const struct virtual_engine *ve =
			rb_entry(rb, typeof(*ve), nodes[engine->id].rb);
		const struct intel_engine_cs *inflight =
			intel_context_inflight(&ve->context);

		if (!inflight || inflight == engine) {
			struct i915_request *next;

			rcu_read_lock();
			next = READ_ONCE(ve->request);
			if (next)
				hint = max(hint, rq_prio(next));
			rcu_read_unlock();
		}
	}

	if (!list_is_last(&rq->sched.link, &engine->active.requests))
		hint = max(hint, rq_prio(list_next_entry(rq, sched.link)));

	GEM_BUG_ON(hint >= I915_PRIORITY_UNPREEMPTABLE);
	return hint >= effective_prio(rq);
}

static bool
timeslice_yield(const struct intel_engine_execlists *el,
		const struct i915_request *rq)
{
	/*
	 * Once bitten, forever smitten!
	 *
	 * If the active context ever busy-waited on a semaphore,
	 * it will be treated as a hog until the end of its timeslice (i.e.
	 * until it is scheduled out and replaced by a new submission,
	 * possibly even its own lite-restore). The HW only sends an interrupt
	 * on the first miss, and we do know if that semaphore has been
	 * signaled, or even if it is now stuck on another semaphore. Play
	 * safe, yield if it might be stuck -- it will be given a fresh
	 * timeslice in the near future.
	 */
	return rq->context->lrc.ccid == READ_ONCE(el->yield);
}

static bool
timeslice_expired(const struct intel_engine_execlists *el,
		  const struct i915_request *rq)
{
	return timer_expired(&el->timer) || timeslice_yield(el, rq);
}

static int
switch_prio(struct intel_engine_cs *engine, const struct i915_request *rq)
{
	if (list_is_last(&rq->sched.link, &engine->active.requests))
		return engine->execlists.queue_priority_hint;

	return rq_prio(list_next_entry(rq, sched.link));
}

static inline unsigned long
timeslice(const struct intel_engine_cs *engine)
{
	return READ_ONCE(engine->props.timeslice_duration_ms);
}

static unsigned long active_timeslice(const struct intel_engine_cs *engine)
{
	const struct intel_engine_execlists *execlists = &engine->execlists;
	const struct i915_request *rq = *execlists->active;

	if (!rq || i915_request_completed(rq))
		return 0;

	if (READ_ONCE(execlists->switch_priority_hint) < effective_prio(rq))
		return 0;

	return timeslice(engine);
}

static void set_timeslice(struct intel_engine_cs *engine)
{
	unsigned long duration;

	if (!intel_engine_has_timeslices(engine))
		return;

	duration = active_timeslice(engine);
	ENGINE_TRACE(engine, "bump timeslicing, interval:%lu", duration);

	set_timer_ms(&engine->execlists.timer, duration);
}

static void start_timeslice(struct intel_engine_cs *engine, int prio)
{
	struct intel_engine_execlists *execlists = &engine->execlists;
	unsigned long duration;

	if (!intel_engine_has_timeslices(engine))
		return;

	WRITE_ONCE(execlists->switch_priority_hint, prio);
	if (prio == INT_MIN)
		return;

	if (timer_pending(&execlists->timer))
		return;

	duration = timeslice(engine);
	ENGINE_TRACE(engine,
		     "start timeslicing, prio:%d, interval:%lu",
		     prio, duration);

	set_timer_ms(&execlists->timer, duration);
}

static void record_preemption(struct intel_engine_execlists *execlists)
{
	(void)I915_SELFTEST_ONLY(execlists->preempt_hang.count++);
}

static unsigned long active_preempt_timeout(struct intel_engine_cs *engine,
					    const struct i915_request *rq)
{
	if (!rq)
		return 0;

	/* Force a fast reset for terminated contexts (ignoring sysfs!) */
	if (unlikely(intel_context_is_banned(rq->context)))
		return 1;

	return READ_ONCE(engine->props.preempt_timeout_ms);
}

static void set_preempt_timeout(struct intel_engine_cs *engine,
				const struct i915_request *rq)
{
	if (!intel_engine_has_preempt_reset(engine))
		return;

	set_timer_ms(&engine->execlists.preempt,
		     active_preempt_timeout(engine, rq));
}

static inline void clear_ports(struct i915_request **ports, int count)
{
	memset_p((void **)ports, NULL, count);
}

static inline void
copy_ports(struct i915_request **dst, struct i915_request **src, int count)
{
	/* A memcpy_p() would be very useful here! */
	while (count--)
		WRITE_ONCE(*dst++, *src++); /* avoid write tearing */
}

static void execlists_dequeue(struct intel_engine_cs *engine)
{
	struct intel_engine_execlists * const execlists = &engine->execlists;
	struct i915_request **port = execlists->pending;
	struct i915_request ** const last_port = port + execlists->port_mask;
	struct i915_request * const *active;
	struct i915_request *last;
	struct rb_node *rb;
	bool submit = false;

	/*
	 * Hardware submission is through 2 ports. Conceptually each port
	 * has a (RING_START, RING_HEAD, RING_TAIL) tuple. RING_START is
	 * static for a context, and unique to each, so we only execute
	 * requests belonging to a single context from each ring. RING_HEAD
	 * is maintained by the CS in the context image, it marks the place
	 * where it got up to last time, and through RING_TAIL we tell the CS
	 * where we want to execute up to this time.
	 *
	 * In this list the requests are in order of execution. Consecutive
	 * requests from the same context are adjacent in the ringbuffer. We
	 * can combine these requests into a single RING_TAIL update:
	 *
	 *              RING_HEAD...req1...req2
	 *                                    ^- RING_TAIL
	 * since to execute req2 the CS must first execute req1.
	 *
	 * Our goal then is to point each port to the end of a consecutive
	 * sequence of requests as being the most optimal (fewest wake ups
	 * and context switches) submission.
	 */

	for (rb = rb_first_cached(&execlists->virtual); rb; ) {
		struct virtual_engine *ve =
			rb_entry(rb, typeof(*ve), nodes[engine->id].rb);
		struct i915_request *rq = READ_ONCE(ve->request);

		if (!rq) { /* lazily cleanup after another engine handled rq */
			rb_erase_cached(rb, &execlists->virtual);
			RB_CLEAR_NODE(rb);
			rb = rb_first_cached(&execlists->virtual);
			continue;
		}

		if (!virtual_matches(ve, rq, engine)) {
			rb = rb_next(rb);
			continue;
		}

		break;
	}

	/*
	 * If the queue is higher priority than the last
	 * request in the currently active context, submit afresh.
	 * We will resubmit again afterwards in case we need to split
	 * the active context to interject the preemption request,
	 * i.e. we will retrigger preemption following the ack in case
	 * of trouble.
	 */
	active = READ_ONCE(execlists->active);

	/*
	 * In theory we can skip over completed contexts that have not
	 * yet been processed by events (as those events are in flight):
	 *
	 * while ((last = *active) && i915_request_completed(last))
	 *	active++;
	 *
	 * However, the GPU cannot handle this as it will ultimately
	 * find itself trying to jump back into a context it has just
	 * completed and barf.
	 */

	if ((last = *active)) {
		if (need_preempt(engine, last, rb)) {
			if (i915_request_completed(last)) {
				tasklet_hi_schedule(&execlists->tasklet);
				return;
			}

			ENGINE_TRACE(engine,
				     "preempting last=%llx:%lld, prio=%d, hint=%d\n",
				     last->fence.context,
				     last->fence.seqno,
				     last->sched.attr.priority,
				     execlists->queue_priority_hint);
			record_preemption(execlists);

			/*
			 * Don't let the RING_HEAD advance past the breadcrumb
			 * as we unwind (and until we resubmit) so that we do
			 * not accidentally tell it to go backwards.
			 */
			ring_set_paused(engine, 1);

			/*
			 * Note that we have not stopped the GPU at this point,
			 * so we are unwinding the incomplete requests as they
			 * remain inflight and so by the time we do complete
			 * the preemption, some of the unwound requests may
			 * complete!
			 */
			__unwind_incomplete_requests(engine);

			last = NULL;
		} else if (need_timeslice(engine, last, rb) &&
			   timeslice_expired(execlists, last)) {
			if (i915_request_completed(last)) {
				tasklet_hi_schedule(&execlists->tasklet);
				return;
			}

			ENGINE_TRACE(engine,
				     "expired last=%llx:%lld, prio=%d, hint=%d, yield?=%s\n",
				     last->fence.context,
				     last->fence.seqno,
				     last->sched.attr.priority,
				     execlists->queue_priority_hint,
				     yesno(timeslice_yield(execlists, last)));

			ring_set_paused(engine, 1);
			defer_active(engine);

			/*
			 * Unlike for preemption, if we rewind and continue
			 * executing the same context as previously active,
			 * the order of execution will remain the same and
			 * the tail will only advance. We do not need to
			 * force a full context restore, as a lite-restore
			 * is sufficient to resample the monotonic TAIL.
			 *
			 * If we switch to any other context, similarly we
			 * will not rewind TAIL of current context, and
			 * normal save/restore will preserve state and allow
			 * us to later continue executing the same request.
			 */
			last = NULL;
		} else {
			/*
			 * Otherwise if we already have a request pending
			 * for execution after the current one, we can
			 * just wait until the next CS event before
			 * queuing more. In either case we will force a
			 * lite-restore preemption event, but if we wait
			 * we hopefully coalesce several updates into a single
			 * submission.
			 */
			if (!list_is_last(&last->sched.link,
					  &engine->active.requests)) {
				/*
				 * Even if ELSP[1] is occupied and not worthy
				 * of timeslices, our queue might be.
				 */
				start_timeslice(engine, queue_prio(execlists));
				return;
			}
		}
	}

	while (rb) { /* XXX virtual is always taking precedence */
		struct virtual_engine *ve =
			rb_entry(rb, typeof(*ve), nodes[engine->id].rb);
		struct i915_request *rq;

		spin_lock(&ve->base.active.lock);

		rq = ve->request;
		if (unlikely(!rq)) { /* lost the race to a sibling */
			spin_unlock(&ve->base.active.lock);
			rb_erase_cached(rb, &execlists->virtual);
			RB_CLEAR_NODE(rb);
			rb = rb_first_cached(&execlists->virtual);
			continue;
		}

		GEM_BUG_ON(rq != ve->request);
		GEM_BUG_ON(rq->engine != &ve->base);
		GEM_BUG_ON(rq->context != &ve->context);

		if (rq_prio(rq) >= queue_prio(execlists)) {
			if (!virtual_matches(ve, rq, engine)) {
				spin_unlock(&ve->base.active.lock);
				rb = rb_next(rb);
				continue;
			}

			if (last && !can_merge_rq(last, rq)) {
				spin_unlock(&ve->base.active.lock);
				start_timeslice(engine, rq_prio(rq));
				return; /* leave this for another sibling */
			}

			ENGINE_TRACE(engine,
				     "virtual rq=%llx:%lld%s, new engine? %s\n",
				     rq->fence.context,
				     rq->fence.seqno,
				     i915_request_completed(rq) ? "!" :
				     i915_request_started(rq) ? "*" :
				     "",
				     yesno(engine != ve->siblings[0]));

			WRITE_ONCE(ve->request, NULL);
			WRITE_ONCE(ve->base.execlists.queue_priority_hint,
				   INT_MIN);
			rb_erase_cached(rb, &execlists->virtual);
			RB_CLEAR_NODE(rb);

			GEM_BUG_ON(!(rq->execution_mask & engine->mask));
			WRITE_ONCE(rq->engine, engine);

			if (__i915_request_submit(rq)) {
				/*
				 * Only after we confirm that we will submit
				 * this request (i.e. it has not already
				 * completed), do we want to update the context.
				 *
				 * This serves two purposes. It avoids
				 * unnecessary work if we are resubmitting an
				 * already completed request after timeslicing.
				 * But more importantly, it prevents us altering
				 * ve->siblings[] on an idle context, where
				 * we may be using ve->siblings[] in
				 * virtual_context_enter / virtual_context_exit.
				 */
				virtual_xfer_context(ve, engine);
				GEM_BUG_ON(ve->siblings[0] != engine);

				submit = true;
				last = rq;
			}
			i915_request_put(rq);

			/*
			 * Hmm, we have a bunch of virtual engine requests,
			 * but the first one was already completed (thanks
			 * preempt-to-busy!). Keep looking at the veng queue
			 * until we have no more relevant requests (i.e.
			 * the normal submit queue has higher priority).
			 */
			if (!submit) {
				spin_unlock(&ve->base.active.lock);
				rb = rb_first_cached(&execlists->virtual);
				continue;
			}
		}

		spin_unlock(&ve->base.active.lock);
		break;
	}

	while ((rb = rb_first_cached(&execlists->queue))) {
		struct i915_priolist *p = to_priolist(rb);
		struct i915_request *rq, *rn;
		int i;

		priolist_for_each_request_consume(rq, rn, p, i) {
			bool merge = true;

			/*
			 * Can we combine this request with the current port?
			 * It has to be the same context/ringbuffer and not
			 * have any exceptions (e.g. GVT saying never to
			 * combine contexts).
			 *
			 * If we can combine the requests, we can execute both
			 * by updating the RING_TAIL to point to the end of the
			 * second request, and so we never need to tell the
			 * hardware about the first.
			 */
			if (last && !can_merge_rq(last, rq)) {
				/*
				 * If we are on the second port and cannot
				 * combine this request with the last, then we
				 * are done.
				 */
				if (port == last_port)
					goto done;

				/*
				 * We must not populate both ELSP[] with the
				 * same LRCA, i.e. we must submit 2 different
				 * contexts if we submit 2 ELSP.
				 */
				if (last->context == rq->context)
					goto done;

				if (i915_request_has_sentinel(last))
					goto done;

				/*
				 * If GVT overrides us we only ever submit
				 * port[0], leaving port[1] empty. Note that we
				 * also have to be careful that we don't queue
				 * the same context (even though a different
				 * request) to the second port.
				 */
				if (ctx_single_port_submission(last->context) ||
				    ctx_single_port_submission(rq->context))
					goto done;

				merge = false;
			}

			if (__i915_request_submit(rq)) {
				if (!merge) {
					*port = execlists_schedule_in(last, port - execlists->pending);
					port++;
					last = NULL;
				}

				GEM_BUG_ON(last &&
					   !can_merge_ctx(last->context,
							  rq->context));
				GEM_BUG_ON(last &&
					   i915_seqno_passed(last->fence.seqno,
							     rq->fence.seqno));

				submit = true;
				last = rq;
			}
		}

		rb_erase_cached(&p->node, &execlists->queue);
		i915_priolist_free(p);
	}

done:
	/*
	 * Here be a bit of magic! Or sleight-of-hand, whichever you prefer.
	 *
	 * We choose the priority hint such that if we add a request of greater
	 * priority than this, we kick the submission tasklet to decide on
	 * the right order of submitting the requests to hardware. We must
	 * also be prepared to reorder requests as they are in-flight on the
	 * HW. We derive the priority hint then as the first "hole" in
	 * the HW submission ports and if there are no available slots,
	 * the priority of the lowest executing request, i.e. last.
	 *
	 * When we do receive a higher priority request ready to run from the
	 * user, see queue_request(), the priority hint is bumped to that
	 * request triggering preemption on the next dequeue (or subsequent
	 * interrupt for secondary ports).
	 */
	execlists->queue_priority_hint = queue_prio(execlists);

	if (submit) {
		*port = execlists_schedule_in(last, port - execlists->pending);
		execlists->switch_priority_hint =
			switch_prio(engine, *execlists->pending);

		/*
		 * Skip if we ended up with exactly the same set of requests,
		 * e.g. trying to timeslice a pair of ordered contexts
		 */
		if (!memcmp(active, execlists->pending,
			    (port - execlists->pending + 1) * sizeof(*port))) {
			do
				execlists_schedule_out(fetch_and_zero(port));
			while (port-- != execlists->pending);

			goto skip_submit;
		}
		clear_ports(port + 1, last_port - port);

		WRITE_ONCE(execlists->yield, -1);
		set_preempt_timeout(engine, *active);
		execlists_submit_ports(engine);
	} else {
		start_timeslice(engine, execlists->queue_priority_hint);
skip_submit:
		ring_set_paused(engine, 0);
	}
}

static void
cancel_port_requests(struct intel_engine_execlists * const execlists)
{
	struct i915_request * const *port;

	for (port = execlists->pending; *port; port++)
		execlists_schedule_out(*port);
	clear_ports(execlists->pending, ARRAY_SIZE(execlists->pending));

	/* Mark the end of active before we overwrite *active */
	for (port = xchg(&execlists->active, execlists->pending); *port; port++)
		execlists_schedule_out(*port);
	clear_ports(execlists->inflight, ARRAY_SIZE(execlists->inflight));

	smp_wmb(); /* complete the seqlock for execlists_active() */
	WRITE_ONCE(execlists->active, execlists->inflight);
}

static inline void
invalidate_csb_entries(const u64 *first, const u64 *last)
{
	clflush((void *)first);
	clflush((void *)last);
}

/*
 * Starting with Gen12, the status has a new format:
 *
 *     bit  0:     switched to new queue
 *     bit  1:     reserved
 *     bit  2:     semaphore wait mode (poll or signal), only valid when
 *                 switch detail is set to "wait on semaphore"
 *     bits 3-5:   engine class
 *     bits 6-11:  engine instance
 *     bits 12-14: reserved
 *     bits 15-25: sw context id of the lrc the GT switched to
 *     bits 26-31: sw counter of the lrc the GT switched to
 *     bits 32-35: context switch detail
 *                  - 0: ctx complete
 *                  - 1: wait on sync flip
 *                  - 2: wait on vblank
 *                  - 3: wait on scanline
 *                  - 4: wait on semaphore
 *                  - 5: context preempted (not on SEMAPHORE_WAIT or
 *                       WAIT_FOR_EVENT)
 *     bit  36:    reserved
 *     bits 37-43: wait detail (for switch detail 1 to 4)
 *     bits 44-46: reserved
 *     bits 47-57: sw context id of the lrc the GT switched away from
 *     bits 58-63: sw counter of the lrc the GT switched away from
 */
static inline bool gen12_csb_parse(const u64 *csb)
{
	bool ctx_away_valid;
	bool new_queue;
	u64 entry;

	/* HSD#22011248461 */
	entry = READ_ONCE(*csb);
	if (unlikely(entry == -1)) {
		preempt_disable();
		if (wait_for_atomic_us((entry = READ_ONCE(*csb)) != -1, 50))
			GEM_WARN_ON("50us CSB timeout");
		preempt_enable();
	}
	WRITE_ONCE(*(u64 *)csb, -1);

	ctx_away_valid = GEN12_CSB_CTX_VALID(upper_32_bits(entry));
	new_queue =
		lower_32_bits(entry) & GEN12_CTX_STATUS_SWITCHED_TO_NEW_QUEUE;

	/*
	 * The context switch detail is not guaranteed to be 5 when a preemption
	 * occurs, so we can't just check for that. The check below works for
	 * all the cases we care about, including preemptions of WAIT
	 * instructions and lite-restore. Preempt-to-idle via the CTRL register
	 * would require some extra handling, but we don't support that.
	 */
	if (!ctx_away_valid || new_queue) {
		GEM_BUG_ON(!GEN12_CSB_CTX_VALID(lower_32_bits(entry)));
		return true;
	}

	/*
	 * switch detail = 5 is covered by the case above and we do not expect a
	 * context switch on an unsuccessful wait instruction since we always
	 * use polling mode.
	 */
	GEM_BUG_ON(GEN12_CTX_SWITCH_DETAIL(upper_32_bits(entry)));
	return false;
}

static inline bool gen8_csb_parse(const u64 *csb)
{
	return *csb & (GEN8_CTX_STATUS_IDLE_ACTIVE | GEN8_CTX_STATUS_PREEMPTED);
}

static void process_csb(struct intel_engine_cs *engine)
{
	struct intel_engine_execlists * const execlists = &engine->execlists;
	const u64 * const buf = execlists->csb_status;
	const u8 num_entries = execlists->csb_size;
	u8 head, tail;

	/*
	 * As we modify our execlists state tracking we require exclusive
	 * access. Either we are inside the tasklet, or the tasklet is disabled
	 * and we assume that is only inside the reset paths and so serialised.
	 */
	GEM_BUG_ON(!tasklet_is_locked(&execlists->tasklet) &&
		   !reset_in_progress(execlists));
	GEM_BUG_ON(!intel_engine_in_execlists_submission_mode(engine));

	/*
	 * Note that csb_write, csb_status may be either in HWSP or mmio.
	 * When reading from the csb_write mmio register, we have to be
	 * careful to only use the GEN8_CSB_WRITE_PTR portion, which is
	 * the low 4bits. As it happens we know the next 4bits are always
	 * zero and so we can simply masked off the low u8 of the register
	 * and treat it identically to reading from the HWSP (without having
	 * to use explicit shifting and masking, and probably bifurcating
	 * the code to handle the legacy mmio read).
	 */
	head = execlists->csb_head;
	tail = READ_ONCE(*execlists->csb_write);
	if (unlikely(head == tail))
		return;

	/*
	 * We will consume all events from HW, or at least pretend to.
	 *
	 * The sequence of events from the HW is deterministic, and derived
	 * from our writes to the ELSP, with a smidgen of variability for
	 * the arrival of the asynchronous requests wrt to the inflight
	 * execution. If the HW sends an event that does not correspond with
	 * the one we are expecting, we have to abandon all hope as we lose
	 * all tracking of what the engine is actually executing. We will
	 * only detect we are out of sequence with the HW when we get an
	 * 'impossible' event because we have already drained our own
	 * preemption/promotion queue. If this occurs, we know that we likely
	 * lost track of execution earlier and must unwind and restart, the
	 * simplest way is by stop processing the event queue and force the
	 * engine to reset.
	 */
	execlists->csb_head = tail;
	ENGINE_TRACE(engine, "cs-irq head=%d, tail=%d\n", head, tail);

	/*
	 * Hopefully paired with a wmb() in HW!
	 *
	 * We must complete the read of the write pointer before any reads
	 * from the CSB, so that we do not see stale values. Without an rmb
	 * (lfence) the HW may speculatively perform the CSB[] reads *before*
	 * we perform the READ_ONCE(*csb_write).
	 */
	rmb();
	do {
		bool promote;

		if (++head == num_entries)
			head = 0;

		/*
		 * We are flying near dragons again.
		 *
		 * We hold a reference to the request in execlist_port[]
		 * but no more than that. We are operating in softirq
		 * context and so cannot hold any mutex or sleep. That
		 * prevents us stopping the requests we are processing
		 * in port[] from being retired simultaneously (the
		 * breadcrumb will be complete before we see the
		 * context-switch). As we only hold the reference to the
		 * request, any pointer chasing underneath the request
		 * is subject to a potential use-after-free. Thus we
		 * store all of the bookkeeping within port[] as
		 * required, and avoid using unguarded pointers beneath
		 * request itself. The same applies to the atomic
		 * status notifier.
		 */

		ENGINE_TRACE(engine, "csb[%d]: status=0x%08x:0x%08x\n",
			     head,
			     upper_32_bits(buf[head]),
			     lower_32_bits(buf[head]));

		if (INTEL_GEN(engine->i915) >= 12)
			promote = gen12_csb_parse(buf + head);
		else
			promote = gen8_csb_parse(buf + head);
		if (promote) {
			struct i915_request * const *old = execlists->active;

			if (GEM_WARN_ON(!*execlists->pending)) {
				execlists->error_interrupt |= ERROR_CSB;
				break;
			}

			ring_set_paused(engine, 0);

			/* Point active to the new ELSP; prevent overwriting */
			WRITE_ONCE(execlists->active, execlists->pending);
			smp_wmb(); /* notify execlists_active() */

			/* cancel old inflight, prepare for switch */
			trace_ports(execlists, "preempted", old);
			while (*old)
				execlists_schedule_out(*old++);

			/* switch pending to inflight */
			GEM_BUG_ON(!assert_pending_valid(execlists, "promote"));
			copy_ports(execlists->inflight,
				   execlists->pending,
				   execlists_num_ports(execlists));
			smp_wmb(); /* complete the seqlock */
			WRITE_ONCE(execlists->active, execlists->inflight);

			/* XXX Magic delay for tgl */
			ENGINE_POSTING_READ(engine, RING_CONTEXT_STATUS_PTR);

			WRITE_ONCE(execlists->pending[0], NULL);
		} else {
			if (GEM_WARN_ON(!*execlists->active)) {
				execlists->error_interrupt |= ERROR_CSB;
				break;
			}

			/* port0 completed, advanced to port1 */
			trace_ports(execlists, "completed", execlists->active);

			/*
			 * We rely on the hardware being strongly
			 * ordered, that the breadcrumb write is
			 * coherent (visible from the CPU) before the
			 * user interrupt is processed. One might assume
			 * that the breadcrumb write being before the
			 * user interrupt and the CS event for the context
			 * switch would therefore be before the CS event
			 * itself...
			 */
			if (GEM_SHOW_DEBUG() &&
			    !i915_request_completed(*execlists->active)) {
				struct i915_request *rq = *execlists->active;
				const u32 *regs __maybe_unused =
					rq->context->lrc_reg_state;

				ENGINE_TRACE(engine,
					     "context completed before request!\n");
				ENGINE_TRACE(engine,
					     "ring:{start:0x%08x, head:%04x, tail:%04x, ctl:%08x, mode:%08x}\n",
					     ENGINE_READ(engine, RING_START),
					     ENGINE_READ(engine, RING_HEAD) & HEAD_ADDR,
					     ENGINE_READ(engine, RING_TAIL) & TAIL_ADDR,
					     ENGINE_READ(engine, RING_CTL),
					     ENGINE_READ(engine, RING_MI_MODE));
				ENGINE_TRACE(engine,
					     "rq:{start:%08x, head:%04x, tail:%04x, seqno:%llx:%d, hwsp:%d}, ",
					     i915_ggtt_offset(rq->ring->vma),
					     rq->head, rq->tail,
					     rq->fence.context,
					     lower_32_bits(rq->fence.seqno),
					     hwsp_seqno(rq));
				ENGINE_TRACE(engine,
					     "ctx:{start:%08x, head:%04x, tail:%04x}, ",
					     regs[CTX_RING_START],
					     regs[CTX_RING_HEAD],
					     regs[CTX_RING_TAIL]);
			}

			execlists_schedule_out(*execlists->active++);

			GEM_BUG_ON(execlists->active - execlists->inflight >
				   execlists_num_ports(execlists));
		}
	} while (head != tail);

	set_timeslice(engine);

	/*
	 * Gen11 has proven to fail wrt global observation point between
	 * entry and tail update, failing on the ordering and thus
	 * we see an old entry in the context status buffer.
	 *
	 * Forcibly evict out entries for the next gpu csb update,
	 * to increase the odds that we get a fresh entries with non
	 * working hardware. The cost for doing so comes out mostly with
	 * the wash as hardware, working or not, will need to do the
	 * invalidation before.
	 */
	invalidate_csb_entries(&buf[0], &buf[num_entries - 1]);
}

static void __execlists_submission_tasklet(struct intel_engine_cs *const engine)
{
	lockdep_assert_held(&engine->active.lock);
	if (!READ_ONCE(engine->execlists.pending[0])) {
		rcu_read_lock(); /* protect peeking at execlists->active */
		execlists_dequeue(engine);
		rcu_read_unlock();
	}
}

static void __execlists_hold(struct i915_request *rq)
{
	LIST_HEAD(list);

	do {
		struct i915_dependency *p;

		if (i915_request_is_active(rq))
			__i915_request_unsubmit(rq);

		clear_bit(I915_FENCE_FLAG_PQUEUE, &rq->fence.flags);
		list_move_tail(&rq->sched.link, &rq->engine->active.hold);
		i915_request_set_hold(rq);
		RQ_TRACE(rq, "on hold\n");

		for_each_waiter(p, rq) {
			struct i915_request *w =
				container_of(p->waiter, typeof(*w), sched);

			/* Leave semaphores spinning on the other engines */
			if (w->engine != rq->engine)
				continue;

			if (!i915_request_is_ready(w))
				continue;

			if (i915_request_completed(w))
				continue;

			if (i915_request_on_hold(w))
				continue;

			list_move_tail(&w->sched.link, &list);
		}

		rq = list_first_entry_or_null(&list, typeof(*rq), sched.link);
	} while (rq);
}

static bool execlists_hold(struct intel_engine_cs *engine,
			   struct i915_request *rq)
{
	if (i915_request_on_hold(rq))
		return false;

	spin_lock_irq(&engine->active.lock);

	if (i915_request_completed(rq)) { /* too late! */
		rq = NULL;
		goto unlock;
	}

	if (rq->engine != engine) { /* preempted virtual engine */
		struct virtual_engine *ve = to_virtual_engine(rq->engine);

		/*
		 * intel_context_inflight() is only protected by virtue
		 * of process_csb() being called only by the tasklet (or
		 * directly from inside reset while the tasklet is suspended).
		 * Assert that neither of those are allowed to run while we
		 * poke at the request queues.
		 */
		GEM_BUG_ON(!reset_in_progress(&engine->execlists));

		/*
		 * An unsubmitted request along a virtual engine will
		 * remain on the active (this) engine until we are able
		 * to process the context switch away (and so mark the
		 * context as no longer in flight). That cannot have happened
		 * yet, otherwise we would not be hanging!
		 */
		spin_lock(&ve->base.active.lock);
		GEM_BUG_ON(intel_context_inflight(rq->context) != engine);
		GEM_BUG_ON(ve->request != rq);
		ve->request = NULL;
		spin_unlock(&ve->base.active.lock);
		i915_request_put(rq);

		rq->engine = engine;
	}

	/*
	 * Transfer this request onto the hold queue to prevent it
	 * being resumbitted to HW (and potentially completed) before we have
	 * released it. Since we may have already submitted following
	 * requests, we need to remove those as well.
	 */
	GEM_BUG_ON(i915_request_on_hold(rq));
	GEM_BUG_ON(rq->engine != engine);
	__execlists_hold(rq);
	GEM_BUG_ON(list_empty(&engine->active.hold));

unlock:
	spin_unlock_irq(&engine->active.lock);
	return rq;
}

static bool hold_request(const struct i915_request *rq)
{
	struct i915_dependency *p;
	bool result = false;

	/*
	 * If one of our ancestors is on hold, we must also be on hold,
	 * otherwise we will bypass it and execute before it.
	 */
	rcu_read_lock();
	for_each_signaler(p, rq) {
		const struct i915_request *s =
			container_of(p->signaler, typeof(*s), sched);

		if (s->engine != rq->engine)
			continue;

		result = i915_request_on_hold(s);
		if (result)
			break;
	}
	rcu_read_unlock();

	return result;
}

static void __execlists_unhold(struct i915_request *rq)
{
	LIST_HEAD(list);

	do {
		struct i915_dependency *p;

		RQ_TRACE(rq, "hold release\n");

		GEM_BUG_ON(!i915_request_on_hold(rq));
		GEM_BUG_ON(!i915_sw_fence_signaled(&rq->submit));

		i915_request_clear_hold(rq);
		list_move_tail(&rq->sched.link,
			       i915_sched_lookup_priolist(rq->engine,
							  rq_prio(rq)));
		set_bit(I915_FENCE_FLAG_PQUEUE, &rq->fence.flags);

		/* Also release any children on this engine that are ready */
		for_each_waiter(p, rq) {
			struct i915_request *w =
				container_of(p->waiter, typeof(*w), sched);

			/* Propagate any change in error status */
			if (rq->fence.error)
				i915_request_set_error_once(w, rq->fence.error);

			if (w->engine != rq->engine)
				continue;

			if (!i915_request_on_hold(w))
				continue;

			/* Check that no other parents are also on hold */
			if (hold_request(w))
				continue;

			list_move_tail(&w->sched.link, &list);
		}

		rq = list_first_entry_or_null(&list, typeof(*rq), sched.link);
	} while (rq);
}

static void execlists_unhold(struct intel_engine_cs *engine,
			     struct i915_request *rq)
{
	spin_lock_irq(&engine->active.lock);

	/*
	 * Move this request back to the priority queue, and all of its
	 * children and grandchildren that were suspended along with it.
	 */
	__execlists_unhold(rq);

	if (rq_prio(rq) > engine->execlists.queue_priority_hint) {
		engine->execlists.queue_priority_hint = rq_prio(rq);
		tasklet_hi_schedule(&engine->execlists.tasklet);
	}

	spin_unlock_irq(&engine->active.lock);
}

struct execlists_capture {
	struct work_struct work;
	struct i915_request *rq;
	struct i915_gpu_coredump *error;
};

static void execlists_capture_work(struct work_struct *work)
{
	struct execlists_capture *cap = container_of(work, typeof(*cap), work);
	const gfp_t gfp = GFP_KERNEL | __GFP_RETRY_MAYFAIL | __GFP_NOWARN;
	struct intel_engine_cs *engine = cap->rq->engine;
	struct intel_gt_coredump *gt = cap->error->gt;
	struct intel_engine_capture_vma *vma;

	/* Compress all the objects attached to the request, slow! */
	vma = intel_engine_coredump_add_request(gt->engine, cap->rq, gfp);
	if (vma) {
		struct i915_vma_compress *compress =
			i915_vma_capture_prepare(gt);

		intel_engine_coredump_add_vma(gt->engine, vma, compress);
		i915_vma_capture_finish(gt, compress);
	}

	gt->simulated = gt->engine->simulated;
	cap->error->simulated = gt->simulated;

	/* Publish the error state, and announce it to the world */
	i915_error_state_store(cap->error);
	i915_gpu_coredump_put(cap->error);

	/* Return this request and all that depend upon it for signaling */
	execlists_unhold(engine, cap->rq);
	i915_request_put(cap->rq);

	kfree(cap);
}

static struct execlists_capture *capture_regs(struct intel_engine_cs *engine)
{
	const gfp_t gfp = GFP_ATOMIC | __GFP_NOWARN;
	struct execlists_capture *cap;

	cap = kmalloc(sizeof(*cap), gfp);
	if (!cap)
		return NULL;

	cap->error = i915_gpu_coredump_alloc(engine->i915, gfp);
	if (!cap->error)
		goto err_cap;

	cap->error->gt = intel_gt_coredump_alloc(engine->gt, gfp);
	if (!cap->error->gt)
		goto err_gpu;

	cap->error->gt->engine = intel_engine_coredump_alloc(engine, gfp);
	if (!cap->error->gt->engine)
		goto err_gt;

	return cap;

err_gt:
	kfree(cap->error->gt);
err_gpu:
	kfree(cap->error);
err_cap:
	kfree(cap);
	return NULL;
}

static struct i915_request *
active_context(struct intel_engine_cs *engine, u32 ccid)
{
	const struct intel_engine_execlists * const el = &engine->execlists;
	struct i915_request * const *port, *rq;

	/*
	 * Use the most recent result from process_csb(), but just in case
	 * we trigger an error (via interrupt) before the first CS event has
	 * been written, peek at the next submission.
	 */

	for (port = el->active; (rq = *port); port++) {
		if (rq->context->lrc.ccid == ccid) {
			ENGINE_TRACE(engine,
				     "ccid found at active:%zd\n",
				     port - el->active);
			return rq;
		}
	}

	for (port = el->pending; (rq = *port); port++) {
		if (rq->context->lrc.ccid == ccid) {
			ENGINE_TRACE(engine,
				     "ccid found at pending:%zd\n",
				     port - el->pending);
			return rq;
		}
	}

	ENGINE_TRACE(engine, "ccid:%x not found\n", ccid);
	return NULL;
}

static u32 active_ccid(struct intel_engine_cs *engine)
{
	return ENGINE_READ_FW(engine, RING_EXECLIST_STATUS_HI);
}

static void execlists_capture(struct intel_engine_cs *engine)
{
	struct execlists_capture *cap;

	if (!IS_ENABLED(CONFIG_DRM_I915_CAPTURE_ERROR))
		return;

	/*
	 * We need to _quickly_ capture the engine state before we reset.
	 * We are inside an atomic section (softirq) here and we are delaying
	 * the forced preemption event.
	 */
	cap = capture_regs(engine);
	if (!cap)
		return;

	spin_lock_irq(&engine->active.lock);
	cap->rq = active_context(engine, active_ccid(engine));
	if (cap->rq) {
		cap->rq = active_request(cap->rq->context->timeline, cap->rq);
		cap->rq = i915_request_get_rcu(cap->rq);
	}
	spin_unlock_irq(&engine->active.lock);
	if (!cap->rq)
		goto err_free;

	/*
	 * Remove the request from the execlists queue, and take ownership
	 * of the request. We pass it to our worker who will _slowly_ compress
	 * all the pages the _user_ requested for debugging their batch, after
	 * which we return it to the queue for signaling.
	 *
	 * By removing them from the execlists queue, we also remove the
	 * requests from being processed by __unwind_incomplete_requests()
	 * during the intel_engine_reset(), and so they will *not* be replayed
	 * afterwards.
	 *
	 * Note that because we have not yet reset the engine at this point,
	 * it is possible for the request that we have identified as being
	 * guilty, did in fact complete and we will then hit an arbitration
	 * point allowing the outstanding preemption to succeed. The likelihood
	 * of that is very low (as capturing of the engine registers should be
	 * fast enough to run inside an irq-off atomic section!), so we will
	 * simply hold that request accountable for being non-preemptible
	 * long enough to force the reset.
	 */
	if (!execlists_hold(engine, cap->rq))
		goto err_rq;

	INIT_WORK(&cap->work, execlists_capture_work);
	schedule_work(&cap->work);
	return;

err_rq:
	i915_request_put(cap->rq);
err_free:
	i915_gpu_coredump_put(cap->error);
	kfree(cap);
}

static void execlists_reset(struct intel_engine_cs *engine, const char *msg)
{
	const unsigned int bit = I915_RESET_ENGINE + engine->id;
	unsigned long *lock = &engine->gt->reset.flags;

	if (!intel_has_reset_engine(engine->gt))
		return;

	if (test_and_set_bit(bit, lock))
		return;

	ENGINE_TRACE(engine, "reset for %s\n", msg);

	/* Mark this tasklet as disabled to avoid waiting for it to complete */
	tasklet_disable_nosync(&engine->execlists.tasklet);

	ring_set_paused(engine, 1); /* Freeze the current request in place */
	execlists_capture(engine);
	intel_engine_reset(engine, msg);

	tasklet_enable(&engine->execlists.tasklet);
	clear_and_wake_up_bit(bit, lock);
}

static bool preempt_timeout(const struct intel_engine_cs *const engine)
{
	const struct timer_list *t = &engine->execlists.preempt;

	if (!CONFIG_DRM_I915_PREEMPT_TIMEOUT)
		return false;

	if (!timer_expired(t))
		return false;

	return READ_ONCE(engine->execlists.pending[0]);
}

/*
 * Check the unread Context Status Buffers and manage the submission of new
 * contexts to the ELSP accordingly.
 */
static void execlists_submission_tasklet(unsigned long data)
{
	struct intel_engine_cs * const engine = (struct intel_engine_cs *)data;
	bool timeout = preempt_timeout(engine);

	process_csb(engine);

	if (unlikely(READ_ONCE(engine->execlists.error_interrupt))) {
		const char *msg;

		/* Generate the error message in priority wrt to the user! */
		if (engine->execlists.error_interrupt & GENMASK(15, 0))
			msg = "CS error"; /* thrown by a user payload */
		else if (engine->execlists.error_interrupt & ERROR_CSB)
			msg = "invalid CSB event";
		else
			msg = "internal error";

		engine->execlists.error_interrupt = 0;
		execlists_reset(engine, msg);
	}

	if (!READ_ONCE(engine->execlists.pending[0]) || timeout) {
		unsigned long flags;

		spin_lock_irqsave(&engine->active.lock, flags);
		__execlists_submission_tasklet(engine);
		spin_unlock_irqrestore(&engine->active.lock, flags);

		/* Recheck after serialising with direct-submission */
		if (unlikely(timeout && preempt_timeout(engine))) {
			cancel_timer(&engine->execlists.preempt);
			execlists_reset(engine, "preemption time out");
		}
	}
}

static void __execlists_kick(struct intel_engine_execlists *execlists)
{
	/* Kick the tasklet for some interrupt coalescing and reset handling */
	tasklet_hi_schedule(&execlists->tasklet);
}

#define execlists_kick(t, member) \
	__execlists_kick(container_of(t, struct intel_engine_execlists, member))

static void execlists_timeslice(struct timer_list *timer)
{
	execlists_kick(timer, timer);
}

static void execlists_preempt(struct timer_list *timer)
{
	execlists_kick(timer, preempt);
}

static void queue_request(struct intel_engine_cs *engine,
			  struct i915_request *rq)
{
	GEM_BUG_ON(!list_empty(&rq->sched.link));
	list_add_tail(&rq->sched.link,
		      i915_sched_lookup_priolist(engine, rq_prio(rq)));
	set_bit(I915_FENCE_FLAG_PQUEUE, &rq->fence.flags);
}

static void __submit_queue_imm(struct intel_engine_cs *engine)
{
	struct intel_engine_execlists * const execlists = &engine->execlists;

	if (reset_in_progress(execlists))
		return; /* defer until we restart the engine following reset */

	__execlists_submission_tasklet(engine);
}

static void submit_queue(struct intel_engine_cs *engine,
			 const struct i915_request *rq)
{
	struct intel_engine_execlists *execlists = &engine->execlists;

	if (rq_prio(rq) <= execlists->queue_priority_hint)
		return;

	execlists->queue_priority_hint = rq_prio(rq);
	__submit_queue_imm(engine);
}

static bool ancestor_on_hold(const struct intel_engine_cs *engine,
			     const struct i915_request *rq)
{
	GEM_BUG_ON(i915_request_on_hold(rq));
	return !list_empty(&engine->active.hold) && hold_request(rq);
}

static void flush_csb(struct intel_engine_cs *engine)
{
	struct intel_engine_execlists *el = &engine->execlists;

	if (READ_ONCE(el->pending[0]) && tasklet_trylock(&el->tasklet)) {
		if (!reset_in_progress(el))
			process_csb(engine);
		tasklet_unlock(&el->tasklet);
	}
}

static void execlists_submit_request(struct i915_request *request)
{
	struct intel_engine_cs *engine = request->engine;
	unsigned long flags;

	/* Hopefully we clear execlists->pending[] to let us through */
	flush_csb(engine);

	/* Will be called from irq-context when using foreign fences. */
	spin_lock_irqsave(&engine->active.lock, flags);

	if (unlikely(ancestor_on_hold(engine, request))) {
		RQ_TRACE(request, "ancestor on hold\n");
		list_add_tail(&request->sched.link, &engine->active.hold);
		i915_request_set_hold(request);
	} else {
		queue_request(engine, request);

		GEM_BUG_ON(RB_EMPTY_ROOT(&engine->execlists.queue.rb_root));
		GEM_BUG_ON(list_empty(&request->sched.link));

		submit_queue(engine, request);
	}

	spin_unlock_irqrestore(&engine->active.lock, flags);
}

static void __execlists_context_fini(struct intel_context *ce)
{
	intel_ring_put(ce->ring);
	i915_vma_put(ce->state);
}

static void execlists_context_destroy(struct kref *kref)
{
	struct intel_context *ce = container_of(kref, typeof(*ce), ref);

	GEM_BUG_ON(!i915_active_is_idle(&ce->active));
	GEM_BUG_ON(intel_context_is_pinned(ce));

	if (ce->state)
		__execlists_context_fini(ce);

	intel_context_fini(ce);
	intel_context_free(ce);
}

static void
set_redzone(void *vaddr, const struct intel_engine_cs *engine)
{
	if (!IS_ENABLED(CONFIG_DRM_I915_DEBUG_GEM))
		return;

	vaddr += engine->context_size;

	memset(vaddr, CONTEXT_REDZONE, I915_GTT_PAGE_SIZE);
}

static void
check_redzone(const void *vaddr, const struct intel_engine_cs *engine)
{
	if (!IS_ENABLED(CONFIG_DRM_I915_DEBUG_GEM))
		return;

	vaddr += engine->context_size;

	if (memchr_inv(vaddr, CONTEXT_REDZONE, I915_GTT_PAGE_SIZE))
		drm_err_once(&engine->i915->drm,
			     "%s context redzone overwritten!\n",
			     engine->name);
}

static void execlists_context_unpin(struct intel_context *ce)
{
	check_redzone((void *)ce->lrc_reg_state - LRC_STATE_OFFSET,
		      ce->engine);
}

static void execlists_context_post_unpin(struct intel_context *ce)
{
	i915_gem_object_unpin_map(ce->state->obj);
}

static u32 *
gen12_emit_timestamp_wa(const struct intel_context *ce, u32 *cs)
{
	*cs++ = MI_LOAD_REGISTER_MEM_GEN8 |
		MI_SRM_LRM_GLOBAL_GTT |
		MI_LRI_LRM_CS_MMIO;
	*cs++ = i915_mmio_reg_offset(GEN8_RING_CS_GPR(0, 0));
	*cs++ = i915_ggtt_offset(ce->state) + LRC_STATE_OFFSET +
		CTX_TIMESTAMP * sizeof(u32);
	*cs++ = 0;

	*cs++ = MI_LOAD_REGISTER_REG |
		MI_LRR_SOURCE_CS_MMIO |
		MI_LRI_LRM_CS_MMIO;
	*cs++ = i915_mmio_reg_offset(GEN8_RING_CS_GPR(0, 0));
	*cs++ = i915_mmio_reg_offset(RING_CTX_TIMESTAMP(0));

	*cs++ = MI_LOAD_REGISTER_REG |
		MI_LRR_SOURCE_CS_MMIO |
		MI_LRI_LRM_CS_MMIO;
	*cs++ = i915_mmio_reg_offset(GEN8_RING_CS_GPR(0, 0));
	*cs++ = i915_mmio_reg_offset(RING_CTX_TIMESTAMP(0));

	return cs;
}

static u32 *
gen12_emit_restore_scratch(const struct intel_context *ce, u32 *cs)
{
	GEM_BUG_ON(lrc_ring_gpr0(ce->engine) == -1);

	*cs++ = MI_LOAD_REGISTER_MEM_GEN8 |
		MI_SRM_LRM_GLOBAL_GTT |
		MI_LRI_LRM_CS_MMIO;
	*cs++ = i915_mmio_reg_offset(GEN8_RING_CS_GPR(0, 0));
	*cs++ = i915_ggtt_offset(ce->state) + LRC_STATE_OFFSET +
		(lrc_ring_gpr0(ce->engine) + 1) * sizeof(u32);
	*cs++ = 0;

	return cs;
}

static u32 *
gen12_emit_cmd_buf_wa(const struct intel_context *ce, u32 *cs)
{
	GEM_BUG_ON(lrc_ring_cmd_buf_cctl(ce->engine) == -1);

	*cs++ = MI_LOAD_REGISTER_MEM_GEN8 |
		MI_SRM_LRM_GLOBAL_GTT |
		MI_LRI_LRM_CS_MMIO;
	*cs++ = i915_mmio_reg_offset(GEN8_RING_CS_GPR(0, 0));
	*cs++ = i915_ggtt_offset(ce->state) + LRC_STATE_OFFSET +
		(lrc_ring_cmd_buf_cctl(ce->engine) + 1) * sizeof(u32);
	*cs++ = 0;

	*cs++ = MI_LOAD_REGISTER_REG |
		MI_LRR_SOURCE_CS_MMIO |
		MI_LRI_LRM_CS_MMIO;
	*cs++ = i915_mmio_reg_offset(GEN8_RING_CS_GPR(0, 0));
	*cs++ = i915_mmio_reg_offset(RING_CMD_BUF_CCTL(0));

	return cs;
}

static u32 *
gen12_emit_indirect_ctx_rcs(const struct intel_context *ce, u32 *cs)
{
	cs = gen12_emit_timestamp_wa(ce, cs);
	cs = gen12_emit_cmd_buf_wa(ce, cs);
	cs = gen12_emit_restore_scratch(ce, cs);

	return cs;
}

static u32 *
gen12_emit_indirect_ctx_xcs(const struct intel_context *ce, u32 *cs)
{
	cs = gen12_emit_timestamp_wa(ce, cs);
	cs = gen12_emit_restore_scratch(ce, cs);

	return cs;
}

static inline u32 context_wa_bb_offset(const struct intel_context *ce)
{
	return PAGE_SIZE * ce->wa_bb_page;
}

static u32 *context_indirect_bb(const struct intel_context *ce)
{
	void *ptr;

	GEM_BUG_ON(!ce->wa_bb_page);

	ptr = ce->lrc_reg_state;
	ptr -= LRC_STATE_OFFSET; /* back to start of context image */
	ptr += context_wa_bb_offset(ce);

	return ptr;
}

static void
setup_indirect_ctx_bb(const struct intel_context *ce,
		      const struct intel_engine_cs *engine,
		      u32 *(*emit)(const struct intel_context *, u32 *))
{
	u32 * const start = context_indirect_bb(ce);
	u32 *cs;

	cs = emit(ce, start);
	GEM_BUG_ON(cs - start > I915_GTT_PAGE_SIZE / sizeof(*cs));
	while ((unsigned long)cs % CACHELINE_BYTES)
		*cs++ = MI_NOOP;

	lrc_ring_setup_indirect_ctx(ce->lrc_reg_state, engine,
				    i915_ggtt_offset(ce->state) +
				    context_wa_bb_offset(ce),
				    (cs - start) * sizeof(*cs));
}

static void
__execlists_update_reg_state(const struct intel_context *ce,
			     const struct intel_engine_cs *engine,
			     u32 head)
{
	struct intel_ring *ring = ce->ring;
	u32 *regs = ce->lrc_reg_state;

	GEM_BUG_ON(!intel_ring_offset_valid(ring, head));
	GEM_BUG_ON(!intel_ring_offset_valid(ring, ring->tail));

	regs[CTX_RING_START] = i915_ggtt_offset(ring->vma);
	regs[CTX_RING_HEAD] = head;
	regs[CTX_RING_TAIL] = ring->tail;
	regs[CTX_RING_CTL] = RING_CTL_SIZE(ring->size) | RING_VALID;

	/* RPCS */
	if (engine->class == RENDER_CLASS) {
		regs[CTX_R_PWR_CLK_STATE] =
			intel_sseu_make_rpcs(engine->gt, &ce->sseu);

		i915_oa_init_reg_state(ce, engine);
	}

	if (ce->wa_bb_page) {
		u32 *(*fn)(const struct intel_context *ce, u32 *cs);

		fn = gen12_emit_indirect_ctx_xcs;
		if (ce->engine->class == RENDER_CLASS)
			fn = gen12_emit_indirect_ctx_rcs;

		/* Mutually exclusive wrt to global indirect bb */
		GEM_BUG_ON(engine->wa_ctx.indirect_ctx.size);
		setup_indirect_ctx_bb(ce, engine, fn);
	}
}

static int
execlists_context_pre_pin(struct intel_context *ce,
			  struct i915_gem_ww_ctx *ww, void **vaddr)
{
	GEM_BUG_ON(!ce->state);
	GEM_BUG_ON(!i915_vma_is_pinned(ce->state));

	*vaddr = i915_gem_object_pin_map(ce->state->obj,
					i915_coherent_map_type(ce->engine->i915) |
					I915_MAP_OVERRIDE);

	return PTR_ERR_OR_ZERO(*vaddr);
}

static int
__execlists_context_pin(struct intel_context *ce,
			struct intel_engine_cs *engine,
			void *vaddr)
{
	ce->lrc.lrca = lrc_descriptor(ce, engine) | CTX_DESC_FORCE_RESTORE;
	ce->lrc_reg_state = vaddr + LRC_STATE_OFFSET;
	__execlists_update_reg_state(ce, engine, ce->ring->tail);

	return 0;
}

static int execlists_context_pin(struct intel_context *ce, void *vaddr)
{
	return __execlists_context_pin(ce, ce->engine, vaddr);
}

static int execlists_context_alloc(struct intel_context *ce)
{
	return __execlists_context_alloc(ce, ce->engine);
}

static void execlists_context_reset(struct intel_context *ce)
{
	CE_TRACE(ce, "reset\n");
	GEM_BUG_ON(!intel_context_is_pinned(ce));

	intel_ring_reset(ce->ring, ce->ring->emit);

	/* Scrub away the garbage */
	execlists_init_reg_state(ce->lrc_reg_state,
				 ce, ce->engine, ce->ring, true);
	__execlists_update_reg_state(ce, ce->engine, ce->ring->tail);

	ce->lrc.desc |= CTX_DESC_FORCE_RESTORE;
}

static const struct intel_context_ops execlists_context_ops = {
	.alloc = execlists_context_alloc,

	.pre_pin = execlists_context_pre_pin,
	.pin = execlists_context_pin,
	.unpin = execlists_context_unpin,
	.post_unpin = execlists_context_post_unpin,

	.enter = intel_context_enter_engine,
	.exit = intel_context_exit_engine,

	.reset = execlists_context_reset,
	.destroy = execlists_context_destroy,
};

static u32 hwsp_offset(const struct i915_request *rq)
{
	const struct intel_timeline_cacheline *cl;

	/* Before the request is executed, the timeline/cachline is fixed */

	cl = rcu_dereference_protected(rq->hwsp_cacheline, 1);
	if (cl)
		return cl->ggtt_offset;

	return rcu_dereference_protected(rq->timeline, 1)->hwsp_offset;
}

static int gen8_emit_init_breadcrumb(struct i915_request *rq)
{
	u32 *cs;

	GEM_BUG_ON(i915_request_has_initial_breadcrumb(rq));
	if (!i915_request_timeline(rq)->has_initial_breadcrumb)
		return 0;

	cs = intel_ring_begin(rq, 6);
	if (IS_ERR(cs))
		return PTR_ERR(cs);

	/*
	 * Check if we have been preempted before we even get started.
	 *
	 * After this point i915_request_started() reports true, even if
	 * we get preempted and so are no longer running.
	 */
	*cs++ = MI_ARB_CHECK;
	*cs++ = MI_NOOP;

	*cs++ = MI_STORE_DWORD_IMM_GEN4 | MI_USE_GGTT;
	*cs++ = hwsp_offset(rq);
	*cs++ = 0;
	*cs++ = rq->fence.seqno - 1;

	intel_ring_advance(rq, cs);

	/* Record the updated position of the request's payload */
	rq->infix = intel_ring_offset(rq, cs);

	__set_bit(I915_FENCE_FLAG_INITIAL_BREADCRUMB, &rq->fence.flags);

	return 0;
}

static int emit_pdps(struct i915_request *rq)
{
	const struct intel_engine_cs * const engine = rq->engine;
	struct i915_ppgtt * const ppgtt = i915_vm_to_ppgtt(rq->context->vm);
	int err, i;
	u32 *cs;

	GEM_BUG_ON(intel_vgpu_active(rq->engine->i915));

	/*
	 * Beware ye of the dragons, this sequence is magic!
	 *
	 * Small changes to this sequence can cause anything from
	 * GPU hangs to forcewake errors and machine lockups!
	 */

	/* Flush any residual operations from the context load */
	err = engine->emit_flush(rq, EMIT_FLUSH);
	if (err)
		return err;

	/* Magic required to prevent forcewake errors! */
	err = engine->emit_flush(rq, EMIT_INVALIDATE);
	if (err)
		return err;

	cs = intel_ring_begin(rq, 4 * GEN8_3LVL_PDPES + 2);
	if (IS_ERR(cs))
		return PTR_ERR(cs);

	/* Ensure the LRI have landed before we invalidate & continue */
	*cs++ = MI_LOAD_REGISTER_IMM(2 * GEN8_3LVL_PDPES) | MI_LRI_FORCE_POSTED;
	for (i = GEN8_3LVL_PDPES; i--; ) {
		const dma_addr_t pd_daddr = i915_page_dir_dma_addr(ppgtt, i);
		u32 base = engine->mmio_base;

		*cs++ = i915_mmio_reg_offset(GEN8_RING_PDP_UDW(base, i));
		*cs++ = upper_32_bits(pd_daddr);
		*cs++ = i915_mmio_reg_offset(GEN8_RING_PDP_LDW(base, i));
		*cs++ = lower_32_bits(pd_daddr);
	}
	*cs++ = MI_NOOP;

	intel_ring_advance(rq, cs);

	return 0;
}

static int execlists_request_alloc(struct i915_request *request)
{
	int ret;

	GEM_BUG_ON(!intel_context_is_pinned(request->context));

	/*
	 * Flush enough space to reduce the likelihood of waiting after
	 * we start building the request - in which case we will just
	 * have to repeat work.
	 */
	request->reserved_space += EXECLISTS_REQUEST_SIZE;

	/*
	 * Note that after this point, we have committed to using
	 * this request as it is being used to both track the
	 * state of engine initialisation and liveness of the
	 * golden renderstate above. Think twice before you try
	 * to cancel/unwind this request now.
	 */

	if (!i915_vm_is_4lvl(request->context->vm)) {
		ret = emit_pdps(request);
		if (ret)
			return ret;
	}

	/* Unconditionally invalidate GPU caches and TLBs. */
	ret = request->engine->emit_flush(request, EMIT_INVALIDATE);
	if (ret)
		return ret;

	request->reserved_space -= EXECLISTS_REQUEST_SIZE;
	return 0;
}

/*
 * In this WA we need to set GEN8_L3SQCREG4[21:21] and reset it after
 * PIPE_CONTROL instruction. This is required for the flush to happen correctly
 * but there is a slight complication as this is applied in WA batch where the
 * values are only initialized once so we cannot take register value at the
 * beginning and reuse it further; hence we save its value to memory, upload a
 * constant value with bit21 set and then we restore it back with the saved value.
 * To simplify the WA, a constant value is formed by using the default value
 * of this register. This shouldn't be a problem because we are only modifying
 * it for a short period and this batch in non-premptible. We can ofcourse
 * use additional instructions that read the actual value of the register
 * at that time and set our bit of interest but it makes the WA complicated.
 *
 * This WA is also required for Gen9 so extracting as a function avoids
 * code duplication.
 */
static u32 *
gen8_emit_flush_coherentl3_wa(struct intel_engine_cs *engine, u32 *batch)
{
	/* NB no one else is allowed to scribble over scratch + 256! */
	*batch++ = MI_STORE_REGISTER_MEM_GEN8 | MI_SRM_LRM_GLOBAL_GTT;
	*batch++ = i915_mmio_reg_offset(GEN8_L3SQCREG4);
	*batch++ = intel_gt_scratch_offset(engine->gt,
					   INTEL_GT_SCRATCH_FIELD_COHERENTL3_WA);
	*batch++ = 0;

	*batch++ = MI_LOAD_REGISTER_IMM(1);
	*batch++ = i915_mmio_reg_offset(GEN8_L3SQCREG4);
	*batch++ = 0x40400000 | GEN8_LQSC_FLUSH_COHERENT_LINES;

	batch = gen8_emit_pipe_control(batch,
				       PIPE_CONTROL_CS_STALL |
				       PIPE_CONTROL_DC_FLUSH_ENABLE,
				       0);

	*batch++ = MI_LOAD_REGISTER_MEM_GEN8 | MI_SRM_LRM_GLOBAL_GTT;
	*batch++ = i915_mmio_reg_offset(GEN8_L3SQCREG4);
	*batch++ = intel_gt_scratch_offset(engine->gt,
					   INTEL_GT_SCRATCH_FIELD_COHERENTL3_WA);
	*batch++ = 0;

	return batch;
}

/*
 * Typically we only have one indirect_ctx and per_ctx batch buffer which are
 * initialized at the beginning and shared across all contexts but this field
 * helps us to have multiple batches at different offsets and select them based
 * on a criteria. At the moment this batch always start at the beginning of the page
 * and at this point we don't have multiple wa_ctx batch buffers.
 *
 * The number of WA applied are not known at the beginning; we use this field
 * to return the no of DWORDS written.
 *
 * It is to be noted that this batch does not contain MI_BATCH_BUFFER_END
 * so it adds NOOPs as padding to make it cacheline aligned.
 * MI_BATCH_BUFFER_END will be added to perctx batch and both of them together
 * makes a complete batch buffer.
 */
static u32 *gen8_init_indirectctx_bb(struct intel_engine_cs *engine, u32 *batch)
{
	/* WaDisableCtxRestoreArbitration:bdw,chv */
	*batch++ = MI_ARB_ON_OFF | MI_ARB_DISABLE;

	/* WaFlushCoherentL3CacheLinesAtContextSwitch:bdw */
	if (IS_BROADWELL(engine->i915))
		batch = gen8_emit_flush_coherentl3_wa(engine, batch);

	/* WaClearSlmSpaceAtContextSwitch:bdw,chv */
	/* Actual scratch location is at 128 bytes offset */
	batch = gen8_emit_pipe_control(batch,
				       PIPE_CONTROL_FLUSH_L3 |
				       PIPE_CONTROL_STORE_DATA_INDEX |
				       PIPE_CONTROL_CS_STALL |
				       PIPE_CONTROL_QW_WRITE,
				       LRC_PPHWSP_SCRATCH_ADDR);

	*batch++ = MI_ARB_ON_OFF | MI_ARB_ENABLE;

	/* Pad to end of cacheline */
	while ((unsigned long)batch % CACHELINE_BYTES)
		*batch++ = MI_NOOP;

	/*
	 * MI_BATCH_BUFFER_END is not required in Indirect ctx BB because
	 * execution depends on the length specified in terms of cache lines
	 * in the register CTX_RCS_INDIRECT_CTX
	 */

	return batch;
}

struct lri {
	i915_reg_t reg;
	u32 value;
};

static u32 *emit_lri(u32 *batch, const struct lri *lri, unsigned int count)
{
	GEM_BUG_ON(!count || count > 63);

	*batch++ = MI_LOAD_REGISTER_IMM(count);
	do {
		*batch++ = i915_mmio_reg_offset(lri->reg);
		*batch++ = lri->value;
	} while (lri++, --count);
	*batch++ = MI_NOOP;

	return batch;
}

static u32 *gen9_init_indirectctx_bb(struct intel_engine_cs *engine, u32 *batch)
{
	static const struct lri lri[] = {
		/* WaDisableGatherAtSetShaderCommonSlice:skl,bxt,kbl,glk */
		{
			COMMON_SLICE_CHICKEN2,
			__MASKED_FIELD(GEN9_DISABLE_GATHER_AT_SET_SHADER_COMMON_SLICE,
				       0),
		},

		/* BSpec: 11391 */
		{
			FF_SLICE_CHICKEN,
			__MASKED_FIELD(FF_SLICE_CHICKEN_CL_PROVOKING_VERTEX_FIX,
				       FF_SLICE_CHICKEN_CL_PROVOKING_VERTEX_FIX),
		},

		/* BSpec: 11299 */
		{
			_3D_CHICKEN3,
			__MASKED_FIELD(_3D_CHICKEN_SF_PROVOKING_VERTEX_FIX,
				       _3D_CHICKEN_SF_PROVOKING_VERTEX_FIX),
		}
	};

	*batch++ = MI_ARB_ON_OFF | MI_ARB_DISABLE;

	/* WaFlushCoherentL3CacheLinesAtContextSwitch:skl,bxt,glk */
	batch = gen8_emit_flush_coherentl3_wa(engine, batch);

	/* WaClearSlmSpaceAtContextSwitch:skl,bxt,kbl,glk,cfl */
	batch = gen8_emit_pipe_control(batch,
				       PIPE_CONTROL_FLUSH_L3 |
				       PIPE_CONTROL_STORE_DATA_INDEX |
				       PIPE_CONTROL_CS_STALL |
				       PIPE_CONTROL_QW_WRITE,
				       LRC_PPHWSP_SCRATCH_ADDR);

	batch = emit_lri(batch, lri, ARRAY_SIZE(lri));

	/* WaMediaPoolStateCmdInWABB:bxt,glk */
	if (HAS_POOLED_EU(engine->i915)) {
		/*
		 * EU pool configuration is setup along with golden context
		 * during context initialization. This value depends on
		 * device type (2x6 or 3x6) and needs to be updated based
		 * on which subslice is disabled especially for 2x6
		 * devices, however it is safe to load default
		 * configuration of 3x6 device instead of masking off
		 * corresponding bits because HW ignores bits of a disabled
		 * subslice and drops down to appropriate config. Please
		 * see render_state_setup() in i915_gem_render_state.c for
		 * possible configurations, to avoid duplication they are
		 * not shown here again.
		 */
		*batch++ = GEN9_MEDIA_POOL_STATE;
		*batch++ = GEN9_MEDIA_POOL_ENABLE;
		*batch++ = 0x00777000;
		*batch++ = 0;
		*batch++ = 0;
		*batch++ = 0;
	}

	*batch++ = MI_ARB_ON_OFF | MI_ARB_ENABLE;

	/* Pad to end of cacheline */
	while ((unsigned long)batch % CACHELINE_BYTES)
		*batch++ = MI_NOOP;

	return batch;
}

static u32 *
gen10_init_indirectctx_bb(struct intel_engine_cs *engine, u32 *batch)
{
	int i;

	/*
	 * WaPipeControlBefore3DStateSamplePattern: cnl
	 *
	 * Ensure the engine is idle prior to programming a
	 * 3DSTATE_SAMPLE_PATTERN during a context restore.
	 */
	batch = gen8_emit_pipe_control(batch,
				       PIPE_CONTROL_CS_STALL,
				       0);
	/*
	 * WaPipeControlBefore3DStateSamplePattern says we need 4 dwords for
	 * the PIPE_CONTROL followed by 12 dwords of 0x0, so 16 dwords in
	 * total. However, a PIPE_CONTROL is 6 dwords long, not 4, which is
	 * confusing. Since gen8_emit_pipe_control() already advances the
	 * batch by 6 dwords, we advance the other 10 here, completing a
	 * cacheline. It's not clear if the workaround requires this padding
	 * before other commands, or if it's just the regular padding we would
	 * already have for the workaround bb, so leave it here for now.
	 */
	for (i = 0; i < 10; i++)
		*batch++ = MI_NOOP;

	/* Pad to end of cacheline */
	while ((unsigned long)batch % CACHELINE_BYTES)
		*batch++ = MI_NOOP;

	return batch;
}

#define CTX_WA_BB_OBJ_SIZE (PAGE_SIZE)

static int lrc_setup_wa_ctx(struct intel_engine_cs *engine)
{
	struct drm_i915_gem_object *obj;
	struct i915_vma *vma;
	int err;

	obj = i915_gem_object_create_shmem(engine->i915, CTX_WA_BB_OBJ_SIZE);
	if (IS_ERR(obj))
		return PTR_ERR(obj);

	vma = i915_vma_instance(obj, &engine->gt->ggtt->vm, NULL);
	if (IS_ERR(vma)) {
		err = PTR_ERR(vma);
		goto err;
	}

	err = i915_ggtt_pin(vma, NULL, 0, PIN_HIGH);
	if (err)
		goto err;

	engine->wa_ctx.vma = vma;
	return 0;

err:
	i915_gem_object_put(obj);
	return err;
}

static void lrc_destroy_wa_ctx(struct intel_engine_cs *engine)
{
	i915_vma_unpin_and_release(&engine->wa_ctx.vma, 0);

	/* Called on error unwind, clear all flags to prevent further use */
	memset(&engine->wa_ctx, 0, sizeof(engine->wa_ctx));
}

typedef u32 *(*wa_bb_func_t)(struct intel_engine_cs *engine, u32 *batch);

static int intel_init_workaround_bb(struct intel_engine_cs *engine)
{
	struct i915_ctx_workarounds *wa_ctx = &engine->wa_ctx;
	struct i915_wa_ctx_bb *wa_bb[2] = { &wa_ctx->indirect_ctx,
					    &wa_ctx->per_ctx };
	wa_bb_func_t wa_bb_fn[2];
	void *batch, *batch_ptr;
	unsigned int i;
	int ret;

	if (engine->class != RENDER_CLASS)
		return 0;

	switch (INTEL_GEN(engine->i915)) {
	case 12:
	case 11:
		return 0;
	case 10:
		wa_bb_fn[0] = gen10_init_indirectctx_bb;
		wa_bb_fn[1] = NULL;
		break;
	case 9:
		wa_bb_fn[0] = gen9_init_indirectctx_bb;
		wa_bb_fn[1] = NULL;
		break;
	case 8:
		wa_bb_fn[0] = gen8_init_indirectctx_bb;
		wa_bb_fn[1] = NULL;
		break;
	default:
		MISSING_CASE(INTEL_GEN(engine->i915));
		return 0;
	}

	ret = lrc_setup_wa_ctx(engine);
	if (ret) {
		drm_dbg(&engine->i915->drm,
			"Failed to setup context WA page: %d\n", ret);
		return ret;
	}

	batch = i915_gem_object_pin_map(wa_ctx->vma->obj, I915_MAP_WB);

	/*
	 * Emit the two workaround batch buffers, recording the offset from the
	 * start of the workaround batch buffer object for each and their
	 * respective sizes.
	 */
	batch_ptr = batch;
	for (i = 0; i < ARRAY_SIZE(wa_bb_fn); i++) {
		wa_bb[i]->offset = batch_ptr - batch;
		if (GEM_DEBUG_WARN_ON(!IS_ALIGNED(wa_bb[i]->offset,
						  CACHELINE_BYTES))) {
			ret = -EINVAL;
			break;
		}
		if (wa_bb_fn[i])
			batch_ptr = wa_bb_fn[i](engine, batch_ptr);
		wa_bb[i]->size = batch_ptr - (batch + wa_bb[i]->offset);
	}
	GEM_BUG_ON(batch_ptr - batch > CTX_WA_BB_OBJ_SIZE);

	__i915_gem_object_flush_map(wa_ctx->vma->obj, 0, batch_ptr - batch);
	__i915_gem_object_release_map(wa_ctx->vma->obj);
	if (ret)
		lrc_destroy_wa_ctx(engine);

	return ret;
}

static void reset_csb_pointers(struct intel_engine_cs *engine)
{
	struct intel_engine_execlists * const execlists = &engine->execlists;
	const unsigned int reset_value = execlists->csb_size - 1;

	ring_set_paused(engine, 0);

	/*
	 * Sometimes Icelake forgets to reset its pointers on a GPU reset.
	 * Bludgeon them with a mmio update to be sure.
	 */
	ENGINE_WRITE(engine, RING_CONTEXT_STATUS_PTR,
		     0xffff << 16 | reset_value << 8 | reset_value);
	ENGINE_POSTING_READ(engine, RING_CONTEXT_STATUS_PTR);

	/*
	 * After a reset, the HW starts writing into CSB entry [0]. We
	 * therefore have to set our HEAD pointer back one entry so that
	 * the *first* entry we check is entry 0. To complicate this further,
	 * as we don't wait for the first interrupt after reset, we have to
	 * fake the HW write to point back to the last entry so that our
	 * inline comparison of our cached head position against the last HW
	 * write works even before the first interrupt.
	 */
	execlists->csb_head = reset_value;
	WRITE_ONCE(*execlists->csb_write, reset_value);
	wmb(); /* Make sure this is visible to HW (paranoia?) */

	/* Check that the GPU does indeed update the CSB entries! */
	memset(execlists->csb_status, -1, (reset_value + 1) * sizeof(u64));
	invalidate_csb_entries(&execlists->csb_status[0],
			       &execlists->csb_status[reset_value]);

	/* Once more for luck and our trusty paranoia */
	ENGINE_WRITE(engine, RING_CONTEXT_STATUS_PTR,
		     0xffff << 16 | reset_value << 8 | reset_value);
	ENGINE_POSTING_READ(engine, RING_CONTEXT_STATUS_PTR);

	GEM_BUG_ON(READ_ONCE(*execlists->csb_write) != reset_value);
}

static void execlists_sanitize(struct intel_engine_cs *engine)
{
	/*
	 * Poison residual state on resume, in case the suspend didn't!
	 *
	 * We have to assume that across suspend/resume (or other loss
	 * of control) that the contents of our pinned buffers has been
	 * lost, replaced by garbage. Since this doesn't always happen,
	 * let's poison such state so that we more quickly spot when
	 * we falsely assume it has been preserved.
	 */
	if (IS_ENABLED(CONFIG_DRM_I915_DEBUG_GEM))
		memset(engine->status_page.addr, POISON_INUSE, PAGE_SIZE);

	reset_csb_pointers(engine);

	/*
	 * The kernel_context HWSP is stored in the status_page. As above,
	 * that may be lost on resume/initialisation, and so we need to
	 * reset the value in the HWSP.
	 */
	intel_timeline_reset_seqno(engine->kernel_context->timeline);

	/* And scrub the dirty cachelines for the HWSP */
	clflush_cache_range(engine->status_page.addr, PAGE_SIZE);
}

static void enable_error_interrupt(struct intel_engine_cs *engine)
{
	u32 status;

	engine->execlists.error_interrupt = 0;
	ENGINE_WRITE(engine, RING_EMR, ~0u);
	ENGINE_WRITE(engine, RING_EIR, ~0u); /* clear all existing errors */

	status = ENGINE_READ(engine, RING_ESR);
	if (unlikely(status)) {
		drm_err(&engine->i915->drm,
			"engine '%s' resumed still in error: %08x\n",
			engine->name, status);
		__intel_gt_reset(engine->gt, engine->mask);
	}

	/*
	 * On current gen8+, we have 2 signals to play with
	 *
	 * - I915_ERROR_INSTUCTION (bit 0)
	 *
	 *    Generate an error if the command parser encounters an invalid
	 *    instruction
	 *
	 *    This is a fatal error.
	 *
	 * - CP_PRIV (bit 2)
	 *
	 *    Generate an error on privilege violation (where the CP replaces
	 *    the instruction with a no-op). This also fires for writes into
	 *    read-only scratch pages.
	 *
	 *    This is a non-fatal error, parsing continues.
	 *
	 * * there are a few others defined for odd HW that we do not use
	 *
	 * Since CP_PRIV fires for cases where we have chosen to ignore the
	 * error (as the HW is validating and suppressing the mistakes), we
	 * only unmask the instruction error bit.
	 */
	ENGINE_WRITE(engine, RING_EMR, ~I915_ERROR_INSTRUCTION);
}

static void enable_execlists(struct intel_engine_cs *engine)
{
	u32 mode;

	assert_forcewakes_active(engine->uncore, FORCEWAKE_ALL);

	intel_engine_set_hwsp_writemask(engine, ~0u); /* HWSTAM */

	if (INTEL_GEN(engine->i915) >= 11)
		mode = _MASKED_BIT_ENABLE(GEN11_GFX_DISABLE_LEGACY_MODE);
	else
		mode = _MASKED_BIT_ENABLE(GFX_RUN_LIST_ENABLE);
	ENGINE_WRITE_FW(engine, RING_MODE_GEN7, mode);

	ENGINE_WRITE_FW(engine, RING_MI_MODE, _MASKED_BIT_DISABLE(STOP_RING));

	ENGINE_WRITE_FW(engine,
			RING_HWS_PGA,
			i915_ggtt_offset(engine->status_page.vma));
	ENGINE_POSTING_READ(engine, RING_HWS_PGA);

	enable_error_interrupt(engine);

	engine->context_tag = GENMASK(BITS_PER_LONG - 2, 0);
}

static bool unexpected_starting_state(struct intel_engine_cs *engine)
{
	bool unexpected = false;

	if (ENGINE_READ_FW(engine, RING_MI_MODE) & STOP_RING) {
		drm_dbg(&engine->i915->drm,
			"STOP_RING still set in RING_MI_MODE\n");
		unexpected = true;
	}

	return unexpected;
}

static int execlists_resume(struct intel_engine_cs *engine)
{
	intel_mocs_init_engine(engine);

	intel_breadcrumbs_reset(engine->breadcrumbs);

	if (GEM_SHOW_DEBUG() && unexpected_starting_state(engine)) {
		struct drm_printer p = drm_debug_printer(__func__);

		intel_engine_dump(engine, &p, NULL);
	}

	enable_execlists(engine);

	return 0;
}

static void execlists_reset_prepare(struct intel_engine_cs *engine)
{
	struct intel_engine_execlists * const execlists = &engine->execlists;
	unsigned long flags;

	ENGINE_TRACE(engine, "depth<-%d\n",
		     atomic_read(&execlists->tasklet.count));

	/*
	 * Prevent request submission to the hardware until we have
	 * completed the reset in i915_gem_reset_finish(). If a request
	 * is completed by one engine, it may then queue a request
	 * to a second via its execlists->tasklet *just* as we are
	 * calling engine->resume() and also writing the ELSP.
	 * Turning off the execlists->tasklet until the reset is over
	 * prevents the race.
	 */
	__tasklet_disable_sync_once(&execlists->tasklet);
	GEM_BUG_ON(!reset_in_progress(execlists));

	/* And flush any current direct submission. */
	spin_lock_irqsave(&engine->active.lock, flags);
	spin_unlock_irqrestore(&engine->active.lock, flags);

	/*
	 * We stop engines, otherwise we might get failed reset and a
	 * dead gpu (on elk). Also as modern gpu as kbl can suffer
	 * from system hang if batchbuffer is progressing when
	 * the reset is issued, regardless of READY_TO_RESET ack.
	 * Thus assume it is best to stop engines on all gens
	 * where we have a gpu reset.
	 *
	 * WaKBLVECSSemaphoreWaitPoll:kbl (on ALL_ENGINES)
	 *
	 * FIXME: Wa for more modern gens needs to be validated
	 */
	ring_set_paused(engine, 1);
	intel_engine_stop_cs(engine);

	engine->execlists.reset_ccid = active_ccid(engine);
}

static void __reset_stop_ring(u32 *regs, const struct intel_engine_cs *engine)
{
	int x;

	x = lrc_ring_mi_mode(engine);
	if (x != -1) {
		regs[x + 1] &= ~STOP_RING;
		regs[x + 1] |= STOP_RING << 16;
	}
}

static void __execlists_reset_reg_state(const struct intel_context *ce,
					const struct intel_engine_cs *engine)
{
	u32 *regs = ce->lrc_reg_state;

	__reset_stop_ring(regs, engine);
}

static void __execlists_reset(struct intel_engine_cs *engine, bool stalled)
{
	struct intel_engine_execlists * const execlists = &engine->execlists;
	struct intel_context *ce;
	struct i915_request *rq;
	u32 head;

	mb(); /* paranoia: read the CSB pointers from after the reset */
	clflush(execlists->csb_write);
	mb();

	process_csb(engine); /* drain preemption events */

	/* Following the reset, we need to reload the CSB read/write pointers */
	reset_csb_pointers(engine);

	/*
	 * Save the currently executing context, even if we completed
	 * its request, it was still running at the time of the
	 * reset and will have been clobbered.
	 */
	rq = active_context(engine, engine->execlists.reset_ccid);
	if (!rq)
		goto unwind;

	ce = rq->context;
	GEM_BUG_ON(!i915_vma_is_pinned(ce->state));

	if (i915_request_completed(rq)) {
		/* Idle context; tidy up the ring so we can restart afresh */
		head = intel_ring_wrap(ce->ring, rq->tail);
		goto out_replay;
	}

	/* We still have requests in-flight; the engine should be active */
	GEM_BUG_ON(!intel_engine_pm_is_awake(engine));

	/* Context has requests still in-flight; it should not be idle! */
	GEM_BUG_ON(i915_active_is_idle(&ce->active));

	rq = active_request(ce->timeline, rq);
	head = intel_ring_wrap(ce->ring, rq->head);
	GEM_BUG_ON(head == ce->ring->tail);

	/*
	 * If this request hasn't started yet, e.g. it is waiting on a
	 * semaphore, we need to avoid skipping the request or else we
	 * break the signaling chain. However, if the context is corrupt
	 * the request will not restart and we will be stuck with a wedged
	 * device. It is quite often the case that if we issue a reset
	 * while the GPU is loading the context image, that the context
	 * image becomes corrupt.
	 *
	 * Otherwise, if we have not started yet, the request should replay
	 * perfectly and we do not need to flag the result as being erroneous.
	 */
	if (!i915_request_started(rq))
		goto out_replay;

	/*
	 * If the request was innocent, we leave the request in the ELSP
	 * and will try to replay it on restarting. The context image may
	 * have been corrupted by the reset, in which case we may have
	 * to service a new GPU hang, but more likely we can continue on
	 * without impact.
	 *
	 * If the request was guilty, we presume the context is corrupt
	 * and have to at least restore the RING register in the context
	 * image back to the expected values to skip over the guilty request.
	 */
	__i915_request_reset(rq, stalled);

	/*
	 * We want a simple context + ring to execute the breadcrumb update.
	 * We cannot rely on the context being intact across the GPU hang,
	 * so clear it and rebuild just what we need for the breadcrumb.
	 * All pending requests for this context will be zapped, and any
	 * future request will be after userspace has had the opportunity
	 * to recreate its own state.
	 */
out_replay:
	ENGINE_TRACE(engine, "replay {head:%04x, tail:%04x}\n",
		     head, ce->ring->tail);
	__execlists_reset_reg_state(ce, engine);
	__execlists_update_reg_state(ce, engine, head);
	ce->lrc.desc |= CTX_DESC_FORCE_RESTORE; /* paranoid: GPU was reset! */

unwind:
	/* Push back any incomplete requests for replay after the reset. */
	cancel_port_requests(execlists);
	__unwind_incomplete_requests(engine);
}

static void execlists_reset_rewind(struct intel_engine_cs *engine, bool stalled)
{
	unsigned long flags;

	ENGINE_TRACE(engine, "\n");

	spin_lock_irqsave(&engine->active.lock, flags);

	__execlists_reset(engine, stalled);

	spin_unlock_irqrestore(&engine->active.lock, flags);
}

static void nop_submission_tasklet(unsigned long data)
{
	struct intel_engine_cs * const engine = (struct intel_engine_cs *)data;

	/* The driver is wedged; don't process any more events. */
	WRITE_ONCE(engine->execlists.queue_priority_hint, INT_MIN);
}

static void execlists_reset_cancel(struct intel_engine_cs *engine)
{
	struct intel_engine_execlists * const execlists = &engine->execlists;
	struct i915_request *rq, *rn;
	struct rb_node *rb;
	unsigned long flags;

	ENGINE_TRACE(engine, "\n");

	/*
	 * Before we call engine->cancel_requests(), we should have exclusive
	 * access to the submission state. This is arranged for us by the
	 * caller disabling the interrupt generation, the tasklet and other
	 * threads that may then access the same state, giving us a free hand
	 * to reset state. However, we still need to let lockdep be aware that
	 * we know this state may be accessed in hardirq context, so we
	 * disable the irq around this manipulation and we want to keep
	 * the spinlock focused on its duties and not accidentally conflate
	 * coverage to the submission's irq state. (Similarly, although we
	 * shouldn't need to disable irq around the manipulation of the
	 * submission's irq state, we also wish to remind ourselves that
	 * it is irq state.)
	 */
	spin_lock_irqsave(&engine->active.lock, flags);

	__execlists_reset(engine, true);

	/* Mark all executing requests as skipped. */
	list_for_each_entry(rq, &engine->active.requests, sched.link)
		mark_eio(rq);

	/* Flush the queued requests to the timeline list (for retiring). */
	while ((rb = rb_first_cached(&execlists->queue))) {
		struct i915_priolist *p = to_priolist(rb);
		int i;

		priolist_for_each_request_consume(rq, rn, p, i) {
			mark_eio(rq);
			__i915_request_submit(rq);
		}

		rb_erase_cached(&p->node, &execlists->queue);
		i915_priolist_free(p);
	}

	/* On-hold requests will be flushed to timeline upon their release */
	list_for_each_entry(rq, &engine->active.hold, sched.link)
		mark_eio(rq);

	/* Cancel all attached virtual engines */
	while ((rb = rb_first_cached(&execlists->virtual))) {
		struct virtual_engine *ve =
			rb_entry(rb, typeof(*ve), nodes[engine->id].rb);

		rb_erase_cached(rb, &execlists->virtual);
		RB_CLEAR_NODE(rb);

		spin_lock(&ve->base.active.lock);
		rq = fetch_and_zero(&ve->request);
		if (rq) {
			mark_eio(rq);

			rq->engine = engine;
			__i915_request_submit(rq);
			i915_request_put(rq);

			ve->base.execlists.queue_priority_hint = INT_MIN;
		}
		spin_unlock(&ve->base.active.lock);
	}

	/* Remaining _unready_ requests will be nop'ed when submitted */

	execlists->queue_priority_hint = INT_MIN;
	execlists->queue = RB_ROOT_CACHED;

	GEM_BUG_ON(__tasklet_is_enabled(&execlists->tasklet));
	execlists->tasklet.func = nop_submission_tasklet;

	spin_unlock_irqrestore(&engine->active.lock, flags);
}

static void execlists_reset_finish(struct intel_engine_cs *engine)
{
	struct intel_engine_execlists * const execlists = &engine->execlists;

	/*
	 * After a GPU reset, we may have requests to replay. Do so now while
	 * we still have the forcewake to be sure that the GPU is not allowed
	 * to sleep before we restart and reload a context.
	 */
	GEM_BUG_ON(!reset_in_progress(execlists));
	if (!RB_EMPTY_ROOT(&execlists->queue.rb_root))
		execlists->tasklet.func(execlists->tasklet.data);

	if (__tasklet_enable(&execlists->tasklet))
		/* And kick in case we missed a new request submission. */
		tasklet_hi_schedule(&execlists->tasklet);
	ENGINE_TRACE(engine, "depth->%d\n",
		     atomic_read(&execlists->tasklet.count));
}

static int gen8_emit_bb_start_noarb(struct i915_request *rq,
				    u64 offset, u32 len,
				    const unsigned int flags)
{
	u32 *cs;

	cs = intel_ring_begin(rq, 4);
	if (IS_ERR(cs))
		return PTR_ERR(cs);

	/*
	 * WaDisableCtxRestoreArbitration:bdw,chv
	 *
	 * We don't need to perform MI_ARB_ENABLE as often as we do (in
	 * particular all the gen that do not need the w/a at all!), if we
	 * took care to make sure that on every switch into this context
	 * (both ordinary and for preemption) that arbitrartion was enabled
	 * we would be fine.  However, for gen8 there is another w/a that
	 * requires us to not preempt inside GPGPU execution, so we keep
	 * arbitration disabled for gen8 batches. Arbitration will be
	 * re-enabled before we close the request
	 * (engine->emit_fini_breadcrumb).
	 */
	*cs++ = MI_ARB_ON_OFF | MI_ARB_DISABLE;

	/* FIXME(BDW+): Address space and security selectors. */
	*cs++ = MI_BATCH_BUFFER_START_GEN8 |
		(flags & I915_DISPATCH_SECURE ? 0 : BIT(8));
	*cs++ = lower_32_bits(offset);
	*cs++ = upper_32_bits(offset);

	intel_ring_advance(rq, cs);

	return 0;
}

static int gen8_emit_bb_start(struct i915_request *rq,
			      u64 offset, u32 len,
			      const unsigned int flags)
{
	u32 *cs;

	cs = intel_ring_begin(rq, 6);
	if (IS_ERR(cs))
		return PTR_ERR(cs);

	*cs++ = MI_ARB_ON_OFF | MI_ARB_ENABLE;

	*cs++ = MI_BATCH_BUFFER_START_GEN8 |
		(flags & I915_DISPATCH_SECURE ? 0 : BIT(8));
	*cs++ = lower_32_bits(offset);
	*cs++ = upper_32_bits(offset);

	*cs++ = MI_ARB_ON_OFF | MI_ARB_DISABLE;
	*cs++ = MI_NOOP;

	intel_ring_advance(rq, cs);

	return 0;
}

static void gen8_logical_ring_enable_irq(struct intel_engine_cs *engine)
{
	ENGINE_WRITE(engine, RING_IMR,
		     ~(engine->irq_enable_mask | engine->irq_keep_mask));
	ENGINE_POSTING_READ(engine, RING_IMR);
}

static void gen8_logical_ring_disable_irq(struct intel_engine_cs *engine)
{
	ENGINE_WRITE(engine, RING_IMR, ~engine->irq_keep_mask);
}

static int gen8_emit_flush(struct i915_request *request, u32 mode)
{
	u32 cmd, *cs;

	cs = intel_ring_begin(request, 4);
	if (IS_ERR(cs))
		return PTR_ERR(cs);

	cmd = MI_FLUSH_DW + 1;

	/* We always require a command barrier so that subsequent
	 * commands, such as breadcrumb interrupts, are strictly ordered
	 * wrt the contents of the write cache being flushed to memory
	 * (and thus being coherent from the CPU).
	 */
	cmd |= MI_FLUSH_DW_STORE_INDEX | MI_FLUSH_DW_OP_STOREDW;

	if (mode & EMIT_INVALIDATE) {
		cmd |= MI_INVALIDATE_TLB;
		if (request->engine->class == VIDEO_DECODE_CLASS)
			cmd |= MI_INVALIDATE_BSD;
	}

	*cs++ = cmd;
	*cs++ = LRC_PPHWSP_SCRATCH_ADDR;
	*cs++ = 0; /* upper addr */
	*cs++ = 0; /* value */
	intel_ring_advance(request, cs);

	return 0;
}

static int gen8_emit_flush_render(struct i915_request *request,
				  u32 mode)
{
	bool vf_flush_wa = false, dc_flush_wa = false;
	u32 *cs, flags = 0;
	int len;

	flags |= PIPE_CONTROL_CS_STALL;

	if (mode & EMIT_FLUSH) {
		flags |= PIPE_CONTROL_RENDER_TARGET_CACHE_FLUSH;
		flags |= PIPE_CONTROL_DEPTH_CACHE_FLUSH;
		flags |= PIPE_CONTROL_DC_FLUSH_ENABLE;
		flags |= PIPE_CONTROL_FLUSH_ENABLE;
	}

	if (mode & EMIT_INVALIDATE) {
		flags |= PIPE_CONTROL_TLB_INVALIDATE;
		flags |= PIPE_CONTROL_INSTRUCTION_CACHE_INVALIDATE;
		flags |= PIPE_CONTROL_TEXTURE_CACHE_INVALIDATE;
		flags |= PIPE_CONTROL_VF_CACHE_INVALIDATE;
		flags |= PIPE_CONTROL_CONST_CACHE_INVALIDATE;
		flags |= PIPE_CONTROL_STATE_CACHE_INVALIDATE;
		flags |= PIPE_CONTROL_QW_WRITE;
		flags |= PIPE_CONTROL_STORE_DATA_INDEX;

		/*
		 * On GEN9: before VF_CACHE_INVALIDATE we need to emit a NULL
		 * pipe control.
		 */
		if (IS_GEN(request->engine->i915, 9))
			vf_flush_wa = true;

		/* WaForGAMHang:kbl */
		if (IS_KBL_GT_REVID(request->engine->i915, 0, KBL_REVID_B0))
			dc_flush_wa = true;
	}

	len = 6;

	if (vf_flush_wa)
		len += 6;

	if (dc_flush_wa)
		len += 12;

	cs = intel_ring_begin(request, len);
	if (IS_ERR(cs))
		return PTR_ERR(cs);

	if (vf_flush_wa)
		cs = gen8_emit_pipe_control(cs, 0, 0);

	if (dc_flush_wa)
		cs = gen8_emit_pipe_control(cs, PIPE_CONTROL_DC_FLUSH_ENABLE,
					    0);

	cs = gen8_emit_pipe_control(cs, flags, LRC_PPHWSP_SCRATCH_ADDR);

	if (dc_flush_wa)
		cs = gen8_emit_pipe_control(cs, PIPE_CONTROL_CS_STALL, 0);

	intel_ring_advance(request, cs);

	return 0;
}

static int gen11_emit_flush_render(struct i915_request *request,
				   u32 mode)
{
	if (mode & EMIT_FLUSH) {
		u32 *cs;
		u32 flags = 0;

		flags |= PIPE_CONTROL_CS_STALL;

		flags |= PIPE_CONTROL_TILE_CACHE_FLUSH;
		flags |= PIPE_CONTROL_RENDER_TARGET_CACHE_FLUSH;
		flags |= PIPE_CONTROL_DEPTH_CACHE_FLUSH;
		flags |= PIPE_CONTROL_DC_FLUSH_ENABLE;
		flags |= PIPE_CONTROL_FLUSH_ENABLE;
		flags |= PIPE_CONTROL_QW_WRITE;
		flags |= PIPE_CONTROL_STORE_DATA_INDEX;

		cs = intel_ring_begin(request, 6);
		if (IS_ERR(cs))
			return PTR_ERR(cs);

		cs = gen8_emit_pipe_control(cs, flags, LRC_PPHWSP_SCRATCH_ADDR);
		intel_ring_advance(request, cs);
	}

	if (mode & EMIT_INVALIDATE) {
		u32 *cs;
		u32 flags = 0;

		flags |= PIPE_CONTROL_CS_STALL;

		flags |= PIPE_CONTROL_COMMAND_CACHE_INVALIDATE;
		flags |= PIPE_CONTROL_TLB_INVALIDATE;
		flags |= PIPE_CONTROL_INSTRUCTION_CACHE_INVALIDATE;
		flags |= PIPE_CONTROL_TEXTURE_CACHE_INVALIDATE;
		flags |= PIPE_CONTROL_VF_CACHE_INVALIDATE;
		flags |= PIPE_CONTROL_CONST_CACHE_INVALIDATE;
		flags |= PIPE_CONTROL_STATE_CACHE_INVALIDATE;
		flags |= PIPE_CONTROL_QW_WRITE;
		flags |= PIPE_CONTROL_STORE_DATA_INDEX;

		cs = intel_ring_begin(request, 6);
		if (IS_ERR(cs))
			return PTR_ERR(cs);

		cs = gen8_emit_pipe_control(cs, flags, LRC_PPHWSP_SCRATCH_ADDR);
		intel_ring_advance(request, cs);
	}

	return 0;
}

static u32 preparser_disable(bool state)
{
	return MI_ARB_CHECK | 1 << 8 | state;
}

static i915_reg_t aux_inv_reg(const struct intel_engine_cs *engine)
{
	static const i915_reg_t vd[] = {
		GEN12_VD0_AUX_NV,
		GEN12_VD1_AUX_NV,
		GEN12_VD2_AUX_NV,
		GEN12_VD3_AUX_NV,
	};

	static const i915_reg_t ve[] = {
		GEN12_VE0_AUX_NV,
		GEN12_VE1_AUX_NV,
	};

	if (engine->class == VIDEO_DECODE_CLASS)
		return vd[engine->instance];

	if (engine->class == VIDEO_ENHANCEMENT_CLASS)
		return ve[engine->instance];

	GEM_BUG_ON("unknown aux_inv_reg\n");

	return INVALID_MMIO_REG;
}

static u32 *
gen12_emit_aux_table_inv(const i915_reg_t inv_reg, u32 *cs)
{
	*cs++ = MI_LOAD_REGISTER_IMM(1);
	*cs++ = i915_mmio_reg_offset(inv_reg);
	*cs++ = AUX_INV;
	*cs++ = MI_NOOP;

	return cs;
}

static int gen12_emit_flush_render(struct i915_request *request,
				   u32 mode)
{
	if (mode & EMIT_FLUSH) {
		u32 flags = 0;
		u32 *cs;

		flags |= PIPE_CONTROL_TILE_CACHE_FLUSH;
		flags |= PIPE_CONTROL_FLUSH_L3;
		flags |= PIPE_CONTROL_RENDER_TARGET_CACHE_FLUSH;
		flags |= PIPE_CONTROL_DEPTH_CACHE_FLUSH;
		/* Wa_1409600907:tgl */
		flags |= PIPE_CONTROL_DEPTH_STALL;
		flags |= PIPE_CONTROL_DC_FLUSH_ENABLE;
		flags |= PIPE_CONTROL_FLUSH_ENABLE;

		flags |= PIPE_CONTROL_STORE_DATA_INDEX;
		flags |= PIPE_CONTROL_QW_WRITE;

		flags |= PIPE_CONTROL_CS_STALL;

		cs = intel_ring_begin(request, 6);
		if (IS_ERR(cs))
			return PTR_ERR(cs);

		cs = gen12_emit_pipe_control(cs,
					     PIPE_CONTROL0_HDC_PIPELINE_FLUSH,
					     flags, LRC_PPHWSP_SCRATCH_ADDR);
		intel_ring_advance(request, cs);
	}

	if (mode & EMIT_INVALIDATE) {
		u32 flags = 0;
		u32 *cs;

		flags |= PIPE_CONTROL_COMMAND_CACHE_INVALIDATE;
		flags |= PIPE_CONTROL_TLB_INVALIDATE;
		flags |= PIPE_CONTROL_INSTRUCTION_CACHE_INVALIDATE;
		flags |= PIPE_CONTROL_TEXTURE_CACHE_INVALIDATE;
		flags |= PIPE_CONTROL_VF_CACHE_INVALIDATE;
		flags |= PIPE_CONTROL_CONST_CACHE_INVALIDATE;
		flags |= PIPE_CONTROL_STATE_CACHE_INVALIDATE;

		flags |= PIPE_CONTROL_STORE_DATA_INDEX;
		flags |= PIPE_CONTROL_QW_WRITE;

		flags |= PIPE_CONTROL_CS_STALL;

		cs = intel_ring_begin(request, 8 + 4);
		if (IS_ERR(cs))
			return PTR_ERR(cs);

		/*
		 * Prevent the pre-parser from skipping past the TLB
		 * invalidate and loading a stale page for the batch
		 * buffer / request payload.
		 */
		*cs++ = preparser_disable(true);

		cs = gen8_emit_pipe_control(cs, flags, LRC_PPHWSP_SCRATCH_ADDR);

		/* hsdes: 1809175790 */
		cs = gen12_emit_aux_table_inv(GEN12_GFX_CCS_AUX_NV, cs);

		*cs++ = preparser_disable(false);
		intel_ring_advance(request, cs);
	}

	return 0;
}

static int gen12_emit_flush(struct i915_request *request, u32 mode)
{
	intel_engine_mask_t aux_inv = 0;
	u32 cmd, *cs;

	cmd = 4;
	if (mode & EMIT_INVALIDATE)
		cmd += 2;
	if (mode & EMIT_INVALIDATE)
		aux_inv = request->engine->mask & ~BIT(BCS0);
	if (aux_inv)
		cmd += 2 * hweight8(aux_inv) + 2;

	cs = intel_ring_begin(request, cmd);
	if (IS_ERR(cs))
		return PTR_ERR(cs);

	if (mode & EMIT_INVALIDATE)
		*cs++ = preparser_disable(true);

	cmd = MI_FLUSH_DW + 1;

	/* We always require a command barrier so that subsequent
	 * commands, such as breadcrumb interrupts, are strictly ordered
	 * wrt the contents of the write cache being flushed to memory
	 * (and thus being coherent from the CPU).
	 */
	cmd |= MI_FLUSH_DW_STORE_INDEX | MI_FLUSH_DW_OP_STOREDW;

	if (mode & EMIT_INVALIDATE) {
		cmd |= MI_INVALIDATE_TLB;
		if (request->engine->class == VIDEO_DECODE_CLASS)
			cmd |= MI_INVALIDATE_BSD;
	}

	*cs++ = cmd;
	*cs++ = LRC_PPHWSP_SCRATCH_ADDR;
	*cs++ = 0; /* upper addr */
	*cs++ = 0; /* value */

	if (aux_inv) { /* hsdes: 1809175790 */
		struct intel_engine_cs *engine;
		unsigned int tmp;

		*cs++ = MI_LOAD_REGISTER_IMM(hweight8(aux_inv));
		for_each_engine_masked(engine, request->engine->gt,
				       aux_inv, tmp) {
			*cs++ = i915_mmio_reg_offset(aux_inv_reg(engine));
			*cs++ = AUX_INV;
		}
		*cs++ = MI_NOOP;
	}

	if (mode & EMIT_INVALIDATE)
		*cs++ = preparser_disable(false);

	intel_ring_advance(request, cs);

	return 0;
}

static void assert_request_valid(struct i915_request *rq)
{
	struct intel_ring *ring __maybe_unused = rq->ring;

	/* Can we unwind this request without appearing to go forwards? */
	GEM_BUG_ON(intel_ring_direction(ring, rq->wa_tail, rq->head) <= 0);
}

/*
 * Reserve space for 2 NOOPs at the end of each request to be
 * used as a workaround for not being allowed to do lite
 * restore with HEAD==TAIL (WaIdleLiteRestore).
 */
static u32 *gen8_emit_wa_tail(struct i915_request *request, u32 *cs)
{
	/* Ensure there's always at least one preemption point per-request. */
	*cs++ = MI_ARB_CHECK;
	*cs++ = MI_NOOP;
	request->wa_tail = intel_ring_offset(request, cs);

	/* Check that entire request is less than half the ring */
	assert_request_valid(request);

	return cs;
}

static u32 *emit_preempt_busywait(struct i915_request *request, u32 *cs)
{
	*cs++ = MI_SEMAPHORE_WAIT |
		MI_SEMAPHORE_GLOBAL_GTT |
		MI_SEMAPHORE_POLL |
		MI_SEMAPHORE_SAD_EQ_SDD;
	*cs++ = 0;
	*cs++ = intel_hws_preempt_address(request->engine);
	*cs++ = 0;

	return cs;
}

static __always_inline u32*
gen8_emit_fini_breadcrumb_tail(struct i915_request *request, u32 *cs)
{
	*cs++ = MI_USER_INTERRUPT;

	*cs++ = MI_ARB_ON_OFF | MI_ARB_ENABLE;
	if (intel_engine_has_semaphores(request->engine))
		cs = emit_preempt_busywait(request, cs);

	LRI(1, POSTED),
	REG(0x1b0),

	NOP(10),
	LRI(1, 0),
	REG(0x0c8),

	END
};

static const u8 gen12_rcs_offsets[] = {
	NOP(1),
	LRI(13, POSTED),
	REG16(0x244),
	REG(0x034),
	REG(0x030),
	REG(0x038),
	REG(0x03c),
	REG(0x168),
	REG(0x140),
	REG(0x110),
	REG(0x1c0),
	REG(0x1c4),
	REG(0x1c8),
	REG(0x180),
	REG16(0x2b4),

	NOP(5),
	LRI(9, POSTED),
	REG16(0x3a8),
	REG16(0x28c),
	REG16(0x288),
	REG16(0x284),
	REG16(0x280),
	REG16(0x27c),
	REG16(0x278),
	REG16(0x274),
	REG16(0x270),

	LRI(3, POSTED),
	REG(0x1b0),
	REG16(0x5a8),
	REG16(0x5ac),

	NOP(6),
	LRI(1, 0),
	REG(0x0c8),
	NOP(3 + 9 + 1),

	LRI(51, POSTED),
	REG16(0x588),
	REG16(0x588),
	REG16(0x588),
	REG16(0x588),
	REG16(0x588),
	REG16(0x588),
	REG(0x028),
	REG(0x09c),
	REG(0x0c0),
	REG(0x178),
	REG(0x17c),
	REG16(0x358),
	REG(0x170),
	REG(0x150),
	REG(0x154),
	REG(0x158),
	REG16(0x41c),
	REG16(0x600),
	REG16(0x604),
	REG16(0x608),
	REG16(0x60c),
	REG16(0x610),
	REG16(0x614),
	REG16(0x618),
	REG16(0x61c),
	REG16(0x620),
	REG16(0x624),
	REG16(0x628),
	REG16(0x62c),
	REG16(0x630),
	REG16(0x634),
	REG16(0x638),
	REG16(0x63c),
	REG16(0x640),
	REG16(0x644),
	REG16(0x648),
	REG16(0x64c),
	REG16(0x650),
	REG16(0x654),
	REG16(0x658),
	REG16(0x65c),
	REG16(0x660),
	REG16(0x664),
	REG16(0x668),
	REG16(0x66c),
	REG16(0x670),
	REG16(0x674),
	REG16(0x678),
	REG16(0x67c),
	REG(0x068),
	REG(0x084),
	NOP(1),

	END
};

static const u8 xehp_rcs_offsets[] = {
	NOP(1),
	LRI(13, POSTED),
	REG16(0x244),
	REG(0x034),
	REG(0x030),
	REG(0x038),
	REG(0x03c),
	REG(0x168),
	REG(0x140),
	REG(0x110),
	REG(0x1c0),
	REG(0x1c4),
	REG(0x1c8),
	REG(0x180),
	REG16(0x2b4),

	NOP(5),
	LRI(9, POSTED),
	REG16(0x3a8),
	REG16(0x28c),
	REG16(0x288),
	REG16(0x284),
	REG16(0x280),
	REG16(0x27c),
	REG16(0x278),
	REG16(0x274),
	REG16(0x270),

	LRI(3, POSTED),
	REG(0x1b0),
	REG16(0x5a8),
	REG16(0x5ac),

	NOP(6),
	LRI(1, 0),
	REG(0x0c8),

	END
};

#undef END
#undef REG16
#undef REG
#undef LRI
#undef NOP

static const u8 *reg_offsets(const struct intel_engine_cs *engine)
{
	/*
	 * The gen12+ lists only have the registers we program in the basic
	 * default state. We rely on the context image using relative
	 * addressing to automatic fixup the register state between the
	 * physical engines for virtual engine.
	 */
	GEM_BUG_ON(GRAPHICS_VER(engine->i915) >= 12 &&
		   !intel_engine_has_relative_mmio(engine));

	if (engine->class == RENDER_CLASS) {
		if (GRAPHICS_VER_FULL(engine->i915) >= IP_VER(12, 50))
			return xehp_rcs_offsets;
		else if (GRAPHICS_VER(engine->i915) >= 12)
			return gen12_rcs_offsets;
		else if (GRAPHICS_VER(engine->i915) >= 11)
			return gen11_rcs_offsets;
		else if (GRAPHICS_VER(engine->i915) >= 9)
			return gen9_rcs_offsets;
		else
			return gen8_rcs_offsets;
	} else {
		if (GRAPHICS_VER(engine->i915) >= 12)
			return gen12_xcs_offsets;
		else if (GRAPHICS_VER(engine->i915) >= 9)
			return gen9_xcs_offsets;
		else
			return gen8_xcs_offsets;
	}
}

static int lrc_ring_mi_mode(const struct intel_engine_cs *engine)
{
	if (GRAPHICS_VER_FULL(engine->i915) >= IP_VER(12, 50))
		return 0x70;
	else if (GRAPHICS_VER(engine->i915) >= 12)
		return 0x60;
	else if (GRAPHICS_VER(engine->i915) >= 9)
		return 0x54;
	else if (engine->class == RENDER_CLASS)
		return 0x58;
	else
		return -1;
}

static int lrc_ring_gpr0(const struct intel_engine_cs *engine)
{
	if (GRAPHICS_VER_FULL(engine->i915) >= IP_VER(12, 50))
		return 0x84;
	else if (GRAPHICS_VER(engine->i915) >= 12)
		return 0x74;
	else if (GRAPHICS_VER(engine->i915) >= 9)
		return 0x68;
	else if (engine->class == RENDER_CLASS)
		return 0xd8;
	else
		return -1;
}

static int lrc_ring_wa_bb_per_ctx(const struct intel_engine_cs *engine)
{
	if (GRAPHICS_VER(engine->i915) >= 12)
		return 0x12;
	else if (GRAPHICS_VER(engine->i915) >= 9 || engine->class == RENDER_CLASS)
		return 0x18;
	else
		return -1;
}

static int lrc_ring_indirect_ptr(const struct intel_engine_cs *engine)
{
	int x;

	x = lrc_ring_wa_bb_per_ctx(engine);
	if (x < 0)
		return x;

	return x + 2;
}

static int lrc_ring_indirect_offset(const struct intel_engine_cs *engine)
{
	int x;

	x = lrc_ring_indirect_ptr(engine);
	if (x < 0)
		return x;

	return x + 2;
}

static int lrc_ring_cmd_buf_cctl(const struct intel_engine_cs *engine)
{

	if (GRAPHICS_VER_FULL(engine->i915) >= IP_VER(12, 50))
		/*
		 * Note that the CSFE context has a dummy slot for CMD_BUF_CCTL
		 * simply to match the RCS context image layout.
		 */
		return 0xc6;
	else if (engine->class != RENDER_CLASS)
		return -1;
	else if (GRAPHICS_VER(engine->i915) >= 12)
		return 0xb6;
	else if (GRAPHICS_VER(engine->i915) >= 11)
		return 0xaa;
	else
		return -1;
}

static u32
lrc_ring_indirect_offset_default(const struct intel_engine_cs *engine)
{
	switch (GRAPHICS_VER(engine->i915)) {
	default:
		MISSING_CASE(GRAPHICS_VER(engine->i915));
		fallthrough;
	case 12:
		return GEN12_CTX_RCS_INDIRECT_CTX_OFFSET_DEFAULT;
	case 11:
		return GEN11_CTX_RCS_INDIRECT_CTX_OFFSET_DEFAULT;
	case 9:
		return GEN9_CTX_RCS_INDIRECT_CTX_OFFSET_DEFAULT;
	case 8:
		return GEN8_CTX_RCS_INDIRECT_CTX_OFFSET_DEFAULT;
	}
}

static void
lrc_setup_indirect_ctx(u32 *regs,
		       const struct intel_engine_cs *engine,
		       u32 ctx_bb_ggtt_addr,
		       u32 size)
{
	GEM_BUG_ON(!size);
	GEM_BUG_ON(!IS_ALIGNED(size, CACHELINE_BYTES));
	GEM_BUG_ON(lrc_ring_indirect_ptr(engine) == -1);
	regs[lrc_ring_indirect_ptr(engine) + 1] =
		ctx_bb_ggtt_addr | (size / CACHELINE_BYTES);

	GEM_BUG_ON(lrc_ring_indirect_offset(engine) == -1);
	regs[lrc_ring_indirect_offset(engine) + 1] =
		lrc_ring_indirect_offset_default(engine) << 6;
}

static void init_common_regs(u32 * const regs,
			     const struct intel_context *ce,
			     const struct intel_engine_cs *engine,
			     bool inhibit)
{
	u32 ctl;

	ctl = _MASKED_BIT_ENABLE(CTX_CTRL_INHIBIT_SYN_CTX_SWITCH);
	ctl |= _MASKED_BIT_DISABLE(CTX_CTRL_ENGINE_CTX_RESTORE_INHIBIT);
	if (inhibit)
		ctl |= CTX_CTRL_ENGINE_CTX_RESTORE_INHIBIT;
	if (GRAPHICS_VER(engine->i915) < 11)
		ctl |= _MASKED_BIT_DISABLE(CTX_CTRL_ENGINE_CTX_SAVE_INHIBIT |
					   CTX_CTRL_RS_CTX_ENABLE);
	regs[CTX_CONTEXT_CONTROL] = ctl;

	regs[CTX_TIMESTAMP] = ce->runtime.last;
}

static void init_wa_bb_regs(u32 * const regs,
			    const struct intel_engine_cs *engine)
{
	const struct i915_ctx_workarounds * const wa_ctx = &engine->wa_ctx;

	if (wa_ctx->per_ctx.size) {
		const u32 ggtt_offset = i915_ggtt_offset(wa_ctx->vma);

		GEM_BUG_ON(lrc_ring_wa_bb_per_ctx(engine) == -1);
		regs[lrc_ring_wa_bb_per_ctx(engine) + 1] =
			(ggtt_offset + wa_ctx->per_ctx.offset) | 0x01;
	}

	if (wa_ctx->indirect_ctx.size) {
		lrc_setup_indirect_ctx(regs, engine,
				       i915_ggtt_offset(wa_ctx->vma) +
				       wa_ctx->indirect_ctx.offset,
				       wa_ctx->indirect_ctx.size);
	}
}

static void init_ppgtt_regs(u32 *regs, const struct i915_ppgtt *ppgtt)
{
	if (i915_vm_is_4lvl(&ppgtt->vm)) {
		/* 64b PPGTT (48bit canonical)
		 * PDP0_DESCRIPTOR contains the base address to PML4 and
		 * other PDP Descriptors are ignored.
		 */
		ASSIGN_CTX_PML4(ppgtt, regs);
	} else {
		ASSIGN_CTX_PDP(ppgtt, regs, 3);
		ASSIGN_CTX_PDP(ppgtt, regs, 2);
		ASSIGN_CTX_PDP(ppgtt, regs, 1);
		ASSIGN_CTX_PDP(ppgtt, regs, 0);
	}
}

static struct i915_ppgtt *vm_alias(struct i915_address_space *vm)
{
	if (i915_is_ggtt(vm))
		return i915_vm_to_ggtt(vm)->alias;
	else
		return i915_vm_to_ppgtt(vm);
}

static void __reset_stop_ring(u32 *regs, const struct intel_engine_cs *engine)
{
	int x;

	x = lrc_ring_mi_mode(engine);
	if (x != -1) {
		regs[x + 1] &= ~STOP_RING;
		regs[x + 1] |= STOP_RING << 16;
	}
}

static void __lrc_init_regs(u32 *regs,
			    const struct intel_context *ce,
			    const struct intel_engine_cs *engine,
			    bool inhibit)
{
	/*
	 * A context is actually a big batch buffer with several
	 * MI_LOAD_REGISTER_IMM commands followed by (reg, value) pairs. The
	 * values we are setting here are only for the first context restore:
	 * on a subsequent save, the GPU will recreate this batchbuffer with new
	 * values (including all the missing MI_LOAD_REGISTER_IMM commands that
	 * we are not initializing here).
	 *
	 * Must keep consistent with virtual_update_register_offsets().
	 */

	if (inhibit)
		memset(regs, 0, PAGE_SIZE);

	set_offsets(regs, reg_offsets(engine), engine, inhibit);

	init_common_regs(regs, ce, engine, inhibit);
	init_ppgtt_regs(regs, vm_alias(ce->vm));

	init_wa_bb_regs(regs, engine);

	__reset_stop_ring(regs, engine);
}

void lrc_init_regs(const struct intel_context *ce,
		   const struct intel_engine_cs *engine,
		   bool inhibit)
{
	__lrc_init_regs(ce->lrc_reg_state, ce, engine, inhibit);
}

void lrc_reset_regs(const struct intel_context *ce,
		    const struct intel_engine_cs *engine)
{
	__reset_stop_ring(ce->lrc_reg_state, engine);
}

static void
set_redzone(void *vaddr, const struct intel_engine_cs *engine)
{
	if (!IS_ENABLED(CONFIG_DRM_I915_DEBUG_GEM))
		return;

	vaddr += engine->context_size;

	memset(vaddr, CONTEXT_REDZONE, I915_GTT_PAGE_SIZE);
}

static void
check_redzone(const void *vaddr, const struct intel_engine_cs *engine)
{
	if (!IS_ENABLED(CONFIG_DRM_I915_DEBUG_GEM))
		return;

	vaddr += engine->context_size;

	if (memchr_inv(vaddr, CONTEXT_REDZONE, I915_GTT_PAGE_SIZE))
		drm_err_once(&engine->i915->drm,
			     "%s context redzone overwritten!\n",
			     engine->name);
}

void lrc_init_state(struct intel_context *ce,
		    struct intel_engine_cs *engine,
		    void *state)
{
	bool inhibit = true;

	set_redzone(state, engine);

	if (engine->default_state) {
		shmem_read(engine->default_state, 0,
			   state, engine->context_size);
		__set_bit(CONTEXT_VALID_BIT, &ce->flags);
		inhibit = false;
	}

	/* Clear the ppHWSP (inc. per-context counters) */
	memset(state, 0, PAGE_SIZE);

	/*
	 * The second page of the context object contains some registers which
	 * must be set up prior to the first execution.
	 */
	__lrc_init_regs(state + LRC_STATE_OFFSET, ce, engine, inhibit);
}

static struct i915_vma *
__lrc_alloc_state(struct intel_context *ce, struct intel_engine_cs *engine)
{
	struct drm_i915_gem_object *obj;
	struct i915_vma *vma;
	u32 context_size;

	context_size = round_up(engine->context_size, I915_GTT_PAGE_SIZE);

	if (IS_ENABLED(CONFIG_DRM_I915_DEBUG_GEM))
		context_size += I915_GTT_PAGE_SIZE; /* for redzone */

	if (GRAPHICS_VER(engine->i915) == 12) {
		ce->wa_bb_page = context_size / PAGE_SIZE;
		context_size += PAGE_SIZE;
	}

	obj = i915_gem_object_create_lmem(engine->i915, context_size, 0);
	if (IS_ERR(obj))
		obj = i915_gem_object_create_shmem(engine->i915, context_size);
	if (IS_ERR(obj))
		return ERR_CAST(obj);

	vma = i915_vma_instance(obj, &engine->gt->ggtt->vm, NULL);
	if (IS_ERR(vma)) {
		i915_gem_object_put(obj);
		return vma;
	}

	return vma;
}

static struct intel_timeline *
pinned_timeline(struct intel_context *ce, struct intel_engine_cs *engine)
{
	struct intel_timeline *tl = fetch_and_zero(&ce->timeline);

	return intel_timeline_create_from_engine(engine, page_unmask_bits(tl));
}

int lrc_alloc(struct intel_context *ce, struct intel_engine_cs *engine)
{
	struct intel_ring *ring;
	struct i915_vma *vma;
	int err;

	GEM_BUG_ON(ce->state);

	vma = __lrc_alloc_state(ce, engine);
	if (IS_ERR(vma))
		return PTR_ERR(vma);

	ring = intel_engine_create_ring(engine, ce->ring_size);
	if (IS_ERR(ring)) {
		err = PTR_ERR(ring);
		goto err_vma;
	}

	if (!page_mask_bits(ce->timeline)) {
		struct intel_timeline *tl;

		/*
		 * Use the static global HWSP for the kernel context, and
		 * a dynamically allocated cacheline for everyone else.
		 */
		if (unlikely(ce->timeline))
			tl = pinned_timeline(ce, engine);
		else
			tl = intel_timeline_create(engine->gt);
		if (IS_ERR(tl)) {
			err = PTR_ERR(tl);
			goto err_ring;
		}

		ce->timeline = tl;
	}

	ce->ring = ring;
	ce->state = vma;

	return 0;

err_ring:
	intel_ring_put(ring);
err_vma:
	i915_vma_put(vma);
	return err;
}

void lrc_reset(struct intel_context *ce)
{
	GEM_BUG_ON(!intel_context_is_pinned(ce));

	intel_ring_reset(ce->ring, ce->ring->emit);

	/* Scrub away the garbage */
	lrc_init_regs(ce, ce->engine, true);
	ce->lrc.lrca = lrc_update_regs(ce, ce->engine, ce->ring->tail);
}

int
lrc_pre_pin(struct intel_context *ce,
	    struct intel_engine_cs *engine,
	    struct i915_gem_ww_ctx *ww,
	    void **vaddr)
{
	GEM_BUG_ON(!ce->state);
	GEM_BUG_ON(!i915_vma_is_pinned(ce->state));

	*vaddr = i915_gem_object_pin_map(ce->state->obj,
					 i915_coherent_map_type(ce->engine->i915,
								ce->state->obj,
								false) |
					 I915_MAP_OVERRIDE);

	return PTR_ERR_OR_ZERO(*vaddr);
}

int
lrc_pin(struct intel_context *ce,
	struct intel_engine_cs *engine,
	void *vaddr)
{
	ce->lrc_reg_state = vaddr + LRC_STATE_OFFSET;

	if (!__test_and_set_bit(CONTEXT_INIT_BIT, &ce->flags))
		lrc_init_state(ce, engine, vaddr);

	ce->lrc.lrca = lrc_update_regs(ce, engine, ce->ring->tail);
	return 0;
}

void lrc_unpin(struct intel_context *ce)
{
	check_redzone((void *)ce->lrc_reg_state - LRC_STATE_OFFSET,
		      ce->engine);
}

void lrc_post_unpin(struct intel_context *ce)
{
	i915_gem_object_unpin_map(ce->state->obj);
}

void lrc_fini(struct intel_context *ce)
{
	if (!ce->state)
		return;

	intel_ring_put(fetch_and_zero(&ce->ring));
	i915_vma_put(fetch_and_zero(&ce->state));
}

void lrc_destroy(struct kref *kref)
{
	struct intel_context *ce = container_of(kref, typeof(*ce), ref);

	GEM_BUG_ON(!i915_active_is_idle(&ce->active));
	GEM_BUG_ON(intel_context_is_pinned(ce));

	lrc_fini(ce);

	intel_context_fini(ce);
	intel_context_free(ce);
}

static u32 *
gen12_emit_timestamp_wa(const struct intel_context *ce, u32 *cs)
{
	*cs++ = MI_LOAD_REGISTER_MEM_GEN8 |
		MI_SRM_LRM_GLOBAL_GTT |
		MI_LRI_LRM_CS_MMIO;
	*cs++ = i915_mmio_reg_offset(GEN8_RING_CS_GPR(0, 0));
	*cs++ = i915_ggtt_offset(ce->state) + LRC_STATE_OFFSET +
		CTX_TIMESTAMP * sizeof(u32);
	*cs++ = 0;

	*cs++ = MI_LOAD_REGISTER_REG |
		MI_LRR_SOURCE_CS_MMIO |
		MI_LRI_LRM_CS_MMIO;
	*cs++ = i915_mmio_reg_offset(GEN8_RING_CS_GPR(0, 0));
	*cs++ = i915_mmio_reg_offset(RING_CTX_TIMESTAMP(0));

	*cs++ = MI_LOAD_REGISTER_REG |
		MI_LRR_SOURCE_CS_MMIO |
		MI_LRI_LRM_CS_MMIO;
	*cs++ = i915_mmio_reg_offset(GEN8_RING_CS_GPR(0, 0));
	*cs++ = i915_mmio_reg_offset(RING_CTX_TIMESTAMP(0));

	return cs;
}

static u32 *
gen12_emit_restore_scratch(const struct intel_context *ce, u32 *cs)
{
	GEM_BUG_ON(lrc_ring_gpr0(ce->engine) == -1);

	*cs++ = MI_LOAD_REGISTER_MEM_GEN8 |
		MI_SRM_LRM_GLOBAL_GTT |
		MI_LRI_LRM_CS_MMIO;
	*cs++ = i915_mmio_reg_offset(GEN8_RING_CS_GPR(0, 0));
	*cs++ = i915_ggtt_offset(ce->state) + LRC_STATE_OFFSET +
		(lrc_ring_gpr0(ce->engine) + 1) * sizeof(u32);
	*cs++ = 0;

	return cs;
}

static u32 *
gen12_emit_cmd_buf_wa(const struct intel_context *ce, u32 *cs)
{
	GEM_BUG_ON(lrc_ring_cmd_buf_cctl(ce->engine) == -1);

	*cs++ = MI_LOAD_REGISTER_MEM_GEN8 |
		MI_SRM_LRM_GLOBAL_GTT |
		MI_LRI_LRM_CS_MMIO;
	*cs++ = i915_mmio_reg_offset(GEN8_RING_CS_GPR(0, 0));
	*cs++ = i915_ggtt_offset(ce->state) + LRC_STATE_OFFSET +
		(lrc_ring_cmd_buf_cctl(ce->engine) + 1) * sizeof(u32);
	*cs++ = 0;

	*cs++ = MI_LOAD_REGISTER_REG |
		MI_LRR_SOURCE_CS_MMIO |
		MI_LRI_LRM_CS_MMIO;
	*cs++ = i915_mmio_reg_offset(GEN8_RING_CS_GPR(0, 0));
	*cs++ = i915_mmio_reg_offset(RING_CMD_BUF_CCTL(0));

	return cs;
}

static u32 *
gen12_emit_indirect_ctx_rcs(const struct intel_context *ce, u32 *cs)
{
	cs = gen12_emit_timestamp_wa(ce, cs);
	cs = gen12_emit_cmd_buf_wa(ce, cs);
	cs = gen12_emit_restore_scratch(ce, cs);

	return cs;
}

static u32 *
gen12_emit_indirect_ctx_xcs(const struct intel_context *ce, u32 *cs)
{
	cs = gen12_emit_timestamp_wa(ce, cs);
	cs = gen12_emit_restore_scratch(ce, cs);

	return cs;
}

static u32 context_wa_bb_offset(const struct intel_context *ce)
{
	return PAGE_SIZE * ce->wa_bb_page;
}

static u32 *context_indirect_bb(const struct intel_context *ce)
{
	void *ptr;

	GEM_BUG_ON(!ce->wa_bb_page);

	ptr = ce->lrc_reg_state;
	ptr -= LRC_STATE_OFFSET; /* back to start of context image */
	ptr += context_wa_bb_offset(ce);

	return ptr;
}

static void
setup_indirect_ctx_bb(const struct intel_context *ce,
		      const struct intel_engine_cs *engine,
		      u32 *(*emit)(const struct intel_context *, u32 *))
{
	u32 * const start = context_indirect_bb(ce);
	u32 *cs;

	cs = emit(ce, start);
	GEM_BUG_ON(cs - start > I915_GTT_PAGE_SIZE / sizeof(*cs));
	while ((unsigned long)cs % CACHELINE_BYTES)
		*cs++ = MI_NOOP;

	lrc_setup_indirect_ctx(ce->lrc_reg_state, engine,
			       i915_ggtt_offset(ce->state) +
			       context_wa_bb_offset(ce),
			       (cs - start) * sizeof(*cs));
}

/*
 * The context descriptor encodes various attributes of a context,
 * including its GTT address and some flags. Because it's fairly
 * expensive to calculate, we'll just do it once and cache the result,
 * which remains valid until the context is unpinned.
 *
 * This is what a descriptor looks like, from LSB to MSB::
 *
 *      bits  0-11:    flags, GEN8_CTX_* (cached in ctx->desc_template)
 *      bits 12-31:    LRCA, GTT address of (the HWSP of) this context
 *      bits 32-52:    ctx ID, a globally unique tag (highest bit used by GuC)
 *      bits 53-54:    mbz, reserved for use by hardware
 *      bits 55-63:    group ID, currently unused and set to 0
 *
 * Starting from Gen11, the upper dword of the descriptor has a new format:
 *
 *      bits 32-36:    reserved
 *      bits 37-47:    SW context ID
 *      bits 48:53:    engine instance
 *      bit 54:        mbz, reserved for use by hardware
 *      bits 55-60:    SW counter
 *      bits 61-63:    engine class
 *
 * On Xe_HP, the upper dword of the descriptor has a new format:
 *
 *      bits 32-37:    virtual function number
 *      bit 38:        mbz, reserved for use by hardware
 *      bits 39-54:    SW context ID
 *      bits 55-57:    reserved
 *      bits 58-63:    SW counter
 *
 * engine info, SW context ID and SW counter need to form a unique number
 * (Context ID) per lrc.
 */
static u32 lrc_descriptor(const struct intel_context *ce)
{
	u32 desc;

	desc = INTEL_LEGACY_32B_CONTEXT;
	if (i915_vm_is_4lvl(ce->vm))
		desc = INTEL_LEGACY_64B_CONTEXT;
	desc <<= GEN8_CTX_ADDRESSING_MODE_SHIFT;

	desc |= GEN8_CTX_VALID | GEN8_CTX_PRIVILEGE;
	if (GRAPHICS_VER(ce->vm->i915) == 8)
		desc |= GEN8_CTX_L3LLC_COHERENT;

	return i915_ggtt_offset(ce->state) | desc;
}

u32 lrc_update_regs(const struct intel_context *ce,
		    const struct intel_engine_cs *engine,
		    u32 head)
{
	struct intel_ring *ring = ce->ring;
	u32 *regs = ce->lrc_reg_state;

	GEM_BUG_ON(!intel_ring_offset_valid(ring, head));
	GEM_BUG_ON(!intel_ring_offset_valid(ring, ring->tail));

	regs[CTX_RING_START] = i915_ggtt_offset(ring->vma);
	regs[CTX_RING_HEAD] = head;
	regs[CTX_RING_TAIL] = ring->tail;
	regs[CTX_RING_CTL] = RING_CTL_SIZE(ring->size) | RING_VALID;

	/* RPCS */
	if (engine->class == RENDER_CLASS) {
		regs[CTX_R_PWR_CLK_STATE] =
			intel_sseu_make_rpcs(engine->gt, &ce->sseu);

		i915_oa_init_reg_state(ce, engine);
	}

	if (ce->wa_bb_page) {
		u32 *(*fn)(const struct intel_context *ce, u32 *cs);

		fn = gen12_emit_indirect_ctx_xcs;
		if (ce->engine->class == RENDER_CLASS)
			fn = gen12_emit_indirect_ctx_rcs;

		/* Mutually exclusive wrt to global indirect bb */
		GEM_BUG_ON(engine->wa_ctx.indirect_ctx.size);
		setup_indirect_ctx_bb(ce, engine, fn);
	}

	return lrc_descriptor(ce) | CTX_DESC_FORCE_RESTORE;
}

void lrc_update_offsets(struct intel_context *ce,
			struct intel_engine_cs *engine)
{
	set_offsets(ce->lrc_reg_state, reg_offsets(engine), engine, false);
}

void lrc_check_regs(const struct intel_context *ce,
		    const struct intel_engine_cs *engine,
		    const char *when)
{
	const struct intel_ring *ring = ce->ring;
	u32 *regs = ce->lrc_reg_state;
	bool valid = true;
	int x;

	if (regs[CTX_RING_START] != i915_ggtt_offset(ring->vma)) {
		pr_err("%s: context submitted with incorrect RING_START [%08x], expected %08x\n",
		       engine->name,
		       regs[CTX_RING_START],
		       i915_ggtt_offset(ring->vma));
		regs[CTX_RING_START] = i915_ggtt_offset(ring->vma);
		valid = false;
	}

	if ((regs[CTX_RING_CTL] & ~(RING_WAIT | RING_WAIT_SEMAPHORE)) !=
	    (RING_CTL_SIZE(ring->size) | RING_VALID)) {
		pr_err("%s: context submitted with incorrect RING_CTL [%08x], expected %08x\n",
		       engine->name,
		       regs[CTX_RING_CTL],
		       (u32)(RING_CTL_SIZE(ring->size) | RING_VALID));
		regs[CTX_RING_CTL] = RING_CTL_SIZE(ring->size) | RING_VALID;
		valid = false;
	}

	x = lrc_ring_mi_mode(engine);
	if (x != -1 && regs[x + 1] & (regs[x + 1] >> 16) & STOP_RING) {
		pr_err("%s: context submitted with STOP_RING [%08x] in RING_MI_MODE\n",
		       engine->name, regs[x + 1]);
		regs[x + 1] &= ~STOP_RING;
		regs[x + 1] |= STOP_RING << 16;
		valid = false;
	}

	WARN_ONCE(!valid, "Invalid lrc state found %s submission\n", when);
}

/*
 * In this WA we need to set GEN8_L3SQCREG4[21:21] and reset it after
 * PIPE_CONTROL instruction. This is required for the flush to happen correctly
 * but there is a slight complication as this is applied in WA batch where the
 * values are only initialized once so we cannot take register value at the
 * beginning and reuse it further; hence we save its value to memory, upload a
 * constant value with bit21 set and then we restore it back with the saved value.
 * To simplify the WA, a constant value is formed by using the default value
 * of this register. This shouldn't be a problem because we are only modifying
 * it for a short period and this batch in non-premptible. We can ofcourse
 * use additional instructions that read the actual value of the register
 * at that time and set our bit of interest but it makes the WA complicated.
 *
 * This WA is also required for Gen9 so extracting as a function avoids
 * code duplication.
 */
static u32 *
gen8_emit_flush_coherentl3_wa(struct intel_engine_cs *engine, u32 *batch)
{
	/* NB no one else is allowed to scribble over scratch + 256! */
	*batch++ = MI_STORE_REGISTER_MEM_GEN8 | MI_SRM_LRM_GLOBAL_GTT;
	*batch++ = i915_mmio_reg_offset(GEN8_L3SQCREG4);
	*batch++ = intel_gt_scratch_offset(engine->gt,
					   INTEL_GT_SCRATCH_FIELD_COHERENTL3_WA);
	*batch++ = 0;

	*batch++ = MI_LOAD_REGISTER_IMM(1);
	*batch++ = i915_mmio_reg_offset(GEN8_L3SQCREG4);
	*batch++ = 0x40400000 | GEN8_LQSC_FLUSH_COHERENT_LINES;

	batch = gen8_emit_pipe_control(batch,
				       PIPE_CONTROL_CS_STALL |
				       PIPE_CONTROL_DC_FLUSH_ENABLE,
				       0);

	*batch++ = MI_LOAD_REGISTER_MEM_GEN8 | MI_SRM_LRM_GLOBAL_GTT;
	*batch++ = i915_mmio_reg_offset(GEN8_L3SQCREG4);
	*batch++ = intel_gt_scratch_offset(engine->gt,
					   INTEL_GT_SCRATCH_FIELD_COHERENTL3_WA);
	*batch++ = 0;

	return batch;
}

/*
 * Typically we only have one indirect_ctx and per_ctx batch buffer which are
 * initialized at the beginning and shared across all contexts but this field
 * helps us to have multiple batches at different offsets and select them based
 * on a criteria. At the moment this batch always start at the beginning of the page
 * and at this point we don't have multiple wa_ctx batch buffers.
 *
 * The number of WA applied are not known at the beginning; we use this field
 * to return the no of DWORDS written.
 *
 * It is to be noted that this batch does not contain MI_BATCH_BUFFER_END
 * so it adds NOOPs as padding to make it cacheline aligned.
 * MI_BATCH_BUFFER_END will be added to perctx batch and both of them together
 * makes a complete batch buffer.
 */
static u32 *gen8_init_indirectctx_bb(struct intel_engine_cs *engine, u32 *batch)
{
	/* WaDisableCtxRestoreArbitration:bdw,chv */
	*batch++ = MI_ARB_ON_OFF | MI_ARB_DISABLE;

	/* WaFlushCoherentL3CacheLinesAtContextSwitch:bdw */
	if (IS_BROADWELL(engine->i915))
		batch = gen8_emit_flush_coherentl3_wa(engine, batch);

	/* WaClearSlmSpaceAtContextSwitch:bdw,chv */
	/* Actual scratch location is at 128 bytes offset */
	batch = gen8_emit_pipe_control(batch,
				       PIPE_CONTROL_FLUSH_L3 |
				       PIPE_CONTROL_STORE_DATA_INDEX |
				       PIPE_CONTROL_CS_STALL |
				       PIPE_CONTROL_QW_WRITE,
				       LRC_PPHWSP_SCRATCH_ADDR);

	*batch++ = MI_ARB_ON_OFF | MI_ARB_ENABLE;

	/* Pad to end of cacheline */
	while ((unsigned long)batch % CACHELINE_BYTES)
		*batch++ = MI_NOOP;

	/*
	 * MI_BATCH_BUFFER_END is not required in Indirect ctx BB because
	 * execution depends on the length specified in terms of cache lines
	 * in the register CTX_RCS_INDIRECT_CTX
	 */

	return batch;
}

struct lri {
	i915_reg_t reg;
	u32 value;
};

static u32 *emit_lri(u32 *batch, const struct lri *lri, unsigned int count)
{
	GEM_BUG_ON(!count || count > 63);

	*batch++ = MI_LOAD_REGISTER_IMM(count);
	do {
		*batch++ = i915_mmio_reg_offset(lri->reg);
		*batch++ = lri->value;
	} while (lri++, --count);
	*batch++ = MI_NOOP;

	return batch;
}

static u32 *gen9_init_indirectctx_bb(struct intel_engine_cs *engine, u32 *batch)
{
	static const struct lri lri[] = {
		/* WaDisableGatherAtSetShaderCommonSlice:skl,bxt,kbl,glk */
		{
			COMMON_SLICE_CHICKEN2,
			__MASKED_FIELD(GEN9_DISABLE_GATHER_AT_SET_SHADER_COMMON_SLICE,
				       0),
		},

		/* BSpec: 11391 */
		{
			FF_SLICE_CHICKEN,
			__MASKED_FIELD(FF_SLICE_CHICKEN_CL_PROVOKING_VERTEX_FIX,
				       FF_SLICE_CHICKEN_CL_PROVOKING_VERTEX_FIX),
		},

		/* BSpec: 11299 */
		{
			_3D_CHICKEN3,
			__MASKED_FIELD(_3D_CHICKEN_SF_PROVOKING_VERTEX_FIX,
				       _3D_CHICKEN_SF_PROVOKING_VERTEX_FIX),
		}
	};

	*batch++ = MI_ARB_ON_OFF | MI_ARB_DISABLE;

	/* WaFlushCoherentL3CacheLinesAtContextSwitch:skl,bxt,glk */
	batch = gen8_emit_flush_coherentl3_wa(engine, batch);

	/* WaClearSlmSpaceAtContextSwitch:skl,bxt,kbl,glk,cfl */
	batch = gen8_emit_pipe_control(batch,
				       PIPE_CONTROL_FLUSH_L3 |
				       PIPE_CONTROL_STORE_DATA_INDEX |
				       PIPE_CONTROL_CS_STALL |
				       PIPE_CONTROL_QW_WRITE,
				       LRC_PPHWSP_SCRATCH_ADDR);

	batch = emit_lri(batch, lri, ARRAY_SIZE(lri));

	/* WaMediaPoolStateCmdInWABB:bxt,glk */
	if (HAS_POOLED_EU(engine->i915)) {
		/*
		 * EU pool configuration is setup along with golden context
		 * during context initialization. This value depends on
		 * device type (2x6 or 3x6) and needs to be updated based
		 * on which subslice is disabled especially for 2x6
		 * devices, however it is safe to load default
		 * configuration of 3x6 device instead of masking off
		 * corresponding bits because HW ignores bits of a disabled
		 * subslice and drops down to appropriate config. Please
		 * see render_state_setup() in i915_gem_render_state.c for
		 * possible configurations, to avoid duplication they are
		 * not shown here again.
		 */
		*batch++ = GEN9_MEDIA_POOL_STATE;
		*batch++ = GEN9_MEDIA_POOL_ENABLE;
		*batch++ = 0x00777000;
		*batch++ = 0;
		*batch++ = 0;
		*batch++ = 0;
	}

	*batch++ = MI_ARB_ON_OFF | MI_ARB_ENABLE;

	/* Pad to end of cacheline */
	while ((unsigned long)batch % CACHELINE_BYTES)
		*batch++ = MI_NOOP;

	return batch;
}

#define CTX_WA_BB_SIZE (PAGE_SIZE)

static int lrc_create_wa_ctx(struct intel_engine_cs *engine)
{
	struct drm_i915_gem_object *obj;
	struct i915_vma *vma;
	int err;

	obj = i915_gem_object_create_shmem(engine->i915, CTX_WA_BB_SIZE);
	if (IS_ERR(obj))
		return PTR_ERR(obj);

	vma = i915_vma_instance(obj, &engine->gt->ggtt->vm, NULL);
	if (IS_ERR(vma)) {
		err = PTR_ERR(vma);
		goto err;
	}

	engine->wa_ctx.vma = vma;
	return 0;

err:
	i915_gem_object_put(obj);
	return err;
}

void lrc_fini_wa_ctx(struct intel_engine_cs *engine)
{
	i915_vma_unpin_and_release(&engine->wa_ctx.vma, 0);
}

typedef u32 *(*wa_bb_func_t)(struct intel_engine_cs *engine, u32 *batch);

void lrc_init_wa_ctx(struct intel_engine_cs *engine)
{
	struct i915_ctx_workarounds *wa_ctx = &engine->wa_ctx;
	struct i915_wa_ctx_bb *wa_bb[] = {
		&wa_ctx->indirect_ctx, &wa_ctx->per_ctx
	};
	wa_bb_func_t wa_bb_fn[ARRAY_SIZE(wa_bb)];
	struct i915_gem_ww_ctx ww;
	void *batch, *batch_ptr;
	unsigned int i;
	int err;

	if (engine->class != RENDER_CLASS)
		return;

	switch (GRAPHICS_VER(engine->i915)) {
	case 12:
	case 11:
		return;
	case 9:
		wa_bb_fn[0] = gen9_init_indirectctx_bb;
		wa_bb_fn[1] = NULL;
		break;
	case 8:
		wa_bb_fn[0] = gen8_init_indirectctx_bb;
		wa_bb_fn[1] = NULL;
		break;
	default:
		MISSING_CASE(GRAPHICS_VER(engine->i915));
		return;
	}

	err = lrc_create_wa_ctx(engine);
	if (err) {
		/*
		 * We continue even if we fail to initialize WA batch
		 * because we only expect rare glitches but nothing
		 * critical to prevent us from using GPU
		 */
		drm_err(&engine->i915->drm,
			"Ignoring context switch w/a allocation error:%d\n",
			err);
		return;
	}

	if (!engine->wa_ctx.vma)
		return;

	i915_gem_ww_ctx_init(&ww, true);
retry:
	err = i915_gem_object_lock(wa_ctx->vma->obj, &ww);
	if (!err)
		err = i915_ggtt_pin(wa_ctx->vma, &ww, 0, PIN_HIGH);
	if (err)
		goto err;

	batch = i915_gem_object_pin_map(wa_ctx->vma->obj, I915_MAP_WB);
	if (IS_ERR(batch)) {
		err = PTR_ERR(batch);
		goto err_unpin;
	}

	/*
	 * Emit the two workaround batch buffers, recording the offset from the
	 * start of the workaround batch buffer object for each and their
	 * respective sizes.
	 */
	batch_ptr = batch;
	for (i = 0; i < ARRAY_SIZE(wa_bb_fn); i++) {
		wa_bb[i]->offset = batch_ptr - batch;
		if (GEM_DEBUG_WARN_ON(!IS_ALIGNED(wa_bb[i]->offset,
						  CACHELINE_BYTES))) {
			err = -EINVAL;
			break;
		}
		if (wa_bb_fn[i])
			batch_ptr = wa_bb_fn[i](engine, batch_ptr);
		wa_bb[i]->size = batch_ptr - (batch + wa_bb[i]->offset);
	}
	GEM_BUG_ON(batch_ptr - batch > CTX_WA_BB_SIZE);

	__i915_gem_object_flush_map(wa_ctx->vma->obj, 0, batch_ptr - batch);
	__i915_gem_object_release_map(wa_ctx->vma->obj);

	/* Verify that we can handle failure to setup the wa_ctx */
	if (!err)
		err = i915_inject_probe_error(engine->i915, -ENODEV);

err_unpin:
	if (err)
		i915_vma_unpin(wa_ctx->vma);
err:
	if (err == -EDEADLK) {
		err = i915_gem_ww_ctx_backoff(&ww);
		if (!err)
			goto retry;
	}
	i915_gem_ww_ctx_fini(&ww);

	if (err) {
		i915_vma_put(engine->wa_ctx.vma);

		/* Clear all flags to prevent further use */
		memset(wa_ctx, 0, sizeof(*wa_ctx));
	}
}

static void st_update_runtime_underflow(struct intel_context *ce, s32 dt)
{
#if IS_ENABLED(CONFIG_DRM_I915_SELFTEST)
	ce->runtime.num_underflow++;
	ce->runtime.max_underflow = max_t(u32, ce->runtime.max_underflow, -dt);
#endif
}

void lrc_update_runtime(struct intel_context *ce)
{
	u32 old;
	s32 dt;

	if (intel_context_is_barrier(ce))
		return;

	old = ce->runtime.last;
	ce->runtime.last = lrc_get_runtime(ce);
	dt = ce->runtime.last - old;

	if (unlikely(dt < 0)) {
		CE_TRACE(ce, "runtime underflow: last=%u, new=%u, delta=%d\n",
			 old, ce->runtime.last, dt);
		st_update_runtime_underflow(ce, dt);
		return;
	}

	ewma_runtime_add(&ce->runtime.avg, dt);
	ce->runtime.total += dt;
}

#if IS_ENABLED(CONFIG_DRM_I915_SELFTEST)
#include "selftest_lrc.c"
#endif
