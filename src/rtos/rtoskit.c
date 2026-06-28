// SPDX-License-Identifier: GPL-2.0-or-later

/*
 * rtoskit.c - OpenOCD RTOS awareness for the rtos-toolkit scheduler.
 *
 * Target kernel: the in-house preemptive SMP scheduler used by the pi-pico
 * project (RP2040 / RP2350), exposed via the CMSIS-RTOSv2 API.  See
 * include/rtos/rtos-toolkit/scheduler.h for the kernel ABI this driver
 * mirrors.
 *
 * Lookup contract:
 *   scheduler        - struct scheduler * global; validated via its marker
 *   current_task     - core-local struct task *; per-core slot in .core_data
 *   __core_data      - LMA of the .core_data section (compiled CLS layout)
 *   __core_local_0/1 - RAM region holding each core's instance of .core_data
 *   __core_data_size - size of the per-core block (bounds check)
 *
 * Behaviour:
 *   - update_threads walks scheduler->tasks (a sched_list) to enumerate every
 *     live task, validates each TCB marker, and annotates running-on-core
 *     based on the per-core current_task slot.
 *   - gdb_target_for_threadid routes register reads for RUNNING tasks to the
 *     target hosting the live core; the SMP head is used for non-running
 *     tasks (memory reads only).
 *   - get_thread_reg_list returns the live register cache for RUNNING tasks
 *     and unwinds the saved scheduler_frame for the rest.  Stacking is
 *     selected from the target's compile-time FPU configuration (the C
 *     scheduler_frame layout is fixed by __FPU_USED, not by EXC_RETURN).
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "rtos.h"
#include "rtos_standard_stackings.h"
#include "target/armv7m.h"
#include "target/cortex_m.h"
#include "target/register.h"
#include "target/smp.h"
#include "server/gdb_server.h"
#include "helper/log.h"
#include "helper/types.h"

/* ------------------------------------------------------------------------- */
/* Kernel ABI constants — mirror struct task / struct scheduler layouts.     */
/* Computed for ARM32 (4-byte pointer, 4-byte unsigned long, 4-byte enum).   */
/* Field reference: include/rtos/rtos-toolkit/scheduler.h.                   */
/* ------------------------------------------------------------------------- */

/* struct task — fields accessed by this driver. */
#define RTOSKIT_TCB_PSP              0x00
#define RTOSKIT_TCB_STATE            0x0c
#define RTOSKIT_TCB_CORE             0x10
#define RTOSKIT_TCB_BASE_PRIORITY    0x18
#define RTOSKIT_TCB_CURRENT_PRIORITY 0x1c
#define RTOSKIT_TCB_SCHEDULER_NODE   0x2c
#define RTOSKIT_TCB_MARKER           0x54

/* struct scheduler — fields accessed by this driver.  locked and critical
 * carry __aligned(16) in the kernel; the offsets below already account for
 * the padding.
 */
#define RTOSKIT_SCHED_TASKS          0x10
#define RTOSKIT_SCHED_RUNNING        0x24
#define RTOSKIT_SCHED_MARKER         0x48

/* enum task_state values (scheduler.h). */
#define RTOSKIT_TASK_TERMINATED 1
#define RTOSKIT_TASK_BLOCKED    2
#define RTOSKIT_TASK_SUSPENDED  4
#define RTOSKIT_TASK_READY      5
#define RTOSKIT_TASK_RUNNING    6

/* Validation markers (scheduler.h). */
#define RTOSKIT_SCHED_MARKER_VALUE 0x13700731UL
#define RTOSKIT_TASK_MARKER_VALUE  0x137aa731UL

/* struct scheduler_frame field offsets.  The frame is laid out at firmware
 * compile time by __FPU_USED: 80 bytes with no FPU, 212 bytes with FPU (the
 * FPU layout inserts s16-s31 between psplim and r0 and appends s0-s15 + fpscr
 * after xpsr).  On ARMv8-M a per-thread psplim slot sits at 0x2c, immediately
 * after r11; it has no GDB register number and is skipped by the stacking
 * tables below.
 */
#define RTOSKIT_FRAME_EXEC_RETURN     0x00 /* both layouts */
#define RTOSKIT_FRAME_R4              0x0c /* both layouts */
#define RTOSKIT_FRAME_PSPLIM          0x2c /* ARMv8-M only; not exposed to GDB */

#define RTOSKIT_FRAME_NOFP_R0         0x30
#define RTOSKIT_FRAME_NOFP_R12        0x40
#define RTOSKIT_FRAME_NOFP_LR         0x44
#define RTOSKIT_FRAME_NOFP_PC         0x48
#define RTOSKIT_FRAME_NOFP_XPSR       0x4c
#define RTOSKIT_FRAME_NOFP_SIZE       0x50

#define RTOSKIT_FRAME_FP_R0           0x70
#define RTOSKIT_FRAME_FP_R12          0x80
#define RTOSKIT_FRAME_FP_LR           0x84
#define RTOSKIT_FRAME_FP_PC           0x88
#define RTOSKIT_FRAME_FP_XPSR         0x8c
#define RTOSKIT_FRAME_FP_SIZE         0xd4

/* Hard cap on tasks list traversal to defend against corruption. */
#define RTOSKIT_MAX_TASKS             256

/* ------------------------------------------------------------------------- */
/* Symbol manifest                                                           */
/* ------------------------------------------------------------------------- */

enum rtoskit_symbol {
	RTOSKIT_SYM_SCHEDULER = 0,
	RTOSKIT_SYM_CURRENT_TASK,
	RTOSKIT_SYM_CORE_DATA,
	RTOSKIT_SYM_CORE_LOCAL_0,
	RTOSKIT_SYM_CORE_LOCAL_1,
	RTOSKIT_SYM_CORE_DATA_SIZE,
	RTOSKIT_SYM_COUNT,
};

struct rtoskit_symbol_def {
	const char *name;
	bool optional;
};

/* ------------------------------------------------------------------------- */
/* Stacking tables — mirror struct scheduler_frame for non-running tasks.    */
/* ------------------------------------------------------------------------- */

static const struct stack_register_offset rtoskit_nofp_offsets[ARMV7M_NUM_CORE_REGS] = {
	{ ARMV7M_R0,   RTOSKIT_FRAME_NOFP_R0,        32 },
	{ ARMV7M_R1,   RTOSKIT_FRAME_NOFP_R0 + 0x04, 32 },
	{ ARMV7M_R2,   RTOSKIT_FRAME_NOFP_R0 + 0x08, 32 },
	{ ARMV7M_R3,   RTOSKIT_FRAME_NOFP_R0 + 0x0c, 32 },
	{ ARMV7M_R4,   RTOSKIT_FRAME_R4,             32 },
	{ ARMV7M_R5,   RTOSKIT_FRAME_R4 + 0x04,      32 },
	{ ARMV7M_R6,   RTOSKIT_FRAME_R4 + 0x08,      32 },
	{ ARMV7M_R7,   RTOSKIT_FRAME_R4 + 0x0c,      32 },
	{ ARMV7M_R8,   RTOSKIT_FRAME_R4 + 0x10,      32 },
	{ ARMV7M_R9,   RTOSKIT_FRAME_R4 + 0x14,      32 },
	{ ARMV7M_R10,  RTOSKIT_FRAME_R4 + 0x18,      32 },
	{ ARMV7M_R11,  RTOSKIT_FRAME_R4 + 0x1c,      32 },
	{ ARMV7M_R12,  RTOSKIT_FRAME_NOFP_R12,       32 },
	{ ARMV7M_R13,  -2,                           32 }, /* sp recomputed */
	{ ARMV7M_R14,  RTOSKIT_FRAME_NOFP_LR,        32 },
	{ ARMV7M_PC,   RTOSKIT_FRAME_NOFP_PC,        32 },
	{ ARMV7M_XPSR, RTOSKIT_FRAME_NOFP_XPSR,      32 },
};

static const struct stack_register_offset rtoskit_fp_offsets[ARMV7M_NUM_CORE_REGS] = {
	{ ARMV7M_R0,   RTOSKIT_FRAME_FP_R0,          32 },
	{ ARMV7M_R1,   RTOSKIT_FRAME_FP_R0 + 0x04,   32 },
	{ ARMV7M_R2,   RTOSKIT_FRAME_FP_R0 + 0x08,   32 },
	{ ARMV7M_R3,   RTOSKIT_FRAME_FP_R0 + 0x0c,   32 },
	{ ARMV7M_R4,   RTOSKIT_FRAME_R4,             32 },
	{ ARMV7M_R5,   RTOSKIT_FRAME_R4 + 0x04,      32 },
	{ ARMV7M_R6,   RTOSKIT_FRAME_R4 + 0x08,      32 },
	{ ARMV7M_R7,   RTOSKIT_FRAME_R4 + 0x0c,      32 },
	{ ARMV7M_R8,   RTOSKIT_FRAME_R4 + 0x10,      32 },
	{ ARMV7M_R9,   RTOSKIT_FRAME_R4 + 0x14,      32 },
	{ ARMV7M_R10,  RTOSKIT_FRAME_R4 + 0x18,      32 },
	{ ARMV7M_R11,  RTOSKIT_FRAME_R4 + 0x1c,      32 },
	{ ARMV7M_R12,  RTOSKIT_FRAME_FP_R12,         32 },
	{ ARMV7M_R13,  -2,                           32 }, /* sp recomputed */
	{ ARMV7M_R14,  RTOSKIT_FRAME_FP_LR,          32 },
	{ ARMV7M_PC,   RTOSKIT_FRAME_FP_PC,          32 },
	{ ARMV7M_XPSR, RTOSKIT_FRAME_FP_XPSR,        32 },
};

static target_addr_t rtoskit_nofp_calc_sp(struct target *t,
		const uint8_t *stack_data, const struct rtos_register_stacking *stacking,
		target_addr_t stack_ptr)
{
	return rtos_cortex_m_stack_align(t, stack_data, stacking, stack_ptr,
			RTOSKIT_FRAME_NOFP_XPSR);
}

static target_addr_t rtoskit_fp_calc_sp(struct target *t,
		const uint8_t *stack_data, const struct rtos_register_stacking *stacking,
		target_addr_t stack_ptr)
{
	return rtos_cortex_m_stack_align(t, stack_data, stacking, stack_ptr,
			RTOSKIT_FRAME_FP_XPSR);
}

static const struct rtos_register_stacking rtoskit_stacking_nofp = {
	.stack_registers_size = RTOSKIT_FRAME_NOFP_SIZE,
	.stack_growth_direction = -1,
	.num_output_registers = ARMV7M_NUM_CORE_REGS,
	.calculate_process_stack = rtoskit_nofp_calc_sp,
	.register_offsets = rtoskit_nofp_offsets,
};

static const struct rtos_register_stacking rtoskit_stacking_fp = {
	.stack_registers_size = RTOSKIT_FRAME_FP_SIZE,
	.stack_growth_direction = -1,
	.num_output_registers = ARMV7M_NUM_CORE_REGS,
	.calculate_process_stack = rtoskit_fp_calc_sp,
	.register_offsets = rtoskit_fp_offsets,
};

static const struct rtoskit_symbol_def rtoskit_symbol_defs[RTOSKIT_SYM_COUNT] = {
	[RTOSKIT_SYM_SCHEDULER]       = { "scheduler",       false },
	[RTOSKIT_SYM_CURRENT_TASK]    = { "current_task",    false },
	[RTOSKIT_SYM_CORE_DATA]       = { "__core_data",     false },
	[RTOSKIT_SYM_CORE_LOCAL_0]    = { "__core_local_0",  false },
	[RTOSKIT_SYM_CORE_LOCAL_1]    = { "__core_local_1",  true  },
	[RTOSKIT_SYM_CORE_DATA_SIZE]  = { "__core_data_size", true },
};

/* ------------------------------------------------------------------------- */
/* Driver-private state                                                      */
/* ------------------------------------------------------------------------- */

struct rtoskit_params {
	int num_cores;              /* derived from presence of __core_local_1 */
	bool has_fpu;               /* derived from target->armv7m->fp_feature */
};

/* ------------------------------------------------------------------------- */
/* Forward declarations                                                      */
/* ------------------------------------------------------------------------- */

static bool rtoskit_detect_rtos(struct target *target);
static int  rtoskit_create(struct target *target);
static int  rtoskit_smp_init(struct target *target);
static int  rtoskit_update_threads(struct rtos *rtos);
static int  rtoskit_get_thread_reg_list(struct rtos *rtos, int64_t thread_id,
		struct rtos_reg **reg_list, int *num_regs);
static int  rtoskit_get_symbol_list_to_lookup(struct symbol_table_elem *symbol_list[]);
static int  rtoskit_gdb_target_for_threadid(struct connection *connection,
		int64_t thread_id, struct target **p_target);
static int  rtoskit_unwind_saved_frame(struct rtos *rtos, uint32_t tcb_ptr,
		uint32_t state, struct rtos_reg **reg_list, int *num_regs);

const struct rtos_type rtoskit_rtos = {
	.name                      = "rtoskit",
	.detect_rtos               = rtoskit_detect_rtos,
	.create                    = rtoskit_create,
	.smp_init                  = rtoskit_smp_init,
	.update_threads            = rtoskit_update_threads,
	.get_thread_reg_list       = rtoskit_get_thread_reg_list,
	.get_symbol_list_to_lookup = rtoskit_get_symbol_list_to_lookup,
};

/* ------------------------------------------------------------------------- */
/* Helpers                                                                   */
/* ------------------------------------------------------------------------- */

static symbol_address_t rtoskit_sym(const struct rtos *rtos, enum rtoskit_symbol s)
{
	return rtos->symbols ? rtos->symbols[s].address : 0;
}

static int rtoskit_get_symbol_list_to_lookup(struct symbol_table_elem *symbol_list[])
{
	*symbol_list = calloc(RTOSKIT_SYM_COUNT + 1, sizeof(struct symbol_table_elem));
	if (!*symbol_list)
		return ERROR_FAIL;

	for (size_t i = 0; i < RTOSKIT_SYM_COUNT; ++i) {
		(*symbol_list)[i].symbol_name = rtoskit_symbol_defs[i].name;
		(*symbol_list)[i].optional    = rtoskit_symbol_defs[i].optional;
	}
	/* Terminator left zeroed by calloc(). */
	return ERROR_OK;
}

/* ------------------------------------------------------------------------- */
/* detect_rtos                                                               */
/*                                                                           */
/* Verify the kernel is actually present and running by reading the          */
/* scheduler marker and the scheduler->running flag through the resolved     */
/* `scheduler` global.                                                       */
/* ------------------------------------------------------------------------- */

static bool rtoskit_detect_rtos(struct target *target)
{
	struct rtos *rtos = target->rtos;
	int retval;

	if (!rtos || !rtos->symbols) {
		LOG_DEBUG("rtoskit: detect: no symbols resolved yet");
		return false;
	}

	symbol_address_t sched_sym   = rtoskit_sym(rtos, RTOSKIT_SYM_SCHEDULER);
	symbol_address_t curtask_sym = rtoskit_sym(rtos, RTOSKIT_SYM_CURRENT_TASK);
	symbol_address_t cdata       = rtoskit_sym(rtos, RTOSKIT_SYM_CORE_DATA);
	symbol_address_t clocal0     = rtoskit_sym(rtos, RTOSKIT_SYM_CORE_LOCAL_0);
	symbol_address_t clocal1     = rtoskit_sym(rtos, RTOSKIT_SYM_CORE_LOCAL_1);
	symbol_address_t cdata_size  = rtoskit_sym(rtos, RTOSKIT_SYM_CORE_DATA_SIZE);

	LOG_DEBUG("rtoskit: detect: scheduler=0x%08" PRIx64 " current_task=0x%08" PRIx64
			" __core_data=0x%08" PRIx64 " __core_local_0=0x%08" PRIx64
			" __core_local_1=0x%08" PRIx64 " __core_data_size=0x%08" PRIx64,
			(uint64_t)sched_sym, (uint64_t)curtask_sym, (uint64_t)cdata,
			(uint64_t)clocal0, (uint64_t)clocal1, (uint64_t)cdata_size);

	if (!sched_sym || !curtask_sym || !cdata || !clocal0) {
		LOG_DEBUG("rtoskit: detect: missing mandatory symbol(s)");
		return false;
	}

	uint32_t sched_ptr = 0;
	retval = target_read_u32(target, sched_sym, &sched_ptr);
	if (retval != ERROR_OK) {
		LOG_DEBUG("rtoskit: detect: cannot read `scheduler` global");
		return false;
	}
	if (!sched_ptr) {
		LOG_DEBUG("rtoskit: detect: `scheduler` global is NULL — kernel not initialised");
		return false;
	}

	uint32_t marker = 0;
	retval = target_read_u32(target, sched_ptr + RTOSKIT_SCHED_MARKER, &marker);
	if (retval != ERROR_OK) {
		LOG_DEBUG("rtoskit: detect: cannot read scheduler.marker at 0x%08" PRIx32,
				sched_ptr + RTOSKIT_SCHED_MARKER);
		return false;
	}

	uint32_t running = 0;
	retval = target_read_u32(target, sched_ptr + RTOSKIT_SCHED_RUNNING, &running);
	if (retval != ERROR_OK) {
		LOG_DEBUG("rtoskit: detect: cannot read scheduler.running");
		return false;
	}

	LOG_DEBUG("rtoskit: detect: scheduler@0x%08" PRIx32 " marker=0x%08" PRIx32
			" running=%" PRIu32, sched_ptr, marker, running);

	if (marker != RTOSKIT_SCHED_MARKER_VALUE) {
		LOG_DEBUG("rtoskit: detect: marker mismatch (expected 0x%08lx)",
				RTOSKIT_SCHED_MARKER_VALUE);
		return false;
	}
	if (!running) {
		LOG_DEBUG("rtoskit: detect: scheduler not running yet");
		return false;
	}

	LOG_INFO("rtoskit: detected rtos-toolkit scheduler @ 0x%08" PRIx32, sched_ptr);
	return true;
}

/* ------------------------------------------------------------------------- */
/* create                                                                    */
/* ------------------------------------------------------------------------- */

static int rtoskit_create(struct target *target)
{
	struct rtoskit_params *p = calloc(1, sizeof(*p));
	if (!p)
		return ERROR_FAIL;

	const char *tname = target_type_name(target);
	if (!tname || (strcmp(tname, "cortex_m") != 0 && strcmp(tname, "hla_target") != 0)) {
		LOG_ERROR("rtoskit: create: unsupported target type %s", tname ? tname : "(null)");
		free(p);
		return ERROR_FAIL;
	}

	p->num_cores = 0;   /* re-derived from __core_local_1 in update_threads */
	p->has_fpu = false; /* re-derived in update_threads after target examination */

	target->rtos->rtos_specific_params = p;
	target->rtos->gdb_target_for_threadid = rtoskit_gdb_target_for_threadid;

	LOG_DEBUG("rtoskit: create: target=%s coreid=%" PRId32,
			target_name(target), target->coreid);
	return ERROR_OK;
}

/* Detect FPU presence on the head target.  fp_feature is set during target
 * examination, which always completes before update_threads first runs.
 */
static void rtoskit_refresh_fpu(struct rtos *rtos)
{
	struct rtoskit_params *p = rtos->rtos_specific_params;
	if (!p)
		return;

	bool prev = p->has_fpu;
	struct armv7m_common *armv7m = target_to_armv7m(rtos->target);
	if (armv7m && armv7m->common_magic == ARMV7M_COMMON_MAGIC)
		p->has_fpu = (armv7m->fp_feature != FP_NONE);
	else
		p->has_fpu = false;

	if (p->has_fpu != prev || !prev)
		LOG_DEBUG("rtoskit: fpu: target=%s arch=%d fp_feature=%d has_fpu=%d",
				target_name(rtos->target),
				armv7m ? (int)armv7m->arm.arch : -1,
				armv7m ? armv7m->fp_feature : -1,
				p->has_fpu);
}

/* ------------------------------------------------------------------------- */
/* smp_init — share one rtos instance across the SMP group                   */
/*                                                                           */
/* Called by `target smp` when more than one member of the SMP group was     */
/* configured with `-rtos rtoskit`.  We adopt the first member's rtos and    */
/* drop the duplicates so that GDB sees a single thread list.                */
/* ------------------------------------------------------------------------- */

static int rtoskit_smp_init(struct target *target)
{
	struct target_list *tl;

	LOG_DEBUG("rtoskit: smp_init: head=%s", target_name(target));

	foreach_smp_target(tl, target->smp_targets) {
		struct target *m = tl->target;
		if (m == target)
			continue;
		if (m->rtos && m->rtos != target->rtos) {
			free(m->rtos->symbols);
			free(m->rtos->rtos_specific_params);
			free(m->rtos);
		}
		m->rtos = target->rtos;
		LOG_DEBUG("rtoskit: smp_init: bound %s to head rtos", target_name(m));
	}
	return ERROR_OK;
}

/* ------------------------------------------------------------------------- */
/* update_threads                                                            */
/*                                                                           */
/* Rebuilds rtos->thread_details by walking scheduler->tasks and tags each   */
/* TCB with its kernel state, priority, and (if currently running) which     */
/* core it occupies.  Robust against the three "kernel not yet there" cases  */
/* GDB can reach: symbol unresolved, scheduler global NULL (scheduler_init   */
/* not run), empty task list.                                                */
/* ------------------------------------------------------------------------- */

static int rtoskit_update_threads(struct rtos *rtos)
{
	if (!rtos)
		return ERROR_FAIL;
	if (!rtos->symbols) {
		LOG_ERROR("rtoskit: update_threads: no symbols");
		return ERROR_FAIL;
	}

	rtos_free_threadlist(rtos);
	rtoskit_refresh_fpu(rtos);

	struct rtoskit_params *p = rtos->rtos_specific_params;
	int retval;

	symbol_address_t sched_sym   = rtoskit_sym(rtos, RTOSKIT_SYM_SCHEDULER);
	symbol_address_t curtask_sym = rtoskit_sym(rtos, RTOSKIT_SYM_CURRENT_TASK);
	symbol_address_t cdata       = rtoskit_sym(rtos, RTOSKIT_SYM_CORE_DATA);
	symbol_address_t clocal0     = rtoskit_sym(rtos, RTOSKIT_SYM_CORE_LOCAL_0);
	symbol_address_t clocal1     = rtoskit_sym(rtos, RTOSKIT_SYM_CORE_LOCAL_1);
	symbol_address_t cdata_size  = rtoskit_sym(rtos, RTOSKIT_SYM_CORE_DATA_SIZE);

	LOG_DEBUG("rtoskit: update_threads: symbols: scheduler=0x%08" PRIx64
			" current_task=0x%08" PRIx64 " __core_data=0x%08" PRIx64
			" __core_local_0=0x%08" PRIx64 " __core_local_1=0x%08" PRIx64
			" __core_data_size=0x%08" PRIx64,
			(uint64_t)sched_sym, (uint64_t)curtask_sym, (uint64_t)cdata,
			(uint64_t)clocal0, (uint64_t)clocal1, (uint64_t)cdata_size);

	if (!sched_sym) {
		LOG_ERROR("rtoskit: update_threads: `scheduler` symbol unresolved — "
				"GDB has no matching symbol file loaded, or the firmware "
				"does not link rtos-toolkit");
		rtos->thread_details = calloc(1, sizeof(struct thread_detail));
		if (!rtos->thread_details)
			return ERROR_FAIL;
		rtos->thread_details[0].threadid = 1;
		rtos->thread_details[0].exists = true;
		rtos->thread_details[0].thread_name_str = strdup("rtoskit:no-symbols");
		rtos->thread_details[0].extra_info_str = strdup("scheduler symbol unresolved");
		rtos->thread_count = 1;
		rtos->current_thread = 1;
		return ERROR_OK;
	}

	uint32_t sched_ptr = 0;
	retval = target_read_u32(rtos->target, sched_sym, &sched_ptr);
	if (retval != ERROR_OK) {
		LOG_ERROR("rtoskit: update_threads: cannot read `scheduler` @ 0x%08" PRIx64,
				(uint64_t)sched_sym);
		return retval;
	}

	if (!sched_ptr) {
		LOG_WARNING("rtoskit: update_threads: scheduler global is NULL — "
				"scheduler_init() has not run yet");
		rtos->thread_details = calloc(1, sizeof(struct thread_detail));
		if (!rtos->thread_details)
			return ERROR_FAIL;
		rtos->thread_details[0].threadid = 1;
		rtos->thread_details[0].exists = true;
		rtos->thread_details[0].thread_name_str = strdup("rtoskit:not-started");
		rtos->thread_details[0].extra_info_str = strdup("scheduler_init() not yet run");
		rtos->thread_count = 1;
		rtos->current_thread = 1;
		return ERROR_OK;
	}

	uint32_t marker = 0;
	(void)target_read_u32(rtos->target, sched_ptr + RTOSKIT_SCHED_MARKER, &marker);
	uint32_t running = 0;
	(void)target_read_u32(rtos->target, sched_ptr + RTOSKIT_SCHED_RUNNING, &running);

	uint32_t tasks_head = sched_ptr + RTOSKIT_SCHED_TASKS;
	uint32_t cls_offset = (uint32_t)(curtask_sym - cdata);
	int num_cores = clocal1 ? 2 : 1;
	if (p)
		p->num_cores = num_cores;

	/* Sanity-check the CLS offset against the per-core block size, when known. */
	if (cdata_size && cls_offset >= (uint32_t)cdata_size) {
		LOG_ERROR("rtoskit: update_threads: current_task cls_offset 0x%" PRIx32
				" exceeds __core_data_size 0x%" PRIx32,
				cls_offset, (uint32_t)cdata_size);
		return ERROR_FAIL;
	}

	LOG_DEBUG("rtoskit: update_threads: scheduler@0x%08" PRIx32 " marker=0x%08" PRIx32
			" running=%" PRIu32 " tasks_head@0x%08" PRIx32 " num_cores=%d",
			sched_ptr, marker, running, tasks_head, num_cores);

	/* Read per-core current_task pointers so we can flag the matching TCBs. */
	uint32_t cur_tcb[2] = { 0, 0 };
	for (int c = 0; c < num_cores; ++c) {
		uint32_t clocal_base = (c == 0) ? (uint32_t)clocal0 : (uint32_t)clocal1;
		uint32_t cls_addr = clocal_base + cls_offset;
		(void)target_read_u32(rtos->target, cls_addr, &cur_tcb[c]);
		LOG_DEBUG("rtoskit: update_threads: core%d: current_task@0x%08" PRIx32
				" = 0x%08" PRIx32, c, cls_addr, cur_tcb[c]);
	}

	/* Walk scheduler->tasks via the embedded sched_list head.
	 *   head.next points at the first task's scheduler_node;
	 *   tcb_addr = node_addr - offsetof(scheduler_node).
	 * Bound the walk to RTOSKIT_MAX_TASKS to defend against corruption.
	 */
	uint32_t node = 0;
	retval = target_read_u32(rtos->target, tasks_head, &node); /* head.next */
	if (retval != ERROR_OK) {
		LOG_ERROR("rtoskit: update_threads: cannot read tasks_head.next @ 0x%08" PRIx32,
				tasks_head);
		return retval;
	}

	uint32_t tcbs[RTOSKIT_MAX_TASKS];
	int n_tcbs = 0;
	while (node != tasks_head && n_tcbs < RTOSKIT_MAX_TASKS) {
		uint32_t tcb_ptr = node - RTOSKIT_TCB_SCHEDULER_NODE;
		tcbs[n_tcbs++] = tcb_ptr;
		uint32_t next = 0;
		retval = target_read_u32(rtos->target, node, &next);
		if (retval != ERROR_OK) {
			LOG_ERROR("rtoskit: update_threads: cannot follow node.next @ 0x%08" PRIx32,
					node);
			return retval;
		}
		LOG_DEBUG("rtoskit: walk: node@0x%08" PRIx32 " tcb=0x%08" PRIx32
				" next=0x%08" PRIx32, node, tcb_ptr, next);
		node = next;
	}
	if (n_tcbs == RTOSKIT_MAX_TASKS)
		LOG_ERROR("rtoskit: update_threads: hit RTOSKIT_MAX_TASKS=%d cap — list may be corrupt",
				RTOSKIT_MAX_TASKS);

	LOG_DEBUG("rtoskit: update_threads: walked %d tasks (head@0x%08" PRIx32
			" cur=[0x%08" PRIx32 ", 0x%08" PRIx32 "])",
			n_tcbs, tasks_head, cur_tcb[0], cur_tcb[1]);

	if (n_tcbs == 0) {
		/* No live tasks — show a synthetic placeholder so GDB has something. */
		rtos->thread_details = calloc(1, sizeof(struct thread_detail));
		if (!rtos->thread_details)
			return ERROR_FAIL;
		rtos->thread_details[0].threadid = 1;
		rtos->thread_details[0].exists = true;
		rtos->thread_details[0].thread_name_str = strdup("rtoskit:empty");
		rtos->thread_details[0].extra_info_str = strdup("scheduler->tasks list empty");
		rtos->thread_count = 1;
		rtos->current_thread = 1;
		return ERROR_OK;
	}

	rtos->thread_details = calloc(n_tcbs, sizeof(struct thread_detail));
	if (!rtos->thread_details)
		return ERROR_FAIL;
	rtos->thread_count = n_tcbs;
	rtos->current_thread = 0;

	for (int i = 0; i < n_tcbs; ++i) {
		uint32_t tcb_ptr = tcbs[i];

		uint32_t tmarker = 0;
		retval = target_read_u32(rtos->target, tcb_ptr + RTOSKIT_TCB_MARKER, &tmarker);
		bool bad_marker = (retval != ERROR_OK) || (tmarker != RTOSKIT_TASK_MARKER_VALUE);

		uint32_t state = 0, tcore = 0, bp = 0, cp = 0, psp = 0;
		(void)target_read_u32(rtos->target, tcb_ptr + RTOSKIT_TCB_PSP, &psp);
		(void)target_read_u32(rtos->target, tcb_ptr + RTOSKIT_TCB_STATE, &state);
		(void)target_read_u32(rtos->target, tcb_ptr + RTOSKIT_TCB_CORE, &tcore);
		(void)target_read_u32(rtos->target, tcb_ptr + RTOSKIT_TCB_BASE_PRIORITY, &bp);
		(void)target_read_u32(rtos->target, tcb_ptr + RTOSKIT_TCB_CURRENT_PRIORITY, &cp);

		const char *state_name;
		switch (state) {
		case RTOSKIT_TASK_TERMINATED: state_name = "TERMINATED"; break;
		case RTOSKIT_TASK_BLOCKED:    state_name = "BLOCKED";    break;
		case RTOSKIT_TASK_SUSPENDED:  state_name = "SUSPENDED";  break;
		case RTOSKIT_TASK_READY:      state_name = "READY";      break;
		case RTOSKIT_TASK_RUNNING:    state_name = "RUNNING";    break;
		default:                      state_name = "?";          break;
		}

		int running_on = -1;
		for (int c = 0; c < num_cores; ++c)
			if (cur_tcb[c] == tcb_ptr)
				running_on = c;

		LOG_DEBUG("rtoskit: task[%d]: tcb=0x%08" PRIx32 " psp=0x%08" PRIx32
				" state=%s(%" PRIu32 ") tcb.core=%" PRIu32
				" prio=%" PRIu32 "/%" PRIu32 " running_on=%d%s",
				i, tcb_ptr, psp, state_name, state, tcore, bp, cp,
				running_on, bad_marker ? " BAD_MARKER" : "");

		rtos->thread_details[i].threadid = (threadid_t)tcb_ptr;
		rtos->thread_details[i].exists = true;
		rtos->thread_details[i].thread_name_str =
				alloc_printf("task@0x%08" PRIx32, tcb_ptr);
		if (bad_marker)
			rtos->thread_details[i].extra_info_str = alloc_printf(
					"BAD_MARKER=0x%08" PRIx32 " state=%s prio=%" PRIu32 "/%" PRIu32,
					tmarker, state_name, bp, cp);
		else if (running_on >= 0)
			rtos->thread_details[i].extra_info_str = alloc_printf(
					"state=%s core=%" PRIu32 " (RUNNING on cm%d) prio=%" PRIu32
					"/%" PRIu32, state_name, tcore, running_on, bp, cp);
		else
			rtos->thread_details[i].extra_info_str = alloc_printf(
					"state=%s prio=%" PRIu32 "/%" PRIu32,
					state_name, bp, cp);

		/* Pick rtos->current_thread = the task running on this rtos's head core. */
		if (running_on == rtos->target->coreid)
			rtos->current_thread = (threadid_t)tcb_ptr;
	}

	if (rtos->current_thread == 0) {
		rtos->current_thread = rtos->thread_details[0].threadid;
		LOG_DEBUG("rtoskit: update_threads: no running task on head core %" PRId32
				" — falling back to thread[0]", rtos->target->coreid);
	}

	return ERROR_OK;
}

/* ------------------------------------------------------------------------- */
/* gdb_target_for_threadid                                                   */
/*                                                                           */
/* Route register reads to the correct hardware target.  Synthetic per-core  */
/* placeholder IDs (1..num_cores, emitted for idle / no-current-task cores)  */
/* map directly to the matching core.  Real TCB IDs are resolved by reading  */
/* TCB.state and TCB.core; when state == RUNNING the request goes to that    */
/* core's target so GDB sees the live register cache.  Non-running tasks     */
/* are served by the SMP head — memory reads only, used by the saved-frame   */
/* unwinder in get_thread_reg_list.                                          */
/* ------------------------------------------------------------------------- */

static int rtoskit_gdb_target_for_threadid(struct connection *connection,
		int64_t thread_id, struct target **p_target)
{
	struct target *head = get_target_from_connection(connection);
	struct rtos *rtos = head ? head->rtos : NULL;
	struct target_list *tl;
	struct rtoskit_params *p = rtos ? rtos->rtos_specific_params : NULL;
	int num_cores = p ? p->num_cores : 0;

	if (!rtos || !head) {
		LOG_WARNING("rtoskit: gdb_target_for_threadid: no rtos/head, tid=0x%" PRIx64,
				(uint64_t)thread_id);
		if (p_target)
			*p_target = head;
		return ERROR_OK;
	}

	int target_core = -1;

	if (thread_id > 0 && thread_id <= num_cores) {
		/* Synthetic idle / invalid id: 1 => core 0, 2 => core 1. */
		target_core = (int)(thread_id - 1);
		LOG_DEBUG("rtoskit: gdb_target_for_threadid: synthetic tid=%" PRId64
				" -> core %d", thread_id, target_core);
	} else if (thread_id > 0) {
		/* Real TCB pointer.  Validate marker, then route by state/core. */
		uint32_t tcb_ptr = (uint32_t)thread_id;
		uint32_t tmarker = 0, state = 0, tcore = 0;
		int retval;
		retval = target_read_u32(head, tcb_ptr + RTOSKIT_TCB_MARKER, &tmarker);
		if (retval == ERROR_OK && tmarker == RTOSKIT_TASK_MARKER_VALUE) {
			(void)target_read_u32(head, tcb_ptr + RTOSKIT_TCB_STATE, &state);
			(void)target_read_u32(head, tcb_ptr + RTOSKIT_TCB_CORE, &tcore);
			if (state == RTOSKIT_TASK_RUNNING)
				target_core = (int)tcore;
			LOG_DEBUG("rtoskit: gdb_target_for_threadid: tcb=0x%08" PRIx32
					" state=%" PRIu32 " core=%" PRIu32 " -> core %d",
					tcb_ptr, state, tcore, target_core);
		} else {
			LOG_DEBUG("rtoskit: gdb_target_for_threadid: tid=0x%" PRIx64
					" failed marker check", (uint64_t)thread_id);
		}
	}

	if (target_core >= 0 && head->smp_targets) {
		foreach_smp_target(tl, head->smp_targets) {
			if (tl->target->coreid == target_core) {
				if (p_target)
					*p_target = tl->target;
				return ERROR_OK;
			}
		}
		LOG_WARNING("rtoskit: gdb_target_for_threadid: tid=0x%" PRIx64
				" core %d not found in SMP group",
				(uint64_t)thread_id, target_core);
	}

	if (p_target)
		*p_target = head;
	return ERROR_OK;
}

/* ------------------------------------------------------------------------- */
/* fill_live_regs — copy a target's live GDB register cache into rtos_reg[]  */
/*                                                                           */
/* Mirrors the pattern in hwthread_get_thread_reg_list (hwthread.c) so SMP   */
/* "this thread is on core N" lookups return the live cm<N> register set.   */
/* ------------------------------------------------------------------------- */

static int rtoskit_fill_live_regs(struct target *t,
		struct rtos_reg **out_list, int *out_num)
{
	if (!target_was_examined(t))
		return ERROR_FAIL;

	struct reg **reg_list = NULL;
	int reg_list_size = 0;
	int retval = target_get_gdb_reg_list(t, &reg_list, &reg_list_size,
			REG_CLASS_GENERAL);
	if (retval != ERROR_OK)
		return retval;

	int kept = 0;
	for (int i = 0; i < reg_list_size; i++)
		if (reg_list[i] && reg_list[i]->exist && !reg_list[i]->hidden)
			kept++;

	*out_list = calloc(kept, sizeof(struct rtos_reg));
	if (!*out_list) {
		free(reg_list);
		return ERROR_FAIL;
	}
	*out_num = kept;

	int j = 0;
	for (int i = 0; i < reg_list_size; i++) {
		struct reg *r = reg_list[i];
		if (!r || !r->exist || r->hidden)
			continue;
		if (!r->valid) {
			retval = r->type->get(r);
			if (retval != ERROR_OK) {
				LOG_TARGET_ERROR(t, "rtoskit: cannot read register %s",
						r->name);
				free(reg_list);
				free(*out_list);
				*out_list = NULL;
				return retval;
			}
		}
		(*out_list)[j].number = r->number;
		(*out_list)[j].size = r->size;
		memcpy((*out_list)[j].value, r->value, DIV_ROUND_UP(r->size, 8));
		j++;
	}
	free(reg_list);
	return ERROR_OK;
}

static struct target *rtoskit_target_for_coreid(struct target *head, int coreid)
{
	if (!head)
		return NULL;
	if (!head->smp_targets) {
		if (head->coreid == coreid)
			return head;
		return NULL;
	}
	struct target_list *tl;
	foreach_smp_target(tl, head->smp_targets) {
		if (tl->target->coreid == coreid)
			return tl->target;
	}
	return NULL;
}

/* ------------------------------------------------------------------------- */
/* rtoskit_unwind_saved_frame                                                */
/*                                                                           */
/* Read the saved scheduler_frame from a non-running task's stack and return */
/* the GDB register file via rtos_generic_stack_read.                        */
/*                                                                           */
/* The struct scheduler_frame layout is fixed at firmware compile time by    */
/* __FPU_USED, NOT by the runtime EXC_RETURN value.  When the firmware is    */
/* built with the FPU enabled, the FP register slots are always present in   */
/* memory (208-byte frame); EXC_RETURN.FType only indicates whether those    */
/* slots hold valid data for this particular task.  Stacking selection here  */
/* therefore tracks the firmware's compile-time FPU setting (proxied via the */
/* target's hardware FPU presence), not EXC_RETURN.                          */
/* ------------------------------------------------------------------------- */

static int rtoskit_unwind_saved_frame(struct rtos *rtos, uint32_t tcb_ptr,
		uint32_t state, struct rtos_reg **reg_list, int *num_regs)
{
	struct rtoskit_params *p = rtos->rtos_specific_params;
	int retval;

	uint32_t psp = 0;
	retval = target_read_u32(rtos->target, tcb_ptr + RTOSKIT_TCB_PSP, &psp);
	if (retval != ERROR_OK || psp == 0) {
		LOG_WARNING("rtoskit: unwind: tcb=0x%08" PRIx32 " bad psp=0x%08" PRIx32
				" (retval=%d)", tcb_ptr, psp, retval);
		return ERROR_FAIL;
	}

	const struct rtos_register_stacking *stk;
	const char *stk_name;
	if (p && p->has_fpu) {
		stk = &rtoskit_stacking_fp;
		stk_name = "fpu";
	} else {
		stk = &rtoskit_stacking_nofp;
		stk_name = "nofp";
	}

	uint32_t exc_return = 0;
	(void)target_read_u32(rtos->target, psp + RTOSKIT_FRAME_EXEC_RETURN, &exc_return);
	LOG_DEBUG("rtoskit: unwind: tcb=0x%08" PRIx32 " psp=0x%08" PRIx32
			" exc_return=0x%08" PRIx32 " state=%" PRIu32 " stacking=%s",
			tcb_ptr, psp, exc_return, state, stk_name);

	return rtos_generic_stack_read(rtos->target, stk, psp, reg_list, num_regs);
}

/* ------------------------------------------------------------------------- */
/* get_thread_reg_list                                                       */
/*                                                                           */
/* gdb_server bypasses its live-CPU fast path when the target is in an SMP   */
/* group, so the driver must serve register reads even for the currently     */
/* running task.  Synthetic placeholder IDs and RUNNING TCBs are answered    */
/* from the routed target's live register cache; non-running TCBs go through */
/* the saved-frame unwinder.                                                 */
/* ------------------------------------------------------------------------- */

static int rtoskit_get_thread_reg_list(struct rtos *rtos, int64_t thread_id,
		struct rtos_reg **reg_list, int *num_regs)
{
	if (!rtos)
		return ERROR_FAIL;

	struct rtoskit_params *p = rtos->rtos_specific_params;
	int num_cores = p ? p->num_cores : 0;
	struct target *head = rtos->target;

	int target_core = -1;
	const char *route_reason = "?";

	if (thread_id > 0 && thread_id <= num_cores) {
		target_core = (int)(thread_id - 1);
		route_reason = "synthetic idle";
	} else if (thread_id > 0) {
		uint32_t tcb_ptr = (uint32_t)thread_id;
		uint32_t tmarker = 0, state = 0, tcore = 0;
		int retval = target_read_u32(head, tcb_ptr + RTOSKIT_TCB_MARKER, &tmarker);
		if (retval == ERROR_OK && tmarker == RTOSKIT_TASK_MARKER_VALUE) {
			(void)target_read_u32(head, tcb_ptr + RTOSKIT_TCB_STATE, &state);
			(void)target_read_u32(head, tcb_ptr + RTOSKIT_TCB_CORE, &tcore);
			if (state == RTOSKIT_TASK_RUNNING) {
				target_core = (int)tcore;
				route_reason = "RUNNING tcb";
			} else {
				/* Non-running: unwind the saved scheduler_frame. */
				return rtoskit_unwind_saved_frame(rtos, tcb_ptr, state,
						reg_list, num_regs);
			}
		} else {
			LOG_WARNING("rtoskit: get_thread_reg_list: tid=0x%" PRIx64
					" failed marker check (marker=0x%08" PRIx32 ")",
					(uint64_t)thread_id, tmarker);
			return ERROR_FAIL;
		}
	} else {
		return ERROR_FAIL;
	}

	struct target *t = rtoskit_target_for_coreid(head, target_core);
	if (!t) {
		LOG_WARNING("rtoskit: get_thread_reg_list: no SMP target for core %d (%s)",
				target_core, route_reason);
		return ERROR_FAIL;
	}

	LOG_DEBUG("rtoskit: get_thread_reg_list: tid=0x%" PRIx64
			" -> core %d (%s) -> %s",
			(uint64_t)thread_id, target_core, route_reason, target_name(t));
	return rtoskit_fill_live_regs(t, reg_list, num_regs);
}
