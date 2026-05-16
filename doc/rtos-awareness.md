# OpenOCD RTOS Awareness — Architecture and Implementation Guide

This document explains how OpenOCD implements *RTOS awareness*: the mechanism
by which a debug session can see the threads of a running RTOS as first-class
GDB threads, switch between them, inspect their stacks, and report current
state. It covers the public API surface, the interaction with GDB's Remote
Serial Protocol (RSP), the data model exported by an RTOS driver, and a
step-by-step recipe for implementing a new (custom) RTOS awareness module.

All file references are relative to the OpenOCD source tree
(`src/rtos/...`).

---

## 1. What RTOS awareness actually does

Without RTOS awareness OpenOCD presents the CPU(s) it is debugging as bare
hardware: one "thread" per physical core whose register state is whatever the
core is currently holding. GDB sees a single (or per-core) execution context.

With RTOS awareness OpenOCD synthesises a virtual thread list from kernel
data structures it reads out of target memory. To GDB it then looks as if
the target is running N pthreads: each TCB becomes a thread ID, each thread's
saved context (typically a register frame pushed on its private stack) can be
read on demand, and GDB commands such as `info threads`, `thread N`,
`bt`, and `thread apply all bt` Just Work.

The implementation is entirely *host side*. The target does not cooperate
beyond exporting a handful of well-known kernel symbols (e.g.
`pxCurrentTCB`, `pxReadyTasksLists`). All thread enumeration and stack
unwinding happens by reading target memory from OpenOCD.

Key consequences of this host-side design:

- No code or hooks are added to the RTOS image.
- The driver must understand the *exact* layout of the RTOS's TCB,
  ready lists, suspended lists, etc.
- Anything the RTOS does that isn't observable through memory (e.g.
  ephemeral state in CPU registers of a context not yet stacked) is
  invisible.
- The driver must implement its own "stack unwinder" that mirrors the
  RTOS's context-save layout for the given CPU/ABI.

---

## 2. Source layout

```
src/rtos/rtos.h                       /* public types and API */
src/rtos/rtos.c                       /* core machinery + GDB packet handlers */

src/rtos/rtos_standard_stackings.[ch] /* prebuilt Cortex-M3/M4F/M4F-FPU/R4 frames */
src/rtos/rtos_chibios_stackings.[ch]
src/rtos/rtos_ecos_stackings.[ch]
src/rtos/rtos_embkernel_stackings.[ch]
src/rtos/rtos_mqx_stackings.[ch]
src/rtos/rtos_nuttx_stackings.[ch]
src/rtos/rtos_riot_stackings.[ch]
src/rtos/rtos_ucos_iii_stackings.[ch]

src/rtos/freertos.c                   /* one driver per supported RTOS */
src/rtos/chibios.c
src/rtos/ecos.c
src/rtos/embkernel.c
src/rtos/hwthread.c                   /* hardware threads (no RTOS) */
src/rtos/linux.c
src/rtos/mqx.c
src/rtos/nuttx.c
src/rtos/riot.c
src/rtos/rtkernel.c
src/rtos/threadx.c
src/rtos/ucos_iii.c
src/rtos/zephyr.c
src/rtos/chromium-ec.c
```

The static table `rtos_types[]` in `src/rtos/rtos.c` registers every
known driver. `hwthread_rtos` is always last because it is the universal
fallback used by `-rtos auto` when nothing else matches.

---

## 3. Data model

### 3.1 `struct rtos_type` — the driver vtable

Defined in `src/rtos/rtos.h`. Each entry is a function pointer that is
invoked at a well-defined point in the lifecycle.

```c
struct rtos_type {
    const char *name;

    bool (*detect_rtos)(struct target *target);
    int  (*create)(struct target *target);
    int  (*smp_init)(struct target *target);
    int  (*update_threads)(struct rtos *rtos);

    int  (*get_thread_reg_list)(struct rtos *rtos, int64_t thread_id,
                                struct rtos_reg **reg_list, int *num_regs);
    int  (*get_thread_reg_value)(struct rtos *rtos, threadid_t thread_id,
                                 uint32_t reg_num, uint32_t *size,
                                 uint8_t **value);

    int  (*get_symbol_list_to_lookup)(struct symbol_table_elem *symbol_list[]);
    int  (*clean)(struct target *target);

    char *(*ps_command)(struct target *target);

    int  (*set_reg)(struct rtos *rtos, uint32_t reg_num, uint8_t *reg_value);

    int  (*read_buffer)(struct rtos *rtos, target_addr_t address,
                        uint32_t size, uint8_t *buffer);
    int  (*write_buffer)(struct rtos *rtos, target_addr_t address,
                         uint32_t size, const uint8_t *buffer);

    bool (*needs_fake_step)(struct target *target, int64_t thread_id);

    struct target *(*swbp_target)(struct rtos *rtos, target_addr_t address,
                                  uint32_t length, enum breakpoint_type type);
};
```

Mandatory callbacks for a minimally functional driver:

- `name` — string used after `-rtos <name>` in config files.
- `detect_rtos` — called when auto-detect mode has resolved every required
  symbol. Returns `true` to claim the target.
- `create` — final, per-driver initialisation; allocate
  `rtos->rtos_specific_params` and pick the appropriate stacking based on
  CPU/ABI. May override `rtos->gdb_thread_packet` and
  `rtos->gdb_target_for_threadid`.
- `update_threads` — rebuild `rtos->thread_details[]` and update
  `rtos->current_thread`. This is the workhorse — every refresh comes
  back through here.
- `get_thread_reg_list` — return the saved general-purpose register file
  for a given `thread_id`. For the *currently running* thread on the
  hardware this should usually defer to the live CPU register cache.
- `get_symbol_list_to_lookup` — return a NULL-terminated
  `struct symbol_table_elem[]` listing the kernel symbols the driver
  needs GDB to resolve. Order in the array implicitly defines the
  per-driver enum values used to index `rtos->symbols[]`.

Optional callbacks:

- `smp_init` — wire the same `struct rtos` instance into every target of
  an SMP group, so `info threads` sees one unified list.
- `get_thread_reg_value` / `set_reg` — finer-grained per-register
  read/write (used for `p $reg = ...` from GDB on a non-current thread).
- `ps_command` — back the `<rtos> ps` Tcl/Telnet command (free-form
  status string).
- `read_buffer` / `write_buffer` — for RTOSes with per-thread address
  spaces (e.g. Linux process VA). Cortex-M class RTOSes leave these
  NULL.
- `needs_fake_step` / `swbp_target` — gdb-protocol corner cases for SMP
  and for the "stepping a non-current thread" workaround.
- `clean` — release driver-private state allocated in `create`.

### 3.2 `struct rtos` — per-target instance

```c
struct rtos {
    const struct rtos_type *type;

    struct symbol_table_elem *symbols; /* resolved kernel symbols */
    struct target *target;

    int64_t   current_threadid;        /* the thread GDB has selected */
    threadid_t current_thread;         /* what the hardware/RTOS thinks */

    struct thread_detail *thread_details;
    int thread_count;

    int (*gdb_thread_packet)(struct connection *, char const *, int);
    int (*gdb_target_for_threadid)(struct connection *, int64_t,
                                   struct target **);

    void *rtos_specific_params;        /* driver-owned blob */
};
```

`current_threadid` is the thread *GDB* has issued `Hg <tid>` for.
`current_thread` is the one the RTOS scheduler last ran on a hardware
core. When they differ, `rtos_get_gdb_reg_list()` synthesises the
register file from the saved stack frame; when they match, it falls
through to the normal target register cache.

### 3.3 `struct thread_detail` — one entry per task

```c
struct thread_detail {
    threadid_t  threadid;        /* opaque to GDB; usually the TCB address */
    bool        exists;
    char       *thread_name_str; /* e.g. "IDLE", "tcpip_task" */
    char       *extra_info_str;  /* e.g. "State: Blocked, Prio: 5"   */
};
```

Memory for `thread_name_str` / `extra_info_str` is owned by the driver
and freed by `rtos_free_threadlist()`. Use `strdup`/`alloc_printf`.

`threadid` must be non-zero (GDB treats 0 as "any thread") and unique
within the snapshot. Using the TCB address is the conventional choice.

### 3.4 `struct symbol_table_elem` — symbol manifest

```c
struct symbol_table_elem {
    const char       *symbol_name;
    symbol_address_t  address;     /* filled in by rtos_qsymbol() */
    bool              optional;    /* mandatory symbols force the auto-detect to fail */
};
```

The driver populates the names; `rtos_qsymbol()` populates the
addresses via the GDB `qSymbol` exchange. Optional symbols are used for
features that may be compiled out of the RTOS (e.g. `xSuspendedTaskList`
when `INCLUDE_vTaskSuspend == 0`).

### 3.5 `struct rtos_reg` — one register value

```c
struct rtos_reg {
    uint32_t number;     /* GDB register number */
    uint32_t size;       /* in bits */
    uint8_t  value[16];  /* little-endian; holds up to 128 bits */
};
```

For Cortex-M `number` matches the `ARMV7M_*` enum from
`target/armv7m.h`.

### 3.6 `struct rtos_register_stacking` — the unwinder description

```c
struct stack_register_offset {
    unsigned short number;      /* gdb register number */
    signed short   offset;      /* byte offset in stack frame
                                   -1 => not stacked, value undefined
                                   -2 => this is the stack pointer */
    unsigned short width_bits;
};

struct rtos_register_stacking {
    unsigned char stack_registers_size;
    signed char   stack_growth_direction;  /* +1 grows up, -1 grows down */
    unsigned char num_output_registers;

    target_addr_t (*calculate_process_stack)(struct target *,
                          const uint8_t *stack_data,
                          const struct rtos_register_stacking *,
                          target_addr_t stack_ptr);

    const struct stack_register_offset *register_offsets;

    int (*read_stack)(struct target *target,
                      int64_t stack_ptr,
                      const struct rtos_register_stacking *,
                      uint8_t *stack_data);
};
```

`calculate_process_stack` lets the unwinder account for ABI quirks such
as the optional 8-byte alignment padding indicated by Cortex-M XPSR
bit 9. `read_stack` is used by RTOSes whose context save format needs
fix-up after the raw memory read (Zephyr's lazy-FP handling, ChibiOS
half-stored frames, etc.). When both fields are NULL the generic
helper `rtos_generic_stack_read()` does a single `target_read_buffer`
and applies the offset table directly.

`rtos_standard_stackings.c` provides four ready-made tables:

- `rtos_standard_cortex_m3_stacking` — basic 16-register frame (ARMv6-M
  and ARMv7-M without FPU).
- `rtos_standard_cortex_m4f_stacking` — Cortex-M4F when the FPU is
  enabled but the context save *does not* include S0–S31.
- `rtos_standard_cortex_m4f_fpu_stacking` — full lazy-stacking with
  S0–S31 + FPSCR.
- `rtos_standard_cortex_r4_stacking` — Cortex-R4 with FPA placeholders.

For a new Cortex-M RTOS, picking the right one (or a small variant) is
usually all that's needed.

---

## 4. Lifecycle

### 4.1 Registration

A driver exposes a single `const struct rtos_type` (e.g.
`freertos_rtos`) and references itself in two places:

1. `extern const struct rtos_type my_rtos;` in `src/rtos/rtos.h`.
2. An entry in the `rtos_types[]` array in `src/rtos/rtos.c`, in
   alphabetical order. `hwthread_rtos` *must* stay last.

The driver source must be added to `src/rtos/Makefile.am`.

### 4.2 Configuration

The Tcl command `<target> configure -rtos <name>` (parsed in
`target/target.c`) eventually calls `rtos_create()` in
`src/rtos/rtos.c`. Three modes:

- `-rtos none` — `target->rtos` stays NULL; no awareness.
- `-rtos <name>` — `os_alloc_create()` allocates `struct rtos`, sets
  `type`, calls `type->create()`.
- `-rtos auto` — `os_alloc()` allocates `struct rtos` and points it at
  the first driver in `rtos_types[]`; `target->rtos_auto_detect` is set
  to `true`. The actual driver is chosen later during the `qSymbol`
  exchange.

### 4.3 Symbol resolution

When GDB attaches, it sends `qSymbol::` (empty). `rtos_qsymbol()` in
`src/rtos/rtos.c` is the state machine that walks the
`get_symbol_list_to_lookup()` array, replying with `qSymbol:<hex-name>`
for the next symbol the driver wants. GDB looks the name up in its
loaded symbol files and replies `qSymbol:<addr>:<hex-name>`. The
process iterates until either:

- the symbol list is exhausted (driver selected manually) — `detect_rtos`
  is *not* called, the driver is taken as given;
- the symbol list is exhausted (auto-detect mode) — `detect_rtos` is
  invoked. If it returns true, the driver is locked in; if false,
  `rtos_try_next()` advances to the next driver and the qSymbol loop
  restarts.
- a mandatory symbol is missing — auto-detect advances to the next
  driver; an explicit selection logs a warning and bails.

Special case: LTO. If the first lookup of `name` fails,
`rtos_qsymbol()` retries with `name.lto_priv.0` to handle GCC's LTO
mangling of file-static symbols.

Once a driver has been selected by auto-detection, `rtos_thread_packet()`
calls `type->create()` (now that the driver is final) and then
`type->update_threads()` once.

### 4.4 Steady state

Every time the target stops, GDB sends a `g`/`Hg`/`qfThreadInfo`/
`qThreadExtraInfo` burst. The handler in `rtos_thread_packet()`:

- `qfThreadInfo` — emit the comma-separated list of
  `thread_details[i].threadid`.
- `qsThreadInfo` — always reply `l` (end of list); the whole list goes
  in one `qfThreadInfo` reply.
- `qThreadExtraInfo,<tid>` — hexified
  `"Name: <name>, <extra_info>"`.
- `T<tid>` — alive check, by lookup in `thread_details[]`.
- `qC` — emit `QC<current_thread>` in hex.
- `Hg<tid>` — record GDB's selected thread in `current_threadid`.

When GDB asks for the register file (`g` or `p <reg>`),
`gdb_server.c` calls `rtos_get_gdb_reg_list()` /
`rtos_get_gdb_reg()`. If `current_threadid` differs from
`current_thread` (or the target is SMP), the RTOS path is taken and the
driver synthesises the register frame; otherwise the normal live-CPU
register cache is used. This is the key invariant — the driver only
needs to unwind contexts that *are not* the currently running one.

`update_threads` is invoked from `rtos_update_threads()` whenever the
debug server detects that the target halted; the gdb_server layer
forces a refresh before letting GDB observe the new state.

### 4.5 Teardown

`rtos_destroy()` calls `os_free()`, which frees
`rtos->symbols`, walks `thread_details[]` freeing the strings, frees
the array, and frees the `struct rtos`. The driver's
`rtos_specific_params` is the driver's responsibility — if you allocate
it in `create`, free it in `clean` (the core does not currently invoke
`clean`, so most drivers leak `rtos_specific_params`; if you allocate a
heavy blob, hook teardown by overriding the destroy path or, more
practically, plumbing it through `clean` and adding the call).

---

## 5. GDB protocol mapping (cheat sheet)

| GDB packet                | Handler                                       | RTOS driver touchpoint                       |
|---------------------------|-----------------------------------------------|----------------------------------------------|
| `qSymbol::` / `qSymbol:`  | `rtos_qsymbol()`                              | `get_symbol_list_to_lookup`, `detect_rtos`   |
| `qfThreadInfo`            | `rtos_thread_packet()`                        | `thread_details[].threadid`                  |
| `qsThreadInfo`            | `rtos_thread_packet()`                        | (constant `l`)                               |
| `qThreadExtraInfo,<tid>`  | `rtos_thread_packet()`                        | `thread_name_str`, `extra_info_str`          |
| `qC`                      | `rtos_thread_packet()`                        | `current_thread`                             |
| `T<tid>`                  | `rtos_thread_packet()`                        | `thread_details[].exists`                    |
| `Hg<tid>` / `Hc<tid>`     | `rtos_thread_packet()`                        | `current_threadid`                           |
| `g`                       | `gdb_server.c` → `rtos_get_gdb_reg_list()`    | `get_thread_reg_list`                        |
| `p <n>`                   | `gdb_server.c` → `rtos_get_gdb_reg()`         | `get_thread_reg_value` or `get_thread_reg_list` |
| `P <n>=<v>`               | `gdb_server.c` → `rtos_set_reg()`             | `set_reg`                                    |
| `m`/`M` (with VA)         | `rtos_read_buffer` / `rtos_write_buffer`      | `read_buffer` / `write_buffer`               |

---

## 6. Implementing a custom RTOS awareness module

This is the recipe to add a new driver for an in-house RTOS. The
running example is a generic Cortex-M kernel called `mykit` with:

- `mykit_current_tcb` — pointer to current `struct tcb`.
- `mykit_task_list` — head of a doubly linked list of all tasks.
- `mykit_task_count` — `uint32_t` total task count.
- `struct tcb` containing at known offsets: `next`, `prev`, `name[16]`,
  `state` (enum), `priority`, `stack_top` (saved SP).

### 6.1 Skeleton file `src/rtos/mykit.c`

```c
// SPDX-License-Identifier: GPL-2.0-or-later

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "rtos.h"
#include "rtos_standard_stackings.h"
#include "target/armv7m.h"
#include "target/cortex_m.h"
#include "helper/log.h"
#include "helper/types.h"

#define MYKIT_TCB_NEXT_OFF        0x00
#define MYKIT_TCB_PREV_OFF        0x04
#define MYKIT_TCB_NAME_OFF        0x08
#define MYKIT_TCB_NAME_LEN        16
#define MYKIT_TCB_STATE_OFF       0x18
#define MYKIT_TCB_PRIO_OFF        0x1c
#define MYKIT_TCB_STACK_TOP_OFF   0x20

enum mykit_symbol_index {
    MYKIT_SYM_CURRENT_TCB = 0,
    MYKIT_SYM_TASK_LIST,
    MYKIT_SYM_TASK_COUNT,
};

static const char * const mykit_symbol_names[] = {
    [MYKIT_SYM_CURRENT_TCB] = "mykit_current_tcb",
    [MYKIT_SYM_TASK_LIST]   = "mykit_task_list",
    [MYKIT_SYM_TASK_COUNT]  = "mykit_task_count",
    NULL,
};

struct mykit_params {
    const struct rtos_register_stacking *stacking;
};

static bool mykit_detect_rtos(struct target *target);
static int  mykit_create(struct target *target);
static int  mykit_update_threads(struct rtos *rtos);
static int  mykit_get_thread_reg_list(struct rtos *rtos, int64_t thread_id,
                                      struct rtos_reg **reg_list, int *num_regs);
static int  mykit_get_symbol_list_to_lookup(struct symbol_table_elem *list[]);

const struct rtos_type mykit_rtos = {
    .name                       = "mykit",
    .detect_rtos                = mykit_detect_rtos,
    .create                     = mykit_create,
    .update_threads             = mykit_update_threads,
    .get_thread_reg_list        = mykit_get_thread_reg_list,
    .get_symbol_list_to_lookup  = mykit_get_symbol_list_to_lookup,
};
```

### 6.2 Symbol manifest

The list returned to the core is allocated fresh every time, even if the
content is constant — the core takes ownership and frees it on teardown.

```c
static int mykit_get_symbol_list_to_lookup(struct symbol_table_elem *list[])
{
    size_t n = ARRAY_SIZE(mykit_symbol_names);

    *list = calloc(n, sizeof(struct symbol_table_elem));
    if (!*list)
        return ERROR_FAIL;

    for (size_t i = 0; i < n - 1; ++i) {
        (*list)[i].symbol_name = mykit_symbol_names[i];
        (*list)[i].optional    = false;
    }
    /* last element terminates with NULL symbol_name (calloc already zeroed). */
    return ERROR_OK;
}
```

### 6.3 Detection

For auto-detect to lock onto the driver, return true only when the
target memory state matches what a live `mykit` kernel would look like.
A reasonable heuristic: scheduler counter is non-zero and the current
TCB pointer is non-NULL and well-aligned.

```c
static bool mykit_detect_rtos(struct target *target)
{
    if (!target->rtos->symbols)
        return false;

    if (target->rtos->symbols[MYKIT_SYM_CURRENT_TCB].address == 0)
        return false;

    uint32_t count = 0;
    if (target_read_u32(target,
            target->rtos->symbols[MYKIT_SYM_TASK_COUNT].address,
            &count) != ERROR_OK)
        return false;

    return count > 0 && count < 1024;
}
```

### 6.4 Create

Pick the correct stacking based on the target's CPU and FPU
configuration. The standard table already covers vanilla Cortex-M.

```c
static int mykit_create(struct target *target)
{
    struct mykit_params *p = calloc(1, sizeof(*p));
    if (!p)
        return ERROR_FAIL;

    /* Discriminate Cortex-M0/M0+/M3 vs M4F/M7 vs M4F-lazy-FP at runtime
     * by reading target->coreid or by inspecting CPACR / FPCCR. The
     * simplest, target-name based approach mirrors what freertos.c does.
     */
    const char *tname = target_type_name(target);
    if (strcmp(tname, "cortex_m") == 0 || strcmp(tname, "hla_target") == 0) {
        /* Default to plain M3 stacking; replace at runtime if FPU is on. */
        p->stacking = &rtos_standard_cortex_m3_stacking;
    } else {
        free(p);
        LOG_ERROR("mykit: unsupported target type %s", tname);
        return ERROR_FAIL;
    }

    target->rtos->rtos_specific_params = p;
    return ERROR_OK;
}
```

### 6.5 Walking the task list

`update_threads` is the heart of the driver. The pattern is:

1. Free any previous list (`rtos_free_threadlist`).
2. Read `current_tcb` and the count.
3. Allocate `thread_details` of the right size.
4. Walk the kernel's task list following `next` pointers until either
   the count is hit or the list closes back on the head.
5. For each TCB, fill in `threadid` (the TCB address), `exists`,
   `thread_name_str`, `extra_info_str`.
6. Identify which entry is the current one and set `current_thread`.

```c
static int mykit_update_threads(struct rtos *rtos)
{
    int retval;

    if (!rtos->symbols)
        return ERROR_FAIL;

    rtos_free_threadlist(rtos);

    uint32_t task_count = 0;
    retval = target_read_u32(rtos->target,
            rtos->symbols[MYKIT_SYM_TASK_COUNT].address, &task_count);
    if (retval != ERROR_OK)
        return retval;

    uint32_t current = 0;
    retval = target_read_u32(rtos->target,
            rtos->symbols[MYKIT_SYM_CURRENT_TCB].address, &current);
    if (retval != ERROR_OK)
        return retval;

    if (task_count == 0) {
        /* Show a single pseudo-thread for the live CPU, so GDB sees
         * *something* when the scheduler hasn't started. */
        rtos->thread_details = calloc(1, sizeof(struct thread_detail));
        rtos->thread_details[0].threadid        = 1;
        rtos->thread_details[0].exists          = true;
        rtos->thread_details[0].thread_name_str = strdup("[scheduler not started]");
        rtos->thread_count    = 1;
        rtos->current_thread  = 1;
        return ERROR_OK;
    }

    rtos->thread_details = calloc(task_count, sizeof(struct thread_detail));
    if (!rtos->thread_details)
        return ERROR_FAIL;

    /* mykit_task_list is the address of the list head; first real TCB is
     * head->next. Adjust the read pattern to match the actual layout. */
    uint32_t head = rtos->symbols[MYKIT_SYM_TASK_LIST].address;
    uint32_t tcb;
    retval = target_read_u32(rtos->target, head + MYKIT_TCB_NEXT_OFF, &tcb);
    if (retval != ERROR_OK)
        return retval;

    unsigned int i = 0;
    while (tcb != head && i < task_count) {
        char name[MYKIT_TCB_NAME_LEN + 1] = { 0 };
        retval = target_read_buffer(rtos->target, tcb + MYKIT_TCB_NAME_OFF,
                                    MYKIT_TCB_NAME_LEN, (uint8_t *)name);
        if (retval != ERROR_OK)
            return retval;
        name[MYKIT_TCB_NAME_LEN] = '\0';

        uint32_t state = 0, prio = 0;
        target_read_u32(rtos->target, tcb + MYKIT_TCB_STATE_OFF, &state);
        target_read_u32(rtos->target, tcb + MYKIT_TCB_PRIO_OFF,  &prio);

        rtos->thread_details[i].threadid        = tcb;
        rtos->thread_details[i].exists          = true;
        rtos->thread_details[i].thread_name_str = strdup(name);
        rtos->thread_details[i].extra_info_str  =
            alloc_printf("State: %" PRIu32 ", Prio: %" PRIu32,
                         state, prio);
        ++i;

        retval = target_read_u32(rtos->target, tcb + MYKIT_TCB_NEXT_OFF, &tcb);
        if (retval != ERROR_OK)
            return retval;
    }
    rtos->thread_count   = i;
    rtos->current_thread = current ? current : 0;
    return ERROR_OK;
}
```

Notes:

- Always sanity-check the loop with a hard upper bound (`i < task_count`)
  so a corrupted list cannot wedge OpenOCD in an infinite read loop.
- `target_read_u32` honours target endianness. Do not byte-swap
  manually.
- Memory accessors return negative `int` error codes; propagate them.
- If the kernel keeps multiple lists (ready by priority, blocked,
  delayed, suspended) walk each and union them. FreeRTOS (see
  `src/rtos/freertos.c`) is the canonical example of this pattern.

### 6.6 Returning register frames

For the currently running thread, the live CPU register cache is
authoritative. For every other thread the driver must read the
saved-context frame off the task's stack and convert it to a list of
`struct rtos_reg`. The helper `rtos_generic_stack_read()` does both
the read and the offset-table application.

```c
static int mykit_get_thread_reg_list(struct rtos *rtos, int64_t thread_id,
                                     struct rtos_reg **reg_list, int *num_regs)
{
    const struct mykit_params *p = rtos->rtos_specific_params;
    if (!p)
        return ERROR_FAIL;

    uint32_t stack_top = 0;
    int retval = target_read_u32(rtos->target,
                                 thread_id + MYKIT_TCB_STACK_TOP_OFF,
                                 &stack_top);
    if (retval != ERROR_OK)
        return retval;

    return rtos_generic_stack_read(rtos->target, p->stacking,
                                   stack_top, reg_list, num_regs);
}
```

If the saved frame is not a verbatim Cortex-M exception frame (e.g. the
kernel pushes its own callee-saved set in a different order, or skips
LR), build a custom `struct stack_register_offset[]` and
`struct rtos_register_stacking`. The pattern is identical to
`rtos_standard_stackings.c`.

If the FPU may or may not be active on a per-task basis, switch
stackings inside `mykit_get_thread_reg_list` by reading the EXC_RETURN
value or a per-task flag, then picking
`rtos_standard_cortex_m4f_stacking` vs `..._fpu_stacking`.

### 6.7 Registration

Edit `src/rtos/rtos.h`:

```c
extern const struct rtos_type mykit_rtos;
```

Edit `src/rtos/rtos.c`, alphabetical, **before** `hwthread_rtos`:

```c
&mykit_rtos,
```

Edit `src/rtos/Makefile.am`, alphabetical:

```
%D%/mykit.c \
```

Rebuild OpenOCD; configure the target with `-rtos mykit` (or
`-rtos auto`).

### 6.8 Minimal validation

A driver is ready for first light when, with target halted and the
RTOS booted:

```text
> openocd -f interface/cmsis-dap.cfg -f target/rp2350.cfg \
          -c "rp2350.cpu0 configure -rtos mykit"
(gdb) target extended-remote :3333
(gdb) info threads
  Id   Target Id                Frame
* 1    Thread 0x20002a00 (Name: main, State: 2, Prio: 8)   ...
  2    Thread 0x20003120 (Name: idle, State: 0, Prio: 0)   ...
(gdb) thread 2
(gdb) bt
```

If `info threads` lists nothing, instrument `mykit_update_threads`
with `LOG_DEBUG` calls and inspect `openocd -d3` output. If the list
is correct but `bt` produces garbage on a non-current thread, the
stack offset table is wrong — verify it against the kernel's actual
context-save sequence in the PendSV/SVC handler.

---

## 7. SMP considerations

For SMP RTOSes (FreeRTOS-SMP, Zephyr-SMP, Linux), each physical core
has its own OpenOCD `struct target`, but all should share a single
`struct rtos` so that the unified thread list is presented to GDB
once.

The hook is `smp_init`. The convention is:

1. The first target in the SMP group runs `os_alloc_create()` and gets
   `target->rtos`.
2. `smp_init` walks the SMP target list and copies that pointer into
   every member.
3. `update_threads` must mark *N* of the threads as "currently on
   core 0/1/…"; gdb_server adapts the `qC` reply per-core.

`current_thread` becomes multi-valued in effect — see `hwthread.c` and
`zephyr.c` for working implementations.

---

## 8. Pitfalls and idioms

- **Always free the previous thread list** at the top of
  `update_threads` (`rtos_free_threadlist(rtos)`). Forgetting to do so
  leaks one `thread_name_str` and `extra_info_str` per refresh.
- **Use `target_read_u32` / `target_buffer_get_u32`** rather than raw
  byte-shuffling. These honour the target's endianness.
- **Don't trust kernel pointers blindly.** Bound-check them against
  RAM ranges before dereferencing. A boot-time race or a corrupted
  list will otherwise crash OpenOCD by trying to read garbage
  addresses.
- **The "currently running" thread is special.** Do not synthesise a
  stack frame for it: the gdb_server fast-path returns live CPU
  registers when `current_threadid == current_thread`. Driver code
  should still list it in `thread_details[]` but `get_thread_reg_list`
  will not be called for it.
- **Stack pointer encoding.** Use offset `-2` in
  `stack_register_offset` to tell the unwinder "the value of this
  register is the *recomputed* SP", not a slot inside the frame.
- **XPSR bit 9 alignment.** When the ARM core takes an exception with
  an unaligned SP, it pushes 4 bytes of padding and sets XPSR[9]. The
  helper `rtos_cortex_m_stack_align()` accounts for this; reuse it in
  `calculate_process_stack` unless you have a kernel-specific reason
  not to.
- **Optional symbols.** Mark a symbol `optional = true` when the
  kernel feature it points at can be configured out. Mandatory
  missing symbols cause auto-detect to skip the driver.
- **Naming.** The `name` field is what users type after
  `-rtos`. Pick something stable and lowercase; it ends up in user
  Tcl configs.
- **`-rtos auto` ordering.** Drivers earlier in `rtos_types[]` are
  tried first. If your detector is loose (matches generic patterns),
  put it later or tighten the heuristic so it doesn't pre-empt
  others.
- **LTO.** GCC's LTO can mangle file-static symbol names to
  `name.lto_priv.0`. `rtos_qsymbol()` already retries with that
  suffix, but only the first such symbol. If your kernel exposes
  many file-static symbols compiled with LTO, expose them as global
  or non-static instead.

---

## 9. Reference reading

The cleanest end-to-end examples in tree are:

- `src/rtos/freertos.c` — multi-list traversal, optional symbols,
  CPU/FPU stacking selection.
- `src/rtos/zephyr.c` — SMP, custom `read_stack` for lazy-FP.
- `src/rtos/hwthread.c` — minimal pseudo-driver, no kernel; one
  thread per hardware core.
- `src/rtos/chibios.c` — compact, single ready list, good template
  for a small kernel.

The GDB Remote Serial Protocol packets relevant here are documented
in the GDB manual under "Remote Protocol", in particular the
sections "Thread-related Packets" and
"General Query Packets" (`qSymbol`, `qfThreadInfo`,
`qsThreadInfo`, `qThreadExtraInfo`, `qC`).

---

## 10. Quick checklist for a new driver

- [ ] `const struct rtos_type mykit_rtos` defined with at minimum
  `name`, `detect_rtos`, `create`, `update_threads`,
  `get_thread_reg_list`, `get_symbol_list_to_lookup`.
- [ ] `extern` declaration added to `src/rtos/rtos.h`.
- [ ] Entry added to `rtos_types[]` in `src/rtos/rtos.c`, alphabetical,
  before `hwthread_rtos`.
- [ ] Source file added to `src/rtos/Makefile.am`.
- [ ] Symbol manifest returned by `get_symbol_list_to_lookup` is
  NULL-terminated and allocated with `calloc`.
- [ ] `update_threads` calls `rtos_free_threadlist` before rebuilding.
- [ ] `current_thread` set to a non-zero, unique TCB-derived ID.
- [ ] Stacking table matches the kernel's context-save sequence
  exactly; SP offset is `-2`, unstacked regs are `-1`.
- [ ] Bound checks on list traversal so a corrupted list cannot loop
  forever.
- [ ] `LOG_ERROR` / `LOG_WARNING` on every recoverable failure path so
  the user can diagnose without a debugger on OpenOCD itself.
