/*
 *  LatticeMico32 helper routines.
 *
 *  Copyright (c) 2010 Michael Walle <michael@walle.cc>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <string.h>
#include <assert.h>

#include "config.h"
#include "cpu.h"
#include "exec-all.h"
#include "host-utils.h"

#define D(x)

int cpu_lm32_handle_mmu_fault (CPUState *env, target_ulong address, int rw,
                               int mmu_idx, int is_softmmu)
{
    int r;
    int prot;

    address &= TARGET_PAGE_MASK;
    prot = PAGE_BITS;
    r = tlb_set_page(env, address, address, prot, mmu_idx, is_softmmu);
    return r;
}

target_phys_addr_t cpu_get_phys_page_debug(CPUState * env, target_ulong addr)
{
    return addr & TARGET_PAGE_MASK;
}

void do_interrupt(CPUState *env)
{
    qemu_log_mask(CPU_LOG_INT,
            "exception at pc=%x type=%x\n", env->pc, env->exception_index);

    switch (env->exception_index) {
        case EXCP_INSN_BUS_ERROR:
        case EXCP_DATA_BUS_ERROR:
        case EXCP_DIVIDE_BY_ZERO:
        case EXCP_IRQ:
        case EXCP_SYSTEMCALL:
            /* non-debug exceptions */
            env->regs[R_EA] = env->pc;
            env->ie |= (env->ie & IE_IE) ? IE_EIE : 0;
            env->ie &= ~IE_IE;
            if (env->dc & DC_RE) {
                env->pc = env->deba + (env->exception_index * 32);
            } else {
                env->pc = env->eba + (env->exception_index * 32);
            }
            log_cpu_state_mask(CPU_LOG_INT, env, 0);
            break;
        case EXCP_BREAKPOINT:
        case EXCP_WATCHPOINT:
            /* debug exceptions */
            env->regs[R_BA] = env->pc;
            env->ie |= (env->ie & IE_IE) ? IE_BIE : 0;
            env->ie &= ~IE_IE;
            if (env->dc & DC_RE) {
                env->pc = env->deba + (env->exception_index * 32);
            } else {
                env->pc = env->eba + (env->exception_index * 32);
            }
            log_cpu_state_mask(CPU_LOG_INT, env, 0);
            break;
        default:
            cpu_abort(env, "unhandled exception type=%d\n",
                      env->exception_index);
            break;
    }
}

typedef struct {
    const char *name;
    uint32_t revision;
    uint32_t num_interrupts;
    uint32_t num_breakpoints;
    uint32_t num_watchpoints;
    uint32_t features;
} lm32_def_t;

static lm32_def_t lm32_defs[] = {
    {
        .name = "lm32-basic",
        .revision = 3,
        .num_interrupts = 32,
        .num_breakpoints = 4,
        .num_watchpoints = 4,
        .features = (  LM32_FEATURE_SHIFT
                     | LM32_FEATURE_SIGN_EXTEND
                     | LM32_FEATURE_CYCLE_COUNT),
    },
    {
        .name = "lm32-standard",
        .revision = 3,
        .num_interrupts = 32,
        .num_breakpoints = 4,
        .num_watchpoints = 4,
        .features = (  LM32_FEATURE_MULTIPLY
                     | LM32_FEATURE_DIVIDE
                     | LM32_FEATURE_SHIFT
                     | LM32_FEATURE_SIGN_EXTEND
                     | LM32_FEATURE_I_CACHE
                     | LM32_FEATURE_CYCLE_COUNT),
    },
    {
        .name = "lm32-full",
        .revision = 3,
        .num_interrupts = 32,
        .num_breakpoints = 4,
        .num_watchpoints = 4,
        .features = (  LM32_FEATURE_MULTIPLY
                     | LM32_FEATURE_DIVIDE
                     | LM32_FEATURE_SHIFT
                     | LM32_FEATURE_SIGN_EXTEND
                     | LM32_FEATURE_I_CACHE
                     | LM32_FEATURE_D_CACHE
                     | LM32_FEATURE_CYCLE_COUNT),
    }
};

void cpu_lm32_list(FILE *f, int (*cpu_fprintf)(FILE *f, const char *fmt, ...))
{
    int i;

    (*cpu_fprintf)(f, "Available CPUs:\n");
    for (i = 0; i < ARRAY_SIZE(lm32_defs); i++) {
        (*cpu_fprintf)(f, "  %s\n", lm32_defs[i].name);
    }
}

static const lm32_def_t *cpu_lm32_find_by_name(const char *name)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(lm32_defs); i++)
    {
        if (strcasecmp(name, lm32_defs[i].name) == 0) {
            return &lm32_defs[i];
        }
    }

    return NULL;
}

static uint32_t cfg_by_def(const lm32_def_t *def)
{
    uint32_t cfg = 0;

    if (def->features & LM32_FEATURE_MULTIPLY)
        cfg |= CFG_M;

    if (def->features & LM32_FEATURE_DIVIDE)
        cfg |= CFG_D;

    if (def->features & LM32_FEATURE_SHIFT)
        cfg |= CFG_S;

    if (def->features & LM32_FEATURE_SIGN_EXTEND)
        cfg |= CFG_X;

    if (def->features & LM32_FEATURE_I_CACHE)
        cfg |= CFG_IC;

    if (def->features & LM32_FEATURE_D_CACHE)
        cfg |= CFG_DC;

    if (def->features & LM32_FEATURE_CYCLE_COUNT)
        cfg |= CFG_CC;

    cfg |= (def->num_interrupts << CFG_INT_SHIFT);
    cfg |= (def->num_breakpoints << CFG_BP_SHIFT);
    cfg |= (def->num_watchpoints << CFG_WP_SHIFT);
    cfg |= (def->revision << CFG_REV_SHIFT);

    return cfg;
}

CPUState *cpu_lm32_init(const char *cpu_model)
{
    CPUState *env;
    const lm32_def_t *def;
    static int tcg_initialized = 0;

    def = cpu_lm32_find_by_name(cpu_model);
    if (!def)
        return NULL;

    env = qemu_mallocz(sizeof(CPUState));

    env->features = def->features;
    env->cfg = cfg_by_def(def);

    cpu_exec_init(env);
    cpu_reset(env);

    if (!tcg_initialized) {
        tcg_initialized = 1;
        lm32_translate_init();
    }

    return env;
}

void cpu_reset(CPUState *env)
{
    if (qemu_loglevel_mask(CPU_LOG_RESET)) {
        qemu_log("CPU Reset (CPU %d)\n", env->cpu_index);
        log_cpu_state(env, 0);
    }

    tlb_flush(env, 1);

    /* FIXME reset breakpoints and watchpoints */
}

