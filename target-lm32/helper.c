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
