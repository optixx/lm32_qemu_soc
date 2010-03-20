/*
 *  LatticeMico32 virtual CPU header.
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

#ifndef CPU_LM32_H
#define CPU_LM32_H

#define TARGET_LONG_BITS 32

#define CPUState struct CPULM32State

#include "cpu-defs.h"
struct CPULM32State;

#define TARGET_HAS_ICE 1

#define ELF_MACHINE EM_LATTICEMICO32

#define NB_MMU_MODES 1
#define TARGET_PAGE_BITS 12
static inline int cpu_mmu_index(CPUState *env)
{
    return 0;
}

/* Exceptions indices */
#define EXCP_RESET           0
#define EXCP_BREAKPOINT      1
#define EXCP_INSN_BUS_ERROR  2
#define EXCP_WATCHPOINT      3
#define EXCP_DATA_BUS_ERROR  4
#define EXCP_DIVIDE_BY_ZERO  5
#define EXCP_IRQ             6
#define EXCP_SYSTEMCALL      7

/* Registers */
#define R_R0     0
#define R_R1     1
#define R_R2     2
#define R_R3     3
#define R_R4     4
#define R_R5     5
#define R_R6     6
#define R_R7     7
#define R_R8     8
#define R_R9     9
#define R_R10    10
#define R_R11    11
#define R_R12    12
#define R_R13    13
#define R_R14    14
#define R_R15    15
#define R_R16    16
#define R_R17    17
#define R_R18    18
#define R_R19    19
#define R_R20    20
#define R_R21    21
#define R_R22    22
#define R_R23    23
#define R_R24    24
#define R_R25    25
#define R_R26    26
#define R_R27    27
#define R_R28    28
#define R_R29    29
#define R_R30    30
#define R_R31    31
#define R_NUM    32

/* Register aliases */
#define R_GP     26
#define R_FP     27
#define R_SP     28
#define R_RA     29
#define R_EA     30
#define R_BA     31

/* Special registers */
#define SR_PC    0
#define SR_IE    1
#define SR_IM    2
#define SR_IP    3
#define SR_ICC   4
#define SR_DCC   5
#define SR_CC    6
#define SR_CFG   7
#define SR_EBA   8
#define SR_NUM   9

/* IE flags */
#define IE_IE   (1<<0)
#define IE_EIE  (1<<1)
#define IE_BIE  (1<<2)

/* DC flags */
#define DC_SS   (1<<0)
#define DC_RE   (1<<1)
#define DC_C0   (1<<2)
#define DC_C1   (1<<3)
#define DC_C2   (1<<4)
#define DC_C3   (1<<5)

/* CFG mask */
#define CFG_M_MASK   0x00000001
#define CFG_D_MASK   0x00000002
#define CFG_S_MASK   0x00000004
#define CFG_U_MASK   0x00000008
#define CFG_X_MASK   0x00000010
#define CFG_CC_MASK  0x00000020
#define CFG_IC_MASK  0x00000040
#define CFG_DC_MASK  0x00000080
#define CFG_G_MASK   0x00000100
#define CFG_H_MASK   0x00000200
#define CFG_R_MASK   0x00000400
#define CFG_J_MASK   0x00000800
#define CFG_INT_MASK 0x0003f000
#define CFG_BP_MASK  0x003c0000
#define CFG_WP_MASK  0x03c00800
#define CFG_REV_MASK 0xfc000000

/* CSRs */
#define CSR_IE    0x00
#define CSR_IM    0x01
#define CSR_IP    0x02
#define CSR_ICC   0x03
#define CSR_DCC   0x04
#define CSR_CC    0x05
#define CSR_CFG   0x06
#define CSR_EBA   0x07
#define CSR_DC    0x08
#define CSR_DEBA  0x09
#define CSR_JTX   0x0e
#define CSR_JRX   0x0f
#define CSR_BP0   0x10
#define CSR_BP1   0x11
#define CSR_BP2   0x12
#define CSR_BP3   0x13
#define CSR_WP0   0x18
#define CSR_WP1   0x19
#define CSR_WP2   0x1a
#define CSR_WP3   0x1b

typedef struct CPULM32State {
    /* general registers */
    uint32_t regs[R_NUM];

    /* special registers */
    uint32_t pc;        /* program counter */
    uint32_t ie;        /* interrupt enable */
    uint32_t icc;       /* instruction cache control */
    uint32_t dcc;       /* data cache control */
    uint32_t cc;        /* cycle counter */
    uint32_t cfg;       /* configuration */
    uint32_t eba;       /* exception base address */

    /* debug registers */
    uint32_t dc;        /* debug control */
    uint32_t deba;      /* debug exception base address */
    uint32_t jtx;       /* JTAG UART transmit */
    uint32_t jrx;       /* JTAG UART receive */
    uint32_t bp[4];     /* breakpoint addresses */
    uint32_t wp[4];     /* watchpoint addresses */

    /* interrupt controller handle for callbacks */
    struct lm32_pic *pic_handle;

    CPU_COMMON
} CPULM32State;

CPUState *cpu_lm32_init(const char *cpu_model);
int cpu_lm32_exec(CPUState *s);
void cpu_lm32_close(CPUState *s);
void do_interrupt(CPUState *env);
/* you can call this signal handler from your SIGBUS and SIGSEGV
   signal handlers to inform the virtual CPU of exceptions. non zero
   is returned if the signal was handled by the virtual CPU.  */
int cpu_lm32_signal_handler(int host_signum, void *pinfo,
                          void *puc);

#define cpu_init cpu_lm32_init
#define cpu_exec cpu_lm32_exec
#define cpu_gen_code cpu_lm32_gen_code
#define cpu_signal_handler cpu_lm32_signal_handler

#define CPU_SAVE_VERSION 1

int cpu_lm32_handle_mmu_fault(CPUState *env, target_ulong address, int rw,
                            int mmu_idx, int is_softmmu);
#define cpu_handle_mmu_fault cpu_lm32_handle_mmu_fault

#if defined(CONFIG_USER_ONLY)
static inline void cpu_clone_regs(CPUState *env, target_ulong newsp)
{
    if (newsp)
        env->regs[R_SP] = newsp;
    env->regs[3] = 0;
}
#endif

static inline void cpu_set_tls(CPUState *env, target_ulong newtls)
{
}

static inline int cpu_interrupts_enabled(CPUState *env)
{
    return env->ie & IE_IE;
}

#include "cpu-all.h"
#include "exec-all.h"

static inline void cpu_pc_from_tb(CPUState *env, TranslationBlock *tb)
{
    env->pc = tb->pc;
}

static inline target_ulong cpu_get_pc(CPUState *env)
{
    return env->pc;
}

static inline void cpu_get_tb_cpu_state(CPUState *env, target_ulong *pc,
                                        target_ulong *cs_base, int *flags)
{
    *pc = env->pc;
    *cs_base = 0;
    *flags = 0;
}

void do_unassigned_access(target_phys_addr_t addr, int is_write, int is_exec,
                          int is_asi, int size);
#endif
