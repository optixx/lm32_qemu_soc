/*
 *  LatticeMico32 main translation routines.
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

#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <assert.h>

#include "cpu.h"
#include "exec-all.h"
#include "disas.h"
#include "helper.h"
#include "tcg-op.h"
#include "lm32-decode.h"
#include "qemu-common.h"

#define GEN_HELPER 1
#include "helper.h"

#define DISAS_LM32 1
#if DISAS_LM32
#  define LOG_DIS(...) qemu_log_mask(CPU_LOG_TB_IN_ASM, ## __VA_ARGS__)
#else
#  define LOG_DIS(...) do { } while (0)
#endif

#define D(x)

#define EXTRACT_FIELD(src, start, end) \
            (((src) >> start) & ((1 << (end - start + 1)) - 1))

#define MEM_INDEX 0

static TCGv_ptr cpu_env;
static TCGv cpu_R[R_NUM];
static TCGv cpu_pc;
static TCGv cpu_ie;
static TCGv cpu_im;
static TCGv cpu_ip;
static TCGv cpu_icc;
static TCGv cpu_dcc;
static TCGv cpu_cc;
static TCGv cpu_cfg;
static TCGv cpu_eba;
static TCGv cpu_dc;
static TCGv cpu_deba;
static TCGv cpu_jrx;
static TCGv cpu_jtx;
static TCGv cpu_bp[4];
static TCGv cpu_wp[4];

#include "gen-icount.h"

enum {
    OP_FMT_RI,
    OP_FMT_RR,
    OP_FMT_CR,
    OP_FMT_I
};

/* This is the state at translation time.  */
typedef struct DisasContext {
    CPUState *env;
    target_ulong pc;

    /* Decoder.  */
    int format;
    uint32_t ir;
    uint8_t opcode;
    uint8_t r0, r1, r2, csr;
    uint16_t imm5;
    uint16_t imm16;
    uint32_t imm26;

    unsigned int delayed_branch;
    unsigned int tb_flags, synced_flags; /* tb dependent flags.  */
    int is_jmp;

#define JMP_NOJMP    0
#define JMP_DIRECT   1
#define JMP_INDIRECT 2
    unsigned int jmp;
    uint32_t jmp_pc;

    int nr_nops;
    struct TranslationBlock *tb;
    int singlestep_enabled;
} DisasContext;

static const char *regnames[] =
{
    "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7",
    "r8", "r9", "r10", "r11", "r12", "r13", "r14", "r15",
    "r16", "r17", "r18", "r19", "r20", "r21", "r22", "r23",
    "r24", "r25", "r26/gp", "r27/fp", "r28/sp", "r29/ra",
    "r30/ea", "r31/ba",
};

static inline int zero_extend(unsigned int val, int width)
{
    return val & ((1 << width) - 1);
}

static inline int sign_extend(unsigned int val, int width)
{
    int sval;

    /* LSL.  */
    val <<= 32 - width;
    sval = val;
    /* ASR.  */
    sval >>= 32 - width;

    return sval;
}

static inline void t_gen_raise_exception(DisasContext *dc, uint32_t index)
{
    TCGv_i32 tmp = tcg_const_i32(index);

    tcg_gen_movi_tl(cpu_pc, dc->pc);
    gen_helper_raise_exception(tmp);
    tcg_temp_free_i32(tmp);
}

/* FIXME have a look at microblaze target */
static void gen_goto_tb(DisasContext *dc, int n, target_ulong dest)
{
    tcg_gen_movi_tl(cpu_pc, dest);
    tcg_gen_exit_tb(0);
}

static void dec_add(DisasContext *dc)
{
    if (dc->format == OP_FMT_RI) {
        LOG_DIS("addi r%d, r%d, %d\n", dc->r1, dc->r0,
                sign_extend(dc->imm16, 16));
    } else {
        LOG_DIS("add r%d, r%d, r%d\n", dc->r2, dc->r0, dc->r1);
    }

    if (dc->format == OP_FMT_RI) {
        tcg_gen_addi_tl(cpu_R[dc->r1], cpu_R[dc->r0], sign_extend(dc->imm16, 16));
    } else {
        tcg_gen_add_tl(cpu_R[dc->r2], cpu_R[dc->r0], cpu_R[dc->r1]);
    }
}

static void dec_and(DisasContext *dc)
{
    if (dc->format == OP_FMT_RI) {
        LOG_DIS("andi r%d, r%d, %d\n", dc->r1, dc->r0,
                zero_extend(dc->imm16, 16));
    } else {
        LOG_DIS("and r%d, r%d, r%d\n", dc->r2, dc->r0, dc->r1);
    }

    if (dc->format == OP_FMT_RI) {
        tcg_gen_andi_tl(cpu_R[dc->r1], cpu_R[dc->r0], zero_extend(dc->imm16, 16));
    } else  {
        tcg_gen_and_tl(cpu_R[dc->r2], cpu_R[dc->r0], cpu_R[dc->r1]);
    }
}

static void dec_andhi(DisasContext *dc)
{
    LOG_DIS("andhi r%d, r%d, %d\n", dc->r2, dc->r0, dc->imm16);

    tcg_gen_andi_tl(cpu_R[dc->r1], cpu_R[dc->r0], (dc->imm16 << 16));
}

static void dec_b(DisasContext *dc)
{
    LOG_DIS("b r%d\n", dc->r0);

    /* restore IE.IE in case of an eret */
    if (dc->r0 == R_EA) {
        TCGv t0 = tcg_temp_new();
        int l1 = gen_new_label();
        tcg_gen_andi_tl(t0, cpu_ie, IE_EIE);
        tcg_gen_ori_tl(cpu_ie, cpu_ie, IE_IE);
        tcg_gen_brcondi_tl(TCG_COND_EQ, t0, IE_EIE, l1);
        tcg_gen_andi_tl(cpu_ie, cpu_ie, ~IE_IE);
        gen_set_label(l1);
        tcg_temp_free(t0);
    } else if (dc->r0 == R_BA) {
        TCGv t0 = tcg_temp_new();
        int l1 = gen_new_label();
        tcg_gen_andi_tl(t0, cpu_ie, IE_BIE);
        tcg_gen_ori_tl(cpu_ie, cpu_ie, IE_IE);
        tcg_gen_brcondi_tl(TCG_COND_EQ, t0, IE_BIE, l1);
        tcg_gen_andi_tl(cpu_ie, cpu_ie, ~IE_IE);
        gen_set_label(l1);
        tcg_temp_free(t0);
    }
    tcg_gen_mov_tl(cpu_pc, cpu_R[dc->r0]);

    dc->is_jmp = DISAS_JUMP;
}

static void dec_bi(DisasContext *dc)
{
    LOG_DIS("bi %d\n", sign_extend(dc->imm26 << 2, 26));

    tcg_gen_movi_tl(cpu_pc, dc->pc + (sign_extend(dc->imm26 << 2, 26)));

    dc->is_jmp = DISAS_JUMP;
}

static inline void gen_cond_branch(DisasContext *dc, int cond)
{
    int l1,l2;

    l1 = gen_new_label();
    l2 = gen_new_label();
    tcg_gen_brcond_tl(cond, cpu_R[dc->r0], cpu_R[dc->r1], l1);
    tcg_gen_movi_tl(cpu_pc, dc->pc + 4);
    tcg_gen_br(l2);
    gen_set_label(l1);
    tcg_gen_movi_tl(cpu_pc, dc->pc + (sign_extend(dc->imm16 << 2, 16)));
    gen_set_label(l2);

    dc->is_jmp = DISAS_JUMP;
}

static void dec_be(DisasContext *dc)
{
    LOG_DIS("be r%d, r%d, %d\n", dc->r1, dc->r0, sign_extend(dc->imm16, 16));

    gen_cond_branch(dc, TCG_COND_EQ);
}

static void dec_bg(DisasContext *dc)
{
    LOG_DIS("bg r%d, r%d, %d\n", dc->r1, dc->r0, sign_extend(dc->imm16, 16));

    gen_cond_branch(dc, TCG_COND_GT);
}

static void dec_bge(DisasContext *dc)
{
    LOG_DIS("bge r%d, r%d, %d\n", dc->r1, dc->r0, sign_extend(dc->imm16, 16));

    gen_cond_branch(dc, TCG_COND_GE);
}

static void dec_bgeu(DisasContext *dc)
{
    LOG_DIS("bg r%d, r%d, %d\n", dc->r1, dc->r0, sign_extend(dc->imm16, 16));

    gen_cond_branch(dc, TCG_COND_GEU);
}

static void dec_bgu(DisasContext *dc)
{
    LOG_DIS("bgu r%d, r%d, %d\n", dc->r1, dc->r0, sign_extend(dc->imm16, 16));

    gen_cond_branch(dc, TCG_COND_GTU);
}

static void dec_bne(DisasContext *dc)
{
    LOG_DIS("bne r%d, r%d, %d\n", dc->r1, dc->r0, sign_extend(dc->imm16, 16));

    gen_cond_branch(dc, TCG_COND_NE);
}

static void dec_call(DisasContext *dc)
{
    LOG_DIS("call r%d\n", dc->r0);

    tcg_gen_movi_tl(cpu_R[R_RA], dc->pc + 4);
    tcg_gen_mov_tl(cpu_pc, cpu_R[dc->r0]);

    dc->is_jmp = DISAS_JUMP;
}

static void dec_calli(DisasContext *dc)
{
    LOG_DIS("calli %d\n", sign_extend(dc->imm26, 26));

    tcg_gen_movi_tl(cpu_R[R_RA], dc->pc + 4);
    tcg_gen_movi_tl(cpu_pc, dc->pc + (sign_extend(dc->imm26 << 2, 26)));

    dc->is_jmp = DISAS_JUMP;
}

static inline void gen_compare(DisasContext *dc, int cond)
{
    int rX = (dc->format == OP_FMT_RR) ? dc->r2 : dc->r1;
    int rY = (dc->format == OP_FMT_RR) ? dc->r0 : dc->r0;
    int rZ = (dc->format == OP_FMT_RR) ? dc->r1 : -1;

    int l1,l2;

    l1 = gen_new_label();
    l2 = gen_new_label();

    if (dc->format == OP_FMT_RI) {
        tcg_gen_brcondi_tl(cond, cpu_R[rY], sign_extend(dc->imm16, 16), l1);
    } else {
        tcg_gen_brcond_tl(cond, cpu_R[rY], cpu_R[rZ], l1);
    }
    tcg_gen_movi_tl(cpu_R[rX], 0);
    tcg_gen_br(l2);
    gen_set_label(l1);
    tcg_gen_movi_tl(cpu_R[rX], 1);
    gen_set_label(l2);
}

static void dec_cmpe(DisasContext *dc)
{
    if (dc->format == OP_FMT_RI) {
        LOG_DIS("cmpei r%d, r%d, %d\n", dc->r0, dc->r1,
                sign_extend(dc->imm16, 16));
    } else {
        LOG_DIS("cmpe r%d, r%d, r%d\n", dc->r2, dc->r0, dc->r1);
    }

    gen_compare(dc, TCG_COND_EQ);
}

static void dec_cmpg(DisasContext *dc)
{
    if (dc->format == OP_FMT_RI) {
        LOG_DIS("cmpgi r%d, r%d, %d\n", dc->r0, dc->r1,
                sign_extend(dc->imm16, 16));
    } else {
        LOG_DIS("cmpg r%d, r%d, r%d\n", dc->r2, dc->r0, dc->r1);
    }

    gen_compare(dc, TCG_COND_GT);
}

static void dec_cmpge(DisasContext *dc)
{
    if (dc->format == OP_FMT_RI) {
        LOG_DIS("cmpgei r%d, r%d, %d\n", dc->r0, dc->r1,
                sign_extend(dc->imm16, 16));
    } else {
        LOG_DIS("cmpge r%d, r%d, r%d\n", dc->r2, dc->r0, dc->r1);
    }

    gen_compare(dc, TCG_COND_GE);
}

static void dec_cmpgeu(DisasContext *dc)
{
    if (dc->format == OP_FMT_RI) {
        LOG_DIS("cmpgeui r%d, r%d, %d\n", dc->r0, dc->r1,
                sign_extend(dc->imm16, 16));
    } else {
        LOG_DIS("cmpgeu r%d, r%d, r%d\n", dc->r2, dc->r0, dc->r1);
    }

    gen_compare(dc, TCG_COND_GEU);
}

static void dec_cmpgu(DisasContext *dc)
{
    if (dc->format == OP_FMT_RI) {
        LOG_DIS("cmpgui r%d, r%d, %d\n", dc->r0, dc->r1,
                sign_extend(dc->imm16, 16));
    } else {
        LOG_DIS("cmpgu r%d, r%d, r%d\n", dc->r2, dc->r0, dc->r1);
    }

    gen_compare(dc, TCG_COND_GTU);
}

static void dec_cmpne(DisasContext *dc)
{
    if (dc->format == OP_FMT_RI) {
        LOG_DIS("cmpnei r%d, r%d, %d\n", dc->r0, dc->r1,
                sign_extend(dc->imm16, 16));
    } else {
        LOG_DIS("cmpne r%d, r%d, r%d\n", dc->r2, dc->r0, dc->r1);
    }

    gen_compare(dc, TCG_COND_NE);
}

static void dec_divu(DisasContext *dc)
{
    LOG_DIS("divu r%d, r%d, r%d\n", dc->r2, dc->r0, dc->r1);

    int l1 = gen_new_label();

    tcg_gen_brcondi_tl(TCG_COND_NE, cpu_R[dc->r1], 0, l1);
    t_gen_raise_exception(dc, EXCP_DIVIDE_BY_ZERO);
    gen_set_label(l1);
    tcg_gen_div_tl(cpu_R[dc->r2], cpu_R[dc->r0], cpu_R[dc->r1]);
}

static void dec_lb(DisasContext *dc)
{
    LOG_DIS("lb r%d, (r%d+%d)\n", dc->r1, dc->r0, dc->imm16);

    TCGv t0 = tcg_temp_new();
    tcg_gen_addi_tl(t0, cpu_R[dc->r0], sign_extend(dc->imm16, 16));
    tcg_gen_qemu_ld8s(cpu_R[dc->r1], t0, MEM_INDEX);
    tcg_temp_free(t0);
}

static void dec_lbu(DisasContext *dc)
{
    LOG_DIS("lbu r%d, (r%d+%d)\n", dc->r1, dc->r0, dc->imm16);

    TCGv t0 = tcg_temp_new();
    tcg_gen_addi_tl(t0, cpu_R[dc->r0], sign_extend(dc->imm16, 16));
    tcg_gen_qemu_ld8u(cpu_R[dc->r1], t0, MEM_INDEX);
    tcg_temp_free(t0);
}

static void dec_lh(DisasContext *dc)
{
    LOG_DIS("lh r%d, (r%d+%d)\n", dc->r1, dc->r0, dc->imm16);

    TCGv t0 = tcg_temp_new();
    tcg_gen_addi_tl(t0, cpu_R[dc->r0], sign_extend(dc->imm16, 16));
    tcg_gen_qemu_ld16s(cpu_R[dc->r1], t0, MEM_INDEX);
    tcg_temp_free(t0);
}

static void dec_lhu(DisasContext *dc)
{
    LOG_DIS("lhu r%d, (r%d+%d)\n", dc->r1, dc->r0, dc->imm16);

    TCGv t0 = tcg_temp_new();
    tcg_gen_addi_tl(t0, cpu_R[dc->r0], sign_extend(dc->imm16, 16));
    tcg_gen_qemu_ld16u(cpu_R[dc->r1], t0, MEM_INDEX);
    tcg_temp_free(t0);
}

static void dec_lw(DisasContext *dc)
{
    LOG_DIS("lw r%d, (r%d+%d)\n", dc->r1, dc->r0, dc->imm16);

    TCGv t0 = tcg_temp_new();
    tcg_gen_addi_tl(t0, cpu_R[dc->r0], sign_extend(dc->imm16, 16));
    tcg_gen_qemu_ld32s(cpu_R[dc->r1], t0, MEM_INDEX);
    tcg_temp_free(t0);
}

static void dec_modu(DisasContext *dc)
{
    LOG_DIS("modu r%d, r%d, %d\n", dc->r2, dc->r0, dc->r1);

    tcg_gen_remu_tl(cpu_R[dc->r2], cpu_R[dc->r0], cpu_R[dc->r1]);
}

static void dec_mul(DisasContext *dc)
{
    if (dc->format == OP_FMT_RI) {
        LOG_DIS("muli r%d, r%d, %d\n", dc->r0, dc->r1,
                sign_extend(dc->imm16, 16));
    } else {
        LOG_DIS("mul r%d, r%d, r%d\n", dc->r2, dc->r0, dc->r1);
    }

    if (dc->format == OP_FMT_RI) {
        tcg_gen_muli_tl(cpu_R[dc->r1], cpu_R[dc->r0], sign_extend(dc->imm16, 16));
    } else {
        tcg_gen_mul_tl(cpu_R[dc->r2], cpu_R[dc->r0], cpu_R[dc->r1]);
    }
}

static void dec_nor(DisasContext *dc)
{
    if (dc->format == OP_FMT_RI) {
        LOG_DIS("nori r%d, r%d, %d\n", dc->r0, dc->r1,
                zero_extend(dc->imm16, 16));
    } else {
        LOG_DIS("nor r%d, r%d, r%d\n", dc->r2, dc->r0, dc->r1);
    }

    if (dc->format == OP_FMT_RI) {
        TCGv t0 = tcg_temp_new();
        tcg_gen_movi_tl(t0, zero_extend(dc->imm16, 16));
        tcg_gen_nor_tl(cpu_R[dc->r1], cpu_R[dc->r0], t0);
        tcg_temp_free(t0);
    } else {
        tcg_gen_nor_tl(cpu_R[dc->r2], cpu_R[dc->r0], cpu_R[dc->r1]);
    }
}

static void dec_or(DisasContext *dc)
{
    if (dc->format == OP_FMT_RI) {
        LOG_DIS("ori r%d, r%d, %d\n", dc->r1, dc->r0,
                zero_extend(dc->imm16, 16));
    } else {
        LOG_DIS("or r%d, r%d, r%d\n", dc->r2, dc->r0, dc->r1);
    }

    if (dc->format == OP_FMT_RI) {
        tcg_gen_ori_tl(cpu_R[dc->r1], cpu_R[dc->r0],
                zero_extend(dc->imm16, 16));
    } else {
        tcg_gen_or_tl(cpu_R[dc->r2], cpu_R[dc->r0], cpu_R[dc->r1]);
    }
}

static void dec_orhi(DisasContext *dc)
{
    LOG_DIS("orhi r%d, r%d, %d\n", dc->r1, dc->r0, dc->imm16);

    tcg_gen_ori_tl(cpu_R[dc->r1], cpu_R[dc->r0], (dc->imm16 << 16));
}

static void dec_raise(DisasContext *dc)
{
    LOG_DIS("break\n");

    t_gen_raise_exception(dc, EXCP_SYSTEMCALL);
}

static void dec_rcsr(DisasContext *dc)
{
    LOG_DIS("rcsr r%d, %d\n", dc->r2, dc->csr);

    switch(dc->csr)
    {
        case CSR_IE:
            tcg_gen_mov_tl(cpu_R[dc->r2], cpu_ie);
            break;
        case CSR_IM:
            tcg_gen_mov_tl(cpu_R[dc->r2], cpu_im);
            break;
        case CSR_IP:
            tcg_gen_mov_tl(cpu_R[dc->r2], cpu_ip);
            break;
        case CSR_CC:
            tcg_gen_mov_tl(cpu_R[dc->r2], cpu_cc);
            break;
        case CSR_CFG:
            tcg_gen_mov_tl(cpu_R[dc->r2], cpu_cfg);
            break;
        case CSR_EBA:
            tcg_gen_mov_tl(cpu_R[dc->r2], cpu_eba);
            break;
        case CSR_DC:
            tcg_gen_mov_tl(cpu_R[dc->r2], cpu_dc);
            break;
        case CSR_DEBA:
            tcg_gen_mov_tl(cpu_R[dc->r2], cpu_deba);
            break;
        case CSR_JTX:
            tcg_gen_mov_tl(cpu_R[dc->r2], cpu_jtx);
            break;
        case CSR_JRX:
            tcg_gen_mov_tl(cpu_R[dc->r2], cpu_jrx);
            break;
        case CSR_ICC:
        case CSR_DCC:
        case CSR_BP0:
        case CSR_BP1:
        case CSR_BP2:
        case CSR_BP3:
        case CSR_WP0:
        case CSR_WP1:
        case CSR_WP2:
        case CSR_WP3:
            cpu_abort(dc->env, "invalid read access csr=%x\n", dc->csr);
            break;
        default:
            cpu_abort(dc->env, "read_csr: unknown csr=%x\n", dc->csr);
            break;
    }
}

static void dec_sb(DisasContext *dc)
{
    LOG_DIS("sb (r%d+%d), r%d\n", dc->r0, dc->imm16, dc->r1);

    TCGv t0 = tcg_temp_new();
    tcg_gen_addi_tl(t0, cpu_R[dc->r0], sign_extend(dc->imm16, 16));
    tcg_gen_qemu_st8(cpu_R[dc->r1], t0, MEM_INDEX);
    tcg_temp_free(t0);
}

static void dec_sextb(DisasContext *dc)
{
    LOG_DIS("sextb r%d, r%d\n", dc->r2, dc->r0);

    tcg_gen_ext8s_tl(cpu_R[dc->r2], cpu_R[dc->r0]);
}

static void dec_sexth(DisasContext *dc)
{
    LOG_DIS("sexth r%d, r%d\n", dc->r2, dc->r0);

    tcg_gen_ext16s_tl(cpu_R[dc->r2], cpu_R[dc->r0]);
}

static void dec_sh(DisasContext *dc)
{
    LOG_DIS("sh (r%d+%d), r%d\n", dc->r0, dc->imm16, dc->r1);

    TCGv t0 = tcg_temp_new();
    tcg_gen_addi_tl(t0, cpu_R[dc->r0], sign_extend(dc->imm16, 16));
    tcg_gen_qemu_st16(cpu_R[dc->r1], t0, MEM_INDEX);
    tcg_temp_free(t0);
}

static void dec_sl(DisasContext *dc)
{
    if (dc->format == OP_FMT_RI) {
        LOG_DIS("sli r%d, r%d, %d\n", dc->r1, dc->r0, dc->imm5);
    } else {
        LOG_DIS("sl r%d, r%d, r%d\n", dc->r2, dc->r0, dc->r1);
    }

    if (dc->format == OP_FMT_RI) {
        tcg_gen_shli_tl(cpu_R[dc->r1], cpu_R[dc->r0], dc->imm5);
    } else {
        TCGv t0 = tcg_temp_new();
        tcg_gen_andi_tl(t0, cpu_R[dc->r1], 0x1f);
        tcg_gen_shl_tl(cpu_R[dc->r2], cpu_R[dc->r0], t0);
        tcg_temp_free(t0);
    }
}

static void dec_sr(DisasContext *dc)
{
    if (dc->format == OP_FMT_RI) {
        LOG_DIS("sri r%d, r%d, %d\n", dc->r1, dc->r0, dc->imm5);
    } else {
        LOG_DIS("sr r%d, r%d, r%d\n", dc->r2, dc->r0, dc->r1);
    }

    if (dc->format == OP_FMT_RI) {
        tcg_gen_sari_tl(cpu_R[dc->r1], cpu_R[dc->r0], dc->imm5);
    } else {
        TCGv t0 = tcg_temp_new();
        tcg_gen_andi_tl(t0, cpu_R[dc->r1], 0x1f);
        tcg_gen_sar_tl(cpu_R[dc->r2], cpu_R[dc->r0], t0);
        tcg_temp_free(t0);
    }
}

static void dec_sru(DisasContext *dc)
{
    if (dc->format == OP_FMT_RI) {
        LOG_DIS("srui r%d, r%d, %d\n", dc->r1, dc->r0, dc->imm5);
    } else {
        LOG_DIS("sru r%d, r%d, r%d\n", dc->r2, dc->r0, dc->r1);
    }

    if (dc->format == OP_FMT_RI) {
        tcg_gen_shri_tl(cpu_R[dc->r1], cpu_R[dc->r0], dc->imm5);
    } else {
        TCGv t0 = tcg_temp_new();
        tcg_gen_andi_tl(t0, cpu_R[dc->r1], 0x1f);
        tcg_gen_shr_tl(cpu_R[dc->r2], cpu_R[dc->r0], t0);
        tcg_temp_free(t0);
    }
}

static void dec_sub(DisasContext *dc)
{
    LOG_DIS("sub r%d, r%d, r%d\n", dc->r2, dc->r0, dc->r1);

    tcg_gen_sub_tl(cpu_R[dc->r2], cpu_R[dc->r0], cpu_R[dc->r1]);
}

static void dec_sw(DisasContext *dc)
{
    LOG_DIS("sw (r%d+%d), r%d\n", dc->r0, dc->imm16, dc->r1);

    TCGv t0 = tcg_temp_new();
    tcg_gen_addi_tl(t0, cpu_R[dc->r0], sign_extend(dc->imm16, 16));
    tcg_gen_qemu_st32(cpu_R[dc->r1], t0, MEM_INDEX);
    tcg_temp_free(t0);
}

static void dec_user(DisasContext *dc)
{
    LOG_DIS("user");

    cpu_abort(dc->env, "user insn undefined");
}

static void dec_wcsr(DisasContext *dc)
{
    LOG_DIS("wcsr r%d, %d\n", dc->r1, dc->csr);

    TCGv t0;

    switch(dc->csr) {
        case CSR_IE:
            tcg_gen_mov_tl(cpu_ie, cpu_R[dc->r1]);
            break;
        case CSR_IM:
            tcg_gen_mov_tl(cpu_im, cpu_R[dc->r1]);
            break;
        case CSR_IP:
            /* ack interrupt */
            t0 = tcg_temp_new();
            tcg_gen_not_tl(t0, cpu_R[dc->r1]);
            tcg_gen_and_tl(cpu_ip, cpu_ip, t0);
            tcg_temp_free(t0);
            break;
        case CSR_ICC:
            /* TODO */
            break;
        case CSR_DCC:
            /* TODO */
            break;
        case CSR_EBA:
            tcg_gen_mov_tl(cpu_eba, cpu_R[dc->r1]);
            break;
        case CSR_DEBA:
            tcg_gen_mov_tl(cpu_deba, cpu_R[dc->r1]);
            break;
        case CSR_JTX:
            tcg_gen_mov_tl(cpu_jtx, cpu_R[dc->r1]);
            break;
        case CSR_JRX:
            tcg_gen_mov_tl(cpu_jrx, cpu_R[dc->r1]);
            break;
        case CSR_DC:
            tcg_gen_mov_tl(cpu_dc, cpu_R[dc->r1]);
            break;
        case CSR_BP0:
            tcg_gen_mov_tl(cpu_bp[0], cpu_R[dc->r1]);
            break;
        case CSR_BP1:
            tcg_gen_mov_tl(cpu_bp[1], cpu_R[dc->r1]);
            break;
        case CSR_BP2:
            tcg_gen_mov_tl(cpu_bp[2], cpu_R[dc->r1]);
            break;
        case CSR_BP3:
            tcg_gen_mov_tl(cpu_bp[3], cpu_R[dc->r1]);
            break;
        case CSR_WP0:
            tcg_gen_mov_tl(cpu_wp[0], cpu_R[dc->r1]);
            break;
        case CSR_WP1:
            tcg_gen_mov_tl(cpu_wp[1], cpu_R[dc->r1]);
            break;
        case CSR_WP2:
            tcg_gen_mov_tl(cpu_wp[2], cpu_R[dc->r1]);
            break;
        case CSR_WP3:
            tcg_gen_mov_tl(cpu_wp[3], cpu_R[dc->r1]);
            break;
        case CSR_CC:
        case CSR_CFG:
            cpu_abort(dc->env, "invalid write access csr=%x\n", dc->csr);
            break;
        default:
            cpu_abort(dc->env, "write_csr unknown csr=%x\n", dc->csr);
            break;
    }
}

static void dec_xnor(DisasContext *dc)
{
    if (dc->format == OP_FMT_RI) {
        LOG_DIS("xnori r%d, r%d, %d\n", dc->r0, dc->r1,
                zero_extend(dc->imm16, 16));
    } else {
        LOG_DIS("xnor r%d, r%d, r%d\n", dc->r2, dc->r0, dc->r1);
    }

    if (dc->format == OP_FMT_RI)
    {
        tcg_gen_xori_tl(cpu_R[dc->r1], cpu_R[dc->r0],
                zero_extend(dc->imm16, 16));
        tcg_gen_not_tl(cpu_R[dc->r1], cpu_R[dc->r1]);
    } else {
        tcg_gen_xor_tl(cpu_R[dc->r2], cpu_R[dc->r0], cpu_R[dc->r1]);
        tcg_gen_not_tl(cpu_R[dc->r2], cpu_R[dc->r2]);
    }
}

static void dec_xor(DisasContext *dc)
{
    if (dc->format == OP_FMT_RI) {
        LOG_DIS("xori r%d, r%d, %d\n", dc->r0, dc->r1,
                zero_extend(dc->imm16, 16));
    } else {
        LOG_DIS("xor r%d, r%d, r%d\n", dc->r2, dc->r0, dc->r1);
    }

    if (dc->format == OP_FMT_RI) {
        tcg_gen_xori_tl(cpu_R[dc->r1], cpu_R[dc->r0],
                zero_extend(dc->imm16, 16));
    } else {
        tcg_gen_xor_tl(cpu_R[dc->r2], cpu_R[dc->r0], cpu_R[dc->r1]);
    }
}

static struct decoder_info {
    struct {
        uint32_t bits;
        uint32_t mask;
    };
    void (*dec)(DisasContext *dc);
} decinfo[] = {
    {DEC_ADD, dec_add},
    {DEC_AND, dec_and},
    {DEC_ANDHI, dec_andhi},
    {DEC_B, dec_b},
    {DEC_BI, dec_bi},
    {DEC_BE, dec_be},
    {DEC_BG, dec_bg},
    {DEC_BGE, dec_bge},
    {DEC_BGEU, dec_bgeu},
    {DEC_BGU, dec_bgu},
    {DEC_BNE, dec_bne},
    {DEC_CALL, dec_call},
    {DEC_CALLI, dec_calli},
    {DEC_CMPE, dec_cmpe},
    {DEC_CMPG, dec_cmpg},
    {DEC_CMPGE, dec_cmpge},
    {DEC_CMPGEU, dec_cmpgeu},
    {DEC_CMPGU, dec_cmpgu},
    {DEC_CMPNE, dec_cmpne},
    {DEC_DIVU, dec_divu},
    {DEC_LB, dec_lb},
    {DEC_LBU, dec_lbu},
    {DEC_LH, dec_lh},
    {DEC_LHU, dec_lhu},
    {DEC_LW, dec_lw},
    {DEC_MODU, dec_modu},
    {DEC_MUL, dec_mul},
    {DEC_NOR, dec_nor},
    {DEC_OR, dec_or},
    {DEC_ORHI, dec_orhi},
    {DEC_RAISE, dec_raise},
    {DEC_RCSR, dec_rcsr},
    {DEC_SB, dec_sb},
    {DEC_SEXTB, dec_sextb},
    {DEC_SEXTH, dec_sexth},
    {DEC_SH, dec_sh},
    {DEC_SL, dec_sl},
    {DEC_SR, dec_sr},
    {DEC_SRU, dec_sru},
    {DEC_SUB, dec_sub},
    {DEC_SW, dec_sw},
    {DEC_USER, dec_user},
    {DEC_WCSR, dec_wcsr},
    {DEC_XNOR, dec_xnor},
    {DEC_XOR, dec_xor},
};

static inline void decode(DisasContext *dc)
{
    uint32_t ir;
    int i;

    if (unlikely(qemu_loglevel_mask(CPU_LOG_TB_OP)))
        tcg_gen_debug_insn_start(dc->pc);

    dc->ir = ir = ldl_code(dc->pc);
    LOG_DIS("%8.8x\t", dc->ir);

    /* try guessing 'empty' instruction memory, although it may be a valid
     * instruction sequence (eg. srui r0, r0, 0) */
    if (dc->ir) {
        dc->nr_nops = 0;
    } else {
        LOG_DIS("nr_nops=%d\t", dc->nr_nops);
        dc->nr_nops++;
        if (dc->nr_nops > 4)
            cpu_abort(dc->env, "fetching nop sequence\n");
    }

    dc->opcode = EXTRACT_FIELD(ir, 26, 31);

    dc->imm5 = EXTRACT_FIELD(ir, 0, 4);
    dc->imm16 = EXTRACT_FIELD(ir, 0, 15);
    dc->imm26 = EXTRACT_FIELD(ir, 0, 25);

    dc->csr = EXTRACT_FIELD(ir, 21, 25);
    dc->r0 = EXTRACT_FIELD(ir, 21, 25);
    dc->r1 = EXTRACT_FIELD(ir, 16, 20);
    dc->r2 = EXTRACT_FIELD(ir, 11, 15);

    /* bit 31 seems to indicate insn type.  */
    if (ir & (1 << 31)) {
        dc->format = OP_FMT_RR;
    } else {
        dc->format = OP_FMT_RI;
    }

    /* Large switch for all insns.  */
    for (i = 0; i < ARRAY_SIZE(decinfo); i++) {
        if ((dc->opcode & decinfo[i].mask) == decinfo[i].bits) {
            decinfo[i].dec(dc);
            break;
        }
    }
}

static void check_breakpoint(CPUState *env, DisasContext *dc)
{
    CPUBreakpoint *bp;

    if (unlikely(!QTAILQ_EMPTY(&env->breakpoints))) {
        QTAILQ_FOREACH(bp, &env->breakpoints, entry) {
            if (bp->pc == dc->pc) {
                t_gen_raise_exception(dc, EXCP_DEBUG);
                dc->is_jmp = DISAS_UPDATE;
             }
        }
    }
}

/* generate intermediate code for basic block 'tb'.  */
static void
gen_intermediate_code_internal(CPUState *env, TranslationBlock *tb,
                               int search_pc)
{
    struct DisasContext ctx, *dc = &ctx;
    uint16_t *gen_opc_end;
    uint32_t pc_start;
    int j, lj;
    uint32_t next_page_start;
    int num_insns;
    int max_insns;

    qemu_log_try_set_file(stderr);

    pc_start = tb->pc;
    dc->env = env;
    dc->tb = tb;

    gen_opc_end = gen_opc_buf + OPC_MAX_SIZE;

    dc->is_jmp = DISAS_NEXT;
    dc->jmp = 0;
    dc->pc = pc_start;
    dc->singlestep_enabled = env->singlestep_enabled;
    dc->nr_nops = 0;

    if (pc_start & 3) {
        cpu_abort(env, "LM32: unaligned PC=%x\n", pc_start);
    }

    if (qemu_loglevel_mask(CPU_LOG_TB_IN_ASM)) {
        qemu_log("-----------------------------------------\n");
        log_cpu_state(env, 0);
    }

    next_page_start = (pc_start & TARGET_PAGE_MASK) + TARGET_PAGE_SIZE;
    lj = -1;
    num_insns = 0;
    max_insns = tb->cflags & CF_COUNT_MASK;
    if (max_insns == 0)
        max_insns = CF_COUNT_MASK;

    gen_icount_start();
    do {
        check_breakpoint(env, dc);

        if (search_pc) {
            j = gen_opc_ptr - gen_opc_buf;
            if (lj < j) {
                lj++;
                while (lj < j)
                    gen_opc_instr_start[lj++] = 0;
            }
            gen_opc_pc[lj] = dc->pc;
            gen_opc_instr_start[lj] = 1;
            gen_opc_icount[lj] = num_insns;
        }

        /* Pretty disas.  */
        LOG_DIS("%8.8x:\t", dc->pc);

        if (num_insns + 1 == max_insns && (tb->cflags & CF_LAST_IO))
            gen_io_start();

        decode(dc);
        dc->pc += 4;
        num_insns++;

        if (env->singlestep_enabled)
            break;
    } while (!dc->is_jmp
         && gen_opc_ptr < gen_opc_end
         && !env->singlestep_enabled
         && !singlestep
         && (dc->pc < next_page_start)
         && num_insns < max_insns);

    if (tb->cflags & CF_LAST_IO) {
        gen_io_end();
    }

    if (unlikely(env->singlestep_enabled)) {
        t_gen_raise_exception(dc, EXCP_DEBUG);
        if (dc->is_jmp == DISAS_NEXT)
            tcg_gen_movi_tl(cpu_pc, dc->pc);
    } else {
        switch(dc->is_jmp) {
            case DISAS_NEXT:
                gen_goto_tb(dc, 1, dc->pc);
                break;
            default:
            case DISAS_JUMP:
            case DISAS_UPDATE:
                /* indicate that the hash table must be used
                   to find the next TB */
                tcg_gen_exit_tb(0);
                break;
            case DISAS_TB_JUMP:
                /* nothing more to generate */
                break;
        }
    }

    gen_icount_end(tb, num_insns);
    *gen_opc_ptr = INDEX_op_end;
    if (search_pc) {
        j = gen_opc_ptr - gen_opc_buf;
        lj++;
        while (lj <= j)
            gen_opc_instr_start[lj++] = 0;
    } else {
        tb->size = dc->pc - pc_start;
        tb->icount = num_insns;
    }

#ifdef DEBUG_DISAS
    if (qemu_loglevel_mask(CPU_LOG_TB_IN_ASM)) {
        qemu_log("\n");
        log_target_disas(pc_start, dc->pc - pc_start, 0);
        qemu_log("\nisize=%d osize=%zd\n",
            dc->pc - pc_start, gen_opc_ptr - gen_opc_buf);
    }
#endif
}

void gen_intermediate_code(CPUState *env, struct TranslationBlock *tb)
{
    gen_intermediate_code_internal(env, tb, 0);
}

void gen_intermediate_code_pc(CPUState *env, struct TranslationBlock *tb)
{
    gen_intermediate_code_internal(env, tb, 1);
}

void cpu_dump_state(CPUState *env, FILE *f,
                     int (*cpu_fprintf)(FILE *f, const char *fmt, ...),
                     int flags)
{
    int i;

    if (!env || !f)
        return;

    cpu_fprintf(f, "IN: PC=%x %s\n",
                env->pc, lookup_symbol(env->pc));
    
    cpu_fprintf(f, "ie=%8.8x (IE=%x EIE=%x BIE=%x) im=%8.8x ip=%8.8x\n",
             env->ie,
             (env->ie & IE_IE) ? 1 : 0,
             (env->ie & IE_EIE) ? 1 : 0,
             (env->ie & IE_BIE) ? 1 : 0,
             env->im, env->ip);
    cpu_fprintf(f, "eba=%8.8x deba=%8.8x\n",
             env->eba,
             env->deba);

    for (i = 0; i < 32; i++) {
        cpu_fprintf(f, "r%2.2d=%8.8x ", i, env->regs[i]);
        if ((i + 1) % 4 == 0)
            cpu_fprintf(f, "\n");
        }
    cpu_fprintf(f, "\n\n");
}

CPUState *cpu_lm32_init(const char *cpu_model)
{
    CPUState *env;
    static int tcg_initialized = 0;
    int i;

    env = qemu_mallocz(sizeof(CPUState));

    cpu_exec_init(env);
    cpu_reset(env);

#define LM32_REV 3
    env->cfg = CFG_M_MASK \
               | CFG_D_MASK \
               | CFG_S_MASK \
               | CFG_X_MASK \
               | CFG_IC_MASK \
               | CFG_DC_MASK \
               | (32 << 12) \
               | (LM32_REV << 26);

    if (tcg_initialized)
        return env;

    tcg_initialized = 1;

    cpu_env = tcg_global_reg_new_ptr(TCG_AREG0, "env");

    for (i = 0; i < ARRAY_SIZE(cpu_R); i++) {
        cpu_R[i] = tcg_global_mem_new(TCG_AREG0,
                          offsetof(CPUState, regs[i]),
                          regnames[i]);
    }

    for (i = 0; i < ARRAY_SIZE(cpu_bp); i++) {
        cpu_bp[i] = tcg_global_mem_new(TCG_AREG0,
                          offsetof(CPUState, bp[i]),
                          "XXXbp");
    }

    for (i = 0; i < ARRAY_SIZE(cpu_wp); i++) {
        cpu_wp[i] = tcg_global_mem_new(TCG_AREG0,
                          offsetof(CPUState, wp[i]),
                          "XXXwp");
    }

    cpu_pc = tcg_global_mem_new(TCG_AREG0,
                    offsetof(CPUState, pc),
                    "pc");
    cpu_ie = tcg_global_mem_new(TCG_AREG0,
                    offsetof(CPUState, ie),
                    "ie");
    cpu_im = tcg_global_mem_new(TCG_AREG0,
                    offsetof(CPUState, im),
                    "im");
    cpu_ip = tcg_global_mem_new(TCG_AREG0,
                    offsetof(CPUState, ip),
                    "ip");
    cpu_icc = tcg_global_mem_new(TCG_AREG0,
                    offsetof(CPUState, icc),
                    "icc");
    cpu_dcc = tcg_global_mem_new(TCG_AREG0,
                    offsetof(CPUState, dcc),
                    "dcc");
    cpu_cc = tcg_global_mem_new(TCG_AREG0,
                    offsetof(CPUState, cc),
                    "cc");
    cpu_cfg = tcg_global_mem_new(TCG_AREG0,
                    offsetof(CPUState, cfg),
                    "cfg");
    cpu_eba = tcg_global_mem_new(TCG_AREG0,
                    offsetof(CPUState, eba),
                    "eba");
    cpu_dc = tcg_global_mem_new(TCG_AREG0,
                    offsetof(CPUState, dc),
                    "dc");
    cpu_deba = tcg_global_mem_new(TCG_AREG0,
                    offsetof(CPUState, deba),
                    "deba");
    cpu_jrx = tcg_global_mem_new(TCG_AREG0,
                    offsetof(CPUState, jrx),
                    "jrx");
    cpu_jtx = tcg_global_mem_new(TCG_AREG0,
                    offsetof(CPUState, jtx),
                    "jtx");
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

void gen_pc_load(CPUState *env, struct TranslationBlock *tb,
                 unsigned long searched_pc, int pc_pos, void *puc)
{
    env->pc = gen_opc_pc[pc_pos];
}

