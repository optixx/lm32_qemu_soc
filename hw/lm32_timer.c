/*
 *  QEMU model of the LatticeMico32 timer block.
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
 *
 *
 * Specification available at:
 *   http://www.latticesemi.com/documents/mico32timer.pdf
 */

#include "sysbus.h"
#include "sysemu.h"
#include "qemu-timer.h"

#define D(x)

#define SR_TO      1
#define SR_RUN     2

#define CR_ITO     1
#define CR_CONT    2
#define CR_START   4
#define CR_STOP    8

#define R_SR       0
#define R_CR       1
#define R_PERIOD   2
#define R_SNAPSHOT 3
#define R_MAX      4

struct lm32_timer
{
    SysBusDevice busdev;

    QEMUBH *bh;
    ptimer_state *ptimer;

    qemu_irq irq;
    uint32_t freq_hz;

    uint32_t r_status;
    uint32_t r_control;
    uint32_t r_period;
    uint32_t r_snapshot;
};

static void timer_update_irq(struct lm32_timer *t)
{
    if (t->r_status & SR_TO && t->r_control & CR_ITO) {
        qemu_set_irq(t->irq, 1);
    }
}

static uint32_t timer_read(void *opaque, target_phys_addr_t addr)
{
    struct lm32_timer *t = opaque;
    uint32_t r = 0;

    addr >>= 2;
    switch (addr)
    {
        case R_SR:
            r = t->r_status;
            break;
        case R_CR:
            r = t->r_control;
            break;
        case R_PERIOD:
            r = t->r_period;
            break;
        case R_SNAPSHOT:
            r = t->r_snapshot;
            break;
        default:
            hw_error("lm32_timer: read access to unkown register 0x"
					TARGET_FMT_plx, addr);
            break;

    }
    D(printf("%s " TARGET_FMT_plx "=%x\n", __func__, addr * 4, r));
    return r;
}

static void timer_write(void *opaque, target_phys_addr_t addr, uint32_t value)
{
    struct lm32_timer *t = opaque;

    addr >>= 2;
    D(printf("%s addr=" TARGET_FMT_plx " val=%x (off=%d)\n",
             __func__, addr * 4, value, (int)addr & 3));
    switch (addr) 
    {
        case R_SR:
            t->r_status &= ~SR_TO;
            break;
        case R_CR:
            t->r_control = value;
            if (t->r_control & CR_START) {
                ptimer_run(t->ptimer, 1);
            }
            if (t->r_control & CR_STOP) {
                ptimer_stop(t->ptimer);
            }
            break;
        case R_PERIOD:
            t->r_period = value;
            ptimer_set_count(t->ptimer, value);
            break;
        case R_SNAPSHOT:
            t->r_snapshot = (uint32_t)ptimer_get_count(t->ptimer);
            break;
        default:
            hw_error("lm32_timer: read access to unkown register 0x"
					TARGET_FMT_plx, addr);
            break;
    }
    timer_update_irq(t);
}

static CPUReadMemoryFunc* const timer_read_fn[] = {
    NULL,
    NULL,
    &timer_read,
};

static CPUWriteMemoryFunc* const timer_write_fn[] = {
    NULL,
    NULL,
    &timer_write,
};

static void timer_hit(void *opaque)
{
    struct lm32_timer *t = opaque;

    D(printf("%s\n", __func__));

    t->r_status |= SR_TO;

    if (t->r_control & CR_CONT)
    {
        ptimer_set_count(t->ptimer, t->r_period);
        ptimer_run(t->ptimer, 1);
    }

    timer_update_irq(t);
}

static int lm32_timer_init(SysBusDevice *dev)
{
    struct lm32_timer *t = FROM_SYSBUS(typeof (*t), dev);
    int timer_regs;

    sysbus_init_irq(dev, &t->irq);

    t->bh = qemu_bh_new(timer_hit, t);
    t->ptimer = ptimer_init(t->bh);
    ptimer_set_freq(t->ptimer, t->freq_hz);

    timer_regs = cpu_register_io_memory(timer_read_fn, timer_write_fn, t);
    sysbus_init_mmio(dev, R_MAX * 4, timer_regs);
    return 0;
}

static SysBusDeviceInfo lm32_timer_info = {
    .init = lm32_timer_init,
    .qdev.name  = "lm32,timer",
    .qdev.size  = sizeof(struct lm32_timer),
    .qdev.props = (Property[]) {
        DEFINE_PROP_UINT32(
                "frequency", struct lm32_timer, freq_hz, 50*1000000
        ),
        DEFINE_PROP_END_OF_LIST(),
    }
};

static void lm32_timer_register(void)
{
    sysbus_register_withprop(&lm32_timer_info);
}

device_init(lm32_timer_register)
