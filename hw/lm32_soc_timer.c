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

#define D(x) x

// Wishbone Timer
//
// Register Description:
//
//    0x00 TCR0
//    0x04 COMPARE0
//    0x08 COUNTER0
//    0x0C TCR1
//    0x10 COMPARE1
//    0x14 COUNTER1
//
// TCRx:  
//    +-------------------+-------+-------+-------+-------+
//    |     28'b0         |  EN   |  AR   | IRQEN |  TRIG |
//    +-------------------+-------+-------+-------+-------+
//
//   EN i  (rw)   if set to '1', COUNTERX counts upwards until it reaches
//                COMPAREX
//   AR    (rw)   AutoRecwstartload -- if COUNTER reaches COMPAREX, shall we 
//                restart at 1, or disable this counter?
//   IRQEN (rw)   Indicate interrupt condition when triggered?
//   TRIG  (ro)   
//
//---------------------------------------------------------------------------

#define R_TCR0          0
#define R_COMPARE0      1
#define R_COUNTER0      2
#define R_TCR1          3
#define R_COMPARE1      4
#define R_COUNTER1      5
#define R_MAX           6

#define BIT_TRIG    (1<<0)
#define BIT_IRQEN   (1<<1)
#define BIT_AR      (1<<2)
#define BIT_EN      (1<<3)


struct lm32_soc_timer
{
    SysBusDevice busdev;

    QEMUBH *bh;
    ptimer_state *ptimer;

    qemu_irq irq;
    
    uint32_t freq_hz;

    uint32_t r_trc0;
    uint32_t r_compare0;
    uint32_t r_counter0;
    uint32_t r_trc1;
    uint32_t r_compare1;
    uint32_t r_counter1;
};

static void timer_update_irq(struct lm32_soc_timer *t)
{
    //qemu_set_irq(t->irq, t->r_status & SR_TO && t->r_control & CR_ITO);
}

static uint32_t timer_read(void *opaque, target_phys_addr_t addr)
{
    struct lm32_soc_timer *t = opaque;
    uint32_t r = 0;

    addr >>= 2;
    switch (addr)
    {
        case R_TCR0:
            r = t->r_trc0;
            break;
        case R_COMPARE0:
            r = t->r_compare0;
            break;
        case R_COUNTER0:
            r = t->r_counter0;
            break;
        case R_TCR1:
            r = t->r_trc1;
            break;
        case R_COMPARE1:
            r = t->r_compare1;
            break;
        case R_COUNTER1:
            r = t->r_counter1;
            break;
        default:
            hw_error("lm32_soc_timer: read access to unkown register 0x"
					TARGET_FMT_plx, addr);
            break;

    }
    sleep(1000 * 100);
    D(printf("%s " TARGET_FMT_plx "=%x\n", __func__, addr * 4, r));
    return r;
}

static void timer_write(void *opaque, target_phys_addr_t addr, uint32_t value)
{
    struct lm32_soc_timer *t = opaque;

    addr >>= 2;
    D(printf("%s addr=" TARGET_FMT_plx " val=%x (off=%d)\n",
             __func__, addr * 4, value, (int)addr & 3));
    switch (addr) 
    {
        case R_TCR0:
            printf("TCR0=%x\n", value); 
            t->r_trc0 |= value;
            t->r_trc0 &= ~BIT_TRIG;
            if (t->r_trc0 & BIT_EN) {
                ptimer_run(t->ptimer, 1);
                printf("TCR0 start timer\n");
            }
            if (t->r_trc0 & ~BIT_EN) {
                ptimer_stop(t->ptimer);
            }
            break;
        case R_COMPARE0:
            printf("COMPARE0=%x\n",value);
            t->r_compare0 = value;
            ptimer_set_count(t->ptimer, value);
            break;
        case R_COUNTER0:
            printf("COUNTER0=%x\n",value);
            t->r_counter0 = value;
            break;
        case R_TCR1:
            t->r_trc1 |= value;
            if (t->r_trc1 & BIT_EN) {
                ptimer_run(t->ptimer, 1);
            }
            if (t->r_trc1 & ~BIT_EN) {
                ptimer_stop(t->ptimer);
            }
            break;
        case R_COMPARE1:
            t->r_compare1 = value;
            ptimer_set_count(t->ptimer, value);
            break;
        case R_COUNTER1:
            t->r_counter1 = value;
            break;
        default:
            hw_error("lm32_soc_timer: write access to unkown register 0x"
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
    struct lm32_soc_timer *t = opaque;

    D(printf("%s\n", __func__));

    t->r_trc0 |= BIT_TRIG;

    if (t->r_trc0 & BIT_AR)
    {
        ptimer_set_count(t->ptimer, t->r_compare0);
        ptimer_run(t->ptimer, 1);
    }
    
    if (t->r_trc0 & BIT_IRQEN){
        timer_update_irq(t);
    }
}

static int lm32_soc_timer_init(SysBusDevice *dev)
{
    struct lm32_soc_timer *t = FROM_SYSBUS(typeof (*t), dev);
    int timer_regs;

    sysbus_init_irq(dev, &t->irq);

    t->bh = qemu_bh_new(timer_hit, t);
    t->ptimer = ptimer_init(t->bh);
    ptimer_set_freq(t->ptimer, t->freq_hz);

    timer_regs = cpu_register_io_memory(timer_read_fn, timer_write_fn, t);
    sysbus_init_mmio(dev, R_MAX * 4, timer_regs);
    return 0;
}

static SysBusDeviceInfo lm32_soc_timer_info = {
    .init = lm32_soc_timer_init,
    .qdev.name  = "lm32_soc,timer",
    .qdev.size  = sizeof(struct lm32_soc_timer),
    .qdev.props = (Property[]) {
        DEFINE_PROP_UINT32(
                "frequency", struct lm32_soc_timer, freq_hz, 50*1000000
        ),
        DEFINE_PROP_END_OF_LIST(),
    }
};

static void lm32_soc_timer_register(void)
{
    sysbus_register_withprop(&lm32_soc_timer_info);
}

device_init(lm32_soc_timer_register)
